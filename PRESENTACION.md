# Guión de la presentación — Marc Rodríguez

Sección dentro de la presentación grupal *Projecte de Drons — UPC AEROS 2026*.
Continúa la sección de Arnau Martínez (`Searching Tello`).

**Estimación de tiempo:** 5–6 minutos.
**9 diapositivas**, formato "OPERATOR CONSOLE" siguiendo el estilo de Pedro/Arnau.

---

## Slide 1 — Portada (00:00 – 00:20)

> **"Buenos días, soy Marc Rodríguez. Continuando con el trabajo de Arnau,
> mi sección se centra en escalar la búsqueda autónoma de un dron a un
> enjambre coordinado de hasta cuatro drones."**

Visual: título grande "Tello Swarm", subtítulo "Coordinated multi-drone
search with A\* pathfinding and obstacle avoidance". Tags al pie:
`MULTI-DRONE · A* · MQTT · OBSTACLE AVOIDANCE`.

---

## Slide 2 — Objetivos (00:20 – 01:00)

> **"Partiendo del trabajo de Arnau donde un dron busca un objeto, los
> objetivos eran tres: primero, soportar varios drones a la vez con un
> mapa zenital unificado; segundo, que cuando uno detecte el objetivo
> el resto vaya a ayudarle; y tercero, que todos sepan volver a casa
> esquivando obstáculos sin chocar entre ellos."**

Visual:
- 3 cajas grandes con iconos:
  - **01 · MULTI-DRONE** — Hasta 4 drones, locales o remotos.
  - **02 · CONVERGENCIA** — Quien detecta es el "ancla"; los demás
    convergen a su posición.
  - **03 · NAVEGACIÓN SEGURA** — A\* sobre obstáculos + separación 3D.

---

## Slide 3 — Arquitectura del sistema (01:00 – 01:45)

> **"El sistema se estructura en 4 capas. Abajo, la abstracción de drones
> que permite intercambiar dron real, simulado o remoto sin tocar la
> lógica. En medio, el Fleet que mantiene el estado compartido del
> enjambre. Encima, la lógica de misión: búsqueda, convergencia y
> retorno. Y arriba, la UI con el mapa zenital donde se ve todo. El
> bridge MQTT, opcional, permite que dos PCs compartan flota."**

Visual: diagrama de cajas en pirámide invertida:
```
        ┌───────────────────────┐
        │   UI (Tkinter + Mapa) │
        └───────────┬───────────┘
                    │
        ┌───────────┴───────────┐
        │  mission_logic / v2   │
        │  YOLO · A* · Conv     │
        └───────────┬───────────┘
                    │
        ┌───────────┴───────────┐
        │   Fleet (estado +     │◄──MQTT──► peer
        │   eventos)            │
        └───────────┬───────────┘
                    │
        ┌───────────┴───────────┐
        │  drone_iface          │
        │  Real / Fake / Remote │
        └───────────────────────┘
```

Tags al pie: `PYTHON 3.10 · TKINTER · DJITELLOPY · YOLOV8 · PAHO-MQTT`

---

## Slide 4 — Detección de objetos (01:45 – 02:30)

> **"La detección reutiliza el código de Arnau: YOLOv8n con la base de
> datos COCO, 80 categorías, excluyendo 'persona'. La selección inicial
> se hace con la webcam del portátil, exigiendo 50 frames consecutivos
> para confirmar el objeto y evitar falsos positivos. Una vez en vuelo,
> el dron real ejecuta YOLO sobre su feed de vídeo y, cuando detecta
> el objeto durante 5 frames seguidos, lo reporta a la flota."**

Visual:
- Columna izquierda: snapshot de la ventana de detección de Arnau (con
  caja roja sobre el objeto y contador `[N/50]`).
- Columna derecha: snapshot de la ventana del dron en vuelo, con el HUD
  `TARGET cell phone [3/5]  conf=0.83`.
- Tag inferior: `YOLOV8N · COCO · CONF 0.5 · 5 FRAMES CONFIRM`.

**🎬 QR a vídeo 1:** *Detección de imagen + inicio de misión + vista en
panel*

---

## Slide 5 — Lógica de búsqueda multi-dron (02:30 – 03:15)

> **"Cada dron arranca en una dirección cardinal distinta gracias a la
> función spread_headings. Hace un barrido tipo serpiente: gira 30
> grados a un lado, 60 al otro, 30 al centro, avanza 75 cm, y repite.
> Si detecta un obstáculo delante con segment_crosses_obstacle, gira
> 90 grados y reintenta. La cámara del dron real va escaneando con
> YOLO en paralelo."**

Visual: diagrama mostrando 4 drones saliendo en cruz desde el origen,
con flechas curvas indicando rotation_iteration. Bloque inferior con
la pseudo-código del bucle:

```
loop:
  if obstacle ahead → rotate 90°
  else → rotate_sweep + forward 75 cm
```

Tag inferior: `4 DIRECTIONS · ROTATION SWEEP · OBSTACLE AVOID`

---

## Slide 6 — Convergencia + Pathfinding A\* (03:15 – 04:15)

> **"Cuando un dron detecta el objetivo, marca su posición como
> target_found_at. Los demás lo ven en el siguiente ciclo y entran en
> modo CONVERGING. Aquí entra el A\*: construimos un grafo de
> visibilidad con los extremos de cada obstáculo desplazados
> perpendicularmente. Los nodos son alcanzables si la línea entre ellos
> no cruza ningún muro y mantiene la clearance mínima. A\* devuelve la
> ruta más corta. Si está bloqueada, hay 3 niveles de fallback,
> empezando por la opción más holgada y bajando a clearance reducida
> solo si no hay otra."**

Visual: diagrama del grafo de visibilidad superpuesto a un mapa con
muros:
- Drone (rojo) en una esquina.
- Target (cruz roja) al otro lado de 2 muros.
- Líneas finas grises = aristas del grafo de visibilidad.
- Línea verde gruesa = camino A\* elegido.
- Nodos amarillos = puntos perpendiculares en los extremos de cada
  obstáculo.

Bloque inferior con tags: `VISIBILITY GRAPH · A* · 3-LEVEL FALLBACK`.

**🎬 QR a vídeo 2:** *Esquiva de obstáculos + SIM DETECT en un fake + el
dron real va a buscarlo*

---

## Slide 7 — Seguridad 3D + Return to Home (04:15 – 05:00)

> **"Para evitar que dos drones convergiendo al mismo punto choquen,
> usamos un criterio determinista por nombre: alpha sube por encima,
> delta baja por debajo. Cada dron aplica la misma regla, así no hay
> conflicto. El Return to Home funciona igual que la convergencia: usa
> A\* desde la posición actual al origen. Esto permite que el dron
> vuelva a casa por la ruta más corta sin chocar con muros, incluso
> si está muy lejos."**

Visual:
- Columna izquierda: vista lateral de 4 drones, alpha arriba, delta
  abajo, separados 50 cm en altura. Etiqueta: "Capas por nombre".
- Columna derecha: vista cenital. Dron en un punto lejano, línea
  punteada del A\* serpenteando entre muros hasta el origen.

Tags: `BY-NAME LAYERING · DIRECT A* RTH · NO COLLISIONS`.

**🎬 QR a vídeo 3:** *Dron desde un punto lejano volviendo al origen*

---

## Slide 8 — Comportamiento ante diferentes situaciones (05:00 – 05:30)

> **"Una breve tabla del comportamiento: si no hay obstáculo, va recto;
> si hay un muro, lo rodea con un waypoint; si hay un laberinto, A\*
> encuentra la ruta multi-hop; y si está atrapado contra una pared,
> entra en modo escape que permite salir aunque pase rozando. Si está
> literalmente encerrado en una caja, hover y avisa."**

Visual: tabla 4×3 con casos:

| Situación              | Resultado                            |
|-----------------------|--------------------------------------|
| Camino libre          | Línea recta                          |
| 1 muro                | Rodeo de 1 waypoint                  |
| Laberinto             | A\* multi-hop                        |
| Pegado a una pared    | Escape mode (clearance reducida)     |
| Encerrado             | Hover + log de aviso, no atraviesa   |

---

## Slide 9 — Conclusiones (05:30 – 06:00)

> **"Conclusiones rápidas. Conseguido: enjambre de 4 drones con
> convergencia, A\* y RTH. Lo que el Tello hace difícil: sin GPS ni
> odometría XY fiable, todo es dead-reckoning, que diverge con cada
> rotación. Como mejora pendiente: añadir un Kalman para fusionar
> visión y posición estimada, o pasarlo a Tello con tracking externo
> tipo OptiTrack. Aplicaciones reales: búsqueda en interiores tras un
> sismo, mapeo de almacenes, inspección de naves industriales."**

Visual: 3 columnas:

**ACHIEVED**
- 4-drone swarm
- A\* path planning
- RTH with obstacles
- 3D safety layering
- MQTT multi-PC

**TELLO LIMITS**
- No GPS
- No XY odometry
- Dead reckoning drift
- Limited camera FoV
- Discrete rotation commands

**FUTURE / REAL-WORLD USE**
- Kalman filter fusion
- External tracking (OptiTrack)
- Indoor search & rescue
- Warehouse mapping
- Industrial inspection

Tag al pie: `BUILD STATUS · MULTI-DRONE COMPLETE`.

---

## Demos en vivo / fallback

Si los vídeos no cargan los QR, **demo en directo**:

1. Lanza `python main.py`. Conecta 4 fakes, despega, escoge OBJETO,
   START. Espera 20 s a que se esparzan.
2. Pulsa SIM en alpha. Los otros 3 convergen visiblemente.
3. Dibuja un muro entre alpha y un converging (con DRAW OBS). El path
   se recalcula en pantalla.
4. Pulsa HOME en cualquiera. Vuelve esquivando.
5. FULL RESET para limpiar y empezar de nuevo.

Tiempo total demo: ~2 minutos.

---

## Notas para vídeos a grabar

| # | Vídeo                                | Duración | Slide |
|---|-------------------------------------|---------|-------|
| 1 | Detección de imagen + inicio misión | 30-45s  | 4     |
| 2 | Muros + SIM en fake + real converge | 45-60s  | 6     |
| 3 | RTH desde punto lejano              | 30-45s  | 7     |

**Tips de grabación:**
- Graba la pantalla con OBS o ShareX en 1080p.
- Subraya cursor visual (en Windows: opción "Cursor visual" del PowerToys o
  "Mouse pointer trails").
- Voz en off opcional explicando lo que se hace.
- Sube los vídeos a YouTube (puede ser "No listado") y genera QR de cada
  URL con `https://www.qr-code-generator.com/` o similar. Sustituye los
  placeholders del .pptx.
