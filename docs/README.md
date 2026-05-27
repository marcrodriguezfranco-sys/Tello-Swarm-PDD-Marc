# Tello Multi-Drone Swarm

Sistema de control multi-dron para DJI RoboMaster TT / Tello EDU, con búsqueda
autónoma de objetos, convergencia coordinada, evasión de obstáculos mediante
A\* sobre grafo de visibilidad y modo distribuido vía MQTT.

Proyecto de la asignatura **Projecte de Drons** 

- **Multi-dron coordinado** (hasta 4 drones simultáneos, locales o remotos).
- **Convergencia automática**: cuando un dron detecta el objetivo, los demás
  navegan hacia él.
- **Pathfinding A\*** sobre grafo de visibilidad para evitar obstáculos.
- **Return to Home** directo con esquiva de obstáculos.
- **Seguridad 3D**: separación vertical  entre los drones  por nombre (alpha estará más
  alto, delta más bajo).
- **Modo distribuido**: dos PCs pueden compartir flota mediante MQTT.

---

## Tabla de contenidos

1. [Instalación](#instalación)
2. [Ejecución rápida](#ejecución-rápida)
3. [Arquitectura](#arquitectura)
4. [Configuración multi-dron real](#configuración-multi-dron-real-station-mode)
5. [Modo distribuido (MQTT)](#modo-distribuido-mqtt)
6. [Cómo usar la UI](#cómo-usar-la-ui)
7. [Limitaciones conocidas](#limitaciones-conocidas)
8. [Créditos](#créditos)

---

## Instalación

Requisitos: **Python 3.10+**, **Windows / Linux / macOS**.

```bash
git clone <url-repo>
cd V3_Tello_Arnau
pip install -r requirements.txt
```

Dependencias principales:

| Paquete            | Para qué                                            |
|--------------------|-----------------------------------------------------|
| `djitellopy`       | SDK del Tello (solo si conectas dron real)          |
| `ultralytics`      | YOLOv8 para detección de objetos                    |
| `opencv-python`    | Captura de vídeo + dibujado                         |
| `numpy`            | Arrays para los frames del Tello                    |
| `paho-mqtt`        | Bridge multi-PC (opcional)                          |

**El modelo `yolov8n.pt` se descarga la primera vez** que ejecutas la app
(unos 6 MB, lo descarga Ultralytics).

---

## Ejecución rápida

### Sin hardware (solo simulación)

```bash
python main.py
```

1. En las 4 tarjetas (`alpha`, `bravo`, `charlie`, `delta`) deja `TYPE = FAKE`
   y pulsa `CONN` en cada una.
2. Pulsa `↑` en cada tarjeta para que despeguen (a alturas escalonadas y vertical_safety=50 en mission_logic:
   alpha=250, bravo=200, charlie=150, delta=100 cm).
3. Pulsa `ESCOGER OBJETO`, apunta la webcam del portátil a algo (botella,
   teléfono, etc.) durante 50 frames consecutivos.
4. Pulsa `START`. Los drones empiezan rotation_iteration en sus 4
   direcciones cardinales.
5. Pulsa ` SIM` en cualquier tarjeta para simular que ese dron detectó
   el objetivo. Los otros 3 convergerán hacia él respetando obstáculos
   y seguridad 3D.
6. Pulsa `↩ HOME` para volver a casa por la ruta más corta esquivando
   muros (A\*).

### Con dron real

Conecta el PC al WiFi del Tello, ejecuta:

```bash
python main.py
```

En la tarjeta deseada cambia `TYPE = REAL`, deja `IP` vacío (usa el AP por
defecto 192.168.10.1) y pulsa `CONN`. Aparecerá una ventana de vídeo
`Drone <id> - Vision` con la detección YOLO superpuesta.

Para **2 drones reales a la vez en un PC**, ver [configuración multi-dron real](#configuración-multi-dron-real-station-mode).

---

## Arquitectura

```
┌──────────────────────────────────────────────────────────────┐
│ main.py                                                       │
│ ───────                                                       │
│ UI Tkinter: 4 tarjetas, mapa zenital, controles globales      │
│ Wire-up de events Fleet → UI                                  │
└──────────────────┬───────────────────────────────────────────┘
                   │
        ┌──────────┴──────────┐
        │                     │
┌───────▼────────┐    ┌──────▼─────────┐
│ mission_logic  │    │   mission_v2   │
│ ──────────────  │    │   ──────────    │
│ run_drone_      │    │ run_real_drone │
│  mission        │    │  _vision       │
│ converge_to     │    │ (YOLO loop)    │
│ return_to_      │    │                │
│  origin         │    │                │
│ execute_manual_ │    │                │
│  nav            │    │                │
│ _astar_         │    │                │
│  visibility     │    │                │
└───────┬────────┘    └──────┬─────────┘
        │                     │
        └─────────┬───────────┘
                  │
        ┌─────────▼──────────┐         ┌─────────────────┐
        │     fleet.py        │◄────────►   mqtt_bridge   │
        │     ─────────       │         │   ──────────    │
        │ Fleet (state)       │         │ paho-mqtt       │
        │ DroneTracker        │         │ RemoteTello sync│
        │ DroneState          │         │ Obstáculos +    │
        │ EventBus (queue)    │         │ target found    │
        └─────────┬──────────┘         └─────────────────┘
                  │
        ┌─────────▼──────────┐
        │   drone_iface.py    │
        │  ───────────────    │
        │ DroneInterface      │
        │ ├─ RealTello        │  ← djitellopy + cmd_lock
        │ ├─ FakeTello        │  ← simulación con time.sleep
        │ └─ RemoteTello      │  ← peer remoto vía MQTT
        └────────────────────┘
```

### Componentes clave

| Módulo            | Responsabilidad                                            |
|-------------------|------------------------------------------------------------|
| `main.py`         | UI completa (Tkinter), event loop, mapa zenital            |
| `fleet.py`        | Estado compartido del enjambre, eventos, obstáculos        |
| `mission_logic.py`| Búsqueda, convergencia, RTH, A\*, seguridad 3D             |
| `mission_v2.py`   | Visión YOLO para drones reales                             |
| `drone_iface.py`  | Abstracción Real / Fake / Remote                           |
| `detection.py`    | Selección inicial del objetivo (webcam)                    |
| `image_detect.py` | Detección sobre imagen estática (DETECT IMG)               |
| `mqtt_bridge.py`  | Sincronización multi-PC vía MQTT                           |

---

## Configuración multi-dron real (station mode)

Para tener 2 Tello EDU/TT a la vez en un mismo PC, ambos deben unirse a un
**router WiFi** (modo estación). Por defecto cada Tello expone su propio AP
y el PC solo puede unirse a uno.

### Una sola vez por dron

1. Enciende un solo Tello. Conecta el PC a su WiFi (`TELLO-XXXXXX`).
2. Ejecuta:
   ```bash
   python configure_tello_raw.py "MiRouter" "miPassword"
   ```
   El SSID debe ser **sin acentos ni espacios** y el router debe estar en
   **2.4 GHz** (los Tello no aceptan 5 GHz).
3. El dron responde `OK,drone will reboot in 3s` y se reinicia.
4. Repite con el siguiente dron.

### En cada uso

1. Conecta el PC al router.
2. Averigua la IP de cada Tello (panel del router → DHCP clients).
3. En la app, en cada tarjeta: `TYPE = REAL`, IP = la del dron, `CONN`.
4. Cada dron real obtiene un puerto de vídeo único (alpha=11111, bravo=11112,
   charlie=11113, delta=11114) → los streams no chocan.

---

## Modo distribuido (MQTT)

Permite que **dos PCs distintos** compartan vista de la flota. Útil cuando
no se puede juntar todos los drones en un solo PC (p. ej. uno tiene un
Tello viejo que no soporta station mode).

### Setup

1. **En uno de los PCs** (o un tercero), instala y lanza un broker MQTT:
   ```bash
   # Windows: descarga mosquitto desde https://mosquitto.org/download/
   mosquitto -v
   ```
2. En la app, panel **MODO DISTRIBUIDO**:
   - `BROKER`: IP del PC con mosquitto
   - `PEER ID`: identificador único de este PC (`pc1`, `pc2`...)
   - Pulsa `BRIDGE`
3. Los drones del otro PC aparecen en el mapa con **borde gris**. Su pose,
   estado, batería y altitud se sincronizan en tiempo real. Los obstáculos
   se comparten bidireccionalmente.

### Qué se sincroniza

| Cosa                       | Sincronizado |
|----------------------------|:------------:|
| Pose (x, y, z, heading)    | ✅           |
| Status del dron            | ✅           |
| Batería                    | ✅           |
| Target encontrado          | ✅           |
| Obstáculos                 | ✅ bidireccional |
| Comandos (takeoff, land…)  | ❌ (cada peer controla los suyos) |
| Pausa de misión            | ❌ (local)   |

---

## Cómo usar la UI

### Layout

```
┌─────────────────────────────────────────────────────────────┐
│ TELLO CONTROL CENTER · MULTI                          status │
├─────────────────────────────────────────────────────────────┤
│ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐                              │
│ │ALPHA│ │BRAVO│ │CHARL│ │DELTA│   ← 4 tarjetas              │
│ └─────┘ └─────┘ └─────┘ └─────┘                              │
├─────────────────────────────────────────────────────────────┤
│ OBJETO | START | REC | MISIÓN                                │
│ INFO   | DETECT IMG                                          │
│ DRAW OBS | CLEAR OBS | FULL RESET                            │
│ MQTT bridge                                                  │
├──────────────────────┬──────────────────────────────────────┤
│ LOG                  │   MAPA ZENITAL                        │
│                      │   ─ trayectorias                      │
│                      │   ▲ drones (color por slot)           │
│                      │   ─ obstáculos (naranja)              │
│                      │   ⊕ target (parpadea si finder)       │
└──────────────────────┴──────────────────────────────────────┘
```

### Tarjeta de dron

| Botón     | Función                                                |
|-----------|--------------------------------------------------------|
| `CONN`    | Conecta al dron (Real o Fake según el dropdown)        |
| `↑`       | Despegar a la altura del campo ALTURA + offset por slot|
| `↓`       | Aterrizar                                              |
| `↩ HOME`  | Volver al origen (0,0) con A\* esquivando obstáculos   |
| `✎`       | Editar pose (x, y, z, heading). En suelo: declara pose inicial. En aire: navega allí. |
| `SIM`     | Simular detección (para testing convergencia)          |

### Mapa

| Acción                          | Resultado                       |
|----------------------------------|---------------------------------|
| Rueda del ratón                 | Zoom (centrado en el cursor)    |
| Click derecho + arrastrar       | Pan                             |
| Click sobre un triángulo        | Abre PoseDialog del dron        |
| Click+arrastrar (con DRAW OBS ON) | Dibuja un muro                |
| Botón `⤢ FIT`                   | Reset zoom+pan al auto-fit      |

### Botones de reset

| Botón         | Qué hace                                                  |
|---------------|-----------------------------------------------------------|
| `↺ MISIÓN`    | Reinicia la misión: trackers a (0,0), drones en hover. Mantiene conexiones, objetivo y obstáculos. |
| `✕ FULL RESET`| Desconecta todos los drones. Mantiene objetivo y obstáculos. |

---

## Limitaciones conocidas

- **Sin GPS ni odometría XY**: el Tello no tiene posicionamiento absoluto.
  Toda la posición es **dead-reckoning** (es decir el dron irá integrando y calculando su posición con los comandos enviados).
  Tras varios giros la posición real diverge ligeramente de la estimada acumulando error.
- **A\* es 2D**: los obstáculos son segmentos en el plano. No hay
  obstáculos volumétricos ni alturas (toda evasión es horizontal o
  ajuste de altitud para drones).
- **Pausa no se sincroniza por MQTT**: cada peer pausa solo sus drones
  locales. No hay paro de emergencia global.
- **Solo Tello EDU/TT pueden ir en grupo en un PC**: el Tello estándar
  solo expone su AP y no acepta el comando `wifi`.

---

## Créditos y Anexo:

- **Federico Pompeo** — Reconstrucción 3D.
- **Pedro Fiol** — Tello Interceptor (intercepción autónoma ).
- **Arnau Martínez** — Searching Tello (búsqueda mono-dron base).
- **Marc Rodríguez** — Multi-dron, convergencia, A\*, MQTT, RTH.

https://www.google.com/url?q=https://youtu.be/_VPuLkShhwY&sa=D&source=editors&ust=1779874888170564&usg=AOvVaw0f_CWWjI9DXOVGBOUfVhui
https://www.google.com/url?q=https://youtu.be/u7ULQgY_3qo&sa=D&source=editors&ust=1779874906385968&usg=AOvVaw0E-TdYoEV3BGCXFDlV13BE
https://www.google.com/url?q=https://youtu.be/8YznvRXAbGc&sa=D&source=editors&ust=1779874924104877&usg=AOvVaw3Is3yHv3G8NSsYZ13mRYOb

**Universitat Politècnica de Catalunya — AEROS**
*Projecte de Drons — 2026 Q2*
