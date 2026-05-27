# 🚁 Tello Multi-Drone Control System

## Objetivo

Sistema autónomo para múltiples drones DJI Tello que:
- Buscan un objeto simultáneamente
- Cuando uno lo encuentra, los demás convergen a esa posición
- Todos esperan comando manual de aterrizaje
- Visualizan sus trayectorias en tiempo real en un mapa zenital

---

## Arquitectura

### Capa 1: Abstracción de drones (`drone_iface.py`)

```python
DroneInterface          # contrato común
├─ RealTello           # wrapper sobre djitellopy.Tello (import lazy)
└─ FakeTello           # simulador sin hardware (velocidades reales)
```

**Características:**
- `RealTello` solo importa djitellopy cuando se instancia → permite tests sin instalarlo
- `FakeTello` simula tiempos reales (50 cm/s forward, 90 °/s rotate)
- Ambas generan frames sintéticos para `cv2.imshow()`

---

### Capa 2: Estado compartido (`fleet.py`)

```python
Fleet                   # registro central + bus de eventos
├─ drones[id]          # DroneState por dron
│  ├─ drone            # instancia DroneInterface
│  ├─ tracker          # DroneTracker (dead-reckoning)
│  ├─ status           # DroneStatus (enum)
│  ├─ color            # color asignado (cyan/rojo/ámbar/púrpura)
│  ├─ is_real          # True si corre YOLO
│  └─ blinking         # parpadea si encontró
├─ target_found_at     # (x, y, finder_id) o None
├─ events              # queue.Queue de eventos
└─ methods
   ├─ add_drone()
   ├─ set_status()
   ├─ emit_pose()
   ├─ report_target_found()
   └─ spread_headings() # reparte heading inicial (0°/90°/180°/270°)
```

**DroneStatus enum:**
```
IDLE → CONNECTED → TAKING_OFF → SEARCHING
                                    ├─→ CONVERGING → AT_TARGET (hover esperando land)
                                    └─→ FOUND (parpadea, hover esperando land)
                                        ↓
                                    LANDING → LANDED
```

**DroneTracker (dead-reckoning):**
- Integra rotaciones (`rotate_cw`, `rotate_ccw`)
- Integra avances (`move_forward`)
- Mantiene `path = [(x, y), ...]` para dibujar trayectoria
- Thread-safe (Lock interno)

---

### Capa 3: Lógica de misión (`mission_logic.py`)

```python
run_drone_mission(fleet, drone_id, stop_event)
├─ Búsqueda: rotation_iteration (6 rotaciones + avance 75 cm)
├─ Detección: comprueba fleet.get_target()
├─ Convergencia: si otro encontró → converge_to()
└─ Estados terminales: FOUND/AT_TARGET → hover

converge_to(fleet, drone_id, tx, ty, stop_event)
├─ Calcula bearing a (tx, ty) desde posición actual
├─ Gira al ángulo más corto
├─ Avanza en chunks (SDK admite 20-500 cm)
└─ Status final: AT_TARGET
```

**Parámetros búsqueda:**
- `SEARCH_ROTATION = 30°` (sub-giros de la rotación)
- `SEARCH_FORWARD = 75 cm` (avance entre iteraciones)

---

### Capa 4: UI (`main.py`)

```
┌─────────────────────────────────────────────────────────┐
│ TELLO CONTROL CENTER · MULTI                      status │
├──────────────────────────────────────────────────────────┤
│ ┌────┐ ┌────┐ ┌────┐ ┌────┐                             │
│ │ALFA│ │BRAV│ │CHAR│ │DELT│   ← 4 tarjetas de dron     │
│ │... │ │... │ │... │ │... │                             │
│ └────┘ └────┘ └────┘ └────┘                             │
├──────────────────────────────────────────────────────────┤
│ TARGET: PERSON  [SELECT]  [▶ START]  [REC OFF]  ALT:100 │
├──────────────────────────────────────────────────────────┤
│ ┌────────────────┐  ┌──────────────────────┐             │
│ │ LOG            │  │  MAPA ZENITAL        │             │
│ │                │  │  (paths multi-dron)  │             │
│ │                │  │  (finder parpadeando)│             │
│ └────────────────┘  └──────────────────────┘             │
└──────────────────────────────────────────────────────────┘
```

**Cada tarjeta de dron:**
- Selector TIPO (REAL/FAKE)
- Campo IP (solo si REAL)
- Indicador batería + estado
- Botones: CONNECT, ↑ TAKEOFF, ↓ LAND, 🐛 SIM DETECT
- Eventos: actualiza desde `fleet.events` queue

**Mapa zenital:**
- Cuadrícula cada 100 cm con escala automática
- Trayectoria de cada dron en su color asignado
- Triángulo apuntando en heading actual (parpadea si FOUND)
- Cruz roja marcando posición del target
- Barra de escala adaptativa

---

## Flujo de uso (prueba sin hardware)

### Setup
```bash
# Instalar dependencias
pip install ultralytics opencv-python

# Lanzar
python main.py
```

### Pasos

1. **Conectar drones**
   - Cada tarjeta: deja `TIPO=FAKE`, pulsa `CONNECT` (4×)
   - Estado → `CONNECTED`, batería actualiza cada 10s

2. **Despegar**
   - Pulsa `↑` en cada tarjeta
   - Altura por defecto 100 cm (ajustable arriba)
   - Heading inicial: 0°, 90°, 180°, 270° (reparte automáticamente)
   - Estado → `SEARCHING`

3. **Seleccionar objeto**
   - Botón global `[SELECT]` abre webcam del portátil
   - Apunta a un objeto durante 50 frames consecutivos
   - Se confirma → estado `TARGET: nombre`

4. **Iniciar misión**
   - Botón `[▶ START]` lanza `run_drone_mission()` por cada dron
   - Todos hacen rotation_iteration en paralelo
   - Se visualiza en mapa: paths separándose

5. **Simular detección**
   - En tarjeta alfa: pulsa `[🐛 SIM DETECT]`
   - Estado alfa → `FOUND` (parpadea, cruz roja en su posición)
   - Bravo/Charlie/Delta → estado `CONVERGING` (van hacia alfa)
   - Cuando llegan → estado `AT_TARGET` (hover)
   - Todos esperan comando manual de land

6. **Aterrizar**
   - Pulsa `↓` en cada tarjeta **individualmente**
   - Estado → `LANDING` → `LANDED`

---

## Características implementadas

| Aspecto | Estado | Notas |
|--------|--------|-------|
| **Multi-dron** | ✅ | Hasta 4 drones simultáneamente |
| **Abstracción** | ✅ | DroneInterface + RealTello/FakeTello |
| **Dead-reckoning** | ✅ | Tracker con x, y, heading, path |
| **Búsqueda** | ✅ | rotation_iteration + avance |
| **Convergencia** | ✅ | Línea recta a coordenadas |
| **Mapa zenital** | ✅ | Multi-dron, colores, parpadeo |
| **UI tarjetas** | ✅ | 4 tarjetas compactas, REAL/FAKE |
| **Bus eventos** | ✅ | Fleet → UI via queue + after() |
| **YOLO (RealTello)** | ⏳ | Paso 4 |
| **Grabación video** | ⏳ | Paso 4 |
| **Reverse-path** | ⏳ | Paso 5 |

---

## Archivos del proyecto

```
drone_iface.py      (Paso 1) Abstracción de drones
fleet.py            (Paso 2) Estado compartido + eventos
mission_logic.py    (Paso 3) Búsqueda + convergencia
main.py             (Paso 3) UI con tarjetas + mapa
test_fleet.py       (Prueba) Test integrado con 2 fakes
test_fake.py        (Prueba) Test unitario de FakeTello

detection.py        (Existente) YOLO webcam para seleccionar objeto
mission_v2.py       (Existente, sin tocar aún)
mission_good.py     (Existente, sin usar)
```

---

## Paso 4 (próximo)

**Objetivo:** Integrar YOLO en RealTello

1. Adaptar `mission_v2.py` para recibir `fleet` y `drone_id`
2. Solo drones con `is_real=True` ejecutan YOLO en `mission_loop()`
3. Drones fake continúan con rotation_iteration sin YOLO
4. Video preview en ventana `cv2.imshow()`
5. Grabación de video si `record_enabled=True`

**Cambios esperados:**
- `mission_v2.py` reescrito para multi-dron + FSM
- `main.py` lanza `mission_v2.mission_loop()` en lugar de `mission_logic.run_drone_mission()` para RealTello
- Fakes siguen usando `mission_logic`

---

## Paso 5 (futuro)

**Objetivo:** Reverse-path en lugar de línea recta

Cuando un dron converge, en lugar de ir directo a (tx, ty):
1. Reproducir el path del finder al revés
2. Llegar al mismo punto habiendo seguido su trayectoria inversa
3. Más "realista" pero más lento

---

## Notas técnicas

- **Heading:** 0° = norte (y+), sentido horario
- **Bearing:** `atan2(dx, dy)` porque y es norte
- **Giro más corto:** si rotación > 180°, gira CCW; si no, CW
- **Thread-safety:** `DroneTracker` usa Lock; `Fleet` usa RLock
- **Eventos:** queue sin tamaño máximo, consume cada 100 ms
- **Batería:** polling cada 10s por dron
- **Spread headings:** llamado tras cada takeoff para redistribuir
