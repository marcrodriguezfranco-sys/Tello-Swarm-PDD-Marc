# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project does

Multi-drone search-and-converge system for the DJI Tello / RoboMaster TT. The operator selects a target object via the laptop webcam + YOLOv8, then up to **4 drones** (any mix of real Tellos and simulated `FakeTello` instances) take off and search in parallel. When one drone detects the target, the others stop searching and **converge to its position by following its trajectory in reverse** (reverse-path), routing around any obstacles drawn on the map. All drones wait for manual landing.

The system also supports **distributed operation** across multiple PCs via an MQTT bridge: each PC runs its own `main.py` with its local drones, and remote drones from peer PCs appear on the same zenithal map.

## Running the application

```bash
# (Connect to Tello WiFi if you have a real drone, then:)
python main.py
```

Install required dependencies:
```bash
pip install opencv-python numpy ultralytics    # core
pip install djitellopy                          # only needed for real Tellos
pip install paho-mqtt                           # only needed for MQTT bridge (distributed mode)
```

`djitellopy` and `paho-mqtt` are imported lazily — the app launches even if they are not installed; you only get an error when you try to create a `RealTello` or activate the MQTT bridge.

Model weights `yolov8n.pt` must be present in the project root.

## Architecture

```
main.py                ← tkinter GUI, entry point
  ├─ drone_iface.py    ← DroneInterface + RealTello / FakeTello / RemoteTello
  ├─ fleet.py          ← Fleet, DroneState, DroneTracker, Obstacle, event bus
  ├─ mission_logic.py  ← movement loop per drone (rotation_iteration + reverse-path)
  ├─ mission_v2.py     ← YOLO vision loop (only for REAL drones)
  ├─ detection.py      ← webcam-based object selection (YOLOv8)
  ├─ image_detect.py   ← detect objects in a static image from disk
  └─ mqtt_bridge.py    ← optional: sync Fleet across PCs via MQTT
```

### Drone abstraction (`drone_iface.py`)

All flight code targets `DroneInterface`:
- `RealTello` — wrapper over `djitellopy.Tello` (lazy import).
- `FakeTello` — in-process simulator: `time.sleep` proportional to real Tello speeds (50 cm/s forward, 90 °/s rotate), synthetic 960×720 RGB frames, battery drain.
- `RemoteTello` — represents a drone physically owned by another PC. All movement methods are no-ops; pose and status arrive via MQTT. Drawn with a grey outline on the map.

### Fleet state (`fleet.py`)

`Fleet` is the single source of truth, thread-safe (RLock). Holds:
- `drones[drone_id] → DroneState` (drone, tracker, status, color, is_real, blinking)
- `target_found_at: (x, y, finder_id) | None`
- `obstacles: List[Obstacle]` — line segments the drones must avoid
- `events: queue.Queue[Event]` — bus consumed by the UI

`DroneTracker` integrates rotations and forward moves into world coordinates (`x` east, `y` north, heading 0° = north, clockwise). The `path` list is the history of waypoints — the reverse-path converger reads this.

`segment_crosses_obstacle(p1, p2)` returns the first `Obstacle` segment that intersects the trajectory, or `None`.

### States (`DroneStatus`)

`IDLE → CONNECTED → TAKING_OFF → SEARCHING → {CONVERGING → AT_TARGET | FOUND} → LANDING → LANDED`

- `FOUND` — this drone detected the target. Blinks on the map. Hovers awaiting manual land.
- `AT_TARGET` — converged onto the finder's position. Hovers awaiting manual land.
- Manual landing per drone — there is no auto-land.

### Movement loop (`mission_logic.py`)

`run_drone_mission(fleet, drone_id, stop_event)` runs in its own thread per drone:
1. If status is `FOUND` or `AT_TARGET`, hover.
2. If another drone reported `target_found` and we haven't converged, call `converge_to()`.
3. Otherwise execute one `rotation_iteration` (zigzag of 30° sub-turns) and advance 75 cm — but only if `fleet.segment_crosses_obstacle` says the forward segment is clear; if not, rotate 90° CW and retry next iteration.

`converge_to(fleet, drone_id, finder_id, stop_event)` does **reverse-path** convergence:
1. Read the finder's `tracker.path`.
2. Find the waypoint in that path closest to the follower's current position (the "entry point").
3. Replay the path from the entry point to the finder's last position.
4. For each segment, check obstacles via `_avoid_obstacle`. If a segment crosses a wall, insert one perpendicular-offset waypoint of detour (`OBSTACLE_MARGIN = 60 cm`).
5. Each leg is flown by `_go_to_point` (rotate to bearing then `move_forward` in ≤ 500 cm chunks).

### Vision loop (`mission_v2.py`)

`run_real_drone_vision(fleet, drone_id, stop_event, target_class_id)` only runs for `state.is_real == True`. Loads YOLOv8n once (shared across real drones via a module-level lock), reads frames, draws bounding boxes, shows them in an `cv2.imshow` window per drone, optionally writes to `records/mission_<drone_id>_<ts>.mp4`. When the target class is detected with confidence ≥ 0.5, it calls `fleet.report_target_found(drone_id)`, which triggers convergence of the rest of the fleet.

Fake drones never run this loop. Use the `SIM DETECT` button in their card to inject a fake detection.

### GUI (`main.py`)

- 4 drone cards (alpha/bravo/charlie/delta) — each with TYPE (FAKE/REAL), IP, battery, status, CONN / ↑ / ↓ / SIM DETECT.
- Global controls: OBJETO (open webcam), START (kick off mission), REC, ALTURA.
- Buttons: `¿CÓMO FUNCIONA?` (in-app help dialog), `DETECT IMG` (analyze a still image with YOLO).
- Obstacle drawing: `DRAW OBS` toggles a mode where click+drag on the map adds a wall segment (drawn in red). `CLEAR OBS` removes all walls.
- **Distributed mode**: enter a `PEER ID` and click `BRIDGE`. The app connects to `broker.hivemq.com:1883` (public broker, configurable via `MQTT_BROKER`/`MQTT_PORT` env vars). Topics under `swarm01/<peer_id>/...`. Remote drones from other peers appear in `fleet.drones` as `RemoteTello` and are drawn with a grey outline.
- Zenithal map: auto-scales to fit drone paths + obstacles. Cyan/red/amber/purple per drone slot, grid every 100 cm, north arrow, target marker (red blinking ✕), heading triangle.
- Event bus: `fleet.events` is consumed in `consume_events()` every 100 ms.

### MQTT bridge (`mqtt_bridge.py`)

Publishes:
- `swarm/<peer>/drone/<id>/pose` every 0.5 s
- `swarm/<peer>/drone/<id>/status` every 0.5 s
- `swarm/<peer>/drone/<id>/found` once (retained) when target detected
- `swarm/<peer>/obstacle` when an obstacle is drawn
- `swarm/<peer>/obstacles_clear` when obstacles are cleared
- `swarm/<peer>/hello` on connect

Subscribes to all peers via wildcard. Filters out its own messages by peer_id. Remote drones are auto-added to the fleet as `RemoteTello` with id `<peer>/<drone_id>`.

### Image detection (`image_detect.py`)

Tkinter file dialog → loads image with cv2 → resizes if max side > 1280 → runs `detection.model` (shared YOLO instance) → draws bounding boxes → shows result in cv2 window → returns list of detections.

## File inventory

| File | Status |
|---|---|
| `main.py` | Active — multi-drone GUI |
| `drone_iface.py` | Active — DroneInterface + 3 implementations |
| `fleet.py` | Active — central state |
| `mission_logic.py` | Active — movement + reverse-path convergence |
| `mission_v2.py` | Active — YOLO vision loop for real drones |
| `detection.py` | Active — webcam target selection |
| `image_detect.py` | Active — static image detection |
| `mqtt_bridge.py` | Active — optional distributed mode |
| `test_fake.py` | Test — exercises FakeTello |
| `test_fleet.py` | Test — exercises Fleet + 2 fakes + reverse-path |
| `mission.py` | Unused — older state-machine version |
| `mission_good.py` | Unused — near-duplicate with `SEARCH_ROTATION=45°` |

## Known gotchas

**`paho-mqtt` is optional**: `mqtt_bridge.py` imports `paho.mqtt.client` lazily inside `MQTTBridge.connect()`. The `BRIDGE` button raises a clear `ImportError` if you click it without installing paho. The app still launches without it.

**`djitellopy` is optional**: same lazy pattern in `RealTello.__init__`. The app launches with all-FAKE drones even if `djitellopy` is missing.

**YOLO model is loaded once**: `mission_v2._get_model()` caches `YOLO("yolov8n.pt")` behind a module-level lock so multiple real drones share it.

**Takeoff height assumption**: `takeoff_click` assumes the Tello stabilises at exactly 80 cm after `takeoff()`. If the surface is uneven this offset will be wrong.

**Dead-reckoning drift**: tracker position is integrated from issued commands, never read from the drone. Tellos drift; expect tens of cm of error after a few minutes of flight. This is acceptable for visualisation and convergence at room scale.

**Reverse-path needs a path**: if the finder has not moved (path has fewer than 2 waypoints), `converge_to` falls back to a straight line to the finder's pose.

**Obstacle avoidance is single-hop**: `_avoid_obstacle` inserts at most one detour waypoint per segment. If two obstacles block both detour candidates, the system logs a warning and falls back to flying through (the assumption being that a human operator will not draw an impossible scene).

**`records/` directory**: video recordings go to `records/mission_<drone_id>_<timestamp>.mp4`. Created on demand.

**The map's "north" is the dead-reckoning frame, not real-world north**: heading 0° is whatever direction each drone happened to be facing when its tracker was initialized (and `Fleet.spread_headings()` redistributes initial headings around the circle on takeoff).
