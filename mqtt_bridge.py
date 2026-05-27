"""
mqtt_bridge.py — Sincronización de Fleet entre varios PCs vía MQTT.

Cada PC ejecuta su propio main.py con su(s) dron(es) local(es).
Este bridge publica el estado de los drones locales y suscribe el estado
de drones remotos para mostrarlos en el mismo mapa.

Topics (todos bajo SWARM_ID):
    swarm/<peer_id>/drone/<drone_id>/pose       {x, y, heading}
    swarm/<peer_id>/drone/<drone_id>/status     {status, battery}
    swarm/<peer_id>/drone/<drone_id>/found      {x, y}    retain=True
    swarm/<peer_id>/obstacle                    {p1: [x,y], p2: [x,y]}
    swarm/<peer_id>/obstacles_clear             {}
    swarm/<peer_id>/hello                       {peer_id, drones: [...]}

Uso desde main.py:
    bridge = MQTTBridge(fleet, peer_id="alpha")
    bridge.connect()
    bridge.register_local_drone("tello-a")
    bridge.publish_obstacle(p1, p2)
"""
from __future__ import annotations
import json
import os
import threading
import time
from typing import Optional, Any

# paho.mqtt se importa de forma perezosa en connect() para que `import mqtt_bridge`
# no falle si la dependencia no está instalada. Solo es necesaria si activas el bridge.
mqtt: Any = None

from fleet import Fleet, DroneStatus
from drone_iface import RemoteTello

BROKER_HOST_DEFAULT = "broker.hivemq.com"
BROKER_PORT_DEFAULT = 1883
SWARM_ID_DEFAULT    = "swarm01"
POSE_PUBLISH_INTERVAL = 0.5   # segundos


class MQTTBridge:
    def __init__(self, fleet: Fleet,
                 peer_id: Optional[str] = None,
                 swarm_id: Optional[str] = None,
                 broker_host: Optional[str] = None,
                 broker_port: Optional[int] = None):
        self.fleet = fleet
        self.peer_id  = peer_id  or os.getenv("PEER_ID",  "peer1")
        self.swarm_id = swarm_id or os.getenv("SWARM_ID", SWARM_ID_DEFAULT)
        self.broker_host = broker_host or os.getenv("MQTT_BROKER", BROKER_HOST_DEFAULT)
        self.broker_port = broker_port or int(os.getenv("MQTT_PORT", BROKER_PORT_DEFAULT))

        self._client = None
        self._connected = False
        self._stop_event = threading.Event()
        self._local_drones: set[str] = set()
        self._remote_drones: dict[str, str] = {}

    def _topic(self, *parts) -> str:
        return "/".join([self.swarm_id, *parts])

    def _is_my_message(self, topic: str) -> bool:
        parts = topic.split("/")
        return len(parts) >= 2 and parts[1] == self.peer_id

    # ── conexión ───────────────────────────────
    def connect(self):
        # Import perezoso de paho-mqtt — solo se necesita si activas el bridge
        global mqtt
        if mqtt is None:
            try:
                import paho.mqtt.client as _mqtt
                mqtt = _mqtt
            except ImportError:
                raise ImportError(
                    "paho-mqtt no instalado. Para el modo distribuido: "
                    "pip install paho-mqtt"
                )
        self._client = mqtt.Client(f"{self.swarm_id}_{self.peer_id}_{int(time.time())}")
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect
        self._client.reconnect_delay_set(min_delay=2, max_delay=10)
        try:
            self._client.connect(self.broker_host, self.broker_port, keepalive=60)
            self._client.loop_start()
            threading.Thread(target=self._pose_publisher_loop, daemon=True).start()
        except Exception as e:
            self.fleet.log("bridge", f"Error conexión MQTT: {e}")

    def disconnect(self):
        self._stop_event.set()
        if self._client is not None:
            self._client.loop_stop()
            try:
                self._client.disconnect()
            except Exception:
                pass
        self._connected = False

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected = True
            self.fleet.log("bridge", f"MQTT conectado (peer_id={self.peer_id})")
            client.subscribe(self._topic("+", "drone", "+", "+"))
            client.subscribe(self._topic("+", "obstacle"))
            client.subscribe(self._topic("+", "obstacles_clear"))
            client.subscribe(self._topic("+", "hello"))
            self._publish_hello()
        else:
            self.fleet.log("bridge", f"Error rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        self.fleet.log("bridge", f"MQTT desconectado (rc={rc})")

    # ── publicación ────────────────────────────
    def register_local_drone(self, drone_id: str):
        self._local_drones.add(drone_id)

    def unregister_local_drone(self, drone_id: str):
        self._local_drones.discard(drone_id)

    def _publish(self, topic: str, payload: dict, retain: bool = False):
        if not self._connected or self._client is None:
            return
        try:
            self._client.publish(topic, json.dumps(payload), retain=retain)
        except Exception as e:
            self.fleet.log("bridge", f"Error publicando: {e}")

    def _publish_hello(self):
        self._publish(
            self._topic(self.peer_id, "hello"),
            {"peer_id": self.peer_id, "drones": list(self._local_drones)}
        )

    def _pose_publisher_loop(self):
        while not self._stop_event.is_set():
            for drone_id in list(self._local_drones):
                state = self.fleet.get(drone_id)
                if state is None:
                    continue
                x, y, h, _ = state.tracker.get_state()
                # Incluimos Z para que la seguridad 3D funcione entre peers
                self._publish(
                    self._topic(self.peer_id, "drone", drone_id, "pose"),
                    {"x": x, "y": y, "z": state.tracker.z, "heading": h}
                )
                self._publish(
                    self._topic(self.peer_id, "drone", drone_id, "status"),
                    {"status": state.status.value, "battery": state.last_battery}
                )
            time.sleep(POSE_PUBLISH_INTERVAL)

    def publish_target_found(self, drone_id: str, x: float, y: float):
        self._publish(
            self._topic(self.peer_id, "drone", drone_id, "found"),
            {"x": x, "y": y},
            retain=True
        )

    def publish_obstacle(self, p1, p2):
        self._publish(
            self._topic(self.peer_id, "obstacle"),
            {"p1": list(p1), "p2": list(p2)}
        )

    def publish_obstacles_clear(self):
        self._publish(self._topic(self.peer_id, "obstacles_clear"), {})

    # ── recepción ──────────────────────────────
    def _on_message(self, client, userdata, message):
        if self._is_my_message(message.topic):
            return
        try:
            payload = json.loads(message.payload.decode()) if message.payload else {}
        except Exception:
            return

        parts = message.topic.split("/")
        if len(parts) < 3:
            return
        peer_id = parts[1]

        if len(parts) == 3 and parts[2] == "hello":
            self._handle_hello(peer_id, payload)
        elif len(parts) == 3 and parts[2] == "obstacle":
            self._handle_obstacle(payload)
        elif len(parts) == 3 and parts[2] == "obstacles_clear":
            self._handle_obstacles_clear()
        elif len(parts) == 5 and parts[2] == "drone":
            drone_id = parts[3]
            subtopic = parts[4]
            local_id = f"{peer_id}/{drone_id}"
            self._ensure_remote_drone(local_id, peer_id)
            if subtopic == "pose":
                self._handle_pose(local_id, payload)
            elif subtopic == "status":
                self._handle_status(local_id, payload)
            elif subtopic == "found":
                self._handle_found(local_id, payload)

    def _ensure_remote_drone(self, local_id: str, peer_id: str):
        if local_id in self._remote_drones:
            return
        if self.fleet.get(local_id) is not None:
            self._remote_drones[local_id] = peer_id
            return
        remote = RemoteTello(name=local_id)
        try:
            self.fleet.add_drone(local_id, remote, is_real=False,
                                 is_remote=True)
            self._remote_drones[local_id] = peer_id
            self.fleet.log("bridge", f"Dron remoto añadido: {local_id}")
        except ValueError:
            self._remote_drones[local_id] = peer_id

    def _handle_hello(self, peer_id: str, payload: dict):
        for did in payload.get("drones", []):
            self._ensure_remote_drone(f"{peer_id}/{did}", peer_id)
        self._publish_hello()

    def _handle_pose(self, local_id: str, payload: dict):
        state = self.fleet.get(local_id)
        if state is None:
            return
        # APPEND al path (no resetear) → la convergencia reverse-path hacia
        # un dron remoto funciona porque conserva su historial de movimiento.
        z_val = payload.get("z")
        state.tracker.update_remote_pose(
            x=float(payload.get("x", 0)),
            y=float(payload.get("y", 0)),
            heading=float(payload.get("heading", 0)),
            z=float(z_val) if z_val is not None else None,
        )
        self.fleet.emit_pose(local_id)

    def _handle_status(self, local_id: str, payload: dict):
        state = self.fleet.get(local_id)
        if state is None:
            return
        try:
            new_status = DroneStatus(payload.get("status", "idle"))
            if state.status != new_status:
                self.fleet.set_status(local_id, new_status)
        except ValueError:
            pass
        battery = payload.get("battery", -1)
        if battery > 0:
            self.fleet.emit_battery(local_id, int(battery))

    def _handle_found(self, local_id: str, payload: dict):
        if self.fleet.get_target() is not None:
            return
        x = float(payload.get("x", 0))
        y = float(payload.get("y", 0))
        state = self.fleet.get(local_id)
        if state is not None:
            state.tracker.set_position(x, y)
        with self.fleet._lock:
            self.fleet.target_found_at = (x, y, local_id)
        self.fleet.set_status(local_id, DroneStatus.FOUND)
        self.fleet.log("bridge", f"Target remoto de {local_id} en ({x:.0f},{y:.0f})")

    def _handle_obstacle(self, payload: dict):
        p1 = tuple(payload.get("p1", [0, 0]))
        p2 = tuple(payload.get("p2", [0, 0]))
        self.fleet.add_obstacle(p1, p2)

    def _handle_obstacles_clear(self):
        self.fleet.clear_obstacles()