"""
Estado compartido de la flota de drones + bus de eventos + obstáculos.

Concepto:
    - Fleet           → registro central con todos los DroneState
    - DroneState      → un dron + su tracker + status + color
    - DroneTracker    → dead-reckoning (movido aquí desde main.py)
    - Event/EventType → cosas que pasan (POSE_UPDATE, TARGET_FOUND, ...)
    - Obstáculos      → segmentos de línea que los drones deben evitar

La UI consume eventos del Fleet.events queue para refrescar.
La lógica de misión (mission_v2, mission_logic) escribe en el tracker y emite eventos.
"""
from __future__ import annotations
import math
import queue
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple, Dict, List

from drone_iface import DroneInterface


# ─────────────────────────────────────────────
#  Tracker (dead-reckoning)
# ─────────────────────────────────────────────
class DroneTracker:
    """Integra rotaciones y avances para estimar pose. Thread-safe."""
    def __init__(self):
        self._lock = threading.Lock()
        self.x = 0.0        # cm, este positivo
        self.y = 0.0        # cm, norte positivo
        self.z = 0.0        # cm, altitud sobre el suelo
        self.heading = 0.0  # grados, 0=norte, sentido horario
        self.path: List[Tuple[float, float]] = [(0.0, 0.0)]

    def reset(self):
        with self._lock:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.heading = 0.0
            self.path = [(0.0, 0.0)]

    def set_pose(self, x: float = 0.0, y: float = 0.0, heading: float = 0.0,
                 z: Optional[float] = None):
        with self._lock:
            self.x = x
            self.y = y
            if z is not None:
                self.z = z
            self.heading = heading % 360
            self.path = [(x, y)]

    def set_altitude(self, z: float):
        """Fija la altitud absoluta (cm sobre el suelo). No baja de 0."""
        with self._lock:
            self.z = max(0.0, z)

    def rotate_cw(self, deg):
        with self._lock:
            self.heading = (self.heading + deg) % 360

    def rotate_ccw(self, deg):
        with self._lock:
            self.heading = (self.heading - deg) % 360

    def move_forward(self, cm):
        with self._lock:
            rad = math.radians(self.heading)
            self.x += cm * math.sin(rad)
            self.y += cm * math.cos(rad)
            self.path.append((self.x, self.y))

    def move_up(self, cm):
        with self._lock:
            self.z += cm

    def move_down(self, cm):
        with self._lock:
            self.z = max(0.0, self.z - cm)

    def set_position(self, x: float, y: float):
        """Sobrescribe la posición sin cambiar heading. Útil para convergencia."""
        with self._lock:
            self.x = x
            self.y = y
            self.path.append((x, y))

    def update_remote_pose(self, x: float, y: float, heading: float,
                           z: Optional[float] = None):
        """
        Actualiza pose desde una fuente externa (MQTT) APPENDIENDO al path
        en vez de reiniciarlo. Solo añade al path si la diferencia con el
        último punto es >= 20 cm (para no saturar el path con micro-cambios).
        """
        with self._lock:
            if (not self.path or
                math.hypot(x - self.path[-1][0], y - self.path[-1][1]) >= 20):
                self.path.append((x, y))
            self.x = x
            self.y = y
            self.heading = heading % 360
            if z is not None:
                self.z = z

    def get_state(self):
        """Devuelve (x, y, heading, copia_de_path). Altitud disponible vía .z"""
        with self._lock:
            return self.x, self.y, self.heading, list(self.path)


# ─────────────────────────────────────────────
#  Estados y eventos
# ─────────────────────────────────────────────
class DroneStatus(Enum):
    IDLE        = "idle"
    CONNECTED   = "connected"
    TAKING_OFF  = "taking_off"
    SEARCHING   = "searching"
    CONVERGING  = "converging"
    AT_TARGET   = "at_target"
    FOUND       = "found"
    RETURNING   = "returning"   # volviendo a (0,0) por reverse-path del propio dron
    LANDING     = "landing"
    LANDED      = "landed"
    ERROR       = "error"


class EventType(Enum):
    POSE_UPDATE     = "pose_update"
    TARGET_FOUND    = "target_found"
    STATUS_CHANGED  = "status_changed"
    BATTERY         = "battery"
    LOG             = "log"


@dataclass
class Event:
    type: EventType
    drone_id: str
    payload: dict = field(default_factory=dict)


# ─────────────────────────────────────────────
#  Estado por dron
# ─────────────────────────────────────────────
@dataclass
class DroneState:
    drone_id: str
    drone: DroneInterface
    tracker: DroneTracker = field(default_factory=DroneTracker)
    status: DroneStatus = DroneStatus.IDLE
    blinking: bool = False
    color: str = "#00e5ff"
    is_real: bool = False
    # True para drones controlados por OTRO PC (vienen vía MQTT bridge).
    # Su pose/status la gestiona el peer dueño; nosotros solo los visualizamos
    # y los excluimos de operaciones locales (reset, spread_headings, land...).
    is_remote: bool = False
    last_battery: int = -1
    record_enabled: bool = False
    target_class_id: Optional[int] = None
    # Cuando no es None, run_drone_mission llamará a execute_manual_nav
    # con (x_target, y_target, z_target) y luego lo pondrá a None.
    manual_target: Optional[Tuple[float, float, float]] = None
    # True si el usuario fijó la pose inicial manualmente (vía PoseDialog en
    # suelo). En ese caso takeoff y spread_headings la respetan.
    manual_pose: bool = False


# ─────────────────────────────────────────────
#  Obstáculos
# ─────────────────────────────────────────────
@dataclass
class Obstacle:
    """Segmento de pared que los drones deben evitar."""
    p1: Tuple[float, float]   # (x, y) en cm
    p2: Tuple[float, float]


def _ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(p1, p2, p3, p4) -> bool:
    """¿Se cortan los segmentos (p1-p2) y (p3-p4)?"""
    return (_ccw(p1, p3, p4) != _ccw(p2, p3, p4) and
            _ccw(p1, p2, p3) != _ccw(p1, p2, p4))


# ─────────────────────────────────────────────
#  Flota
# ─────────────────────────────────────────────
class Fleet:
    """Registro central."""

    DRONE_COLORS = ["#00e5ff", "#ff4c4c", "#ffab00", "#bb86fc"]

    def __init__(self):
        self._lock = threading.RLock()
        self.drones: Dict[str, DroneState] = {}
        self.target_found_at: Optional[Tuple[float, float, str]] = None
        self.events: "queue.Queue[Event]" = queue.Queue()
        self.global_target_class_id: Optional[int] = None
        self.obstacles: List[Obstacle] = []
        # Evento de pausa global. Cuando está set, los bucles de movimiento
        # esperan en lugar de avanzar (el dron hover libremente).
        self.mission_paused: threading.Event = threading.Event()

    # ── drones ─────────────────────────────────
    def add_drone(self, drone_id, drone, is_real=False,
                  is_remote=False, color=None) -> DroneState:
        with self._lock:
            if drone_id in self.drones:
                raise ValueError(f"drone_id '{drone_id}' ya registrado")
            # El color lo fija la tarjeta (color por slot fijo). Si no se pasa
            # (p.ej. dron remoto sin tarjeta), se asigna por orden de conexión.
            if color is None:
                color = self.DRONE_COLORS[len(self.drones) % len(self.DRONE_COLORS)]
            state = DroneState(drone_id=drone_id, drone=drone, color=color,
                               is_real=is_real, is_remote=is_remote)
            self.drones[drone_id] = state
            return state

    def remove_drone(self, drone_id):
        with self._lock:
            self.drones.pop(drone_id, None)

    def get(self, drone_id) -> Optional[DroneState]:
        with self._lock:
            return self.drones.get(drone_id)

    def all(self) -> List[DroneState]:
        with self._lock:
            return list(self.drones.values())

    def count(self) -> int:
        with self._lock:
            return len(self.drones)

    # ── eventos ────────────────────────────────
    def publish(self, event: Event):
        self.events.put(event)

    def log(self, drone_id, msg):
        self.publish(Event(EventType.LOG, drone_id, {"msg": msg}))

    def emit_pose(self, drone_id):
        state = self.get(drone_id)
        if not state:
            return
        x, y, h, path = state.tracker.get_state()
        self.publish(Event(EventType.POSE_UPDATE, drone_id, {
            "x": x, "y": y, "heading": h, "path_len": len(path),
        }))

    def emit_battery(self, drone_id, value):
        with self._lock:
            state = self.drones.get(drone_id)
            if state:
                state.last_battery = value
        self.publish(Event(EventType.BATTERY, drone_id, {"value": value}))

    def set_status(self, drone_id, new_status: DroneStatus):
        with self._lock:
            state = self.drones.get(drone_id)
            if not state:
                return
            old = state.status
            state.status = new_status
            state.blinking = (new_status == DroneStatus.FOUND)
        self.publish(Event(EventType.STATUS_CHANGED, drone_id, {
            "old": old.value, "new": new_status.value,
        }))

    def report_target_found(self, drone_id):
        state = self.get(drone_id)
        if not state:
            return
        x, y, _, _ = state.tracker.get_state()
        with self._lock:
            if self.target_found_at is not None:
                return
            self.target_found_at = (x, y, drone_id)
        self.set_status(drone_id, DroneStatus.FOUND)
        self.publish(Event(EventType.TARGET_FOUND, drone_id, {"x": x, "y": y}))

    # ── consultas ──────────────────────────────
    def get_target(self) -> Optional[Tuple[float, float, str]]:
        with self._lock:
            return self.target_found_at

    def reset_mission(self):
        """Limpia target encontrado y blinks LOCALES. No toca drones remotos
        (su status lo gestiona el peer dueño)."""
        with self._lock:
            self.target_found_at = None
            for state in self.drones.values():
                if state.is_remote:
                    continue
                if state.status == DroneStatus.FOUND:
                    state.status = DroneStatus.SEARCHING
                state.blinking = False

    def spread_headings(self, base=0.0):
        """
        Reparte headings cardinales. Conserva el "slot" de cada dron local
        para que el reparto sea estable.

        Excluye drones que están volando con pose REAL (SEARCHING, CONVERGING,
        AT_TARGET, FOUND, RETURNING, LANDING) — su tracker tiene posición
        significativa y resetearlo a (0,0) sería un bug.

        INCLUYE drones en TAKING_OFF: en ese estado el tracker se acaba de
        resetear (en takeoff_click), así que es exactamente el momento de
        asignarles su heading inicial.

        También excluye `manual_pose=True` (lo gestiona el usuario) y
        `is_remote=True` (lo gestiona el peer dueño).
        """
        # NOTA: TAKING_OFF NO está aquí a propósito (ver docstring)
        SKIP_STATUSES = (DroneStatus.SEARCHING, DroneStatus.CONVERGING,
                         DroneStatus.AT_TARGET, DroneStatus.FOUND,
                         DroneStatus.RETURNING, DroneStatus.LANDING)
        with self._lock:
            locals_ordered = [s for s in self.drones.values()
                              if not s.is_remote]
            n_total = len(locals_ordered)
            if n_total == 0:
                return
            step = 360.0 / n_total
            for i, state in enumerate(locals_ordered):
                if state.manual_pose or state.status in SKIP_STATUSES:
                    continue   # mantiene su slot pero no se toca su pose
                state.tracker.set_pose(0.0, 0.0, (base + i * step) % 360)

    # ── obstáculos ─────────────────────────────
    def add_obstacle(self, p1: Tuple[float, float], p2: Tuple[float, float]):
        """Añade un segmento como obstáculo. p1, p2 en coords mundo (cm)."""
        with self._lock:
            self.obstacles.append(Obstacle(p1=p1, p2=p2))

    def clear_obstacles(self):
        with self._lock:
            self.obstacles.clear()

    def get_obstacles(self) -> List[Obstacle]:
        with self._lock:
            return list(self.obstacles)

    def segment_crosses_obstacle(self, p1, p2) -> Optional[Obstacle]:
        """Devuelve el primer obstáculo que cruza p1→p2, o None si está libre."""
        with self._lock:
            obs_list = list(self.obstacles)
        for o in obs_list:
            if segments_intersect(p1, p2, o.p1, o.p2):
                return o
        return None