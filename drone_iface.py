"""
Capa de abstracción para drones.

DroneInterface define el contrato.
- RealTello envuelve djitellopy.Tello
- FakeTello simula tiempos y frames para probar el sistema sin hardware
- RemoteTello representa un dron físico que vive en OTRO PC, sincronizado vía MQTT
"""
from abc import ABC, abstractmethod
import threading
import time
import numpy as np
import cv2


class FrameRead:
    """Imita el objeto frame_read de djitellopy: expone .frame como property."""
    def __init__(self, get_frame_fn):
        self._get_frame = get_frame_fn

    @property
    def frame(self):
        return self._get_frame()


# ─────────────────────────────────────────────
#  Interfaz
# ─────────────────────────────────────────────
class DroneInterface(ABC):
    name: str = "drone"

    @abstractmethod
    def connect(self): ...
    @abstractmethod
    def takeoff(self): ...
    @abstractmethod
    def land(self): ...
    @abstractmethod
    def rotate_clockwise(self, deg: int): ...
    @abstractmethod
    def rotate_counter_clockwise(self, deg: int): ...
    @abstractmethod
    def move_forward(self, cm: int): ...
    @abstractmethod
    def move_up(self, cm: int): ...
    @abstractmethod
    def move_down(self, cm: int): ...
    @abstractmethod
    def get_battery(self) -> int: ...
    @abstractmethod
    def streamon(self): ...
    @abstractmethod
    def streamoff(self): ...
    @abstractmethod
    def get_frame_read(self) -> FrameRead: ...

    # No abstracto: por defecto no hace nada (fake/remote lo ignoran).
    def set_speed(self, cm_s: int):
        pass


# ─────────────────────────────────────────────
#  Real (djitellopy)
# ─────────────────────────────────────────────
class RealTello(DroneInterface):
    """
    Wrapper sobre djitellopy.Tello.

    IMPORTANTE: todos los comandos de control van protegidos por `_cmd_lock`.
    El Tello usa un único socket UDP de comandos (puerto 8889) y solo acepta
    UN comando a la vez. Sin el lock, si dos hilos mandan comandos a la vez
    (p.ej. takeoff_click + run_drone_mission) el stream se corrompe y el dron
    responde 'error Not joystick' / 'error No valid imu'. El lock serializa
    todo: mientras un takeoff está en curso, una rotación espera su turno.

    get_frame_read NO se bloquea: el vídeo va por otro socket (stream) y leer
    frames no manda comandos.
    """
    def __init__(self, name: str = "tello", host: str | None = None,
                 vs_port: int | None = None):
        """
        name    — nombre lógico del dron (alpha/bravo/...).
        host    — IP del dron (None = AP por defecto 192.168.10.1).
        vs_port — puerto UDP para el stream de video. Si conectas varios Tellos
                  al mismo PC, cada uno necesita su propio puerto o los videos
                  chocan (todos por defecto usan 11111). Sugerencia:
                      alpha=11111, bravo=11112, charlie=11113, delta=11114
        """
        try:
            from djitellopy import Tello
        except ImportError:
            raise ImportError("djitellopy no instalado. pip install djitellopy")
        self.name = name
        kwargs = {}
        if host:
            kwargs["host"] = host
        if vs_port is not None:
            kwargs["vs_udp"] = int(vs_port)
        self._tello = Tello(**kwargs) if kwargs else Tello()
        self._cmd_lock = threading.Lock()

    def connect(self):
        with self._cmd_lock: self._tello.connect()
    def takeoff(self):
        with self._cmd_lock: self._tello.takeoff()
    def land(self):
        with self._cmd_lock: self._tello.land()
    def rotate_clockwise(self, deg):
        with self._cmd_lock: self._tello.rotate_clockwise(deg)
    def rotate_counter_clockwise(self, deg):
        with self._cmd_lock: self._tello.rotate_counter_clockwise(deg)
    def move_forward(self, cm):
        with self._cmd_lock: self._tello.move_forward(cm)
    def move_up(self, cm):
        with self._cmd_lock: self._tello.move_up(cm)
    def move_down(self, cm):
        with self._cmd_lock: self._tello.move_down(cm)
    def get_battery(self) -> int:
        with self._cmd_lock: return self._tello.get_battery()
    def streamon(self):
        with self._cmd_lock: self._tello.streamon()
    def streamoff(self):
        with self._cmd_lock: self._tello.streamoff()
    def set_speed(self, cm_s: int):
        # Velocidad lineal del Tello (10-100 cm/s). Acelera los avances.
        with self._cmd_lock: self._tello.set_speed(int(cm_s))

    def get_frame_read(self) -> FrameRead:
        # Sin lock: el vídeo va por otro socket, no manda comandos de control.
        fr = self._tello.get_frame_read()
        return FrameRead(lambda: fr.frame)


# ─────────────────────────────────────────────
#  Fake (simulado en proceso)
# ─────────────────────────────────────────────
class FakeTello(DroneInterface):
    SPEED_FORWARD  = 50
    SPEED_ROTATE   = 90
    SPEED_VERTICAL = 40
    # Latencia fija por comando, para imitar el ritmo del Tello real: cada
    # comando del Tello tarda ~1-2 s extra (round-trip de red + acelerar,
    # ejecutar, frenar y estabilizar antes de responder 'ok'). Sin esto el
    # fake va mucho más rápido que el dron real y se desincronizan.
    CMD_OVERHEAD   = 1.5

    def __init__(self, name: str = "fake", initial_battery: int = 90):
        self.name = name
        self._battery_initial = initial_battery
        self._battery_drain_per_min = 3
        self._connected = False
        self._streaming = False
        self._airborne = False
        self._start_time = time.time()
        self.simulate_detection = False
        self._lock = threading.Lock()

    def _battery_now(self) -> int:
        elapsed_min = (time.time() - self._start_time) / 60.0
        return max(5, int(self._battery_initial - self._battery_drain_per_min * elapsed_min))

    def _log(self, msg): print(f"[FAKE {self.name}] {msg}")

    def connect(self):
        time.sleep(0.4); self._connected = True; self._log("connected")
    def takeoff(self):
        time.sleep(2.5); self._airborne = True; self._log("takeoff")
    def land(self):
        time.sleep(2.0); self._airborne = False; self._log("land")
    def rotate_clockwise(self, deg):
        time.sleep(self.CMD_OVERHEAD + deg / self.SPEED_ROTATE)
        self._log(f"rotate_cw {deg}°")
    def rotate_counter_clockwise(self, deg):
        time.sleep(self.CMD_OVERHEAD + deg / self.SPEED_ROTATE)
        self._log(f"rotate_ccw {deg}°")
    def move_forward(self, cm):
        time.sleep(self.CMD_OVERHEAD + cm / self.SPEED_FORWARD)
        self._log(f"move_forward {cm}cm")
    def move_up(self, cm):
        time.sleep(self.CMD_OVERHEAD + cm / self.SPEED_VERTICAL)
        self._log(f"move_up {cm}cm")
    def move_down(self, cm):
        time.sleep(self.CMD_OVERHEAD + cm / self.SPEED_VERTICAL)
        self._log(f"move_down {cm}cm")
    def get_battery(self): return self._battery_now()
    def streamon(self):  self._streaming = True
    def streamoff(self): self._streaming = False

    def get_frame_read(self) -> FrameRead:
        def make_frame():
            frame = np.zeros((720, 960, 3), dtype=np.uint8)
            cv2.rectangle(frame, (0, 0), (960, 60), (40, 40, 60), -1)
            cv2.putText(frame, f"FAKE TELLO  ·  {self.name.upper()}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 229, 255), 2)
            cv2.putText(frame, time.strftime("%H:%M:%S"), (20, 700),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            cv2.putText(frame, f"BAT {self._battery_now()}%", (820, 700),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 230, 118), 2)
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return FrameRead(make_frame)


# ─────────────────────────────────────────────
#  Remote (dron físico controlado por OTRO PC)
# ─────────────────────────────────────────────
class RemoteTello(DroneInterface):
    """
    Representa un dron que vive en otro PC.
    Todas las acciones de movimiento son no-ops — el dron real lo controla su PC.
    Su estado (pose, status, batería) se sincroniza vía MQTT por mqtt_bridge.py.

    En la UI aparece como un dron más en el mapa pero los botones de control
    deberían estar deshabilitados (lo gestiona main.py).
    """
    def __init__(self, name: str = "remote"):
        self.name = name
        self._battery = -1

    def connect(self):                            pass
    def takeoff(self):                            pass
    def land(self):                               pass
    def rotate_clockwise(self, deg):              pass
    def rotate_counter_clockwise(self, deg):      pass
    def move_forward(self, cm):                   pass
    def move_up(self, cm):                        pass
    def move_down(self, cm):                      pass
    def streamon(self):                           pass
    def streamoff(self):                          pass
    def get_battery(self) -> int:                 return self._battery

    def get_frame_read(self) -> FrameRead:
        def empty_frame():
            f = np.zeros((720, 960, 3), dtype=np.uint8)
            cv2.putText(f, f"REMOTE - {self.name}", (200, 360),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (100, 100, 100), 3)
            return cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
        return FrameRead(empty_frame)