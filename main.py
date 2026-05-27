"""
Tello Control Center — versión multi-dron (hasta 4).

Estructura:
    - 4 tarjetas de dron (REAL o FAKE), cada una con sus controles.
    - Controles globales: escoger objeto, iniciar misión, REC.
    - Mapa zenital con todos los drones a la vez.

Solo el dron marcado como REAL ejecuta YOLO.
"""
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import font as tkfont, ttk, messagebox

import detection
import image_detect
import mission_logic
import mission_v2
from drone_iface import RealTello, FakeTello, RemoteTello
from fleet import Fleet, DroneStatus, EventType
from mqtt_bridge import MQTTBridge

# ─────────────────────────────────────────────
#  Paleta y constantes
# ─────────────────────────────────────────────
BG = "#0d0f14"
PANEL = "#13161e"
ACCENT = "#00e5ff"
ACCENT2 = "#ff4c4c"
TEXT = "#e8eaf0"
TEXT_DIM = "#4a5060"
SUCCESS = "#00e676"
WARNING = "#ffab00"
BORDER = "#1e2330"
# Color exclusivo para obstáculos en el mapa (distinto del rojo del dron bravo
# para que se distingan visualmente cuando coinciden).
OBSTACLE_COLOR = "#ff8c00"   # naranja

# Mapa ampliado para el panel derecho
MAP_W, MAP_H = 750, 750
MAX_DRONES = 4
DRONE_SLOTS = ["alpha", "bravo", "charlie", "delta"]

# ─────────────────────────────────────────────
#  Estado global de la app
# ─────────────────────────────────────────────
fleet = Fleet()
mission_stop_event = threading.Event()
mission_threads: dict[str, threading.Thread] = {}
record_enabled = False
selected_class_id: int | None = None
ui_tick = 0  # para animación de parpadeo

# Estado del botón START: "idle" → "running" → "paused" → "running" → ...
mission_state = "idle"

# Dibujado de obstáculos
drawing_obstacles = False
_obstacle_start: tuple[float, float] | None = None

# Modo distribuido (bridge MQTT)
mqtt_bridge: MQTTBridge | None = None

# Zoom del mapa zenital:
#   - None  → auto-fit (escala según el extent de drones+obstáculos)
#   - float → extent fijo en cm (mitad del lado representado)
viewport_extent: float | None = None
ZOOM_FACTOR_STEP = 1.25   # paso multiplicativo por scroll/click +/-
MIN_EXTENT = 50.0         # cm — zoom máximo (más cerca)
MAX_EXTENT = 20000.0      # cm — zoom mínimo (más lejos)
# Pan del mapa (en coords mundo). El centro del canvas representa el
# (viewport_pan_x, viewport_pan_y) en el mundo. (0,0) = origen al centro.
viewport_pan_x: float = 0.0
viewport_pan_y: float = 0.0
# Estado del arrastre con click-derecho (pan)
_pan_drag_start = None   # (canvas_x, canvas_y, pan_x_inicial, pan_y_inicial)


# ─────────────────────────────────────────────
#  Tarjeta de dron (Versión Compacta)
# ─────────────────────────────────────────────
class DroneCard:
    def __init__(self, parent, drone_id: str, color: str):
        self.drone_id = drone_id
        self.color = color
        self.connected = False

        self.frame = tk.Frame(parent, bg=PANEL, bd=0, relief="flat",
                              highlightthickness=1, highlightbackground=BORDER)

        # Banda superior con color del dron
        tk.Frame(self.frame, bg=color, height=3).pack(fill="x")

        inner = tk.Frame(self.frame, bg=PANEL, padx=8, pady=6)
        inner.pack(fill="both", expand=True)

        # Header: Nombre + dot color
        head = tk.Frame(inner, bg=PANEL)
        head.pack(fill="x")
        tk.Label(head, text=drone_id.upper(), font=mono_bold,
                 bg=PANEL, fg=color).pack(side="left")
        tk.Label(head, text="●", font=small_f, bg=PANEL, fg=color).pack(side="right")

        # Fila Tipo + IP (Comprimida)
        row_cfg = tk.Frame(inner, bg=PANEL)
        row_cfg.pack(fill="x", pady=2)

        self.type_var = tk.StringVar(value="FAKE")
        type_combo = ttk.Combobox(row_cfg, textvariable=self.type_var,
                                  values=["FAKE", "REAL"], state="readonly",
                                  width=5, font=small_f)
        type_combo.pack(side="left")
        type_combo.bind("<<ComboboxSelected>>", lambda e: self._on_type_change())

        self.ip_var = tk.StringVar(value="")
        self.ip_entry = tk.Entry(row_cfg, textvariable=self.ip_var, width=10,
                                 bg=BORDER, fg=ACCENT, insertbackground=ACCENT,
                                 relief="flat", font=small_f)
        self.ip_entry.pack(side="right")
        self.ip_entry.config(state="disabled")

        # Batería + status (en una sola línea)
        row_st = tk.Frame(inner, bg=PANEL)
        row_st.pack(fill="x")
        self.batt_var = tk.StringVar(value="--%")
        tk.Label(row_st, textvariable=self.batt_var, font=small_f,
                 bg=PANEL, fg=TEXT).pack(side="right")

        self.status_var = tk.StringVar(value="idle")
        self.status_label = tk.Label(row_st, textvariable=self.status_var,
                                     font=small_f, bg=PANEL, fg=TEXT_DIM)
        self.status_label.pack(side="left")

        # Botones de control (Más pequeños)
        btn_row = tk.Frame(inner, bg=PANEL)
        btn_row.pack(fill="x", pady=4)

        self.connect_btn = tk.Button(btn_row, text="CONN", font=small_f,
                                     bg=BORDER, fg=ACCENT, relief="flat", bd=0,
                                     padx=4, pady=2, cursor="hand2",
                                     command=self.connect_click)
        self.connect_btn.pack(side="left", fill="x", expand=True, padx=1)

        self.takeoff_btn = tk.Button(btn_row, text="^", font=mono_bold,
                                     bg=BORDER, fg=ACCENT, relief="flat", bd=0,
                                     pady=2, cursor="hand2", state="disabled",
                                     command=self.takeoff_click)
        self.takeoff_btn.pack(side="left", fill="x", expand=True, padx=1)

        self.land_btn = tk.Button(btn_row, text="v", font=mono_bold,
                                  bg=BORDER, fg=ACCENT2, relief="flat", bd=0,
                                  pady=2, cursor="hand2", state="disabled",
                                  command=self.land_click)
        self.land_btn.pack(side="left", fill="x", expand=True, padx=1)

        # Fila extra: Return to Home + Edit pose + Sim detect
        extra_row = tk.Frame(inner, bg=PANEL)
        extra_row.pack(fill="x")
        self.rth_btn = tk.Button(extra_row, text="↩ HOME", font=small_f,
                                 bg=BORDER, fg=ACCENT, relief="flat", bd=0,
                                 pady=2, cursor="hand2", state="disabled",
                                 command=self.rth_click)
        self.rth_btn.pack(side="left", fill="x", expand=True, padx=1)
        self.pos_btn = tk.Button(extra_row, text="✎", font=mono_bold,
                                 bg=BORDER, fg=ACCENT, relief="flat", bd=0,
                                 pady=2, cursor="hand2", state="disabled",
                                 command=lambda: PoseDialog(self.drone_id))
        self.pos_btn.pack(side="left", fill="x", expand=True, padx=1)
        self.sim_btn = tk.Button(extra_row, text="SIM", font=small_f,
                                 bg=BORDER, fg=WARNING, relief="flat", bd=0,
                                 pady=2, cursor="hand2", state="disabled",
                                 command=self.sim_detect_click)
        self.sim_btn.pack(side="left", fill="x", expand=True, padx=1)

    def _on_type_change(self):
        if self.type_var.get() == "REAL":
            self.ip_entry.config(state="normal")
        else:
            self.ip_entry.config(state="disabled")

    def connect_click(self):
        if self.connected: return
        kind = self.type_var.get()
        ip = self.ip_var.get().strip() or None
        log(f"[{self.drone_id}] Conectando ({kind})…")

        def run():
            try:
                if kind == "REAL":
                    # Puerto de vídeo único por slot para no chocar si conectas
                    # varios Tellos reales al mismo PC (default 11111 para todos).
                    try:
                        slot = DRONE_SLOTS.index(self.drone_id)
                    except ValueError:
                        slot = 0
                    vs_port = 11111 + slot
                    drone = RealTello(name=self.drone_id, host=ip,
                                      vs_port=vs_port)
                else:
                    drone = FakeTello(name=self.drone_id)
                drone.connect()
                drone.streamon()
                fleet.add_drone(self.drone_id, drone, is_real=(kind == "REAL"),
                                color=self.color)
                fleet.set_status(self.drone_id, DroneStatus.CONNECTED)
                batt = drone.get_battery()
                fleet.emit_battery(self.drone_id, batt)
                window.after(0, self._on_connected)
            except Exception as e:
                log(f"[{self.drone_id}] Error conexión: {e}")

        threading.Thread(target=run, daemon=True).start()

    def _on_connected(self):
        self.connected = True
        self.connect_btn.config(text="ON", state="disabled", fg=SUCCESS)
        self.takeoff_btn.config(state="normal")
        self.land_btn.config(state="normal")
        self.sim_btn.config(state="normal")
        self.rth_btn.config(state="normal")
        self.pos_btn.config(state="normal")
        # Registrar en bridge si está activo
        if mqtt_bridge is not None:
            mqtt_bridge.register_local_drone(self.drone_id)

    def reset_card_state(self):
        """Vuelve la tarjeta a estado inicial (sin conectar). Usado por
        FULL RESET. NO toca el fleet ni el drone; solo la UI."""
        self.connected = False
        self.connect_btn.config(text="CONN", state="normal", fg=ACCENT)
        self.takeoff_btn.config(state="disabled")
        self.land_btn.config(state="disabled")
        self.sim_btn.config(state="disabled")
        self.rth_btn.config(state="disabled")
        self.pos_btn.config(state="disabled")
        self.batt_var.set("--%")
        self.status_var.set("idle")
        self.status_label.config(fg=TEXT_DIM)
        self._raw_status = "idle"

    def takeoff_click(self):
        if not self.connected:
            return
        state = fleet.get(self.drone_id)
        if state is None:
            return

        # ⚠ Si el dron ya está en aire, NO hacer takeoff de nuevo:
        # tracker.reset() rompería las coordenadas y el dron creería estar
        # en (0,0). En su lugar, ajustar altitud al target_h si difiere.
        airborne_states = (DroneStatus.TAKING_OFF, DroneStatus.SEARCHING,
                           DroneStatus.CONVERGING, DroneStatus.AT_TARGET,
                           DroneStatus.FOUND, DroneStatus.RETURNING,
                           DroneStatus.LANDING)
        if state.status in airborne_states:
            try:
                target_h = int(height_var.get())
            except ValueError:
                log("Altura no válida")
                return
            log(f"[{self.drone_id}] ya en vuelo → ajustando altitud a {target_h}cm")

            def adjust():
                cur_z = state.tracker.z
                dz = int(round(target_h - cur_z))
                try:
                    if dz >= 20:
                        state.drone.move_up(dz)
                        state.tracker.move_up(dz)
                    elif dz <= -20:
                        state.drone.move_down(abs(dz))
                        state.tracker.move_down(abs(dz))
                    fleet.emit_pose(self.drone_id)
                except Exception as e:
                    log(f"[{self.drone_id}] Error ajuste altitud: {e}")

            threading.Thread(target=adjust, daemon=True).start()
            return

        # Despegue normal (dron en suelo)
        try:
            target_h = int(height_var.get())
        except ValueError:
            log("Altura no válida")
            return
        log(f"[{self.drone_id}] Despegue → {target_h} cm")

        def run():
            try:
                fleet.set_status(self.drone_id, DroneStatus.TAKING_OFF)
                if not state.manual_pose:
                    # Default: reset tracker y reparte heading inicial
                    state.tracker.reset()
                    _redistribute_headings()
                # Si manual_pose: respetamos la pose pre-asignada
                state.drone.takeoff()
                # Velocidad de avance más alta (cm/s) para que no sea tan lento
                try:
                    state.drone.set_speed(60)
                except Exception:
                    pass
                init_h = 80
                state.tracker.set_altitude(init_h)
                # Capa por nombre (alpha más alto, delta más bajo): se separa
                # cada dron por VERTICAL_SAFETY desde el ALTURA base, de modo
                # que nunca conflictúan verticalmente al arrancar la búsqueda.
                if state.manual_pose:
                    effective_h = target_h
                else:
                    try:
                        slot = DRONE_SLOTS.index(self.drone_id)
                    except ValueError:
                        slot = 0
                    n = len(DRONE_SLOTS)
                    effective_h = target_h + (n - 1 - slot) * mission_logic.VERTICAL_SAFETY
                    effective_h = max(mission_logic.MIN_FLIGHT_HEIGHT,
                                      min(mission_logic.MAX_FLIGHT_HEIGHT,
                                          effective_h))
                diff = effective_h - init_h
                if diff > 10:
                    state.drone.move_up(diff)
                    state.tracker.move_up(diff)
                elif diff < -10:
                    state.drone.move_down(abs(diff))
                    state.tracker.move_down(abs(diff))
                fleet.set_status(self.drone_id, DroneStatus.SEARCHING)
                fleet.emit_pose(self.drone_id)
                x, y, h, _ = state.tracker.get_state()
                log(f"[{self.drone_id}] en vuelo: "
                    f"({x:.0f}, {y:.0f}, Z{state.tracker.z:.0f}) HDG{h:.0f}°")

                # Auto-join: si hay misión en curso, este dron también se une
                if mission_state in ("running", "paused"):
                    already = (self.drone_id in mission_threads
                               and mission_threads[self.drone_id].is_alive())
                    if not already:
                        t_move = threading.Thread(
                            target=mission_logic.run_drone_mission,
                            args=(fleet, self.drone_id, mission_stop_event),
                            daemon=True)
                        t_move.start()
                        mission_threads[self.drone_id] = t_move
                        log(f"[{self.drone_id}] ▶ se une a la misión en curso")
                        # Visión YOLO si es REAL y hay objetivo seleccionado
                        if state.is_real and selected_class_id is not None:
                            t_vis = threading.Thread(
                                target=mission_v2.run_real_drone_vision,
                                args=(fleet, self.drone_id, mission_stop_event,
                                      selected_class_id),
                                daemon=True)
                            t_vis.start()
            except Exception as e:
                log(f"[{self.drone_id}] Error vuelo: {e}")
                fleet.set_status(self.drone_id, DroneStatus.ERROR)

        threading.Thread(target=run, daemon=True).start()

    def land_click(self):
        if not self.connected: return
        log(f"[{self.drone_id}] Aterrizando…")

        def run():
            state = fleet.get(self.drone_id)
            if not state: return
            try:
                fleet.set_status(self.drone_id, DroneStatus.LANDING)
                state.drone.land()
                fleet.set_status(self.drone_id, DroneStatus.LANDED)
            except Exception as e:
                log(f"[{self.drone_id}] Error land: {e}")

        threading.Thread(target=run, daemon=True).start()

    def sim_detect_click(self):
        log(f"[{self.drone_id}] SIM DETECT")
        fleet.report_target_found(self.drone_id)

    def rth_click(self):
        """Return to Home: el dron vuelve a (0,0) por reverse-path de su propio recorrido."""
        if not self.connected:
            return
        state = fleet.get(self.drone_id)
        if state is None:
            return
        if state.status in (DroneStatus.IDLE, DroneStatus.LANDED, DroneStatus.CONNECTED):
            log(f"[{self.drone_id}] no está en vuelo, RTH ignorado")
            return
        if state.status == DroneStatus.TAKING_OFF:
            log(f"[{self.drone_id}] espera al despegue antes de RTH")
            return
        if state.status == DroneStatus.RETURNING:
            log(f"[{self.drone_id}] ya está volviendo")
            return

        # Si SOY el finder (clicaron SIM antes o detecté con YOLO), libero el
        # target al pulsar HOME. Sin esto los otros drones seguirían
        # convergiendo hacia mí y bloquearían mi camino de vuelta por
        # seguridad 3D → RTH terminaría sin moverse.
        t = fleet.get_target()
        if t is not None and t[2] == self.drone_id:
            log(f"[{self.drone_id}] (era el finder → libero target, "
                f"los demás vuelven a buscar)")
            fleet.reset_mission()   # limpia target_found_at + reset FOUND/CONVERGING

        log(f"[{self.drone_id}] ↩ Return to Home iniciado")

        # Si la misión está corriendo, el thread existente se encargará al ver
        # status=RETURNING. Si no, lanzar un thread dedicado.
        if (self.drone_id in mission_threads
                and mission_threads[self.drone_id].is_alive()):
            fleet.set_status(self.drone_id, DroneStatus.RETURNING)
            return

        def run():
            try:
                mission_logic.return_to_origin(fleet, self.drone_id,
                                               threading.Event())
            except Exception as e:
                log(f"[{self.drone_id}] Error RTH: {e}")

        threading.Thread(target=run, daemon=True).start()

    def update_battery(self, value: int):
        self.batt_var.set(f"{value}%")

    def update_status(self, status: str):
        # Guarda el status crudo para refrescos posteriores (POSE_UPDATE)
        self._raw_status = status
        self._refresh_status_text()
        colors = {"found": ACCENT2, "at_target": WARNING, "searching": ACCENT,
                  "converging": ACCENT, "returning": WARNING,
                  "taking_off": ACCENT, "connected": SUCCESS,
                  "landed": SUCCESS, "error": ACCENT2}
        self.status_label.config(fg=colors.get(status, TEXT_DIM))

    def _refresh_status_text(self):
        status = getattr(self, "_raw_status", "idle")
        state = fleet.get(self.drone_id)
        z_txt = f"  Z{state.tracker.z:.0f}" if state else ""
        self.status_var.set(f"{status}{z_txt}")

    def on_pose_update(self):
        """Llamado en cada POSE_UPDATE para refrescar el dato de altitud."""
        self._refresh_status_text()


# ─────────────────────────────────────────────
#  Helpers UI
# ─────────────────────────────────────────────
def sep(parent):
    tk.Frame(parent, bg=BORDER, height=1).pack(fill="x", pady=4)


def log(msg: str):
    log_text.config(state="normal")
    ts = time.strftime("%H:%M:%S")
    log_text.insert("end", f"[{ts}] {msg}\n")
    log_text.see("end")
    log_text.config(state="disabled")


def _redistribute_headings():
    fleet.spread_headings()


def set_global_status(msg: str, color: str = TEXT):
    global_status_var.set(msg)
    global_status_label.config(fg=color)


# ─────────────────────────────────────────────
#  Acciones globales
# ─────────────────────────────────────────────
def select_object_click():
    set_global_status("Cámara activa…", ACCENT)
    log("Detección global: abriendo webcam…")
    camera_running = [True]

    def run():
        global selected_class_id
        result = detection.run_object_detection_loop(camera_running)
        selected_class_id = result
        window.after(0, _on_select_done, result)

    threading.Thread(target=run, daemon=True).start()


def _on_select_done(class_id):
    global selected_class_id
    if class_id is not None:
        selected_class_id = class_id
        name = detection.model.names[class_id]
        target_var.set(f"TARGET ▸ {name.upper()}")
        target_label.config(fg=ACCENT)
        set_global_status(f"Objeto: {name}", SUCCESS)
        start_btn.config(state="normal")
    else:
        set_global_status("Selección cancelada", TEXT_DIM)


def start_mission_click():
    """Toggle: START → PAUSE → RESUME → PAUSE → ..."""
    global mission_state

    # --- Pausar ---
    if mission_state == "running":
        fleet.mission_paused.set()
        mission_state = "paused"
        start_btn.config(text="▶ RESUME", fg=WARNING)
        set_global_status("Misión PAUSADA", WARNING)
        log("⏸ Misión pausada (drones en hover).")
        return

    # --- Reanudar ---
    if mission_state == "paused":
        fleet.mission_paused.clear()
        mission_state = "running"
        n = sum(1 for s in fleet.all() if not isinstance(s.drone, RemoteTello)
                and s.status in (DroneStatus.SEARCHING, DroneStatus.CONVERGING,
                                 DroneStatus.AT_TARGET, DroneStatus.FOUND,
                                 DroneStatus.RETURNING))
        start_btn.config(text="❚❚ PAUSE", fg=ACCENT)
        set_global_status(f"Misión activa ({n})", ACCENT)
        log("▶ Misión reanudada.")
        return

    # --- Arrancar (mission_state == "idle") ---
    if selected_class_id is None:
        log("No hay objetivo seleccionado.")
        return
    fleet.global_target_class_id = selected_class_id
    fleet.reset_mission()
    fleet.mission_paused.clear()
    mission_stop_event.clear()
    n = 0
    for state in fleet.all():
        if isinstance(state.drone, RemoteTello):
            continue
        if state.status in (DroneStatus.IDLE, DroneStatus.LANDED, DroneStatus.ERROR):
            continue
        if state.drone_id in mission_threads and mission_threads[state.drone_id].is_alive():
            continue
        if state.status == DroneStatus.CONNECTED:
            log(f"[{state.drone_id}] en suelo, ignorado")
            continue

        t_move = threading.Thread(
            target=mission_logic.run_drone_mission,
            args=(fleet, state.drone_id, mission_stop_event),
            daemon=True)
        t_move.start()
        mission_threads[state.drone_id] = t_move

        if state.is_real:
            t_vis = threading.Thread(
                target=mission_v2.run_real_drone_vision,
                args=(fleet, state.drone_id, mission_stop_event, selected_class_id),
                daemon=True)
            t_vis.start()
            log(f"[{state.drone_id}] visión YOLO arrancada")

        n += 1
    mission_state = "running"
    start_btn.config(text="❚❚ PAUSE", fg=ACCENT)
    log(f"▶ Misión iniciada ({n} drones)")
    set_global_status(f"Misión activa ({n})", ACCENT)


def reset_all_trackers():
    """
    Reset total: detiene la misión, resetea los trackers de todos los drones,
    limpia los flags y deja el sistema en estado 'idle' listo para un nuevo
    START. Los drones físicos se quedan donde estén (en aire o en suelo) —
    el operador los aterriza manualmente.
    """
    global mission_state
    was_active = mission_state in ("running", "paused")

    log("↺ RESET iniciado…")

    # Detectar drones LOCALES en aire (los remotos los gestiona su peer,
    # no los podemos aterrizar desde aquí)
    AIRBORNE = (DroneStatus.TAKING_OFF, DroneStatus.SEARCHING,
                DroneStatus.CONVERGING, DroneStatus.AT_TARGET,
                DroneStatus.FOUND, DroneStatus.RETURNING, DroneStatus.LANDING)
    drones_in_air = [s.drone_id for s in fleet.all()
                     if not s.is_remote and s.status in AIRBORNE]
    drones_on_ground = [s.drone_id for s in fleet.all()
                        if not s.is_remote and s.status not in AIRBORNE]
    log(f"   misión: {mission_state} · en aire: {len(drones_in_air)} "
        f"· en suelo: {len(drones_on_ground)}")

    # 1) Parar misión (si la hubiera)
    if was_active:
        mission_stop_event.set()
        fleet.mission_paused.clear()
        log(f"   misión detenida (los threads acabarán su comando actual)")

    # 2) Resetear trackers. Para drones LOCALES en SUELO reseteamos tracker
    #    y manual_pose. Para drones LOCALES EN AIRE NO tocamos el tracker
    #    (su X/Y/Z son reales y físicamente siguen en el aire), pero sí
    #    rearmamos su status (AT_TARGET/FOUND → SEARCHING) por si se
    #    relanza la misión. Drones remotos se ignoran.
    for s in fleet.all():
        if s.is_remote:
            continue
        if s.status in AIRBORNE:
            # No tocamos tracker — el dron está físicamente volando.
            if s.status in (DroneStatus.AT_TARGET, DroneStatus.FOUND):
                fleet.set_status(s.drone_id, DroneStatus.SEARCHING)
            continue
        # Suelo: full reset
        s.tracker.reset()
        s.manual_pose = False
    fleet.spread_headings()

    # 3) Limpiar estado de misión
    fleet.reset_mission()
    mission_state = "idle"
    start_btn.config(text="START", fg=ACCENT)

    suffix = " (+ misión detenida)" if was_active else ""
    log(f"↺ RESET completo{suffix}.")
    set_global_status("Sistema reseteado · Listo.", SUCCESS)

    # 4) Aviso si hay drones en hover
    if drones_in_air:
        listado = "\n".join(f"   • {d}" for d in drones_in_air)
        messagebox.showinfo(
            "Drones en hover",
            f"Tras el RESET, los siguientes drones siguen en aire:\n\n"
            f"{listado}\n\n"
            f"Quedan en hover libremente. Pulsa el botón ↓ en la "
            f"tarjeta de cada dron para aterrizarlo."
        )


def full_reset():
    """
    RESET TOTAL: detiene misión, desconecta TODOS los drones locales,
    devuelve las tarjetas a estado inicial. Conserva:
      • objetivo seleccionado (selected_class_id)
      • obstáculos dibujados (fleet.obstacles)
      • zoom del mapa
    Es como si la app se hubiera reiniciado pero el "escenario" (laberinto +
    qué buscar) se quedara intacto.
    """
    global mission_state
    if not messagebox.askyesno(
            "FULL RESET",
            "Esto desconecta TODOS los drones y los devuelve a estado "
            "inicial.\n\nSe conservan: objetivo seleccionado y obstáculos.\n\n"
            "¿Continuar?"):
        return

    log("✕ FULL RESET iniciado…")

    # 1) Parar misión
    mission_stop_event.set()
    fleet.mission_paused.clear()

    # 2) Aterrizar (si aplica) y desconectar cada dron LOCAL.
    #    Remotos se ignoran (su peer los gestiona).
    drone_ids = [s.drone_id for s in fleet.all() if not s.is_remote]
    for did in drone_ids:
        st = fleet.get(did)
        if st is None:
            continue
        try:
            st.drone.land()
        except Exception:
            pass
        try:
            st.drone.streamoff()
        except Exception:
            pass
        if mqtt_bridge is not None:
            try:
                mqtt_bridge.unregister_local_drone(did)
            except Exception:
                pass
        fleet.remove_drone(did)
        log(f"   [{did}] desconectado")

    # 3) Reset UI de las tarjetas
    for card in cards.values():
        card.reset_card_state()

    # 4) Limpiar estado de misión (NO toca obstacles ni selected_class_id)
    mission_threads.clear()
    fleet.target_found_at = None
    mission_state = "idle"
    start_btn.config(text="START", fg=ACCENT)
    # Si no había objetivo, dejar START disabled; si había, sigue habilitado
    if selected_class_id is None:
        start_btn.config(state="disabled", bg=TEXT_DIM)
    else:
        start_btn.config(state="normal", bg=PANEL)
    # mission_stop_event listo para una nueva misión
    mission_stop_event.clear()

    log(f"✕ FULL RESET completo. "
        f"Conservados: {len(fleet.get_obstacles())} obstáculos, "
        f"objetivo={'sí' if selected_class_id is not None else 'no'}.")
    set_global_status("Sistema desconectado · Listo para conectar de nuevo",
                      WARNING)


def toggle_record():
    global record_enabled
    record_enabled = not record_enabled
    if record_enabled:
        record_btn.config(text="● REC ON", bg=ACCENT2, fg=TEXT)
        log("Grabación armada.")
    else:
        record_btn.config(text="○ REC OFF", bg=PANEL, fg=ACCENT)
        log("Grabación desactivada.")
    for state in fleet.all(): state.record_enabled = record_enabled


def on_window_close():
    mission_stop_event.set()
    for state in fleet.all():
        try:
            state.drone.land()
            state.drone.streamoff()
        except Exception:
            pass
    if mqtt_bridge is not None:
        mqtt_bridge.disconnect()
    window.destroy()


# ─────────────────────────────────────────────
#  Mapa zenital
# ─────────────────────────────────────────────
def draw_map():
    global ui_tick
    c = map_canvas
    c.delete("all")
    cx, cy = MAP_W / 2, MAP_H / 2
    scale = _compute_scale()

    def to_cv(wx, wy):
        return (cx + (wx - viewport_pan_x) * scale,
                cy - (wy - viewport_pan_y) * scale)

    # Cuadrícula
    grid_px = 100 * scale
    if grid_px >= 8:
        n = int(MAP_W / 2 / grid_px) + 2
        for i in range(-n, n + 1):
            c.create_line(cx + i * grid_px, 0, cx + i * grid_px, MAP_H, fill="#181c28")
            c.create_line(0, cy - i * grid_px, MAP_W, cy - i * grid_px, fill="#181c28")

    # Origen y Ejes
    ox, oy = to_cv(0, 0)
    c.create_line(ox, 0, ox, MAP_H, fill=BORDER, dash=(4, 4))
    c.create_line(0, oy, MAP_W, oy, fill=BORDER, dash=(4, 4))
    c.create_oval(ox - 4, oy - 4, ox + 4, oy + 4, fill=SUCCESS, outline="")

    # Obstáculos (paredes) — en naranja para distinguir del dron rojo
    for obs in fleet.get_obstacles():
        x1, y1 = to_cv(obs.p1[0], obs.p1[1])
        x2, y2 = to_cv(obs.p2[0], obs.p2[1])
        c.create_line(x1, y1, x2, y2, fill=OBSTACLE_COLOR, width=4,
                      capstyle="round")
        c.create_oval(x1 - 3, y1 - 3, x1 + 3, y1 + 3,
                      fill=OBSTACLE_COLOR, outline="")
        c.create_oval(x2 - 3, y2 - 3, x2 + 3, y2 + 3,
                      fill=OBSTACLE_COLOR, outline="")

    # Drones
    for st in fleet.all():
        x, y, heading, path = st.tracker.get_state()
        if len(path) >= 2:
            pts = []
            for p in path:
                px_, py_ = to_cv(p[0], p[1])
                pts += [px_, py_]
            c.create_line(pts, fill=st.color, width=2)

        visible = (not st.blinking) or (ui_tick % 4 < 2)
        if visible:
            dx_, dy_ = to_cv(x, y)
            sz = 12
            r = math.radians(heading)
            tip = (dx_ + sz * math.sin(r), dy_ - sz * math.cos(r))
            left_ = (dx_ + sz * 0.6 * math.sin(r + 2.4), dy_ - sz * 0.6 * math.cos(r + 2.4))
            right_ = (dx_ + sz * 0.6 * math.sin(r - 2.4), dy_ - sz * 0.6 * math.cos(r - 2.4))
            # Drones remotos: borde gris más grueso para distinguirlos
            is_remote = isinstance(st.drone, RemoteTello)
            outline_color = "#888888" if is_remote else TEXT
            outline_width = 2 if is_remote else 1
            c.create_polygon([tip, left_, right_], fill=st.color,
                             outline=outline_color, width=outline_width)

    # Target
    target = fleet.get_target()
    if target:
        tx, ty, finder = target
        tcx, tcy = to_cv(tx, ty)
        if ui_tick % 4 < 2:
            c.create_line(tcx - 10, tcy, tcx + 10, tcy, fill=ACCENT2, width=3)
            c.create_line(tcx, tcy - 10, tcx, tcy + 10, fill=ACCENT2, width=3)

    ui_tick += 1
    window.after(250, draw_map)


# ─────────────────────────────────────────────
#  Loop de Eventos
# ─────────────────────────────────────────────
def consume_events():
    while True:
        try:
            ev = fleet.events.get_nowait()
        except queue.Empty:
            break
        card = cards.get(ev.drone_id)
        if ev.type == EventType.STATUS_CHANGED:
            if card:
                card.update_status(ev.payload["new"])
        elif ev.type == EventType.BATTERY:
            if card:
                card.update_battery(ev.payload["value"])
        elif ev.type == EventType.POSE_UPDATE:
            if card:
                card.on_pose_update()
        elif ev.type == EventType.LOG:
            log(f"[{ev.drone_id}] {ev.payload['msg']}")
        elif ev.type == EventType.TARGET_FOUND:
            log(f"⚡ {ev.drone_id} ENCONTRÓ en "
                f"({ev.payload['x']:.0f}, {ev.payload['y']:.0f})")
            _publish_target_found_if_bridge(ev.drone_id)
    window.after(100, consume_events)


def poll_batteries():
    for state in fleet.all():
        try:
            fleet.emit_battery(state.drone_id, state.drone.get_battery())
        except Exception:
            pass
    window.after(10000, poll_batteries)


# ─────────────────────────────────────────────
#  Detección en imagen, obstáculos, MQTT, ayuda
# ─────────────────────────────────────────────
def detect_image_click():
    log("Abriendo selector de imagen…")

    def run():
        global selected_class_id
        dets = image_detect.detect_in_image(window)
        if not dets:
            log("Sin detecciones o cancelado")
            return
        log(f"Imagen analizada: {len(dets)} objetos")
        for d in dets:
            log(f"  - {d['class_name']} ({d['confidence']:.2f})")
        # Establecer la detección de mayor confianza como objetivo de misión
        best = max(dets, key=lambda d: d['confidence'])
        log(f"⇒ Objetivo de misión: {best['class_name']} "
            f"(conf {best['confidence']:.2f})")
        # _on_select_done toca UI: lanzar en el thread principal
        window.after(0, _on_select_done, best['class_id'])

    threading.Thread(target=run, daemon=True).start()


def toggle_draw_obstacles():
    global drawing_obstacles
    drawing_obstacles = not drawing_obstacles
    if drawing_obstacles:
        draw_obs_btn.config(text="● DRAWING", bg=OBSTACLE_COLOR, fg=TEXT)
        log("Modo dibujo ON. Click+drag en el mapa.")
    else:
        draw_obs_btn.config(text="✎ DRAW OBS", bg=PANEL, fg=OBSTACLE_COLOR)
        log("Modo dibujo OFF.")


def clear_obstacles():
    fleet.clear_obstacles()
    if mqtt_bridge is not None:
        mqtt_bridge.publish_obstacles_clear()
    log("Obstáculos eliminados.")


def _autofit_extent():
    """Extent que abarca todos los drones (paths) y obstáculos. Mínimo 200 cm."""
    all_pts = [(0.0, 0.0)]
    for st in fleet.all():
        _, _, _, path = st.tracker.get_state()
        all_pts.extend(path)
    for o in fleet.get_obstacles():
        all_pts.extend([o.p1, o.p2])
    return max(max((abs(p[0]) for p in all_pts), default=0),
               max((abs(p[1]) for p in all_pts), default=0), 200)


def _compute_extent():
    """Extent actual: viewport_extent si está fijado, si no auto-fit."""
    return viewport_extent if viewport_extent is not None else _autofit_extent()


def _compute_scale():
    """Escala px/cm del mapa."""
    return (min(MAP_W, MAP_H) / 2 - 40) / _compute_extent()


def _canvas_to_world(px, py):
    """Píxel del canvas → coords mundo (cm). Respeta zoom y pan."""
    scale = _compute_scale()
    cx, cy = MAP_W / 2, MAP_H / 2
    wx = (px - cx) / scale + viewport_pan_x
    wy = (cy - py) / scale + viewport_pan_y
    return wx, wy


def _world_to_canvas(wx, wy):
    """Coords mundo (cm) → píxel del canvas. Respeta zoom y pan."""
    scale = _compute_scale()
    cx, cy = MAP_W / 2, MAP_H / 2
    px = cx + (wx - viewport_pan_x) * scale
    py = cy - (wy - viewport_pan_y) * scale
    return px, py


def _drone_at_click(px, py):
    """Devuelve el drone_id del dron LOCAL más cercano al click (dentro de
    un radio en píxeles), o None. Excluye drones remotos."""
    HIT_RADIUS = 18
    best_id, best_d = None, HIT_RADIUS
    for st in fleet.all():
        if isinstance(st.drone, RemoteTello):
            continue
        x, y, _, _ = st.tracker.get_state()
        dx_, dy_ = _world_to_canvas(x, y)
        d = math.hypot(dx_ - px, dy_ - py)
        if d < best_d:
            best_d = d
            best_id = st.drone_id
    return best_id


def on_map_click(event):
    global _obstacle_start
    # Si estamos en modo dibujo, click = inicio de obstáculo
    if drawing_obstacles:
        _obstacle_start = _canvas_to_world(event.x, event.y)
        return
    # Si no, ¿clickamos un dron? → abrir su PoseDialog
    drone_id = _drone_at_click(event.x, event.y)
    if drone_id is not None:
        PoseDialog(drone_id)


def _zoom_to(new_extent: float):
    """Aplica un extent nuevo, clamp a [MIN_EXTENT, MAX_EXTENT]."""
    global viewport_extent
    viewport_extent = max(MIN_EXTENT, min(MAX_EXTENT, new_extent))


def zoom_in():
    _zoom_to(_compute_extent() / ZOOM_FACTOR_STEP)


def zoom_out():
    _zoom_to(_compute_extent() * ZOOM_FACTOR_STEP)


def zoom_fit():
    """Vuelve al modo auto-fit y resetea el pan al origen."""
    global viewport_extent, viewport_pan_x, viewport_pan_y
    viewport_extent = None
    viewport_pan_x = 0.0
    viewport_pan_y = 0.0
    log("Mapa: auto-fit reactivado (pan reseteado)")


def on_map_wheel(event):
    """
    Rueda sobre el mapa = zoom CENTRADO EN EL CURSOR. El punto del mundo
    bajo el cursor permanece fijo en pantalla mientras se hace zoom.
    """
    global viewport_pan_x, viewport_pan_y, viewport_extent
    direction = 0
    if getattr(event, "delta", 0) > 0: direction = 1
    elif getattr(event, "delta", 0) < 0: direction = -1
    elif getattr(event, "num", 0) == 4: direction = 1
    elif getattr(event, "num", 0) == 5: direction = -1
    if direction == 0:
        return

    # 1) Punto del mundo bajo el cursor ANTES del zoom
    wx_before, wy_before = _canvas_to_world(event.x, event.y)

    # 2) Aplicar zoom
    if viewport_extent is None:
        viewport_extent = _autofit_extent()
    if direction > 0:
        new_extent = viewport_extent / ZOOM_FACTOR_STEP
    else:
        new_extent = viewport_extent * ZOOM_FACTOR_STEP
    viewport_extent = max(MIN_EXTENT, min(MAX_EXTENT, new_extent))

    # 3) Ajustar pan para que el mismo punto del mundo siga bajo el cursor.
    #    canvas_x = cx + (wx_before - pan_x) * scale_new
    #    => pan_x = wx_before - (canvas_x - cx) / scale_new
    scale_new = _compute_scale()
    cx, cy = MAP_W / 2, MAP_H / 2
    viewport_pan_x = wx_before - (event.x - cx) / scale_new
    viewport_pan_y = wy_before + (event.y - cy) / scale_new


def on_pan_start(event):
    """Click-derecho: empieza el arrastre del mapa."""
    global _pan_drag_start, viewport_extent
    # Si estábamos en auto-fit, fijamos el extent actual al inicio del pan
    if viewport_extent is None:
        viewport_extent = _autofit_extent()
    _pan_drag_start = (event.x, event.y, viewport_pan_x, viewport_pan_y)


def on_pan_motion(event):
    """Arrastre con click-derecho: actualiza el pan según el delta."""
    global viewport_pan_x, viewport_pan_y
    if _pan_drag_start is None:
        return
    sx, sy, spx, spy = _pan_drag_start
    dx_px = event.x - sx
    dy_px = event.y - sy
    scale = _compute_scale()
    # Mover el ratón a la derecha (dx_px > 0) → el mundo se "desliza" a la
    # derecha bajo el ratón → pan_x DISMINUYE (vemos el lado izquierdo del mundo).
    viewport_pan_x = spx - dx_px / scale
    viewport_pan_y = spy + dy_px / scale


def on_pan_end(event):
    global _pan_drag_start
    _pan_drag_start = None


def on_map_release(event):
    global _obstacle_start
    if not drawing_obstacles or _obstacle_start is None:
        return
    end = _canvas_to_world(event.x, event.y)
    if math.hypot(end[0] - _obstacle_start[0], end[1] - _obstacle_start[1]) > 30:
        fleet.add_obstacle(_obstacle_start, end)
        if mqtt_bridge is not None:
            mqtt_bridge.publish_obstacle(_obstacle_start, end)
        log(f"Obstáculo: ({_obstacle_start[0]:.0f},{_obstacle_start[1]:.0f}) "
            f"→ ({end[0]:.0f},{end[1]:.0f})")
    _obstacle_start = None


# ─────────────────────────────────────────────
#  Diálogo de pose: ver y editar (x, y, z, heading) de un dron
# ─────────────────────────────────────────────
class PoseDialog:
    def __init__(self, drone_id: str):
        state = fleet.get(drone_id)
        if state is None:
            return

        self.drone_id = drone_id
        self.win = tk.Toplevel(window)
        self.win.title(f"Drone {drone_id}")
        self.win.geometry("330x360")
        self.win.configure(bg=BG)
        self.win.transient(window)

        # Banda de color
        tk.Frame(self.win, bg=state.color, height=3).pack(fill="x")
        tk.Label(self.win, text=drone_id.upper(), font=title_f,
                 bg=BG, fg=state.color).pack(pady=(15, 2))
        tk.Label(self.win, text="Click APPLY para mover el dron a esta pose",
                 font=small_f, bg=BG, fg=TEXT_DIM).pack(pady=(0, 10))

        frame = tk.Frame(self.win, bg=PANEL, padx=15, pady=12,
                         highlightthickness=1, highlightbackground=BORDER)
        frame.pack(padx=20, fill="x")

        self.x_var = tk.StringVar()
        self.y_var = tk.StringVar()
        self.z_var = tk.StringVar()
        self.h_var = tk.StringVar()

        def _mk(label, var):
            row = tk.Frame(frame, bg=PANEL)
            row.pack(fill="x", pady=3)
            tk.Label(row, text=label, font=mono, bg=PANEL, fg=TEXT_DIM,
                     width=14, anchor="w").pack(side="left")
            entry = tk.Entry(row, textvariable=var, width=10, bg=BORDER,
                             fg=ACCENT, insertbackground=ACCENT, relief="flat",
                             font=mono_bold, justify="right")
            entry.pack(side="left", padx=5)
            return entry

        self.x_entry = _mk("X (cm)",       self.x_var)
        self.y_entry = _mk("Y (cm)",       self.y_var)
        self.z_entry = _mk("Z altura (cm)", self.z_var)
        self.h_entry = _mk("HEADING (°)",  self.h_var)

        # Info en vivo
        self.info_var = tk.StringVar()
        tk.Label(self.win, textvariable=self.info_var, font=small_f,
                 bg=BG, fg=TEXT_DIM).pack(pady=8)

        # Botones
        btn_row = tk.Frame(self.win, bg=BG)
        btn_row.pack(pady=10)
        tk.Button(btn_row, text="↻ READ", font=small_f, bg=PANEL, fg=ACCENT,
                  relief="flat", padx=8, pady=4, cursor="hand2",
                  command=self.refresh).pack(side="left", padx=3)
        tk.Button(btn_row, text="↺ RESET", font=small_f, bg=PANEL, fg=WARNING,
                  relief="flat", padx=8, pady=4, cursor="hand2",
                  command=self.reset_drone).pack(side="left", padx=3)
        tk.Button(btn_row, text="✓ APPLY", font=mono_bold, bg=PANEL,
                  fg=SUCCESS, relief="flat", padx=12, pady=4, cursor="hand2",
                  command=self.apply).pack(side="left", padx=3)
        tk.Button(btn_row, text="✕ CLOSE", font=small_f, bg=PANEL,
                  fg=TEXT_DIM, relief="flat", padx=8, pady=4, cursor="hand2",
                  command=self.win.destroy).pack(side="left", padx=3)

        self.refresh()
        self._tick()

    # Estados que consideramos "en aire" (Z editable)
    _AIRBORNE = (DroneStatus.TAKING_OFF, DroneStatus.SEARCHING,
                 DroneStatus.CONVERGING, DroneStatus.AT_TARGET,
                 DroneStatus.FOUND, DroneStatus.RETURNING,
                 DroneStatus.LANDING)

    def _is_airborne(self):
        state = fleet.get(self.drone_id)
        return state is not None and state.status in self._AIRBORNE

    def _apply_z_lock(self):
        """Bloquea el campo Z si el dron está en suelo (Z físico = 0)."""
        if self._is_airborne():
            self.z_entry.config(state="normal", fg=ACCENT)
        else:
            # En suelo: forzar Z = 0 y deshabilitar
            self.z_var.set("0")
            self.z_entry.config(state="disabled", fg=TEXT_DIM)

    def refresh(self):
        """Carga la pose actual del dron en los campos."""
        state = fleet.get(self.drone_id)
        if state is None:
            return
        x, y, h, _ = state.tracker.get_state()
        self.x_var.set(f"{x:.0f}")
        self.y_var.set(f"{y:.0f}")
        self.z_var.set(f"{state.tracker.z:.0f}")
        self.h_var.set(f"{h:.0f}")
        self._apply_z_lock()

    def _tick(self):
        """Refresca info en vivo (status, batería) cada 500ms."""
        state = fleet.get(self.drone_id)
        if state is not None:
            self.info_var.set(
                f"status: {state.status.value}   ·   bat: {state.last_battery}%"
            )
        # Re-evaluar lock de Z (por si el dron despegó / aterrizó mientras la
        # ventana está abierta)
        self._apply_z_lock()
        try:
            self.win.after(500, self._tick)
        except tk.TclError:
            pass   # ventana cerrada

    def apply(self):
        try:
            tx = float(self.x_var.get())
            ty = float(self.y_var.get())
            tz = float(self.z_var.get())
            th = float(self.h_var.get())
        except ValueError:
            log(f"[{self.drone_id}] Valores no válidos")
            return

        state = fleet.get(self.drone_id)
        if state is None:
            return

        airborne_states = (DroneStatus.TAKING_OFF, DroneStatus.SEARCHING,
                           DroneStatus.CONVERGING, DroneStatus.AT_TARGET,
                           DroneStatus.FOUND, DroneStatus.RETURNING,
                           DroneStatus.LANDING)

        if state.status not in airborne_states:
            # ── Dron en suelo: declarar pose inicial (no vuela)
            # Z se fuerza a 0 — físicamente está en el suelo, no se puede
            # despegar con Z>0 en el tracker.
            state.tracker.set_pose(x=tx, y=ty, heading=th, z=0)
            state.manual_pose = True
            fleet.emit_pose(self.drone_id)
            log(f"[{self.drone_id}]  Pose inicial fijada: "
                f"({tx:.0f}, {ty:.0f}) HDG{th:.0f}° (Z=0 forzado en suelo)")
            return

        # ── Dron en aire: navegación manual
        state.manual_target = (tx, ty, tz)
        log(f"[{self.drone_id}] ✎ Manual nav → ({tx:.0f}, {ty:.0f}, Z {tz:.0f})")
        if (self.drone_id in mission_threads
                and mission_threads[self.drone_id].is_alive()):
            return

        def run():
            try:
                mission_logic.execute_manual_nav(fleet, self.drone_id,
                                                 threading.Event())
            except Exception as e:
                log(f"[{self.drone_id}] error nav manual: {e}")

        threading.Thread(target=run, daemon=True).start()

    def reset_drone(self):
        """Resetea el tracker del dron al origen (0,0, heading=0, z=0).
        Si el dron está en aire, también lo deja en AT_TARGET para que
        el bucle de misión no le siga mandando comandos (evita que se
        mueva ligeramente tras el reset)."""
        state = fleet.get(self.drone_id)
        if state is None:
            return
        airborne = state.status in (DroneStatus.TAKING_OFF, DroneStatus.SEARCHING,
                                    DroneStatus.CONVERGING, DroneStatus.AT_TARGET,
                                    DroneStatus.FOUND, DroneStatus.RETURNING,
                                    DroneStatus.LANDING)
        state.tracker.reset()
        state.manual_pose = False
        if airborne:
            # Detener cualquier acción de misión sobre este dron
            fleet.set_status(self.drone_id, DroneStatus.AT_TARGET)
        fleet.emit_pose(self.drone_id)
        suffix = " (en hover, status AT_TARGET)" if airborne else ""
        log(f"[{self.drone_id}]  tracker reseteado a origen{suffix}")
        self.refresh()


def _publish_target_found_if_bridge(drone_id):
    """Publica target_found al broker si hay bridge y este peer es el finder."""
    if mqtt_bridge is None:
        return
    target = fleet.get_target()
    if target is not None and target[2] == drone_id:
        x, y, _ = target
        mqtt_bridge.publish_target_found(drone_id, x, y)


def toggle_mqtt_bridge():
    """Conecta/desconecta el bridge MQTT (modo distribuido multi-PC)."""
    global mqtt_bridge
    if mqtt_bridge is not None:
        mqtt_bridge.disconnect()
        mqtt_bridge = None
        bridge_btn.config(text="● BRIDGE OFF", fg=TEXT_DIM)
        peer_entry.config(state="normal")
        log("MQTT bridge desconectado.")
        return

    peer_id = peer_var.get().strip() or "peer1"
    log(f"Conectando MQTT bridge (peer_id={peer_id})…")

    def run():
        global mqtt_bridge
        try:
            mqtt_bridge = MQTTBridge(fleet, peer_id=peer_id)
            mqtt_bridge.connect()
            # Registrar los drones locales conectados (excluyendo remotos)
            for state in fleet.all():
                if not isinstance(state.drone, RemoteTello):
                    mqtt_bridge.register_local_drone(state.drone_id)
            window.after(0, lambda: bridge_btn.config(text="● BRIDGE ON", fg=SUCCESS))
            window.after(0, lambda: peer_entry.config(state="disabled"))
        except Exception as e:
            log(f"Error bridge: {e}")
            mqtt_bridge = None

    threading.Thread(target=run, daemon=True).start()


def show_help():
    win = tk.Toplevel(window)
    win.title("Cómo funciona el sistema")
    win.geometry("700x700")
    win.configure(bg=BG)

    tk.Label(win, text="FLOW DEL SISTEMA", font=title_f,
             bg=BG, fg=ACCENT).pack(pady=(15, 5))
    tk.Label(win, text="Búsqueda colaborativa multi-dron",
             font=small_f, bg=BG, fg=TEXT_DIM).pack(pady=(0, 15))

    frame = tk.Frame(win, bg=PANEL, highlightthickness=1, highlightbackground=BORDER)
    frame.pack(fill="both", expand=True, padx=20, pady=10)

    txt = tk.Text(frame, bg=PANEL, fg=TEXT, font=mono, relief="flat",
                  padx=15, pady=15, wrap="word")
    txt.pack(fill="both", expand=True)

    flow = """
1. CONFIGURAR DRONES
   Por cada dron (hasta 4):
   - Tipo: FAKE (simulado) o REAL (Tello físico)
   - Si REAL: rellenar IP (vacío = AP por defecto 192.168.10.1)
   - CONN → conecta y registra en la flota

2. DESPEGAR
   - Cada dron despega con ^ a la altura del campo ALTURA
   - Los headings se reparten automáticamente

3. SELECCIONAR OBJETIVO (OBJETO)
   Abre la webcam con YOLOv8. Mantén el objeto ~2s en cámara.

4. INICIAR MISIÓN (START / PAUSE / RESUME)
   Cada dron ejecuta rotation_iteration: gira en zigzag + avanza 75cm.
   Si hay obstáculo delante, gira 90° y reintenta.
   El botón START se convierte en PAUSE mientras corre la misión —
   púlsalo para que todos los drones queden en hover. Vuelve a pulsar
   (RESUME) para reanudar exactamente donde estaban.

5. DETECCIÓN
   Los drones REAL ejecutan YOLO. Cuando uno detecta:
   - Su tarjeta se pone roja y parpadea (FOUND)
   - El target aparece como ✕ rojo en el mapa
   - Los demás convergen siguiendo el REVERSE-PATH del finder

6. CONVERGENCIA REVERSE-PATH
   Cada seguidor busca el waypoint más cercano del path del finder
   y reproduce el resto del camino. Si un segmento choca con un
   obstáculo añadido durante la misión, se inserta un waypoint
   de rodeo automático.

7. RETURN TO HOME (↩ HOME en cada tarjeta)
   El dron reproduce su propio path al revés hasta llegar a (0,0).
   Si dibujaste obstáculos durante el vuelo, los rodea automáticamente.
   Al llegar queda en hover esperando land manual.

8. ATERRIZAR (v en cada tarjeta)

────────────────────────────────────────────
OBSTÁCULOS

DRAW OBS    Click+drag en el mapa para dibujar paredes (líneas rojas).
CLEAR OBS   Eliminar todos los obstáculos.

────────────────────────────────────────────
ZOOM DEL MAPA

Rueda del ratón sobre el mapa: zoom in/out.
Botones bajo el mapa:
   −    aleja la vista
   +    acerca la vista
   FIT  vuelve al auto-fit (todo el contenido a pantalla)

────────────────────────────────────────────
EDICIÓN DE POSE (botón ✎ en cada tarjeta o click en el mapa)

Cada tarjeta tiene un botón ✎ que abre la ventana de pose.
También puedes clicar el dron en el mapa (con DRAW OBS apagado).
La ventana muestra X, Y, Z, heading en vivo y deja modificarlos:

  • Si el dron está EN SUELO al pulsar APPLY → declara su pose
    inicial. El takeoff respetará esa pose (no la sobrescribirá).
  • Si el dron está EN AIRE al pulsar APPLY → ejecuta nav manual:
    ajusta altitud primero y vuela XY respetando obstáculos.

Botones del diálogo:
  ↻ READ    Recarga los valores actuales del dron.
  ↺ RESET   Resetea el tracker a (0,0,heading=0). Útil si se queda
            atascado o quieres "olvidar" su recorrido.
  ✓ APPLY   Confirma los cambios (declarar o navegar).
  ✕ CLOSE   Cierra sin tocar nada.

────────────────────────────────────────────
RESET GLOBAL (botón ↺ RESET en la fila principal)

Hace una reinicialización completa:
  • Detiene la misión si está activa (o pausada).
  • Resetea los trackers de TODOS los drones a (0,0,heading=0,z=0)
    y reparte headings cardinales.
  • Limpia los flags `manual_pose` y el target_found.
  • Vuelve el botón START a "START" (puedes lanzar misión de cero).

Los drones físicos NO aterrizan automáticamente. Si están en aire
se quedan en hover hasta que pulses ↓ en cada tarjeta.

────────────────────────────────────────────
AÑADIR DRONES MID-MISIÓN

Si la misión ya está corriendo (o pausada) y conectas + despegas un
dron nuevo, este se une automáticamente a la búsqueda. No hace falta
volver a pulsar START.

────────────────────────────────────────────
MODO DISTRIBUIDO (varios PCs)

PEER ID     Identificador único de este PC (ej. "pc-marc")
BRIDGE      Conecta al broker MQTT y sincroniza con otros PCs.

Cada PC ejecuta su main.py con su propio dron real. Los drones
de otros PCs aparecen como REMOTE en el mapa (no se pueden
controlar desde aquí) pero comparten target_found y obstáculos.
Si un dron remoto detecta el objetivo, tus drones locales
convergen hacia su posición automáticamente.

────────────────────────────────────────────
EXTRAS

DETECT IMG    Analiza una imagen del disco con YOLOv8.
REC           Activa grabación de vídeo de los drones REAL.
SIM DETECT    Simula una detección por dron (para pruebas).
↺ RESET TRACKERS    Resetea posiciones X,Y,heading.
"""
    txt.insert("1.0", flow)
    txt.config(state="disabled")

    tk.Button(win, text="ENTENDIDO", font=mono_bold,
              bg=PANEL, fg=SUCCESS, relief="flat", padx=20, pady=5,
              command=win.destroy).pack(pady=10)


# ─────────────────────────────────────────────
#  CONSTRUCCIÓN UI (NUEVO LAYOUT LATERAL)
# ─────────────────────────────────────────────
window = tk.Tk()
window.title("TELLO CONTROL CENTER · MULTI")
window.geometry("1350x850")  # Ventana más ancha
window.configure(bg=BG)

ttk_style = ttk.Style()
ttk_style.theme_use("clam")
ttk_style.configure("TCombobox", fieldbackground=BORDER, background=PANEL, foreground=ACCENT, arrowcolor=ACCENT,
                    bordercolor=BORDER)

mono = tkfont.Font(family="Courier", size=9)
mono_bold = tkfont.Font(family="Courier", size=10, weight="bold")
title_f = tkfont.Font(family="Courier", size=16, weight="bold")
small_f = tkfont.Font(family="Courier", size=8)

# --- PANEL IZQUIERDO (SIDEBAR) ---
sidebar = tk.Frame(window, bg=BG, width=450)
sidebar.pack(side="left", fill="y", padx=15, pady=15)

tk.Label(sidebar, text="TELLO COMMAND", font=title_f, bg=BG, fg=ACCENT).pack(anchor="w")
global_status_var = tk.StringVar(value="— No conectado")
global_status_label = tk.Label(sidebar, textvariable=global_status_var, font=small_f, bg=BG, fg=TEXT_DIM)
global_status_label.pack(anchor="w", pady=(0, 10))

# Rejilla 2x2 para las tarjetas de drones (Comprimido)
cards_grid = tk.Frame(sidebar, bg=BG)
cards_grid.pack(fill="x", pady=5)
cards: dict[str, DroneCard] = {}
for i, name in enumerate(DRONE_SLOTS):
    card = DroneCard(cards_grid, name, Fleet.DRONE_COLORS[i])
    card.frame.grid(row=i // 2, column=i % 2, padx=4, pady=4, sticky="nsew")
    cards[name] = card

sep(sidebar)

# Controles Globales (Debajo de las tarjetas)
tk.Label(sidebar, text="GLOBAL CONTROLS", font=small_f, bg=BG, fg=TEXT_DIM).pack(anchor="w")
g_ctrl = tk.Frame(sidebar, bg=BG)
g_ctrl.pack(fill="x", pady=5)

target_var = tk.StringVar(value="TARGET ▸ n/a")
target_label = tk.Label(g_ctrl, textvariable=target_var, font=mono_bold, bg=BG, fg=TEXT_DIM)
target_label.pack(anchor="w")

btn_row1 = tk.Frame(sidebar, bg=BG)
btn_row1.pack(fill="x", pady=5)
select_btn = tk.Button(btn_row1, text="OBJETO", font=mono_bold, bg=PANEL, fg=ACCENT, relief="flat", padx=10, pady=5,
                       command=select_object_click)
select_btn.pack(side="left", expand=True, fill="x", padx=2)
start_btn = tk.Button(btn_row1, text="START", font=mono_bold, bg=PANEL, fg=ACCENT, relief="flat", padx=10, pady=5,
                      state="disabled", command=start_mission_click)
start_btn.pack(side="left", expand=True, fill="x", padx=2)
record_btn = tk.Button(btn_row1, text="REC OFF", font=mono_bold, bg=PANEL, fg=ACCENT, relief="flat", padx=10, pady=5,
                       command=toggle_record)
record_btn.pack(side="left", expand=True, fill="x", padx=2)

reset_btn = tk.Button(btn_row1, text="↺ MISIÓN", font=mono_bold,
                      bg=PANEL, fg=WARNING, relief="flat", padx=10, pady=5,
                      cursor="hand2", command=reset_all_trackers)
reset_btn.pack(side="left", expand=True, fill="x", padx=2)

# Fila 2: ayuda + detectar imagen
btn_row2 = tk.Frame(sidebar, bg=BG)
btn_row2.pack(fill="x", pady=(2, 0))
help_btn = tk.Button(btn_row2, text="¿CÓMO FUNCIONA?", font=mono_bold,
                     bg=PANEL, fg=WARNING, relief="flat", padx=10, pady=5,
                     command=show_help)
help_btn.pack(side="left", expand=True, fill="x", padx=2)
img_btn = tk.Button(btn_row2, text="DETECT IMG", font=mono_bold,
                    bg=PANEL, fg=ACCENT, relief="flat", padx=10, pady=5,
                    command=detect_image_click)
img_btn.pack(side="left", expand=True, fill="x", padx=2)

# Fila 3: obstáculos
btn_row3 = tk.Frame(sidebar, bg=BG)
btn_row3.pack(fill="x", pady=(2, 0))
draw_obs_btn = tk.Button(btn_row3, text="✎ DRAW OBS", font=mono_bold,
                         bg=PANEL, fg=OBSTACLE_COLOR, relief="flat",
                         padx=10, pady=5, command=toggle_draw_obstacles)
draw_obs_btn.pack(side="left", expand=True, fill="x", padx=2)
clear_obs_btn = tk.Button(btn_row3, text="✕ CLEAR OBS", font=mono_bold,
                          bg=PANEL, fg=TEXT_DIM, relief="flat", padx=10, pady=5,
                          command=clear_obstacles)
clear_obs_btn.pack(side="left", expand=True, fill="x", padx=2)

full_reset_btn = tk.Button(btn_row3, text="✕ FULL RESET", font=mono_bold,
                           bg=PANEL, fg=ACCENT2, relief="flat",
                           padx=10, pady=5, cursor="hand2",
                           command=full_reset)
full_reset_btn.pack(side="left", expand=True, fill="x", padx=2)

# Modo distribuido (MQTT bridge)
sep(sidebar)
tk.Label(sidebar, text="MODO DISTRIBUIDO", font=small_f,
         bg=BG, fg=TEXT_DIM).pack(anchor="w")

peer_row = tk.Frame(sidebar, bg=BG)
peer_row.pack(fill="x", pady=2)
tk.Label(peer_row, text="PEER ID:", font=small_f, bg=BG, fg=TEXT_DIM).pack(side="left")
peer_var = tk.StringVar(value="peer1")
peer_entry = tk.Entry(peer_row, textvariable=peer_var, width=12,
                      bg=BORDER, fg=ACCENT, insertbackground=ACCENT,
                      relief="flat", font=small_f)
peer_entry.pack(side="left", padx=5)

bridge_btn = tk.Button(sidebar, text="● BRIDGE OFF", font=mono_bold,
                       bg=PANEL, fg=TEXT_DIM, relief="flat", padx=10, pady=5,
                       command=toggle_mqtt_bridge)
bridge_btn.pack(fill="x", pady=2)

h_row = tk.Frame(sidebar, bg=BG)
h_row.pack(fill="x")
tk.Label(h_row, text="ALTURA (cm):", font=small_f, bg=BG, fg=TEXT_DIM).pack(side="left")
height_var = tk.StringVar(value="100")
tk.Entry(h_row, textvariable=height_var, width=5, bg=BORDER, fg=ACCENT, relief="flat", font=mono_bold).pack(side="left",
                                                                                                            padx=5)

# Log (Ocupando el resto de la sidebar)
log_outer = tk.Frame(sidebar, bg=PANEL, highlightthickness=1, highlightbackground=BORDER)
log_outer.pack(fill="both", expand=True, pady=(15, 0))
log_text = tk.Text(log_outer, bg=PANEL, fg=TEXT_DIM, font=small_f, relief="flat", bd=0, state="disabled", wrap="word",
                   padx=8, pady=8)
log_text.pack(fill="both", expand=True)

# --- PANEL DERECHO (MAPA COMPLETO) ---
map_panel = tk.Frame(window, bg=PANEL, highlightthickness=1, highlightbackground=BORDER)
map_panel.pack(side="right", fill="both", expand=True, padx=(0, 15), pady=15)

tk.Frame(map_panel, bg=ACCENT2, height=3).pack(fill="x")
tk.Label(map_panel, text="SISTEMA DE MONITOREO ZENITAL", font=small_f, bg=PANEL, fg=TEXT_DIM).pack(pady=5)

map_canvas = tk.Canvas(map_panel, width=MAP_W, height=MAP_H, bg=BG, highlightthickness=0)
map_canvas.pack(fill="both", expand=True, padx=10, pady=10)
map_canvas.bind("<Button-1>", on_map_click)
map_canvas.bind("<ButtonRelease-1>", on_map_release)
# Zoom con la rueda del ratón (centrado en el cursor)
map_canvas.bind("<MouseWheel>", on_map_wheel)     # Windows / macOS
map_canvas.bind("<Button-4>",   on_map_wheel)     # Linux scroll up
map_canvas.bind("<Button-5>",   on_map_wheel)     # Linux scroll down
# Pan con click-derecho (arrastrar para deslizar la vista)
map_canvas.bind("<Button-3>",        on_pan_start)
map_canvas.bind("<B3-Motion>",       on_pan_motion)
map_canvas.bind("<ButtonRelease-3>", on_pan_end)

# Fila de controles del mapa: reset + zoom
map_ctrl_row = tk.Frame(map_panel, bg=PANEL)
map_ctrl_row.pack(fill="x", padx=10, pady=(0, 8))

tk.Button(map_ctrl_row, text="−", font=mono_bold, bg=PANEL, fg=ACCENT,
          relief="flat", padx=10, pady=5, cursor="hand2",
          command=zoom_out).pack(side="left", padx=2)
tk.Button(map_ctrl_row, text="+", font=mono_bold, bg=PANEL, fg=ACCENT,
          relief="flat", padx=10, pady=5, cursor="hand2",
          command=zoom_in).pack(side="left", padx=2)
tk.Button(map_ctrl_row, text="⤢ FIT", font=small_f, bg=PANEL, fg=WARNING,
          relief="flat", padx=8, pady=5, cursor="hand2",
          command=zoom_fit).pack(side="left", padx=2)

# Arranque
window.protocol("WM_DELETE_WINDOW", on_window_close)
window.after(250, draw_map)
window.after(100, consume_events)
window.after(10000, poll_batteries)
window.mainloop()