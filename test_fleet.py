"""
Test integrado de Fleet + DroneTracker + mapa zenital con 2 FakeTellos.

Lo que verás:
    - Una ventana con un mapa zenital (sin botones, solo visualización).
    - 2 drones (alpha cyan, bravo rojo) ejecutando su rotation_iteration
      en paralelo.
    - A los ~12 segundos, alpha "detecta" el objetivo: queda en FOUND y su
      icono empieza a parpadear.
    - Bravo sigue buscando (la lógica de convergencia llega en el paso 4).

Ejecuta:
    python test_fleet.py
"""
import threading
import time
import math
import tkinter as tk

from drone_iface import FakeTello
from fleet import Fleet, DroneStatus, EventType


# ─── Parámetros de la "misión" simulada ────────────
SEARCH_ROTATION = 30
SEARCH_FORWARD  = 75

MAP_W, MAP_H = 480, 480
BG     = "#0d0f14"
PANEL  = "#13161e"
TEXT   = "#e8eaf0"
DIM    = "#4a5060"
BORDER = "#1e2330"
GREEN  = "#00e676"


# ─── Convergencia: ir en línea recta a una pose dada ───────────
def converge_to(fleet: Fleet, drone_id: str, tx: float, ty: float,
                stop_event: threading.Event):
    """
    Gira hacia (tx, ty) y avanza hasta llegar. En línea recta de momento;
    el reverse-path real llega en el paso 5.
    """
    state = fleet.get(drone_id)
    drone = state.drone
    tracker = state.tracker

    fleet.set_status(drone_id, DroneStatus.CONVERGING)
    x, y, heading, _ = tracker.get_state()
    dx = tx - x
    dy = ty - y
    distance = math.hypot(dx, dy)

    if distance < 25:
        fleet.set_status(drone_id, DroneStatus.AT_TARGET)
        return

    # Bearing desde norte (y+ es norte, x+ es este → atan2(dx, dy))
    target_bearing = math.degrees(math.atan2(dx, dy)) % 360
    rotation = (target_bearing - heading) % 360

    # Elegir el sentido de giro más corto
    if rotation > 180:
        deg = int(round(360 - rotation))
        if deg >= 1 and not stop_event.is_set():
            drone.rotate_counter_clockwise(deg)
            tracker.rotate_ccw(deg)
            fleet.emit_pose(drone_id)
    else:
        deg = int(round(rotation))
        if deg >= 1 and not stop_event.is_set():
            drone.rotate_clockwise(deg)
            tracker.rotate_cw(deg)
            fleet.emit_pose(drone_id)

    # Avanzar a trozos (Tello SDK: 20-500 cm por comando)
    remaining = int(round(distance))
    while remaining >= 20 and not stop_event.is_set():
        chunk = min(remaining, 500)
        drone.move_forward(chunk)
        tracker.move_forward(chunk)
        fleet.emit_pose(drone_id)
        remaining -= chunk

    # Llegó al punto del finder: hover esperando land manual
    fleet.set_status(drone_id, DroneStatus.AT_TARGET)


# ─── Bucle de búsqueda fake (equivalente a rotation_iteration) ─
def search_thread(fleet: Fleet, drone_id: str, stop_event: threading.Event):
    state = fleet.get(drone_id)
    drone = state.drone
    tracker = state.tracker

    fleet.set_status(drone_id, DroneStatus.TAKING_OFF)
    drone.takeoff()
    fleet.set_status(drone_id, DroneStatus.SEARCHING)

    converged = False  # solo convergemos una vez por misión

    def someone_else_found():
        t = fleet.get_target()
        return t is not None and t[2] != drone_id

    while not stop_event.is_set():
        # 1) Si ya estoy en estado terminal (encontré yo, o llegué al target): hover
        if state.status in (DroneStatus.FOUND, DroneStatus.AT_TARGET):
            time.sleep(0.5)
            continue

        # 2) Si otro lo encontró y aún no he convergido: voy
        if someone_else_found() and not converged:
            tx, ty, _ = fleet.get_target()
            converge_to(fleet, drone_id, tx, ty, stop_event)
            converged = True
            continue   # converge_to deja status=AT_TARGET → próxima iteración hover

        # 3) rotation_iteration normal con tracker hooks
        interrupted = False
        for action in [
            ("cw",  SEARCH_ROTATION),
            ("cw",  SEARCH_ROTATION),
            ("ccw", 2 * SEARCH_ROTATION),
            ("ccw", SEARCH_ROTATION),
            ("ccw", SEARCH_ROTATION),
            ("cw",  2 * SEARCH_ROTATION),
        ]:
            if stop_event.is_set() or state.status == DroneStatus.FOUND:
                interrupted = True
                break
            # Interrumpir si llega un target mid-iteración
            if someone_else_found() and not converged:
                interrupted = True
                break
            kind, deg = action
            if kind == "cw":
                drone.rotate_clockwise(deg)
                tracker.rotate_cw(deg)
            else:
                drone.rotate_counter_clockwise(deg)
                tracker.rotate_ccw(deg)
            fleet.emit_pose(drone_id)

        if not interrupted and not stop_event.is_set() and state.status != DroneStatus.FOUND:
            drone.move_forward(SEARCH_FORWARD)
            tracker.move_forward(SEARCH_FORWARD)
            fleet.emit_pose(drone_id)


def detection_simulator(fleet: Fleet, drone_id: str, after_seconds: float):
    """Tras X segundos, marca al dron `drone_id` como 'el que encontró'."""
    time.sleep(after_seconds)
    print(f"\n>>> Simulando detección de {drone_id}\n")
    fleet.report_target_found(drone_id)


# ─── UI mínima (mapa zenital multi-dron) ───────────
class MapWindow:
    def __init__(self, fleet: Fleet):
        self.fleet = fleet
        self.tick = 0   # contador para parpadeo

        self.root = tk.Tk()
        self.root.title("FLEET MAP TEST")
        self.root.configure(bg=BG)

        tk.Label(self.root, text="MAPA ZENITAL · TEST FLOTA",
                 font=("Courier", 12, "bold"), bg=BG, fg="#00e5ff").pack(pady=(12, 4))

        self.canvas = tk.Canvas(self.root, width=MAP_W, height=MAP_H,
                                bg=BG, highlightthickness=0)
        self.canvas.pack(padx=14, pady=8)

        self.status_var = tk.StringVar(value="Iniciando…")
        tk.Label(self.root, textvariable=self.status_var,
                 font=("Courier", 9), bg=BG, fg=DIM).pack(pady=(0, 12))

    def draw(self):
        c = self.canvas
        c.delete("all")
        cx, cy = MAP_W / 2, MAP_H / 2

        # Escala automática a partir de todos los paths
        all_pts = [(0.0, 0.0)]
        for st in self.fleet.all():
            _, _, _, path = st.tracker.get_state()
            all_pts.extend(path)
        extent = max(
            max(abs(p[0]) for p in all_pts),
            max(abs(p[1]) for p in all_pts),
            150,
        )
        scale = (min(MAP_W, MAP_H) / 2 - 30) / extent

        def to_cv(wx, wy):
            return cx + wx * scale, cy - wy * scale

        # Cuadrícula 100 cm
        grid_px = 100 * scale
        if grid_px >= 8:
            n = int(MAP_W / 2 / grid_px) + 2
            for i in range(-n, n + 1):
                c.create_line(cx + i*grid_px, 0, cx + i*grid_px, MAP_H, fill="#181c28")
                c.create_line(0, cy - i*grid_px, MAP_W, cy - i*grid_px, fill="#181c28")

        # Ejes
        ox, oy = to_cv(0, 0)
        c.create_line(ox, 0, ox, MAP_H, fill=BORDER, dash=(4, 4))
        c.create_line(0, oy, MAP_W, oy, fill=BORDER, dash=(4, 4))

        # Norte
        c.create_text(MAP_W - 14, 14, text="N", fill=DIM, font=("Courier", 8))
        c.create_line(MAP_W - 14, 22, MAP_W - 14, 32, fill=DIM, arrow=tk.LAST)

        # Marcador de inicio
        sx, sy = to_cv(0, 0)
        c.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill=GREEN, outline="")

        # Por dron: path + dron
        status_lines = []
        for st in self.fleet.all():
            x, y, heading, path = st.tracker.get_state()

            # Path
            if len(path) >= 2:
                pts = []
                for p in path:
                    px_, py_ = to_cv(p[0], p[1])
                    pts += [px_, py_]
                c.create_line(pts, fill=st.color, width=2)

            # Triángulo del dron (parpadeo si blinking)
            visible = (not st.blinking) or (self.tick % 4 < 2)
            if visible:
                dx_, dy_ = to_cv(x, y)
                sz = 11
                r = math.radians(heading)
                tip   = (dx_ + sz       * math.sin(r),       dy_ - sz       * math.cos(r))
                left_ = (dx_ + sz * 0.6 * math.sin(r + 2.4), dy_ - sz * 0.6 * math.cos(r + 2.4))
                right_= (dx_ + sz * 0.6 * math.sin(r - 2.4), dy_ - sz * 0.6 * math.cos(r - 2.4))
                outline = "#ffffff" if st.blinking else TEXT
                c.create_polygon([tip, left_, right_], fill=st.color, outline=outline, width=1)
                # etiqueta
                c.create_text(dx_ + 14, dy_ - 14, text=st.drone_id.upper(),
                              fill=st.color, font=("Courier", 8, "bold"), anchor="w")

            status_lines.append(
                f"{st.drone_id}: {st.status.value} · "
                f"X{x:+.0f} Y{y:+.0f} HDG{heading:.0f}° · "
                f"path={len(path)}"
            )

        # Marcador del objetivo (si alguien lo encontró)
        target = self.fleet.get_target()
        if target:
            tx, ty, finder = target
            tcx, tcy = to_cv(tx, ty)
            # cruz parpadeante
            if self.tick % 4 < 2:
                c.create_line(tcx-10, tcy, tcx+10, tcy, fill="#ff4c4c", width=3)
                c.create_line(tcx, tcy-10, tcx, tcy+10, fill="#ff4c4c", width=3)
            c.create_text(tcx, tcy + 18, text=f"TARGET ({finder})",
                          fill="#ff4c4c", font=("Courier", 8, "bold"))

        self.status_var.set("\n".join(status_lines))
        self.tick += 1
        self.root.after(250, self.draw)

    def run(self):
        self.root.after(250, self.draw)
        self.root.mainloop()


# ─── Main ──────────────────────────────────────────
def main():
    fleet = Fleet()

    # Crear 2 fakes y registrarlos
    alpha = FakeTello(name="alpha")
    bravo = FakeTello(name="bravo")
    alpha.connect()
    bravo.connect()
    fleet.add_drone("alpha", alpha)
    fleet.add_drone("bravo", bravo)

    # Headings iniciales opuestos: alpha mira al norte (0°), bravo al sur (180°).
    # Con 4 drones esto sería 0°/90°/180°/270°.
    fleet.spread_headings()

    stop_event = threading.Event()

    # Lanzar misiones en paralelo
    t_alpha = threading.Thread(target=search_thread, args=(fleet, "alpha", stop_event), daemon=True)
    t_bravo = threading.Thread(target=search_thread, args=(fleet, "bravo", stop_event), daemon=True)
    t_alpha.start()
    t_bravo.start()

    # A los 12 s, alpha "detecta" el objetivo
    threading.Thread(target=detection_simulator, args=(fleet, "alpha", 12.0), daemon=True).start()

    # Abrir UI (bloquea aquí hasta cerrar la ventana)
    try:
        MapWindow(fleet).run()
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
