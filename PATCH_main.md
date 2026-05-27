# PATCH para main.py — Cambios completos

Este patch añade al main.py existente:
1. Botón "¿CÓMO FUNCIONA?" + "DETECT IMG"
2. Botones DRAW OBS + CLEAR OBS y dibujo de obstáculos en el mapa
3. Arranque de mission_v2 (visión YOLO) para drones REAL en START
4. **Modo distribuido**: panel para conectar a un broker MQTT y sincronizar
   el Fleet con otros PCs. Cada PC ejecuta su main.py con sus drones locales
   y se ven todos los drones en todos los mapas.

---

## 1. Imports nuevos

```python
import image_detect
import mission_v2
from mqtt_bridge import MQTTBridge
```

---

## 2. Estado global nuevo

```python
# Dibujado de obstáculos
drawing_obstacles = False
_obstacle_start = None

# Modo distribuido
mqtt_bridge: MQTTBridge | None = None
```

---

## 3. Funciones de acciones globales

```python
def detect_image_click():
    log("Abriendo selector de imagen…")
    def run():
        dets = image_detect.detect_in_image(window)
        if dets:
            log(f"Imagen analizada: {len(dets)} objetos")
            for d in dets:
                log(f"  - {d['class_name']} ({d['confidence']:.2f})")
        else:
            log("Sin detecciones o cancelado")
    threading.Thread(target=run, daemon=True).start()


def toggle_draw_obstacles():
    global drawing_obstacles
    drawing_obstacles = not drawing_obstacles
    if drawing_obstacles:
        draw_obs_btn.config(text="● DRAWING", bg=ACCENT2, fg=TEXT)
        log("Modo dibujo ON. Click+drag en el mapa.")
    else:
        draw_obs_btn.config(text="✎ DRAW OBS", bg=PANEL, fg=ACCENT2)
        log("Modo dibujo OFF.")


def clear_obstacles():
    fleet.clear_obstacles()
    if mqtt_bridge is not None:
        mqtt_bridge.publish_obstacles_clear()
    log("Obstáculos eliminados.")


def _canvas_to_world(px, py):
    """Convierte coords de píxel del canvas a coords mundo (cm)."""
    all_pts = [(0.0, 0.0)]
    for st in fleet.all():
        _, _, _, path = st.tracker.get_state()
        all_pts.extend(path)
    for o in fleet.get_obstacles():
        all_pts.extend([o.p1, o.p2])
    extent = max(max((abs(p[0]) for p in all_pts), default=0),
                 max((abs(p[1]) for p in all_pts), default=0), 200)
    scale = (min(MAP_W, MAP_H) / 2 - 40) / extent
    cx, cy = MAP_W / 2, MAP_H / 2
    wx = (px - cx) / scale
    wy = (cy - py) / scale
    return wx, wy


def on_map_click(event):
    global _obstacle_start
    if not drawing_obstacles:
        return
    _obstacle_start = _canvas_to_world(event.x, event.y)


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


def toggle_mqtt_bridge():
    """Conecta o desconecta el bridge MQTT (modo distribuido)."""
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
            # Registrar los drones locales que ya están conectados
            for state in fleet.all():
                if state.is_real or isinstance(state.drone, type(state.drone)) \
                        and state.drone.__class__.__name__ in ("RealTello", "FakeTello"):
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

4. INICIAR MISIÓN (START)
   Cada dron ejecuta rotation_iteration: gira en zigzag + avanza 75cm.
   Si hay obstáculo delante, gira 90° y reintenta.

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

7. ATERRIZAR (v en cada tarjeta)

────────────────────────────────────────────
OBSTÁCULOS

DRAW OBS    Click+drag en el mapa para dibujar paredes (líneas rojas).
CLEAR OBS   Eliminar todos los obstáculos.

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
```

---

## 4. Reemplazar `start_mission_click`

```python
def start_mission_click():
    if selected_class_id is None:
        log("No hay objetivo seleccionado.")
        return
    fleet.global_target_class_id = selected_class_id
    fleet.reset_mission()
    mission_stop_event.clear()
    n = 0
    for state in fleet.all():
        # Los drones remotos no se controlan desde aquí
        from drone_iface import RemoteTello
        if isinstance(state.drone, RemoteTello):
            continue
        if state.status in (DroneStatus.IDLE, DroneStatus.LANDED, DroneStatus.ERROR):
            continue
        if state.drone_id in mission_threads and mission_threads[state.drone_id].is_alive():
            continue
        if state.status == DroneStatus.CONNECTED:
            log(f"[{state.drone_id}] en suelo, ignorado")
            continue

        # Thread de movimiento
        t_move = threading.Thread(
            target=mission_logic.run_drone_mission,
            args=(fleet, state.drone_id, mission_stop_event),
            daemon=True)
        t_move.start()
        mission_threads[state.drone_id] = t_move

        # Visión YOLO solo para los REAL
        if state.is_real:
            t_vis = threading.Thread(
                target=mission_v2.run_real_drone_vision,
                args=(fleet, state.drone_id, mission_stop_event, selected_class_id),
                daemon=True)
            t_vis.start()
            log(f"[{state.drone_id}] visión YOLO arrancada")

        n += 1
    log(f"Misión iniciada ({n} drones)")
    set_global_status(f"Misión activa ({n})", ACCENT)
```

---

## 5. Hook en `DroneCard.connect_click` para registrar en bridge

Dentro de `_on_connected()` añade al final:

```python
        # Si el bridge está activo, registrar este dron como local
        if mqtt_bridge is not None:
            mqtt_bridge.register_local_drone(self.drone_id)
```

---

## 6. Hook en `fleet.report_target_found` desde mission_v2

`mission_v2.run_real_drone_vision` ya llama a `fleet.report_target_found(drone_id)`.
Necesitamos que esto también se publique por MQTT cuando hay bridge.

Añade esta función helper junto a las acciones globales:

```python
def _publish_target_found_if_bridge(drone_id):
    """Hook para publicar la detección al broker MQTT si bridge activo."""
    if mqtt_bridge is None:
        return
    target = fleet.get_target()
    if target is not None and target[2] == drone_id:
        x, y, _ = target
        mqtt_bridge.publish_target_found(drone_id, x, y)
```

Y en el `consume_events()`, dentro del bucle, añade este caso:

```python
        elif ev.type == EventType.TARGET_FOUND:
            _publish_target_found_if_bridge(ev.drone_id)
```

---

## 7. Botones nuevos en el sidebar

Después del bloque `btn_row1`, añade:

```python
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

btn_row3 = tk.Frame(sidebar, bg=BG)
btn_row3.pack(fill="x", pady=(2, 0))
draw_obs_btn = tk.Button(btn_row3, text="✎ DRAW OBS", font=mono_bold,
                         bg=PANEL, fg=ACCENT2, relief="flat", padx=10, pady=5,
                         command=toggle_draw_obstacles)
draw_obs_btn.pack(side="left", expand=True, fill="x", padx=2)
clear_obs_btn = tk.Button(btn_row3, text="✕ CLEAR OBS", font=mono_bold,
                          bg=PANEL, fg=TEXT_DIM, relief="flat", padx=10, pady=5,
                          command=clear_obstacles)
clear_obs_btn.pack(side="left", expand=True, fill="x", padx=2)

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
```

---

## 8. Bindings del ratón en el canvas

Justo después de crear `map_canvas`:

```python
map_canvas.bind("<Button-1>", on_map_click)
map_canvas.bind("<ButtonRelease-1>", on_map_release)
```

---

## 9. Dibujar obstáculos y diferenciar drones remotos en `draw_map()`

Dentro de `draw_map()`, después de los ejes y antes del bucle de drones:

```python
    # Obstáculos
    for obs in fleet.get_obstacles():
        x1, y1 = to_cv(obs.p1[0], obs.p1[1])
        x2, y2 = to_cv(obs.p2[0], obs.p2[1])
        c.create_line(x1, y1, x2, y2, fill=ACCENT2, width=4, capstyle="round")
        c.create_oval(x1-3, y1-3, x1+3, y1+3, fill=ACCENT2, outline="")
        c.create_oval(x2-3, y2-3, x2+3, y2+3, fill=ACCENT2, outline="")
```

Y para diferenciar los drones remotos visualmente, reemplaza la línea
`c.create_polygon([tip, left_, right_], fill=st.color, outline=TEXT, width=1)`
por:

```python
            from drone_iface import RemoteTello
            outline_color = "#888888" if isinstance(st.drone, RemoteTello) else TEXT
            outline_width = 2 if isinstance(st.drone, RemoteTello) else 1
            c.create_polygon([tip, left_, right_], fill=st.color,
                             outline=outline_color, width=outline_width)
```

---

## 10. Cleanup al cerrar

En `on_window_close()`, antes de `window.destroy()`:

```python
    if mqtt_bridge is not None:
        mqtt_bridge.disconnect()
```

---

## Verificación final

- `python main.py` arranca sin errores en un PC con bridge desactivado
- Botones: ¿CÓMO FUNCIONA?, DETECT IMG, DRAW OBS, CLEAR OBS visibles
- Panel "MODO DISTRIBUIDO" con campo PEER ID y botón BRIDGE
- Click+drag en mapa con DRAW OBS crea líneas rojas (obstáculos)
- Con bridge activo: al añadir un obstáculo, otro PC con bridge activo y mismo
  SWARM_ID lo recibe y dibuja
- Drones remotos aparecen con borde gris en el mapa
- START solo lanza misión para drones locales, no para remotos
- Detección de un dron real local se propaga y hace converger a drones remotos
  (y viceversa: detección remota hace converger drones locales)
