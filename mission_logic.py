"""
Lógica de misión por dron.

Cambios respecto a versión anterior:
  - converge_to() ahora hace REVERSE-PATH por waypoints (no línea recta).
    El seguidor reproduce el camino del finder al revés desde el waypoint
    más cercano hasta la pose final del finder.
  - Cada segmento del camino se comprueba contra los obstáculos del Fleet.
    Si un segmento cruza un obstáculo, se inserta un waypoint de rodeo.
  - El movimiento entre waypoints es siempre LÍNEA RECTA (girar + avanzar).
"""
from __future__ import annotations
import heapq
import itertools
import math
import threading
import time
from typing import List, Optional, Tuple

from fleet import Fleet, DroneStatus, segments_intersect


SEARCH_ROTATION = 30
SEARCH_FORWARD  = 75

# Margen para el pathfinding respecto a obstáculos (cm).
#   - OBSTACLE_MARGIN: distancia perpendicular a la que se colocan los
#     waypoints de rodeo desde los extremos del muro. Define el ancho del
#     "tubo de seguridad" alrededor de cada esquina.
#   - MIN_FLIGHT_CLEARANCE: distancia mínima que cualquier tramo del vuelo
#     debe mantener contra cualquier muro. Permite pasos por las esquinas
#     pero garantiza que nunca se vuele a menos de esta distancia de un muro.
# El primer valor lo controla el holgura visual; el segundo, la seguridad
# real durante el vuelo.
OBSTACLE_MARGIN      = 70
MIN_FLIGHT_CLEARANCE = 20   # Debe ser < OBSTACLE_MARGIN para que los nav
                            # points (a OBSTACLE_MARGIN del muro) sean
                            # alcanzables por segmentos del grafo.

# Seguridad 3D entre drones en vuelo:
#   - SAFETY_DISTANCE  = umbral de proximidad horizontal (cm). Si dos drones
#                        están dentro de este radio en XY, deben separarse
#                        verticalmente en al menos VERTICAL_SAFETY.
#   - VERTICAL_SAFETY  = separación vertical mínima exigida cuando hay
#                        proximidad horizontal (cm).
#   - MIN_FLIGHT_HEIGHT, MAX_FLIGHT_HEIGHT = límites para ajustes automáticos
#                        de altitud (cm).
SAFETY_DISTANCE    = 100
VERTICAL_SAFETY    = 25
MIN_FLIGHT_HEIGHT  = 30
MAX_FLIGHT_HEIGHT  = 300

# Atajo de RTH: si el dron pasa más cerca que esto del origen durante el
# reverse-path, abandona los waypoints restantes y va directo a (0,0).
HOME_PROXIMITY     = 100

# Estados que consideramos "en aire" para colisiones (los demás se ignoran)
_FLYING_STATES = frozenset({
    DroneStatus.TAKING_OFF,
    DroneStatus.SEARCHING,
    DroneStatus.CONVERGING,
    DroneStatus.AT_TARGET,
    DroneStatus.FOUND,
    DroneStatus.RETURNING,
    DroneStatus.LANDING,
})


# ─────────────────────────────────────────────
#  Helpers de geometría
# ─────────────────────────────────────────────
def _dist(p1, p2) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def _point_to_segment_dist(p, s1, s2) -> float:
    """Distancia mínima del punto p al segmento s1-s2."""
    dx, dy = s2[0] - s1[0], s2[1] - s1[1]
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(p[0] - s1[0], p[1] - s1[1])
    t = max(0.0, min(1.0,
                     ((p[0] - s1[0]) * dx + (p[1] - s1[1]) * dy) / L2))
    proj_x = s1[0] + t * dx
    proj_y = s1[1] + t * dy
    return math.hypot(p[0] - proj_x, p[1] - proj_y)


def _segment_to_segment_distance(p1, p2, p3, p4) -> float:
    """Distancia mínima entre dos segmentos. 0 si se cruzan."""
    if segments_intersect(p1, p2, p3, p4):
        return 0.0
    return min(
        _point_to_segment_dist(p1, p3, p4),
        _point_to_segment_dist(p2, p3, p4),
        _point_to_segment_dist(p3, p1, p2),
        _point_to_segment_dist(p4, p1, p2),
    )


def _segment_safe_from_obstacles(fleet: Fleet, p1, p2,
                                 buffer: float) -> bool:
    """True si el segmento p1-p2 mantiene `buffer` cm contra TODOS los muros."""
    for o in fleet.get_obstacles():
        if _segment_to_segment_distance(p1, p2, o.p1, o.p2) < buffer:
            return False
    return True


def _min_distance_to_others(fleet: Fleet, drone_id: str,
                            point: Tuple[float, float]
                            ) -> Tuple[float, str]:
    """
    Devuelve (distancia_horizontal_mínima, drone_id_más_cercano) entre `point`
    y los otros drones EN AIRE (sin tener en cuenta altitud).
    """
    min_d = float('inf')
    min_id = ''
    for st in fleet.all():
        if st.drone_id == drone_id:
            continue
        if st.status not in _FLYING_STATES:
            continue
        ox, oy, _, _ = st.tracker.get_state()
        d = math.hypot(ox - point[0], oy - point[1])
        if d < min_d:
            min_d = d
            min_id = st.drone_id
    return min_d, min_id


def _conflicting_drones_at(fleet: Fleet, drone_id: str,
                           xy: Tuple[float, float], z: float
                           ) -> List[Tuple[str, float]]:
    """
    Lista de drones que conflictúan con `(xy, z)`:
        - están dentro de SAFETY_DISTANCE horizontalmente, Y
        - su altitud está dentro de VERTICAL_SAFETY de `z`.
    Devuelve [(drone_id, su_z), ...].
    """
    conflicts = []
    for st in fleet.all():
        if st.drone_id == drone_id:
            continue
        if st.status not in _FLYING_STATES:
            continue
        ox, oy, _, _ = st.tracker.get_state()
        d2d = math.hypot(ox - xy[0], oy - xy[1])
        if d2d >= SAFETY_DISTANCE:
            continue
        if abs(z - st.tracker.z) >= VERTICAL_SAFETY:
            continue
        conflicts.append((st.drone_id, st.tracker.z))
    return conflicts


def _name_key(drone_id: str) -> str:
    """Clave de orden por nombre, ignorando el prefijo de peer MQTT
    ('peer2/alpha' → 'alpha')."""
    return drone_id.split("/")[-1]


def _deconflict_altitude(fleet: Fleet, drone_id: str,
                         conflicts: List[Tuple[str, float]]) -> float:
    """
    Altitud DETERMINISTA por orden de nombre (criterio: letras).

    Regla: el nombre más temprano alfabéticamente vuela MÁS ALTO.
      - Drones con nombre anterior al mío (p.ej. alpha si yo soy bravo) deben
        quedar POR ENCIMA → yo me pongo por debajo de ellos.
      - Drones con nombre posterior (p.ej. charlie si yo soy bravo) quedan
        POR DEBAJO → yo me pongo por encima de ellos.

    Como ambos drones aplican la misma regla, dos que confluyen al mismo punto
    se reparten siempre igual: el de letra menor sube, el de letra mayor baja.

    Devuelve la altitud objetivo, clamp a [MIN_FLIGHT_HEIGHT, MAX_FLIGHT_HEIGHT].
    """
    my = _name_key(drone_id)
    earlier_zs = [cz for cid, cz in conflicts if _name_key(cid) < my]  # van encima
    later_zs   = [cz for cid, cz in conflicts if _name_key(cid) > my]  # van debajo

    # Debo estar por DEBAJO de los anteriores y por ENCIMA de los posteriores
    hi = (min(earlier_zs) - VERTICAL_SAFETY) if earlier_zs else None  # techo
    lo = (max(later_zs) + VERTICAL_SAFETY) if later_zs else None      # suelo

    if lo is not None and hi is not None:
        # Hay drones por encima y por debajo de mí: me pongo en medio si cabe.
        # Si no cabe (constraints incompatibles), respeto a los de nombre
        # ANTERIOR (mayor prioridad) y me bajo bajo ellos; los posteriores se
        # apartarán hacia abajo en su propia iteración → la pila se ordena
        # sola en un par de pasos.
        target = (lo + hi) / 2.0 if lo <= hi else hi
    elif lo is not None:
        target = lo            # solo nombres posteriores → subo por encima
    elif hi is not None:
        target = hi            # solo nombres anteriores → bajo por debajo
    else:
        target = fleet.get(drone_id).tracker.z

    return max(MIN_FLIGHT_HEIGHT, min(MAX_FLIGHT_HEIGHT, target))


def _safe_move_forward(fleet: Fleet, drone_id: str, cm: int) -> int:
    """
    Avanza con seguridad 3D:
      1. Predice el punto final del avance.
      2. Si entra en la zona horizontal de otro dron, busca una altitud
         que respete VERTICAL_SAFETY. Si la encuentra, ajusta altitud PRIMERO
         (sube/baja) y luego avanza.
      3. Si no hay altitud viable, reduce el chunk en pasos de 20 cm.
      4. Si ni 20 cm es seguro, devuelve 0 (no se puede avanzar).
    """
    state = fleet.get(drone_id)
    if state is None:
        return 0
    x, y, heading, _ = state.tracker.get_state()
    z = state.tracker.z
    rad = math.radians(heading)

    safe_cm = int(cm)
    target_z = z  # altitud a la que iremos antes de avanzar (puede no cambiar)

    while safe_cm >= 20:
        predicted = (x + safe_cm * math.sin(rad),
                     y + safe_cm * math.cos(rad))
        conflicts = _conflicting_drones_at(fleet, drone_id, predicted, z)
        if not conflicts:
            target_z = z   # sin conflicto, no movemos altitud
            break
        # Hay conflicto. Altitud determinista por nombre (letras).
        new_z = _deconflict_altitude(fleet, drone_id, conflicts)
        # Verificar que esa altitud realmente despeja (por si algún dron no
        # está en su capa todavía)
        if not _conflicting_drones_at(fleet, drone_id, predicted, new_z):
            target_z = new_z
            break
        # No despeja a esa altitud → reducir avance
        safe_cm -= 20

    if safe_cm < 20:
        d_now, who = _min_distance_to_others(fleet, drone_id, (x, y))
        fleet.log(drone_id,
                  f"⚠ Stop seguridad 3D: cerca de {who} a {d_now:.0f}cm "
                  f"sin altitud libre")
        return 0

    # Ajustar altitud primero si hace falta
    if abs(target_z - z) >= 20:
        dz = int(round(target_z - z))
        try:
            if dz > 0:
                state.drone.move_up(dz)
                state.tracker.move_up(dz)
                fleet.log(drone_id, f"↑ +{dz}cm para evitar colisión")
            else:
                state.drone.move_down(abs(dz))
                state.tracker.move_down(abs(dz))
                fleet.log(drone_id, f"↓ {dz}cm para evitar colisión")
            fleet.emit_pose(drone_id)
        except Exception as e:
            fleet.log(drone_id, f"⚠ Error ajustando altitud: {e}")
            return 0

    try:
        state.drone.move_forward(safe_cm)
    except Exception as e:
        fleet.log(drone_id, f"⚠ Error move_forward (SDK): {e}")
        return 0
    state.tracker.move_forward(safe_cm)
    fleet.emit_pose(drone_id)
    return safe_cm


def _closest_waypoint_index(point, path: List[Tuple[float, float]]) -> int:
    """Devuelve el índice del waypoint del path más cercano a point."""
    if not path:
        return 0
    best_i = 0
    best_d = _dist(point, path[0])
    for i, wp in enumerate(path):
        d = _dist(point, wp)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def _avoid_obstacle(fleet: Fleet, start, end
                    ) -> Optional[List[Tuple[float, float]]]:
    """
    Pathfinding desde `start` a `end` evitando los obstáculos del Fleet.

    Estrategia escalonada:
      1. Línea recta libre              → [end]                   (O(N))
      2. 1-hop perpendicular libre      → [detour, end]           (O(N))
      3. A* sobre grafo de visibilidad  → [wp1, wp2, ..., end]    (O(M² × N))
                                          con offsets perpendiculares en cada
                                          extremo de cada obstáculo
      4. Sin camino posible             → None

    El caller decide qué hacer con None (parar, reintentar, etc.).
    """
    # 1. Línea recta directa — debe mantener MIN_FLIGHT_CLEARANCE de TODOS los muros
    if _segment_safe_from_obstacles(fleet, start, end, MIN_FLIGHT_CLEARANCE):
        return [end]

    # 2. 1-hop perpendicular al primer obstáculo que cruza (rápido)
    obs = fleet.segment_crosses_obstacle(start, end)
    if obs is not None:
        one_hop = _try_one_hop(fleet, start, end, obs)
        if one_hop is not None:
            return one_hop

    # 3. A* en 3 niveles, de más seguro a más arriesgado. Usamos el primero
    #    que encuentre ruta → si hay una opción holgada, A* la elige antes
    #    que la pegada al muro.
    #
    #    Nivel A: clearance NORMAL en todo el grafo (incluyendo arias del start).
    #             Es el path más seguro. Si lo hay, ganamos.
    result = _astar_visibility(fleet, start, end,
                               clearance=MIN_FLIGHT_CLEARANCE,
                               escape_mode=False)
    if result is not None:
        return result

    #    Nivel B: clearance reducida (5 cm) en TODO el grafo. Sigue evitando
    #             cruzar muros, pero el camino puede pasar más pegado. Solo
    #             se usa si nivel A no encuentra nada.
    result = _astar_visibility(fleet, start, end,
                               clearance=5.0,
                               escape_mode=False)
    if result is not None:
        return result

    #    Nivel C (último recurso): escape_mode = arias del START no exigen
    #             clearance (solo no cruzar muros). Permite al dron escapar
    #             cuando está acorralado/pegado a una pared. El resto del
    #             camino mantiene clearance reducida para que sea seguro.
    return _astar_visibility(fleet, start, end,
                             clearance=5.0,
                             escape_mode=True)


def _try_one_hop(fleet: Fleet, start, end, obs
                 ) -> Optional[List[Tuple[float, float]]]:
    """Intenta resolver con 1 solo waypoint perpendicular al obstáculo `obs`."""
    a, b = obs.p1, obs.p2
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy) or 1.0
    px, py = -dy / L, dx / L

    candidates = [
        (a[0] + px * OBSTACLE_MARGIN, a[1] + py * OBSTACLE_MARGIN),
        (a[0] - px * OBSTACLE_MARGIN, a[1] - py * OBSTACLE_MARGIN),
        (b[0] + px * OBSTACLE_MARGIN, b[1] + py * OBSTACLE_MARGIN),
        (b[0] - px * OBSTACLE_MARGIN, b[1] - py * OBSTACLE_MARGIN),
    ]

    best, best_dist = None, float('inf')
    for c in candidates:
        # Cada tramo debe mantener OBSTACLE_MARGIN de TODOS los muros
        if not _segment_safe_from_obstacles(fleet, start, c, MIN_FLIGHT_CLEARANCE):
            continue
        if not _segment_safe_from_obstacles(fleet, c, end, MIN_FLIGHT_CLEARANCE):
            continue
        d = _dist(start, c) + _dist(c, end)
        if d < best_dist:
            best_dist, best = d, c
    return [best, end] if best is not None else None


def _astar_visibility(fleet: Fleet, start, end,
                      clearance: float = None,
                      escape_mode: bool = False
                      ) -> Optional[List[Tuple[float, float]]]:
    """
    A* sobre un grafo de visibilidad.

    `clearance`   = distancia mínima del segmento a TODOS los muros.
    `escape_mode` = si True, las aristas que tocan el START (node 0) solo
                    deben NO CRUZAR muros (sin buffer). Usar solo como último
                    recurso cuando el dron está pegado a una pared y los modos
                    normales no encuentran camino.
    """
    if clearance is None:
        clearance = MIN_FLIGHT_CLEARANCE
    obstacles = fleet.get_obstacles()
    if not obstacles:
        return [end]

    # Por cada muro generamos 8 nav points: 4 alrededor de cada extremo, en
    # un semicírculo "hacia fuera" (lejos del otro extremo) a 0°, 45°, 90°,
    # 135° de la perpendicular. Esto permite rodear los extremos del muro
    # con un margen mínimo, no solo pasar perpendicularmente al lado.
    nodes: List[Tuple[float, float]] = [start, end]

    # ESCAPE POINTS alrededor de start: 8 puntos a 50 cm en 8 direcciones.
    # Si el dron está acorralado por muros (su posición no puede alcanzar
    # ningún nav point del grafo principal), estos puntos cercanos le dan
    # opciones de salida. Sin esto, drones cerca de paredes hacen RTH a
    # AT_TARGET instantáneamente porque no pueden conectar con el grafo.
    ESCAPE_RADIUS = 50.0
    for ang in range(0, 360, 45):
        r = math.radians(ang)
        nodes.append((start[0] + ESCAPE_RADIUS * math.sin(r),
                      start[1] + ESCAPE_RADIUS * math.cos(r)))
    SQRT2_INV = 1.0 / math.sqrt(2.0)
    for o in obstacles:
        a, b = o.p1, o.p2
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy) or 1.0
        perp_x, perp_y = -dy / L, dx / L
        par_x, par_y = dx / L, dy / L

        # En A: "out" = dirección -par (lejos de B). 4 puntos:
        for ux, uy in [
            (perp_x, perp_y),                                            # 90° (perp+)
            ((-par_x + perp_x) * SQRT2_INV, (-par_y + perp_y) * SQRT2_INV),  # 45°
            (-par_x, -par_y),                                            # 0°  (par out)
            ((-par_x - perp_x) * SQRT2_INV, (-par_y - perp_y) * SQRT2_INV),  # -45°
            (-perp_x, -perp_y),                                          # -90° (perp-)
        ]:
            nodes.append((a[0] + ux * OBSTACLE_MARGIN,
                          a[1] + uy * OBSTACLE_MARGIN))

        # En B: "out" = +par (lejos de A). 4 puntos simétricos:
        for ux, uy in [
            (perp_x, perp_y),                                            # 90°
            ((par_x + perp_x) * SQRT2_INV, (par_y + perp_y) * SQRT2_INV),    # 45°
            (par_x, par_y),                                              # 0°  (par out)
            ((par_x - perp_x) * SQRT2_INV, (par_y - perp_y) * SQRT2_INV),    # -45°
            (-perp_x, -perp_y),                                          # -90°
        ]:
            nodes.append((b[0] + ux * OBSTACLE_MARGIN,
                          b[1] + uy * OBSTACLE_MARGIN))

    n = len(nodes)
    # Aristas: par (i, j) si la línea mantiene `clearance` de TODOS los muros.
    # Si escape_mode=True, las aristas que tocan al START (node 0) usan SOLO
    # el check de "no cruza" (sin buffer) → permite salir aunque el dron esté
    # pegado a un muro. Esto solo se activa en el último nivel de fallback
    # del caller, NUNCA por defecto.
    adj: List[List[Tuple[int, float]]] = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            touches_start = (i == 0 or j == 0)
            if escape_mode and touches_start:
                safe = fleet.segment_crosses_obstacle(nodes[i], nodes[j]) is None
            else:
                safe = _segment_safe_from_obstacles(fleet, nodes[i], nodes[j],
                                                    clearance)
            if safe:
                d = math.hypot(nodes[j][0] - nodes[i][0],
                               nodes[j][1] - nodes[i][1])
                adj[i].append((j, d))
                adj[j].append((i, d))

    goal = nodes[1]

    def heur(idx: int) -> float:
        return math.hypot(nodes[idx][0] - goal[0], nodes[idx][1] - goal[1])

    counter = itertools.count()   # tiebreaker para heapq cuando f es igual
    # Cada entrada: (f, tiebreak, g, idx, path_tuple)
    open_heap = [(heur(0), next(counter), 0.0, 0, (0,))]
    best_g: dict[int, float] = {0: 0.0}

    while open_heap:
        f, _, g, cur, path = heapq.heappop(open_heap)
        if cur == 1:
            return [nodes[i] for i in path[1:]]
        if g > best_g.get(cur, float('inf')):
            continue
        for nxt, edge_cost in adj[cur]:
            new_g = g + edge_cost
            if new_g >= best_g.get(nxt, float('inf')):
                continue
            best_g[nxt] = new_g
            new_f = new_g + heur(nxt)
            heapq.heappush(open_heap,
                           (new_f, next(counter), new_g, nxt, path + (nxt,)))

    return None  # No hay camino


# ─────────────────────────────────────────────
#  Navegación a un punto (gira + avanza en línea recta)
# ─────────────────────────────────────────────
def _go_to_point(fleet: Fleet, drone_id: str, target: Tuple[float, float],
                 stop_event: threading.Event) -> bool:
    """
    Vuela en línea recta hasta `target` (cm en coords mundo).
    Gira primero y luego avanza en chunks de 500 cm.
    Devuelve True si llegó, False si fue interrumpido.
    """
    state = fleet.get(drone_id)
    if state is None:
        return False
    drone = state.drone
    tracker = state.tracker

    x, y, heading, _ = tracker.get_state()
    dx = target[0] - x
    dy = target[1] - y
    distance = math.hypot(dx, dy)

    if distance < 20:   # ya estamos lo bastante cerca
        return True

    # Bearing desde norte
    bearing = math.degrees(math.atan2(dx, dy)) % 360
    rotation = (bearing - heading) % 360

    try:
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
    except Exception as e:
        fleet.log(drone_id, f"⚠ Error giro (SDK): {e}")
        return False

    remaining = int(round(distance))
    while remaining >= 20 and not stop_event.is_set():
        # Bloqueo por pausa global (no avanza, no consume chunk)
        while fleet.mission_paused.is_set() and not stop_event.is_set():
            time.sleep(0.3)
        if stop_event.is_set():
            break
        chunk = min(remaining, 500)
        moved = _safe_move_forward(fleet, drone_id, chunk)
        if moved == 0:
            return False
        remaining -= moved

    return not stop_event.is_set()


# ─────────────────────────────────────────────
#  Convergencia con reverse-path + obstáculos
# ─────────────────────────────────────────────
def converge_to(fleet: Fleet, drone_id: str, finder_id: str,
                stop_event: threading.Event):
    """
    El dron `drone_id` converge hacia la pose actual del `finder_id`
    reproduciendo SU PATH al revés desde el waypoint más cercano.

    Pasos:
      1. Obtener path del finder
      2. Buscar el waypoint del finder más cercano a la posición actual del seguidor
      3. Ir a ese waypoint
      4. Recorrer el path del finder desde ahí hasta el último waypoint (donde detectó)
      5. Para cada segmento, si choca con un obstáculo, insertar un waypoint de rodeo
    """
    state = fleet.get(drone_id)
    finder_state = fleet.get(finder_id)
    if state is None or finder_state is None:
        return

    fleet.set_status(drone_id, DroneStatus.CONVERGING)
    fleet.log(drone_id, f"Convergiendo hacia {finder_id} por reverse-path")

    # Posición actual del seguidor
    sx, sy, _, _ = state.tracker.get_state()
    follower_pos = (sx, sy)

    # Path del finder (lista de waypoints incluyendo posición actual)
    _, _, _, finder_path = finder_state.tracker.get_state()
    if len(finder_path) < 2:
        # El finder casi no se ha movido: ir directo a su pose
        fx, fy, _, _ = finder_state.tracker.get_state()
        _navigate_segment(fleet, drone_id, follower_pos, (fx, fy), stop_event)
        fleet.set_status(drone_id, DroneStatus.AT_TARGET)
        return

    # 1. Encontrar el waypoint del finder más cercano al seguidor
    entry_idx = _closest_waypoint_index(follower_pos, finder_path)

    # 2. Construir la ruta: [waypoint_entry, ..., posición_actual_del_finder]
    #    Aseguramos que finder_pos esté como último waypoint (objetivo final).
    finder_pos = (finder_path[-1] if finder_path else (0.0, 0.0))
    route = list(finder_path[entry_idx:])
    if not route or route[-1] != finder_pos:
        route.append(finder_pos)
    fleet.log(drone_id,
              f"Entrando al path del finder por waypoint "
              f"{entry_idx}/{len(finder_path)-1}")

    # 3. Recorrer la ruta. Si un waypoint está bloqueado, SALTAR a los
    #    siguientes hasta encontrar uno alcanzable. Solo si NINGUNO es
    #    alcanzable nos rendimos.
    current = follower_pos
    i = 0
    while i < len(route):
        if stop_event.is_set():
            return

        # Buscar el primer waypoint alcanzable desde current
        next_idx = None
        intermediate = None
        for j in range(i, len(route)):
            cand = _avoid_obstacle(fleet, current, route[j])
            if cand is not None:
                next_idx = j
                intermediate = cand
                break

        if next_idx is None:
            # Ningún waypoint alcanzable desde aquí. NO marcamos AT_TARGET:
            # devolvemos para que el bucle principal vuelva a intentarlo más
            # tarde (por si el usuario quita un obstáculo, o por si la
            # posición del finder cambia y abre un camino).
            fleet.log(drone_id,
                      "⚠ Convergencia bloqueada por obstáculos. Reintentaré.")
            return

        if next_idx > i:
            fleet.log(drone_id,
                      f"↪ Saltando waypoints {i}..{next_idx-1} (bloqueados), "
                      f"voy al {next_idx}")

        # Navegar al waypoint (con su detour si lo hubo)
        for wp in intermediate:
            if stop_event.is_set():
                return
            ok = _navigate_segment(fleet, drone_id, current, wp, stop_event)
            if not ok:
                if stop_event.is_set():
                    return
                fleet.set_status(drone_id, DroneStatus.AT_TARGET)
                fleet.log(drone_id, "Parado a distancia segura del target.")
                return
            current = wp

        i = next_idx + 1

    fleet.set_status(drone_id, DroneStatus.AT_TARGET)
    fleet.log(drone_id, "Llegó al target. Hover esperando land manual.")


def _navigate_segment(fleet: Fleet, drone_id: str,
                      _from: Tuple[float, float],
                      _to: Tuple[float, float],
                      stop_event: threading.Event) -> bool:
    """Vuela en línea recta de `_from` a `_to`. Asume que el camino está libre."""
    return _go_to_point(fleet, drone_id, _to, stop_event)


# ─────────────────────────────────────────────
#  Return to Home (ruta DIRECTA al origen con A*)
# ─────────────────────────────────────────────
def return_to_origin(fleet: Fleet, drone_id: str, stop_event: threading.Event):
    """
    El dron vuelve a su posición inicial (0, 0) por la RUTA MÁS DIRECTA,
    esquivando obstáculos con A* (no reproduce el camino al revés).

    Motivo: en un dron real el reverse-path acumula demasiado error de
    dead-reckoning (cada giro/avance del Tello tiene imprecisión), así que
    deshacer toda la secuencia se vuelve errático. Ir directo usa muchos
    menos comandos → menos drift → más fiable.

    Estado final: AT_TARGET (hover en origen, esperando land manual).
    """
    state = fleet.get(drone_id)
    if state is None:
        return

    # Mantener el status RETURNING (sin re-emitir si ya está)
    if state.status != DroneStatus.RETURNING:
        fleet.set_status(drone_id, DroneStatus.RETURNING)

    sx, sy, _, _ = state.tracker.get_state()
    fleet.log(drone_id,
              f"RTH: desde ({sx:.0f},{sy:.0f},Z{state.tracker.z:.0f}) hacia (0,0)")

    # Ya estamos en el origen
    if math.hypot(sx, sy) < 25:
        fleet.log(drone_id, "Ya está en el origen.")
        fleet.set_status(drone_id, DroneStatus.AT_TARGET)
        return

    # Ruta directa a (0,0) esquivando obstáculos (A* si hace falta)
    route = _avoid_obstacle(fleet, (sx, sy), (0.0, 0.0))
    if route is None:
        # A* falló: realmente bloqueado por obstáculos sin salida → AT_TARGET
        fleet.log(drone_id, "⚠ No hay ruta libre a home (obstáculos). Parando.")
        fleet.set_status(drone_id, DroneStatus.AT_TARGET)
        return

    fleet.log(drone_id, f"RTH ruta: {len(route)} waypoint(s)")

    for wp in route:
        if stop_event.is_set():
            return
        if state.status in (DroneStatus.LANDING, DroneStatus.LANDED,
                            DroneStatus.ERROR):
            return
        ok = _go_to_point(fleet, drone_id, wp, stop_event)
        if not ok:
            if stop_event.is_set():
                return
            # _go_to_point devolvió False por seguridad (otro dron pasó cerca).
            # NO marcamos AT_TARGET: dejamos status=RETURNING para que el bucle
            # principal vuelva a llamar a return_to_origin tras el RTH_RETRY_INTERVAL.
            # Así un blocker transitorio (otro dron en sweep) no nos hace abandonar.
            fleet.log(drone_id,
                      f"RTH bloqueado temporalmente en wp ({wp[0]:.0f},{wp[1]:.0f}). "
                      f"Reintentaré.")
            return

    fleet.log(drone_id, "Llegó al origen. Hover esperando land manual.")
    fleet.set_status(drone_id, DroneStatus.AT_TARGET)


# ─────────────────────────────────────────────
#  Navegación manual (set pose desde la UI)
# ─────────────────────────────────────────────
def execute_manual_nav(fleet: Fleet, drone_id: str,
                       stop_event: threading.Event):
    """
    Lleva al dron a `state.manual_target = (x, y, z)`.
    Orden: ajustar altitud primero (move_up/move_down), luego XY con
    `_go_to_point` (respeta obstáculos y seguridad 3D entre drones).
    Estado final: AT_TARGET. Limpia `manual_target`.
    """
    state = fleet.get(drone_id)
    if state is None or state.manual_target is None:
        return

    tx, ty, tz = state.manual_target
    state.manual_target = None    # consumido

    fleet.set_status(drone_id, DroneStatus.CONVERGING)
    fleet.log(drone_id, f"Manual nav → ({tx:.0f}, {ty:.0f}, Z {tz:.0f})")

    # 1) Altitud
    cur_z = state.tracker.z
    dz = int(round(tz - cur_z))
    if abs(dz) >= 20 and not stop_event.is_set():
        try:
            if dz > 0:
                state.drone.move_up(dz)
                state.tracker.move_up(dz)
            else:
                state.drone.move_down(abs(dz))
                state.tracker.move_down(abs(dz))
            fleet.emit_pose(drone_id)
        except Exception as e:
            fleet.log(drone_id, f"⚠ Error ajustando altitud: {e}")

    # 2) Posición XY (respeta obstáculos y safety 3D)
    if not stop_event.is_set():
        x, y, _, _ = state.tracker.get_state()
        if math.hypot(tx - x, ty - y) >= 20:
            intermediate = _avoid_obstacle(fleet, (x, y), (tx, ty))
            if intermediate is None:
                # No se pudo navegar al destino. Volvemos a SEARCHING para
                # que el bucle de misión retome el control desde donde estamos.
                fleet.log(drone_id,
                          "⚠ Obstáculo bloquea la nav manual. Reanudo búsqueda.")
                fleet.set_status(drone_id, DroneStatus.SEARCHING)
                return
            current = (x, y)
            for wp in intermediate:
                if stop_event.is_set():
                    return
                ok = _go_to_point(fleet, drone_id, wp, stop_event)
                if not ok:
                    if stop_event.is_set():
                        return
                    fleet.log(drone_id,
                              "Nav manual cortada por seguridad. Reanudo búsqueda.")
                    break
                current = wp

    if not stop_event.is_set():
        # Tras nav manual, el dron RETOMA la misión: vuelve a SEARCHING para
        # que rotation_iteration/convergencia/etc. sigan funcionando. Antes
        # quedaba en AT_TARGET (hover terminal) y no hacía nada más.
        fleet.set_status(drone_id, DroneStatus.SEARCHING)
        fleet.log(drone_id, "Nav manual completada. Continúo con la misión.")


# ─────────────────────────────────────────────
#  Bucle principal de misión por dron
# ─────────────────────────────────────────────
def run_drone_mission(fleet: Fleet, drone_id: str, stop_event: threading.Event):
    """
    Bucle de movimiento de un dron (FAKE o REAL).
    - Si el dron es REAL, mission_v2.run_real_drone_vision se ejecuta en paralelo
      en otro thread y se encarga de YOLO.
    - Si es FAKE, este es el único thread (busca pero nunca detecta —
      la detección se simula con SIM DETECT desde la UI).
    """
    state = fleet.get(drone_id)
    if state is None:
        return
    drone = state.drone
    tracker = state.tracker

    if state.status not in (DroneStatus.SEARCHING, DroneStatus.TAKING_OFF):
        fleet.set_status(drone_id, DroneStatus.SEARCHING)

    # Timestamp del último intento de convergencia. Si falló (bloqueada por
    # obstáculos), volvemos a intentar tras CONVERGE_RETRY_INTERVAL segundos.
    last_converge_attempt = 0.0
    CONVERGE_RETRY_INTERVAL = 3.0

    # RTH: mismo patrón. Si return_to_origin sale dejando status=RETURNING
    # (porque un dron cruzó su camino), reintentamos cada RTH_RETRY_INTERVAL.
    # Tras MAX_RTH_RETRIES intentos sin éxito, nos rendimos en AT_TARGET.
    last_rth_attempt = 0.0
    rth_retries = 0
    RTH_RETRY_INTERVAL = 3.0
    MAX_RTH_RETRIES = 8

    def someone_else_found() -> bool:
        t = fleet.get_target()
        return t is not None and t[2] != drone_id

    while not stop_event.is_set():
        # 00a) Esperar a que termine el despegue. takeoff_click controla el
        #      dron mientras está en TAKING_OFF; si moviéramos aquí a la vez
        #      los comandos se pisarían en el socket UDP del Tello.
        if state.status == DroneStatus.TAKING_OFF:
            time.sleep(0.3)
            continue

        # 00b) Pausa global: hover hasta que se reanude
        if fleet.mission_paused.is_set():
            time.sleep(0.3)
            continue

        # 0a) Manual nav pedido por la UI: prioridad máxima
        if state.manual_target is not None:
            try:
                execute_manual_nav(fleet, drone_id, stop_event)
            except Exception as e:
                fleet.log(drone_id, f"⚠ Error nav manual: {e}")
                fleet.set_status(drone_id, DroneStatus.AT_TARGET)
            continue

        # 0b) RTH pedido por la UI. return_to_origin puede dejar status en
        #     RETURNING si fue bloqueado por seguridad transitoria → reintentamos
        #     cada RTH_RETRY_INTERVAL segundos hasta MAX_RTH_RETRIES.
        if state.status == DroneStatus.RETURNING:
            now = time.time()
            if now - last_rth_attempt >= RTH_RETRY_INTERVAL:
                last_rth_attempt = now
                try:
                    return_to_origin(fleet, drone_id, stop_event)
                except Exception as e:
                    fleet.log(drone_id, f"⚠ Error en Return to Home: {e}")
                    fleet.set_status(drone_id, DroneStatus.AT_TARGET)
                # Si tras el intento sigue en RETURNING, fue bloqueado: cuenta
                if state.status == DroneStatus.RETURNING:
                    rth_retries += 1
                    if rth_retries >= MAX_RTH_RETRIES:
                        fleet.log(drone_id,
                                  f"⚠ RTH abandonado tras {MAX_RTH_RETRIES} "
                                  f"intentos bloqueados. Hover aquí.")
                        fleet.set_status(drone_id, DroneStatus.AT_TARGET)
                        rth_retries = 0
                else:
                    rth_retries = 0   # éxito o A* sin ruta → reset contador
            else:
                time.sleep(0.4)
            continue

        # 1) Estados terminales: hover hasta land manual (incluye ERROR, p.ej.
        #    si el despegue falló — no queremos seguir mandando comandos)
        if state.status in (DroneStatus.FOUND, DroneStatus.AT_TARGET,
                            DroneStatus.LANDING, DroneStatus.LANDED,
                            DroneStatus.ERROR):
            time.sleep(0.4)
            continue

        # 2) Otro encontró → convergencia. Si la última falló, esperamos al
        #    retry interval antes de probar otra vez. Si tuvo éxito (estado
        #    pasa a AT_TARGET o FOUND) la rama 1 captura en hover y ya está.
        if someone_else_found():
            now = time.time()
            if now - last_converge_attempt >= CONVERGE_RETRY_INTERVAL:
                last_converge_attempt = now
                t = fleet.get_target()
                if t is not None:
                    _, _, finder_id = t
                    try:
                        converge_to(fleet, drone_id, finder_id, stop_event)
                    except Exception as e:
                        fleet.log(drone_id, f"⚠ Error en convergencia: {e}")
            else:
                time.sleep(0.4)
            continue

        # 3) Rotation iteration normal — protegida contra fallos del SDK.
        #    Si un comando del Tello real lanza excepción (timeout, 'error',
        #    etc.) lo registramos y seguimos en vez de morir el hilo (lo que
        #    congelaría el mapa y la evasión de obstáculos).
        try:
            interrupted = False
            # Barrido: mira a un lado, barre al otro, vuelve al centro.
            # Giro neto 0° → el dron se mantiene mirando al frente (no
            # acumula rotación ni espiralea). La cámara cubre un arco de
            # ±SEARCH_ROTATION grados.
            for kind, deg in [
                ("cw",  SEARCH_ROTATION),       # mira a la derecha
                ("ccw", 2 * SEARCH_ROTATION),   # barre a la izquierda
                ("cw",  SEARCH_ROTATION),       # vuelve al centro
            ]:
                if stop_event.is_set():
                    interrupted = True
                    break
                if state.status in (DroneStatus.FOUND, DroneStatus.AT_TARGET,
                                    DroneStatus.RETURNING):
                    interrupted = True
                    break
                if state.manual_target is not None:
                    interrupted = True
                    break
                if someone_else_found():
                    interrupted = True
                    break
                if kind == "cw":
                    drone.rotate_clockwise(deg)
                    tracker.rotate_cw(deg)
                else:
                    drone.rotate_counter_clockwise(deg)
                    tracker.rotate_ccw(deg)
                fleet.emit_pose(drone_id)

            if not interrupted and state.status == DroneStatus.SEARCHING:
                # Avanzar pero comprobar primero obstáculos y zona de seguridad
                x, y, heading, _ = tracker.get_state()
                rad = math.radians(heading)
                forward_target = (x + SEARCH_FORWARD * math.sin(rad),
                                  y + SEARCH_FORWARD * math.cos(rad))
                obs = fleet.segment_crosses_obstacle((x, y), forward_target)
                if obs is not None:
                    fleet.log(drone_id,
                              "⚠ Obstáculo delante, girando para esquivar")
                    drone.rotate_clockwise(90)
                    tracker.rotate_cw(90)
                    fleet.emit_pose(drone_id)
                else:
                    moved = _safe_move_forward(fleet, drone_id, SEARCH_FORWARD)
                    if moved == 0:
                        drone.rotate_clockwise(90)
                        tracker.rotate_cw(90)
                        fleet.emit_pose(drone_id)
        except Exception as e:
            fleet.log(drone_id, f"⚠ Error de movimiento (SDK Tello): {e}")
            time.sleep(1.0)   # evita un bucle rápido de errores repetidos