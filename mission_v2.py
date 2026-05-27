"""
mission_v2.py — Bucle de visión YOLO integrado con Fleet.

Esta función SOLO se llama para drones REAL. La parte de movimiento
(rotation_iteration + convergencia) la sigue haciendo mission_logic.run_drone_mission.

Arquitectura:
    main.py start_mission_click():
        para CADA dron:
            mission_logic.run_drone_mission()   ← movimiento
        para CADA dron REAL:
            mission_v2.run_real_drone_vision()  ← visión + YOLO

Cuando YOLO detecta el objetivo:
    fleet.report_target_found(drone_id)
    → Fleet propaga el evento
    → mission_logic detecta el target en otros drones y los hace converger
"""
import os
import random
import datetime
import threading
import time

import cv2
from ultralytics import YOLO

from fleet import Fleet, DroneStatus


CONFIDENCE_THRESHOLD = 0.5
RECORDS_DIR = "records"
# Frames consecutivos viendo el target antes de reportar (evita falsos
# positivos en cuanto YOLO detecta algo un único frame). El dron se mueve,
# así que un valor alto sería difícil de alcanzar; 5 es buen compromiso.
CONFIRMATION_FRAMES = 5

# Modelo YOLO cargado una sola vez (compartido entre drones reales)
_model = None
_model_lock = threading.Lock()


def _get_model():
    """Carga YOLOv8n perezosamente, una sola vez para todos los drones."""
    global _model
    with _model_lock:
        if _model is None:
            _model = YOLO("yolov8n.pt")
    return _model


def run_real_drone_vision(fleet: Fleet, drone_id: str,
                          stop_event: threading.Event,
                          target_class_id: int):
    """
    Bucle de visión para un dron REAL.

    - Carga YOLO al inicio (compartido si ya estaba cargado)
    - Lee frames del Tello en bucle
    - Si detecta target_class_id con conf >= 0.5 → fleet.report_target_found
    - Si state.record_enabled → graba vídeo con bounding boxes
    - Se detiene cuando: stop_event, alguien encontró el target, o 'q' en la ventana
    """
    state = fleet.get(drone_id)
    if state is None or not state.is_real:
        fleet.log(drone_id, "run_real_drone_vision: no es real, abortando")
        return

    drone = state.drone
    fleet.log(drone_id, "Cargando YOLOv8...")
    model = _get_model()
    fleet.log(drone_id, f"YOLO listo. Buscando clase {target_class_id} "
                        f"({model.names.get(target_class_id, '?')})")

    # Grabación opcional
    video_writer = None
    if state.record_enabled:
        os.makedirs(RECORDS_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(RECORDS_DIR, f"mission_{drone_id}_{ts}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(filename, fourcc, 30.0, (960, 720))
        fleet.log(drone_id, f"Grabando en {filename}")

    frame_read = drone.get_frame_read()
    window_title = f"Drone {drone_id} - Vision"
    detected_locally = False
    target_streak = 0   # frames consecutivos viendo el target

    try:
        # Estados en los que NO buscamos con YOLO (el dron no está en modo
        # búsqueda: está volviendo a casa, convergiendo, parado, despegando…)
        NON_SEARCH = (DroneStatus.FOUND, DroneStatus.AT_TARGET,
                      DroneStatus.RETURNING, DroneStatus.CONVERGING,
                      DroneStatus.TAKING_OFF, DroneStatus.LANDING,
                      DroneStatus.LANDED, DroneStatus.ERROR)

        while not stop_event.is_set():
            # Solo buscamos en SEARCHING. En cualquier otro estado, hover.
            if detected_locally or state.status in NON_SEARCH:
                time.sleep(0.3)
                continue

            # Si otro detectó: paramos la visión (el movimiento converge)
            tgt = fleet.get_target()
            if tgt is not None and tgt[2] != drone_id:
                time.sleep(0.3)
                continue

            frame = frame_read.frame
            if frame is None:
                continue

            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            results = model(frame_bgr, verbose=False)

            found_target = False
            best_target_conf = 0.0
            for box in results[0].boxes:
                conf = float(box.conf[0])
                if conf < CONFIDENCE_THRESHOLD:
                    continue
                cid = int(box.cls[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                if cid == target_class_id:
                    # Target: caja roja gruesa + etiqueta destacada
                    found_target = True
                    best_target_conf = max(best_target_conf, conf)
                    cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 0, 255), 3)
                    label = f">> {model.names.get(cid, cid)} {conf:.2f}"
                    cv2.putText(frame_bgr, label, (x1, max(y1 - 10, 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    # Resto: caja gris fina sin etiqueta (menos ruido visual)
                    cv2.rectangle(frame_bgr, (x1, y1), (x2, y2),
                                  (120, 120, 120), 1)

            # Confirmación por frames consecutivos
            if found_target:
                target_streak += 1
            else:
                target_streak = 0

            # HUD: estado de confirmación
            hud = (f"TARGET {model.names.get(target_class_id, target_class_id)} "
                   f"[{min(target_streak, CONFIRMATION_FRAMES)}/{CONFIRMATION_FRAMES}]"
                   f"  conf={best_target_conf:.2f}")
            cv2.putText(frame_bgr, hud, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 230, 255) if found_target else (160, 160, 160), 2)

            if target_streak >= CONFIRMATION_FRAMES:
                fleet.log(drone_id,
                          f"OBJETIVO CONFIRMADO ({CONFIRMATION_FRAMES} frames) "
                          f"— reportando a la flota")
                fleet.report_target_found(drone_id)
                detected_locally = True

            if video_writer is not None:
                video_writer.write(frame_bgr)

            cv2.imshow(window_title, frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

    except Exception as e:
        fleet.log(drone_id, f"Error en visión: {e}")
    finally:
        if video_writer is not None:
            video_writer.release()
        try:
            cv2.destroyWindow(window_title)
        except Exception:
            pass
        fleet.log(drone_id, "Visión finalizada")