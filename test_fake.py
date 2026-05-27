"""
Test rápido del FakeTello. No requiere hardware ni instalar nada nuevo
(numpy y opencv ya están en el proyecto).

Ejecuta:
    python test_fake.py
"""
import time
import cv2
from drone_iface import FakeTello


def main():
    drone = FakeTello(name="alpha")
    drone.connect()
    print(f"Batería inicial: {drone.get_battery()}%")

    drone.streamon()
    fr = drone.get_frame_read()

    drone.takeoff()

    # Secuencia de prueba: rotar y avanzar mientras vemos el frame
    t0 = time.time()
    for action in [
        ("rotate_cw",  45),
        ("rotate_cw",  45),
        ("rotate_ccw", 90),
        ("forward",    75),
    ]:
        kind, val = action
        if kind == "rotate_cw":
            drone.rotate_clockwise(val)
        elif kind == "rotate_ccw":
            drone.rotate_counter_clockwise(val)
        elif kind == "forward":
            drone.move_forward(val)

        # Mostrar el frame actual durante un instante
        frame = fr.frame
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("FakeTello frame", bgr)
        cv2.waitKey(200)

    drone.land()
    print(f"Tiempo total: {time.time()-t0:.1f}s  (debería rondar 6-7s)")
    print(f"Batería final: {drone.get_battery()}%")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
