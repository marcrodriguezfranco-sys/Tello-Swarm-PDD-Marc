"""
Configura un Tello EDU / RoboMaster TT para que se una a tu router WiFi
(modo estación). Solo hay que ejecutarlo UNA VEZ por dron — el Tello recuerda
las credenciales en su memoria interna.

Uso:
    1) Enciende UN solo Tello.
    2) Conecta el PC al WiFi del Tello (algo como "TELLO-XXXXXX").
    3) Ejecuta:
           python configure_tello.py "NombreDeMiRouter" "passwordDelRouter"
    4) El Tello responde 'OK', parpadea, se reinicia y se une al router.
    5) Repite con el siguiente Tello (apaga el primero, enciende el segundo,
       cambia el WiFi del PC a su AP, lanza el script otra vez).

Después: conecta el PC al MISMO router. Los Tellos serán direccionables por
las IPs que el router les asigne.
"""
import sys
from djitellopy import Tello


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)

    ssid, pwd = sys.argv[1], sys.argv[2]

    print(f"Conectando al Tello (asegúrate de que el WiFi del PC es el del Tello)…")
    t = Tello()
    t.connect()
    batt = t.get_battery()
    print(f"  ✓ Tello respondió. Batería: {batt}%")

    if batt < 50:
        print("  ⚠ Batería < 50%. Recomendable cargar antes; "
              "los Tellos rechazan despegar con poca batería.")

    print(f"Mandando comando para que se una al WiFi '{ssid}'…")
    t.connect_to_wifi(ssid, pwd)
    print("  ✓ Comando enviado.")
    print()
    print("El Tello se reiniciará y se unirá a tu router en ~30 segundos.")
    print("Después podrás verlo en tu router con su nueva IP.")
    print()
    print("Para volver a modo AP (si quieres deshacer esto):")
    print("  - Mantén pulsado el botón de power del Tello hasta que parpadee amarillo.")


if __name__ == "__main__":
    main()
