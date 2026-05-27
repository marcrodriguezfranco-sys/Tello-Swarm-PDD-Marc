"""
Alternativa al configure_tello.py para cuando djitellopy falla por state
packets. Habla con el Tello directamente por UDP raw, sin esperar paquetes
de estado.

Uso:
    1) Conecta el PC al WiFi del Tello (TELLO-XXXXXX).
    2) Ejecuta:
           python configure_tello_raw.py "MiRouter" "miPassword"

Si funciona, el Tello aceptará el comando 'wifi ssid pwd' y se reiniciará.
"""
import socket
import sys
import time


TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
LOCAL_PORT = 9000   # cualquier puerto libre del PC; bind explícito


def send_command(sock, cmd: str, timeout: float = 5.0) -> str:
    """Envía un comando UDP al Tello y espera respuesta. None si timeout."""
    print(f"  >>> {cmd}")
    sock.sendto(cmd.encode('utf-8'), (TELLO_IP, TELLO_PORT))
    sock.settimeout(timeout)
    try:
        data, addr = sock.recvfrom(1024)
        resp = data.decode('utf-8', errors='replace').strip()
        print(f"  <<< {resp}    (de {addr[0]})")
        return resp
    except socket.timeout:
        print(f"  <<< (timeout {timeout}s — sin respuesta)")
        return None


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    ssid, pwd = sys.argv[1], sys.argv[2]

    # Crear socket UDP. Bind explícito al puerto local para tener control.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(('', LOCAL_PORT))
        print(f"Socket UDP bound to local port {LOCAL_PORT}")
    except OSError as e:
        print(f"⚠ No se pudo bind al puerto {LOCAL_PORT}: {e}")
        print("  (intentando sin bind)")

    print()
    print("1) Entrando en modo SDK con 'command'...")
    r = send_command(sock, "command")
    if r != "ok":
        print(f"❌ El Tello no respondió 'ok' a 'command'. Respuesta: {r}")
        print()
        print("Posibles causas:")
        print("  • El PC no está realmente en el WiFi del Tello "
              "(Windows pudo cambiar a otra red conocida).")
        print("  • Otro programa está ocupando el puerto UDP 8889.")
        print("  • Firewall de Windows bloqueando UDP.")
        sock.close()
        sys.exit(1)
    print("✓ Tello en modo SDK")

    print()
    print(f"2) Pidiendo unirse a WiFi '{ssid}'...")
    r = send_command(sock, f"wifi {ssid} {pwd}", timeout=10.0)
    # El Tello puede responder "ok" o algo como "OK,drone will reboot in 3s".
    # Cualquier respuesta que empiece por "ok" (case-insensitive) es éxito.
    if r is None or not r.lower().startswith("ok"):
        print(f"❌ El Tello rechazó el comando wifi. Respuesta: {r}")
        print()
        print("Posibles causas:")
        print("  • El SSID tiene caracteres especiales (acentos, espacios).")
        print("  • El password está mal.")
        print("  • El Tello no es EDU/TT (los estándar no soportan 'wifi').")
        sock.close()
        sys.exit(1)
    print("✓ Comando aceptado — el Tello se reiniciará")

    print()
    print("El Tello se reiniciará y se unirá a tu router en ~30 segundos.")
    print("Después podrás verlo en tu router con su nueva IP.")
    print()
    print("Para volverlo a modo AP: mantén pulsado el botón hasta que "
          "parpadee amarillo.")
    sock.close()


if __name__ == "__main__":
    main()
