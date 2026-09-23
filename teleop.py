"""
teleop.py
Teleoperacion del carrito para PROBAR la cadena PC -> ESP32 -> motores.
(Windows: usa msvcrt para leer el teclado sin bloquear.)

Antes de correr:
  1. Sube carrito_esp32.ino al ESP32.
  2. Conecta la PC al WiFi "CarritoIA" (clave: carrito123).
     -> Mientras estes en esa red no tendras internet; no importa,
        nada del proyecto lo necesita.
  3. Corre: python teleop.py

Controles:
  w = adelante   s = atras   a = izquierda   d = derecha
  espacio = detener          q = salir
"""

# importar librerias
import socket
import time
import msvcrt      # Windows

ESP32_IP = "192.168.4.1"   # IP por defecto del ESP32 en modo AP
PORT = 8888
RESEND_S = 0.1             # reenviar comando cada100ms(mantienevivoelfailsafe)

TECLAS = {
    'w': 'F', 's': 'B', 'a': 'L', 'd': 'R', ' ': 'S',
}


# funcion para teleoperar el carrito desde la PC (Windows) usando teclado y WiFi
def main():
    print(f"Conectando a {ESP32_IP}:{PORT} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    try:
        sock.connect((ESP32_IP, PORT))
    except OSError as e:
        print("No se pudo conectar.")
        print("Revisa que la PC este conectada al WiFi 'CarritoIA'.")
        print("Detalle:", e)
        return
    sock.settimeout(None)
    print("Conectado. w/a/s/d, espacio=detener, q=salir")

    comando = 'S'
    try:
        while True:
            if msvcrt.kbhit():
                tecla = msvcrt.getch().decode('latin-1', 'ignore').lower()
                if tecla == 'q':
                    break
                if tecla in TECLAS:
                    comando = TECLAS[tecla]
                    print("comando ->", comando, "     ", end='\r')

            sock.sendall(comando.encode())   # reenvio periodico
            time.sleep(RESEND_S)
    except (KeyboardInterrupt, OSError):
        pass
    finally:
        try:
            sock.sendall(b'S')   # detener al salir
        except OSError:
            pass
        sock.close()
        print("\nDesconectado. Carrito detenido.")


if __name__ == "__main__":
    main()
