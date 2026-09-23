"""
control.py
Control en lazo cerrado con look-ahead + llegada por cercania + freno reactivo.

Teclas: 1=Bodega  2=Hospital  0/espacio=detener  q=salir
Primero en SIM; cuando el rumbo y los comandos tengan sentido, pasar a REAL.
"""

# importar opencv para capturar video desde la cámara USB
import cv2
# importar librerias de red y tiempo
import socket
# importar librerias matematicas
import time
from math import atan2, cos, sin, pi, radians

# importar modulos propios
from aruco_detector import make_detector, pose_from_corners, open_camera
from homography import load_homography, pixel_to_world, world_to_cell
from mapa import GridMap, CELL_PX, EDIFICIOS, ROBOT_ID
from planner import astar
from vista_ruta import celda_aproximacion, dibujar_ruta

# ---------------- Parametros de red ----------
ENVIAR_COMANDOS = True
CARRITO_IP = "192.168.4.1"
CARRITO_PORT = 8888

# ---------- Parametros de control ----------
LOOKAHEAD = 3             # celdas adelante que se apuntan
UMBRAL_ANGULO = radians(22)   # dentro de esto -> avanza
ARRIVAL_TOL = 1             # celdas: se da por llegado a esta distancia del destino
PULSO_AVANCE = 0.18          # pulso de avance (s) para avanzar 1 celda
PULSO_GIRO = 0.12          # pulso de giro (s) para girar 90 grados
PULSO_GIRO_MIN = 0.04          # pulso de giro minimo (s) para girar 90 grados
ASENTAR = 0.15          # s de espera tras detener
INVERTIR_GIRO = False    # si True, gira al reves (para probar el carrito al reves)
OFFSET_HEADING = 0.0           # rad; corrige como esta pegado el ArUco

COL_RUMBO = (0, 255, 0)
COL_BLANCO = (0, 200, 255)


# clase para manejar la conexion con el carrito
class RobotLink:
    # funcion de inicializacion: abre el socket TCP con el carrito
    def __init__(self):
        self.sock = None
        if not ENVIAR_COMANDOS:
            print("Modo SIMULACION: no se envian comandos.")
            return
        # Intentar abrir el socket TCP con el carrito
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3); s.connect((CARRITO_IP, CARRITO_PORT)); s.settimeout(None)
            self.sock = s
            print("Carrito conectado.")
        # capturar excepcion si no se puede conectar y mostrar mensaje de error
        except OSError as e:
            print("No se pudo conectar -> simulacion. Detalle:", e)

    # funcion para enviar un comando al carrito
    def enviar(self, c):
        if self.sock:
            try: self.sock.sendall(c.encode())
            except OSError: pass

    # funcion para cerrar la conexion con el carrito
    def cerrar(self):
        if self.sock:
            try: self.sock.sendall(b'S')
            except OSError: pass
            self.sock.close()


# funcion para normalizar un angulo a [-pi, pi]
def norm(a):
    while a > pi: a -= 2*pi
    while a < -pi: a += 2*pi
    return a


# funcion para calcular el rumbo del robot en el mundo a partir de la homografia, el centro del ArUco y el angulo del ArUco
def heading_mundo(H, center_px, angle_px, L=30.0):
    fx = center_px[0] + L*cos(angle_px)
    fy = center_px[1] + L*sin(angle_px)
    x0, y0 = pixel_to_world(H, center_px[0], center_px[1])
    x1, y1 = pixel_to_world(H, fx, fy)
    return atan2(y1 - y0, x1 - x0) + OFFSET_HEADING


# funcion para obtener la celda objetivo a partir de la ruta y el lookahead
def objetivo_lookahead(ruta):
    if not ruta or len(ruta) < 2:
        return None
    return ruta[min(LOOKAHEAD, len(ruta) - 1)]


# funcion para decidir el comando a enviar al carrito en base al rumbo, la celda actual y la celda objetivo
def decidir(heading, celda, objetivo):
    dr = objetivo[0] - celda[0]
    dc = objetivo[1] - celda[1]
    err = norm(atan2(dr, dc) - heading)
    if abs(err) < UMBRAL_ANGULO:
        return 'F', err
    gira_pos = err > 0
    if INVERTIR_GIRO:
        gira_pos = not gira_pos
    return ('R' if gira_pos else 'L'), err


# funcion para obtener la celda que el robot tiene justo enfrente segun su rumbo
def celda_frente(celda, heading):
    """Celda que el robot tiene justo enfrente segun su rumbo."""
    dr = int(round(sin(heading)))
    dc = int(round(cos(heading)))
    return (celda[0] + dr, celda[1] + dc)


# funcion para enviar un pulso de comando al carrito, con duracion variable segun el comando y la distancia al objetivo
def pulso(link, cmd, err=0.0, dist_meta=999):
    if cmd == 'F':
        dur = PULSO_AVANCE * (0.5 if dist_meta <= 2 else 1.0)   # mas corto si esta cerca
    else:
        factor = min(1.0, abs(err) / radians(90))               # menos angulo -> pulso mas corto
        dur = max(PULSO_GIRO_MIN, PULSO_GIRO * factor)
    link.enviar(cmd); time.sleep(dur)
    link.enviar('S'); time.sleep(ASENTAR)


# funcion para actualizar la camara y descartar frames viejos, devolviendo el frame mas reciente
def leer_fresco(cap, descartar=4):
    for _ in range(descartar):
        cap.grab()
    return cap.read()


# funcion principal del programa
def main():
    H = load_homography()
    detector = make_detector()
    cap = open_camera()
    mapa = GridMap()
    link = RobotLink()

    destino_id = None
    # variable para almacenar el ultimo comando enviado al carrito
    ultimo_cmd = "-"

    print("1=Bodega 2=Hospital 0=detener q=salir")
    while True:
        ret, frame = leer_fresco(cap)
        if not ret:
            break

        corners, ids, _ = detector.detectMarkers(frame)
        # si no se detectan marcadores, limpiar el mapa y reiniciar variables
        mapa.limpiar()
        pend = []
        robot_head = None

        # si se detectan marcadores, dibujar los marcadores y actualizar el mapa con la posicion del robot y los edificios
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for corner, marker_id in zip(corners, ids.flatten()):
                center, ang = pose_from_corners(corner)
                x_cm, y_cm = pixel_to_world(H, center[0], center[1])
                celda = world_to_cell(x_cm, y_cm)
                mid = int(marker_id)
                if mid == ROBOT_ID:
                    mapa.set_robot(celda)
                    robot_head = heading_mundo(H, center, ang)
                elif mid in EDIFICIOS:
                    mapa.add_edificio(mid, EDIFICIOS[mid], celda)
                    pend.append((mid, center, ang))

        # si no se detectan marcadores, limpiar el mapa y reiniciar variables
        aprox = {}
        for mid, center, ang in pend:
            a = celda_aproximacion(H, center, ang, mapa)
            if a is not None:
                aprox[mid] = a

        # si hay un destino seleccionado, obtener la meta y calcular la ruta y el comando a enviar al carrito
        meta = aprox.get(destino_id)
        ruta = None
        objetivo = None
        cmd = 'S'; err = 0.0; dist_meta = 999

        # si hay un destino seleccionado pero no se detecta el marcador, indicar que no hay acceso
        if destino_id is not None and meta is None:
            ultimo_cmd = "SIN ACCESO"
        elif meta is not None and mapa.robot is not None and robot_head is not None:
            dist_meta = abs(mapa.robot[0]-meta[0]) + abs(mapa.robot[1]-meta[1])
            if dist_meta <= ARRIVAL_TOL:
                ultimo_cmd = "LLEGO"; destino_id = None      # llegada por cercania
            else:
                ruta = astar(mapa, mapa.robot, meta)
                objetivo = objetivo_lookahead(ruta)
                if objetivo is not None:
                    cmd, err = decidir(robot_head, mapa.robot, objetivo)
                    if cmd == 'F':
                        # freno de seguridad: no avanzar hacia celda ocupada / fuera
                        if not mapa.es_libre(celda_frente(mapa.robot, robot_head)):
                            cmd = 'S'; ultimo_cmd = "FRENO"
                        else:
                            ultimo_cmd = cmd
                    else:
                        ultimo_cmd = cmd
                else:
                    ultimo_cmd = "sin ruta"

        # dibujar el mapa logico, la ruta y el rumbo del robot
        grid = mapa.dibujar()
        dibujar_ruta(grid, ruta, meta)
        if objetivo is not None:
            r, c = objetivo
            cv2.circle(grid, (c*CELL_PX+CELL_PX//2, r*CELL_PX+CELL_PX//2),
                       CELL_PX//5, COL_BLANCO, 2)
        if mapa.robot is not None and robot_head is not None:
            r, c = mapa.robot
            cx, cy = c*CELL_PX+CELL_PX//2, r*CELL_PX+CELL_PX//2
            ex = int(cx + CELL_PX*0.6*cos(robot_head))
            ey = int(cy + CELL_PX*0.6*sin(robot_head))
            cv2.arrowedLine(grid, (cx, cy), (ex, ey), COL_RUMBO, 2, tipLength=0.4)

        # mostrar información del estado del sistema
        modo = "SIM" if link.sock is None else "REAL"
        # mostrar el destino y el ultimo comando enviado al carrito
        est = "sin destino" if destino_id is None else EDIFICIOS.get(destino_id, "-")
        col = (0,140,255) if ultimo_cmd in ("SIN ACCESO","FRENO") else (255,255,255)
        cv2.putText(grid, f"[{modo}] destino: {est}   cmd: {ultimo_cmd}",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        # mostrar el frame de la camara y el mapa logico
        cv2.imshow("Camara", frame)
        cv2.imshow("Mapa logico", grid)

        # leer la tecla presionada y actualizar el destino o salir del programa
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'): break
        elif k == ord('1'): destino_id = 1
        elif k == ord('2'): destino_id = 2
        elif k in (ord('0'), ord(' ')): destino_id = None

        if cmd in ('F', 'L', 'R') and link.sock is not None:
            pulso(link, cmd, err, dist_meta)
        elif link.sock is not None:
            link.enviar('S')

    # cerrar la conexion con el carrito y liberar la camara
    link.cerrar()
    cap.release()
    cv2.destroyAllWindows()


# si el script se ejecuta directamente, llamar a la funcion main
if __name__ == "__main__":
    main()
