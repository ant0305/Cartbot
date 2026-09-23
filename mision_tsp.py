"""
mision_tsp.py
Mision completa: elegis varios destinos, el TSP calcula el mejor orden,
y el carrito los visita en secuencia (con pausa en cada uno).

Teclas:
  1 = agregar/quitar Bodega     2 = agregar/quitar Hospital
  ENTER = calcular orden (TSP) e iniciar mision
  c = limpiar     0/espacio = abortar     q = salir

Primero en SIM (ENVIAR_COMANDOS=False en control.py se respeta aca tambien).
"""

# importaciones de librerias
import cv2
import time
from math import cos, sin

# importaciones de modulos propios
from aruco_detector import make_detector, pose_from_corners, open_camera
from homography import load_homography, pixel_to_world, world_to_cell
from mapa import GridMap, CELL_PX, EDIFICIOS, ROBOT_ID
from planner import astar
from vista_ruta import celda_aproximacion, dibujar_ruta
from tsp import ordenar_mision
import control as ctrl   # reutiliza parametros y funciones del control

PAUSA_S = 3.0            # espera en cada destino

# definicion de estados de la mision
SELECC, NAVEGANDO, ESPERANDO, COMPLETADA = "SELECCION", "NAVEGANDO", "ESPERANDO", "COMPLETADA"


# Clase para representar la mision completa (TSP)
class MisionTSP:
    def __init__(self):
        self.seleccion = []     # ids elegidos por el usuario
        self.orden = []         # ids ordenados por el TSP
        self.indice = 0
        self.estado = SELECC
        self.t_llegada = 0.0

    # funcion para agregar/quitar un destino de la mision
    def toggle(self, mid):
        if self.estado != SELECC:
            return
        if mid in self.seleccion:
            self.seleccion.remove(mid)
        else:
            self.seleccion.append(mid)

    # funcion para limpiar la mision
    def limpiar(self):
        self.__init__()

    # funcion para abortar la mision
    def abortar(self):
        self.estado = SELECC
        self.orden = []
        self.indice = 0

    # funcion para iniciar la mision (calcular orden y pasar a NAVEGANDO)
    def iniciar(self, mapa, aprox):
        destinos = {m: aprox[m] for m in self.seleccion if m in aprox}
        if not destinos or mapa.robot is None:
            return "faltan destinos alcanzables"
        orden, total = ordenar_mision(mapa, mapa.robot, destinos)
        if orden is None:
            return "algun destino sin ruta"
        self.orden = orden
        self.indice = 0
        self.estado = NAVEGANDO
        return f"orden: {' -> '.join(EDIFICIOS[i] for i in orden)} ({total} pasos)"

    # funcion para obtener el id del objetivo actual de la mision
    def objetivo(self):
        if self.estado in (NAVEGANDO, ESPERANDO) and self.indice < len(self.orden):
            return self.orden[self.indice]
        return None

    # funcion para actualizar el estado de la mision (llamada cada frame)
    def actualizar(self, dist_meta):
        if self.estado == NAVEGANDO:
            if dist_meta is not None and dist_meta <= ctrl.ARRIVAL_TOL:
                self.estado = ESPERANDO
                self.t_llegada = time.time()
        elif self.estado == ESPERANDO:
            if time.time() - self.t_llegada >= PAUSA_S:
                self.indice += 1
                self.estado = NAVEGANDO if self.indice < len(self.orden) else COMPLETADA

    # funcion para obtener el texto a mostrar en la pantalla
    def texto(self):
        if self.estado == SELECC:
            sel = " + ".join(EDIFICIOS[i] for i in self.seleccion) or "(nada)"
            return f"[SELECCION] {sel}  (ENTER para calcular ruta)"
        prog = " -> ".join(
            (">" + EDIFICIOS[i] if k == self.indice else EDIFICIOS[i])
            for k, i in enumerate(self.orden))
        linea = f"[{self.estado}] {prog}"
        if self.estado == ESPERANDO:
            linea += f"  esperando {PAUSA_S - (time.time()-self.t_llegada):.1f}s"
        return linea

# funcion principal para ejecutar la aplicacion de mision completa (TSP)
def main():
    H = load_homography()
    detector = make_detector()
    cap = open_camera()
    mapa = GridMap()
    link = ctrl.RobotLink()
    mision = MisionTSP()
    aviso = ""

    print("1/2=elegir destinos  ENTER=calcular e iniciar  c=limpiar  q=salir")
    while True:
        ret, frame = ctrl.leer_fresco(cap)
        if not ret:
            break

        corners, ids, _ = detector.detectMarkers(frame)
        mapa.limpiar()
        pend = []
        robot_head = None

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for corner, marker_id in zip(corners, ids.flatten()):
                center, ang = pose_from_corners(corner)
                x_cm, y_cm = pixel_to_world(H, center[0], center[1])
                celda = world_to_cell(x_cm, y_cm)
                mid = int(marker_id)
                if mid == ROBOT_ID:
                    mapa.set_robot(celda)
                    robot_head = ctrl.heading_mundo(H, center, ang)
                elif mid in EDIFICIOS:
                    mapa.add_edificio(mid, EDIFICIOS[mid], celda)
                    pend.append((mid, center, ang))

        aprox = {}
        for mid, center, ang in pend:
            a = celda_aproximacion(H, center, ang, mapa)
            if a is not None:
                aprox[mid] = a

        # --- logica de la mision ---
        obj_id = mision.objetivo()
        meta = aprox.get(obj_id)
        ruta = None
        objetivo = None
        cmd = 'S'; err = 0.0
        dist_meta = None
        # Si hay meta y robot, calcular ruta y decidir comando
        if meta is not None and mapa.robot is not None and robot_head is not None:
            dist_meta = abs(mapa.robot[0]-meta[0]) + abs(mapa.robot[1]-meta[1])
            mision.actualizar(dist_meta)
            if mision.estado == NAVEGANDO:
                ruta = astar(mapa, mapa.robot, meta)
                objetivo = ctrl.objetivo_lookahead(ruta)
                if objetivo is not None:
                    cmd, err = ctrl.decidir(robot_head, mapa.robot, objetivo)
                    if cmd == 'F' and not mapa.es_libre(
                            ctrl.celda_frente(mapa.robot, robot_head)):
                        cmd = 'S'
        elif obj_id is not None:
            mision.actualizar(None)   # deja correr la pausa aunque no haya meta

        # si hay ruta, dibujarla en el mapa logico
        grid = mapa.dibujar()
        dibujar_ruta(grid, ruta, meta)
        if objetivo is not None:
            r, c = objetivo
            cv2.circle(grid, (c*CELL_PX+CELL_PX//2, r*CELL_PX+CELL_PX//2),
                       CELL_PX//5, ctrl.COL_BLANCO, 2)
        if mapa.robot is not None and robot_head is not None:
            r, c = mapa.robot
            cx, cy = c*CELL_PX+CELL_PX//2, r*CELL_PX+CELL_PX//2
            ex = int(cx + CELL_PX*0.6*cos(robot_head))
            ey = int(cy + CELL_PX*0.6*sin(robot_head))
            cv2.arrowedLine(grid, (cx, cy), (ex, ey), ctrl.COL_RUMBO, 2, tipLength=0.4)
        # mostrar estado de la mision y comando en la ventana del mapa logico
        modo = "SIM" if link.sock is None else "REAL"
        cv2.putText(grid, f"[{modo}] {mision.texto()}  cmd:{cmd}", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        if aviso:
            cv2.putText(grid, aviso, (8, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,255), 1)

        cv2.imshow("Camara", frame)
        cv2.imshow("Mapa logico", grid)
        # esperar tecla y procesar comandos
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('1'):
            mision.toggle(1); aviso = ""
        elif k == ord('2'):
            mision.toggle(2); aviso = ""
        elif k == ord('c'):
            mision.limpiar(); aviso = ""
        elif k in (ord('0'), ord(' ')):
            mision.abortar(); aviso = ""
        elif k == 13:   # ENTER
            aviso = mision.iniciar(mapa, aprox)

        # enviar comando al carrito (si hay link activo)
        if cmd in ('F','L','R') and link.sock is not None:
            ctrl.pulso(link, cmd, err, dist_meta if dist_meta else 999)
        elif link.sock is not None:
            link.enviar('S')

    link.cerrar()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
