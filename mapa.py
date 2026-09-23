"""
mapa.py
Construye y dibuja el mapa logico (grid) a partir de lo que ve la camara.
El estado del mundo se reconstruye en cada frame desde los ArUcos:
  - robot (ID 0): su celda
  - edificios (ID 1,2,...): su celda = obstaculo, inflada por el tamano del robot
Nada se guarda fijo -> si mueves un edificio, el mapa se actualiza solo.
"""

import cv2
import numpy as np

from aruco_detector import make_detector, pose_from_corners, open_camera
from homography import (load_homography, pixel_to_world, world_to_cell,
                        GRID_ROWS, GRID_COLS)

ROBOT_ID = 0
EDIFICIOS = {1: "Bodega", 2: "Hospital"}
RADIO_INFLADO = 2              # celdas de margen (robot 20x15 -> radio~12.5cm)
CELL_PX = 60                   # tamano de cada celda en pantalla (pixeles)

# definen colores para dibujar el mapa logico
COL_LIBRE = (60, 60, 60)
COL_OBST = (40, 40, 200)     # rojo = obstaculo (edificio)
COL_MARGEN = (30, 30, 110)     # rojo oscuro = margen de seguridad
COL_ROBOT = (0, 200, 0)       # verde = robot
COL_GRID = (90, 90, 90)
COL_TEXTO = (255, 255, 255)


# Clase para representar el mapa logico (grid) y dibujarlo
class GridMap:
    # funcion para inicializar el mapa logico
    def __init__(self, rows=GRID_ROWS, cols=GRID_COLS):
        self.rows = rows
        self.cols = cols
        self.obstaculos = set()   # celdas ocupadas (fila, col)
        self.inflado = set()      # margen de seguridad alrededor de obstaculos
        self.robot = None         # celda del robot
        self.destinos = {}        # id -> (nombre, celda)

    # funcion para limpiar el mapa logico (reiniciar)
    def limpiar(self):
        self.obstaculos = set()
        self.inflado = set()
        self.robot = None
        self.destinos = {}

    # funcion para actualizar la celda del robot
    def set_robot(self, celda):
        self.robot = celda

    # funcion para agregar un edificio al mapa logico
    def add_edificio(self, marker_id, nombre, celda):
        if celda is None:
            return
        self.destinos[marker_id] = (nombre, celda)
        self.obstaculos.add(celda)
        # Inflar: el robot no es un punto, se agranda el obstaculo
        r, c = celda
        R = RADIO_INFLADO
        for dr in range(-R, R + 1):
            for dc in range(-R, R + 1):
                self.inflado.add((r + dr, c + dc))

    # funcion para verificar si una celda esta libre (no obstaculo ni inflada)
    def es_libre(self, celda):
        r, c = celda
        return (0 <= r < self.rows and 0 <= c < self.cols
                and celda not in self.obstaculos
                and celda not in self.inflado)

    # funcion para dibujar el mapa logico en una imagen (OpenCV)
    def dibujar(self):
        img = np.zeros((self.rows*CELL_PX, self.cols*CELL_PX, 3), dtype=np.uint8)

        for r in range(self.rows):
            for c in range(self.cols):
                x0, y0 = c*CELL_PX, r*CELL_PX
                # El color se decide ANTES de dibujar
                if (r, c) in self.obstaculos:
                    color = COL_OBST
                elif (r, c) in self.inflado:
                    color = COL_MARGEN
                else:
                    color = COL_LIBRE
                cv2.rectangle(img, (x0, y0), (x0+CELL_PX, y0+CELL_PX), color, -1)
                cv2.rectangle(img, (x0, y0), (x0+CELL_PX, y0+CELL_PX), COL_GRID, 1)

        for mid, (nombre, celda) in self.destinos.items():
            r, c = celda
            cv2.putText(img, nombre[:4], (c*CELL_PX+4, r*CELL_PX+CELL_PX//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_TEXTO, 1)

        if self.robot is not None:
            r, c = self.robot
            cx, cy = c*CELL_PX+CELL_PX//2, r*CELL_PX+CELL_PX//2
            cv2.circle(img, (cx, cy), CELL_PX//3, COL_ROBOT, -1)

        return img


# funcion principal para ejecutar la aplicacion de mapa logico
def main():
    H = load_homography()
    detector = make_detector()
    cap = open_camera()
    mapa = GridMap()

    print("Mapa logico en vivo. 'q' para salir.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids, _ = detector.detectMarkers(frame)
        mapa.limpiar()   # reconstruir desde cero cada frame

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for corner, marker_id in zip(corners, ids.flatten()):
                center, _ = pose_from_corners(corner)
                x_cm, y_cm = pixel_to_world(H, center[0], center[1])
                celda = world_to_cell(x_cm, y_cm)
                mid = int(marker_id)
                if mid == ROBOT_ID:
                    mapa.set_robot(celda)
                elif mid in EDIFICIOS:
                    mapa.add_edificio(mid, EDIFICIOS[mid], celda)

        cv2.imshow("Camara", frame)
        cv2.imshow("Mapa logico", mapa.dibujar())
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
