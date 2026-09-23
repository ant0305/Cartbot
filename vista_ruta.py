"""
vista_ruta.py
Mapa en vivo + celda de aproximacion al frente de cada edificio + ruta A*.

Teclas:  1 = ir a Bodega   2 = ir a Hospital   0 = sin destino   q = salir
"""

import cv2
import numpy as np

from aruco_detector import make_detector, pose_from_corners, open_camera
from homography import (load_homography, pixel_to_world, world_to_cell,
                        CELL_W_CM, CELL_H_CM)
from mapa import GridMap, CELL_PX, EDIFICIOS, ROBOT_ID
from planner import astar

COL_RUTA = (0, 220, 220)   # amarillo
COL_APROX = (0, 140, 255)  # naranja


def celda_aproximacion(H, center_px, angle, mapa):
    """
    Celda libre al FRENTE del edificio, segun la orientacion de su ArUco.
    Importante: la direccion se calcula en el MUNDO, no en pixeles,
    porque la homografia deforma los angulos por la perspectiva.
    """
    L = 30.0                      # largo del vector de prueba, en pixeles
    fx = center_px[0] + L * np.cos(angle)
    fy = center_px[1] + L * np.sin(angle)

    x0, y0 = pixel_to_world(H, center_px[0], center_px[1])   # centro
    x1, y1 = pixel_to_world(H, fx, fy)                        # frente

    dx, dy = x1 - x0, y1 - y0
    n = np.hypot(dx, dy)
    if n < 1e-6:
        return None
    dx, dy = dx / n, dy / n       # direccion unitaria en cm

    paso = max(CELL_W_CM, CELL_H_CM)
    for k in (3, 4):              # 1 celda al frente; si no, 2
        celda = world_to_cell(x0 + dx * paso * k, y0 + dy * paso * k)
        if celda is not None and mapa.es_libre(celda):
            return celda
    return None


def dibujar_ruta(img, ruta, aprox):
    """Pinta la ruta y la celda de aproximacion sobre la imagen del grid."""
    if aprox is not None:
        r, c = aprox
        cv2.rectangle(img, (c*CELL_PX+6, r*CELL_PX+6),
                      ((c+1)*CELL_PX-6, (r+1)*CELL_PX-6), COL_APROX, 2)
    if ruta:
        pts = [(c*CELL_PX+CELL_PX//2, r*CELL_PX+CELL_PX//2) for r, c in ruta]
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, a, b, COL_RUTA, 3)
        for p in pts:
            cv2.circle(img, p, 4, COL_RUTA, -1)


# funcion principal para ejecutar la aplicacion de mapa logico + ruta
def main():
    H = load_homography()
    detector = make_detector()
    cap = open_camera()
    mapa = GridMap()
    destino_id = None

    print("1=Bodega  2=Hospital  0=sin destino  q=salir")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids, _ = detector.detectMarkers(frame)
        mapa.limpiar()
        aprox = {}          # id -> celda de aproximacion
        pendientes = []     # edificios a procesar despues de armar el mapa
        # Procesar los marcadores detectados
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for corner, marker_id in zip(corners, ids.flatten()):
                center, angle = pose_from_corners(corner)
                x_cm, y_cm = pixel_to_world(H, center[0], center[1])
                celda = world_to_cell(x_cm, y_cm)
                mid = int(marker_id)

                if mid == ROBOT_ID:
                    mapa.set_robot(celda)
                elif mid in EDIFICIOS:
                    mapa.add_edificio(mid, EDIFICIOS[mid], celda)
                    pendientes.append((mid, center, angle))

        # Las celdas de aproximacion se calculan con el mapa ya completo,
        # para no elegir una celda ocupada por otro edificio.
        for mid, center, angle in pendientes:
            a = celda_aproximacion(H, center, angle, mapa)
            if a is not None:
                aprox[mid] = a

        # Ruta A* hacia el destino elegido
        ruta = None
        meta = aprox.get(destino_id)
        if mapa.robot is not None and meta is not None:
            ruta = astar(mapa, mapa.robot, meta)

        grid = mapa.dibujar()
        dibujar_ruta(grid, ruta, meta)

        estado = "sin destino" if destino_id is None else EDIFICIOS[destino_id]
        if destino_id is not None:
            if meta is None:
                estado += " (sin celda de aprox.)"
            elif ruta is None:
                estado += " (sin ruta)"
            else:
                estado += f" - {len(ruta)-1} pasos"
        cv2.putText(grid, estado, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        cv2.imshow("Camara", frame)
        cv2.imshow("Mapa logico", grid)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('1'):
            destino_id = 1
        elif k == ord('2'):
            destino_id = 2
        elif k == ord('0'):
            destino_id = None

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
