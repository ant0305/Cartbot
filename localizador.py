"""
localizador.py
Detecta en vivo el robot y los edificios, y muestra en que CELDA del grid
esta cada uno usando la homografia ya calibrada (homography.npy).

Requiere haber corrido antes:  python homography.py
"""

import cv2
import numpy as np

from aruco_detector import make_detector, pose_from_corners, open_camera
from homography import load_homography, pixel_to_world, world_to_cell

# variables globales para la aplicacion de localizacion
ROBOT_ID = 0
EDIFICIOS = {
    1: "Bodega",
    2: "Hospital",
    # 3: "Cafeteria",  <- agregar aqui nuevos edificios
}


#  funcion para obtener el nombre de un marker
def etiqueta(marker_id):
    if marker_id == ROBOT_ID:
        return "ROBOT"
    return EDIFICIOS.get(marker_id, f"ID{marker_id}")


# funcion para obtener el color de un marker
def color(marker_id):
    if marker_id == ROBOT_ID:
        return (0, 0, 255)      # rojo = robot
    return (255, 150, 0)        # azul = edificios


# funcion principal de la aplicacion de localizacion
def main():
    H = load_homography()          # matriz de la calibracion previa
    detector = make_detector()
    cap = open_camera()            # abre y configura la camara USB

    win = "Localizador - Q para salir"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    print("Detectando. Presiona 'q' para salir.")

    # bucle principal para leer frames de la camara y mostrar la localizacion de los markers
    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se puede leer frame")
            break

        corners, ids, _ = detector.detectMarkers(frame)

        # Estado del mundo reconstruido en ESTE frame (nada guardado en fijo)
        mundo = {}   # marker_id -> (celda, angulo)

        # Si se detectaron markers, dibujarlos y calcular su celda y angulo
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for corner, marker_id in zip(corners, ids.flatten()):
                center, angle = pose_from_corners(corner)
                cx, cy = int(center[0]), int(center[1])

                # pixel -> mundo (cm) -> celda (fila, columna)
                x_cm, y_cm = pixel_to_world(H, cx, cy)
                celda = world_to_cell(x_cm, y_cm)
                mundo[int(marker_id)] = (celda, angle)

                col = color(marker_id)
                cv2.circle(frame, (cx, cy), 5, col, -1)

                # Flecha de orientacion
                length = 50
                ex = int(cx + length * np.cos(angle))
                ey = int(cy + length * np.sin(angle))
                cv2.arrowedLine(frame, (cx, cy), (ex, ey), (0, 255, 0), 2)

                # Etiqueta con ID y celda
                celda_txt = f"{celda}" if celda else "fuera"
                cv2.putText(frame, f"{etiqueta(marker_id)} {celda_txt}",
                            (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

        # Mostrar en la parte superior izquierda del frame el estado del mundo reconstruido
        y = 25
        for mid in sorted(mundo):
            celda, ang = mundo[mid]
            deg = np.degrees(ang)
            txt = f"{etiqueta(mid)}: celda {celda}  {deg:.0f}deg"
            cv2.putText(frame, txt, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y += 28

        # hacer un refresh de la ventana y esperar 1ms por tecla
        cv2.imshow(win, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # liberar recursos
    cap.release()
    cv2.destroyAllWindows()


# funcion principal para ejecutar la calibracion interactiva de la homografia
if __name__ == "__main__":
    main()
