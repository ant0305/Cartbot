# importar la libreria opencv para capturar video desde la cámara USB
import cv2
# importar numpy para operaciones matemáticas
import numpy as np

# constantes para la detección de marcadores ArUco
DICT = cv2.aruco.DICT_6X6_250
ROBOT_ID = 0
MARKER_SIZE_CM = 13.0        # tamano real del marcador impreso

# Configuracion de la Camara USB
CAM_INDEX = 1
WIDTH, HEIGHT = 640, 480


# Función para abrir la cámara USB y configurar sus propiedades
def open_camera():
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError("Error al abrir la camara USB")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 150)
    cap.set(cv2.CAP_PROP_CONTRAST, 150)
    cap.set(cv2.CAP_PROP_EXPOSURE, 0.5)
    return cap


# Función para crear el detector de ArUco
def make_detector():
    """Detector compatible con OpenCV >= 4.7."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICT)
    params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, params)


# Función para calcular el centro y la orientación de un marcador ArUco
def pose_from_corners(corner):
    """
    corner: array (1,4,2) con las 4 esquinas en orden
            (top-left, top-right, bottom-right, bottom-left).
    Devuelve el centro en pixeles y el angulo de orientacion en radianes.
    """
    c = corner.reshape(4, 2)
    center = c.mean(axis=0)
    x_axis = c[1] - c[0]
    angle = np.arctan2(x_axis[1], x_axis[0])
    return center, angle


def main():
    # Abrir la cámara y crear el detector
    cap = open_camera()
    detector = make_detector()
    # Crear ventana para mostrar el video de la cámara
    win = "ArUco - Q para salir"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    print("Camara abierta. Presiona 'q' para salir.")
    # Bucle principal para capturar y mostrar frames de la cámara
    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se puede leer frame")
            break

        corners, ids, _ = detector.detectMarkers(frame)
        # Dibujar los marcadores detectados y mostrar el centro y la orientación del robot
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for corner, marker_id in zip(corners, ids.flatten()):
                if marker_id != ROBOT_ID:
                    continue

                center, angle = pose_from_corners(corner)
                cx, cy = int(center[0]), int(center[1])

                # Centro del robot
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                # Flecha de orientacion
                length = 60
                ex = int(cx + length * np.cos(angle))
                ey = int(cy + length * np.sin(angle))
                cv2.arrowedLine(frame, (cx, cy), (ex, ey), (0, 255, 0), 2)

                deg = np.degrees(angle)
                cv2.putText(frame, f"ID{marker_id} ({cx},{cy}) {deg:.0f}deg",
                            (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        cv2.imshow(win, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
