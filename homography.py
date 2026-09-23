"""
homography.py
Convierte coordenadas de pixel (camara) a coordenadas del mundo/grid.

Uso:
  1. Calibrar (una sola vez, con la camara en posicion definitiva):
       python homography.py
     Clicar las 4 esquinas del area de trabajo EN ESTE ORDEN:
       1) superior-izquierda
       2) superior-derecha
       3) inferior-derecha
       4) inferior-izquierda
     Se guarda la matriz en homography.npy

  2. En el resto del proyecto:
       from homography import load_homography, pixel_to_world, world_to_cell
"""

import cv2
import numpy as np
import os

# --- Dimensiones reales del area de trabajo (medir con cinta metrica) ---
WORLD_W_CM = 100.0    # ancho del area (direccion X)
WORLD_H_CM = 100.0    # alto del area (direccion Y)

# --- Grid logico ---
# Area cuadrada + grid cuadrado -> celdas cuadradas de 10x10 cm
GRID_COLS = 10
GRID_ROWS = 10

CELL_W_CM = WORLD_W_CM / GRID_COLS   # 10 cm por celda
CELL_H_CM = WORLD_H_CM / GRID_ROWS   # 10 cm por celda

HOMOGRAPHY_FILE = "homography.npy"


# funciones para cargar la homografia
def load_homography(path=HOMOGRAPHY_FILE):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"No existe {path}. Ejecuta 'python homography.py' para calibrar.")
    return np.load(path)


# funcion para convertir coordenadas de pixel a coordenadas del mundo
def pixel_to_world(H, px, py):
    """Pixel (px, py) -> coordenadas del mundo en cm (x, y).
    Origen: esquina superior-izquierda del area de trabajo."""
    p = np.array([px, py, 1.0])
    w = H @ p
    w /= w[2]
    return float(w[0]), float(w[1])


# funcion para convertir coordenadas del mundo a coordenadas de celda del grid
def world_to_cell(x_cm, y_cm):
    """Coordenadas del mundo (cm) -> celda del grid (fila, columna).
    Devuelve None si el punto cae fuera del area de trabajo."""
    col = int(x_cm // CELL_W_CM)
    row = int(y_cm // CELL_H_CM)
    if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
        return row, col
    return None


# funcion para convertir coordenadas de pixel a coordenadas de celda del grid
def pixel_to_cell(H, px, py):
    """Atajo: pixel -> celda del grid en un solo paso."""
    x, y = pixel_to_world(H, px, py)
    return world_to_cell(x, y)


# ----------------------------------------------------------------------
# Calibracion interactiva (ejecutar este archivo directamente)
# ----------------------------------------------------------------------

def calibrate():
    # Reutiliza el selector si aruco_detector.py esta en la misma carpeta
    try:
        from aruco_detector import select_camera, configure_camera
        cap = select_camera()
        configure_camera(cap)
    except ImportError:
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        if not cap.isOpened():
            raise RuntimeError("No se pudo abrir la camara")

    clicks = []
    labels = ["sup-izq", "sup-der", "inf-der", "inf-izq"]

    # funcion de callback para manejar los clics del mouse
    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))
            print(f"Punto {len(clicks)} ({labels[len(clicks)-1]}): ({x}, {y})")

    win = "Calibracion - clic en las 4 esquinas (r=reiniciar, ESC=salir)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    cv2.setMouseCallback(win, on_mouse)

    print("Clic en las 4 esquinas del area de trabajo, en orden:")
    print("  1) sup-izq  2) sup-der  3) inf-der  4) inf-izq")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Dibujar los puntos ya clicados
        for i, (x, y) in enumerate(clicks):
            cv2.circle(frame, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(frame, labels[i], (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if len(clicks) >= 2:
            pts = np.array(clicks, dtype=np.int32)
            cv2.polylines(frame, [pts], len(clicks) == 4, (0, 255, 0), 2)

        # Indicar cual punto sigue
        if len(clicks) < 4:
            cv2.putText(frame, f"Clic en esquina: {labels[len(clicks)]}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (255, 255, 0), 2)
        else:
            cv2.putText(frame, "Listo. ENTER=guardar, r=reiniciar",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0), 2)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            clicks.clear()
            print("Reiniciado. Clic de nuevo en las 4 esquinas.")
        elif key == 27:  # ESC
            print("Cancelado")
            break
        elif key == 13 and len(clicks) == 4:  # ENTER
            src = np.array(clicks, dtype=np.float32)
            dst = np.array([
                [0,          0],
                [WORLD_W_CM, 0],
                [WORLD_W_CM, WORLD_H_CM],
                [0,          WORLD_H_CM],
            ], dtype=np.float32)
            H = cv2.getPerspectiveTransform(src, dst)
            np.save(HOMOGRAPHY_FILE, H)
            print(f"Homografia guardada en {HOMOGRAPHY_FILE}")

            # Mostrar la conversion de pixel a coordenadas del mundo para cada esquina
            for (px, py), lab in zip(clicks, labels):
                x, y = pixel_to_world(H, px, py)
                print(f"  {lab}: pixel ({px},{py}) -> mundo "
                      f"({x:.1f}, {y:.1f}) cm")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    calibrate()
