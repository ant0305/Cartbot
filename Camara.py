# importar la libreria opencv para capturar video desde la cámara USB
import cv2

# Abrir la cámara USB (índice 1) con DirectShow
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# Si la cámara no se abre correctamente, mostrar un mensaje de error y salir
if not cap.isOpened():
    print("Error al abrir cámara USB")
    exit()

# Configurar propiedades de la cámara
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 150)
cap.set(cv2.CAP_PROP_CONTRAST, 150)
cap.set(cv2.CAP_PROP_EXPOSURE, 0.5)

# crear una ventana para mostrar el video de la cámara
cv2.namedWindow('Cámara USB - Presiona Q para salir', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Cámara USB - Presiona Q para salir', 1280, 720)
# tamaño inicial grande

print("Cámara abierta. Maximiza la ventana si quieres."
      " Presiona 'q' para salir.")

# bucle principal para capturar y mostrar frames de la cámara
while True:
    ret, frame = cap.read()
    if not ret:
        print("No se puede leer frame")
        break

    cv2.imshow('Cámara USB - Presiona Q para salir', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la cámara y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
