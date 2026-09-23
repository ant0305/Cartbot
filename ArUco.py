import cv2
from cv2 import aruco

# selecciona el diccionario de marcadores ArUco (6x6 con 250 posibles IDs)
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Tamaño en píxeles
marker_size_pixels = 300

# genera y guarda imágenes de marcadores ArUco para los IDs 0, 1 y 2
ids_a_generar = [0, 1, 2]

# Generar y guardar los marcadores
for id in ids_a_generar:
    marker_image = aruco.generateImageMarker(dictionary, id,
                                             marker_size_pixels)
    filename = f"aruco_6x6_id_{id}_{marker_size_pixels}px.png"
    cv2.imwrite(filename, marker_image)
    print(f"Generado marcador ID {id}: {filename}")

print("\nListo para imprimir:")
print("- Imprime los archivos a tamaño real (100 %, sin escalar ni ajustar).")
print("- Usa papel blanco grueso o cartulina.")
print("- Cada marcador debe medir ≈10–12 cm por lado.")
print("- Pégalos en cartón o directamente en el carrito.")
print("- Recomendación: empieza con ID 0 para pruebas simples.")
