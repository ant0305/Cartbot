"""
tsp.py
Ordena una lista de destinos para minimizar el recorrido total.
TSP exacto por enumeracion de permutaciones (valido porque son pocos destinos).

El costo entre dos puntos = longitud de la ruta A* (esquiva obstaculos),
no la distancia en linea recta. El recorrido arranca desde el robot.
"""

# importar librerias
from itertools import permutations
from planner import astar, longitud


def costo(mapa, a, b, cache):
    """Costo (pasos de A*) entre dos celdas, con memoria para no recalcular."""
    clave = (a, b)
    if clave not in cache:
        L = longitud(astar(mapa, a, b))
        cache[clave] = L
        cache[(b, a)] = L          # el costo es simetrico
    return cache[clave]


def ordenar_mision(mapa, inicio, destinos):
    """
    inicio: celda del robot.
    destinos: dict {id: celda_de_aproximacion}.
    Devuelve (mejor_orden_de_ids, costo_total) o (None, None) si algun
    tramo no tiene ruta.
    """
    ids = list(destinos.keys())
    if not ids or inicio is None:
        return None, None

    cache = {}
    mejor_orden = None
    mejor_costo = float('inf')

    # Probar todos los ordenes posibles de los destinos
    for perm in permutations(ids):
        total = 0
        actual = inicio
        valido = True
        # Calcular el costo de cada tramo del orden actual
        for mid in perm:
            c = costo(mapa, actual, destinos[mid], cache)
            if c is None:            # tramo sin ruta -> orden invalido
                valido = False
                break
            total += c
            actual = destinos[mid]
        # Comparar con el mejor orden encontrado hasta ahora
        if valido and total < mejor_costo:
            mejor_costo = total
            mejor_orden = list(perm)
    # Si no se encontro ningun orden valido, devolver None
    if mejor_orden is None:
        return None, None
    return mejor_orden, mejor_costo


# prueba de TSP con un mapa de juguete
if __name__ == "__main__":
    class MapaPrueba:
        def __init__(self, filas, cols, obst):
            self.rows, self.cols = filas, cols
            self.obstaculos = set(obst)
        def es_libre(self, celda):
            r, c = celda
            return (0 <= r < self.rows and 0 <= c < self.cols
                    and celda not in self.obstaculos)

    m = MapaPrueba(10, 10, [])
    robot = (0, 0)
    destinos = {1: (0, 9), 2: (9, 9), 3: (9, 0)}   # bodega, hospital, zapateria

    orden, total = ordenar_mision(m, robot, destinos)
    nombres = {1: "Bodega", 2: "Hospital", 3: "Zapateria"}
    print("Robot en", robot)
    print("Mejor orden:", " -> ".join(nombres[i] for i in orden))
    print("Costo total:", total, "pasos")
