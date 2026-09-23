"""
planner.py
Busqueda de rutas A* sobre el grid.

Movimiento en 4 direcciones (sin diagonales), heuristica Manhattan.
El mapa solo necesita exponer un metodo es_libre((fila, col)) -> bool,
que es justo lo que ya tiene GridMap en mapa.py.
"""

import heapq

# Movimientos: arriba, abajo, izquierda, derecha
MOVIMIENTOS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def heuristica(a, b):
    """Distancia Manhattan: nunca sobreestima con movimiento en 4 dir.
    Esa propiedad (admisibilidad) es la que garantiza el camino optimo."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def vecinos(mapa, celda):
    """Celdas vecinas transitables."""
    r, c = celda
    for dr, dc in MOVIMIENTOS:
        v = (r + dr, c + dc)
        if mapa.es_libre(v):
            yield v


def reconstruir_camino(padre, actual):
    """Recorre hacia atras la cadena de padres para armar la ruta."""
    camino = [actual]
    while actual in padre:
        actual = padre[actual]
        camino.append(actual)
    camino.reverse()
    return camino


def astar(mapa, inicio, meta):
    """
    Devuelve la lista de celdas [inicio, ..., meta] o None si no hay ruta.
    """
    if inicio is None or meta is None:
        return None
    if not mapa.es_libre(meta):
        return None
    if inicio == meta:
        return [inicio]

    # Cola de abiertos: (f, g, celda), donde f = g + h
    abiertos = [(heuristica(inicio, meta), 0, inicio)]
    padre = {}
    g = {inicio: 0}          # costo real desde el inicio
    cerrados = set()         # ya expandidos definitivamente
    # Bucle principal de A*
    while abiertos:
        f_actual, g_actual, actual = heapq.heappop(abiertos)

        if actual == meta:
            return reconstruir_camino(padre, actual)

        if actual in cerrados:
            continue         # entrada vieja de la cola, ignorar
        cerrados.add(actual)
        # buscar vecinos transitables y actualizar sus costos
        for v in vecinos(mapa, actual):
            if v in cerrados:
                continue
            g_nuevo = g_actual + 1          # cada paso cuesta 1 celda
            if g_nuevo < g.get(v, float('inf')):
                g[v] = g_nuevo
                padre[v] = actual
                f = g_nuevo + heuristica(v, meta)
                heapq.heappush(abiertos, (f, g_nuevo, v))

    return None   # se agotaron los abiertos: no hay ruta


def longitud(camino):
    """Numero de pasos de una ruta (para usar como costo en el TSP)."""
    return None if camino is None else len(camino) - 1


# prueba de A* con un mapa de juguete
if __name__ == "__main__":

    class MapaPrueba:
        """Mapa de juguete para verificar el algoritmo aislado."""
        def __init__(self, filas, cols, obstaculos):
            self.filas, self.cols = filas, cols
            self.obstaculos = set(obstaculos)

        def es_libre(self, celda):
            r, c = celda
            return (0 <= r < self.filas and 0 <= c < self.cols
                    and celda not in self.obstaculos)

    # Muro vertical con una abertura en la fila 7
    muro = [(r, 5) for r in range(0, 7)]
    m = MapaPrueba(10, 10, muro)

    inicio, meta = (0, 0), (0, 9)
    ruta = astar(m, inicio, meta)

    print(f"Inicio {inicio} -> Meta {meta}")
    print(f"Pasos: {longitud(ruta)}")
    print(f"Ruta: {ruta}\n")

    # Dibujo en consola
    for r in range(m.filas):
        linea = ""
        for c in range(m.cols):
            celda = (r, c)
            if celda == inicio: linea += " I"
            elif celda == meta: linea += " M"
            elif celda in m.obstaculos: linea += " #"
            elif ruta and celda in ruta: linea += " ."
            else: linea += " _"
        print(linea)
