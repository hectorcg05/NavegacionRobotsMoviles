import numpy as np
import matplotlib.pyplot as plt
import heapq
import math

class NodoGrid:
    def __init__(self, x, y, costo_g, costo_f, padre):
        self.x = x
        self.y = y
        self.costo_g = costo_g  # Costo desde el inicio
        self.costo_f = costo_f  # Costo total (g + heuristica)
        self.padre = padre

    # Para que heapq sepa cómo comparar nodos (el de menor costo F gana)
    def __lt__(self, otro):
        return self.costo_f < otro.costo_f

class AStarGrid:
    def __init__(self, inicio, meta, obstaculos, tamaño_x, tamaño_y, resolucion):
        self.inicio = [int(inicio[0]/resolucion), int(inicio[1]/resolucion)]
        self.meta = [int(meta[0]/resolucion), int(meta[1]/resolucion)]
        self.tamaño_x = int(tamaño_x / resolucion)
        self.tamaño_y = int(tamaño_y / resolucion)
        self.resolucion = resolucion
        self.mapa_grid = self.crear_grid(obstaculos)

        # Movimientos permitidos (N, S, E, O, y Diagonales)
        self.movimientos = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                            [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)], 
                            [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]

    def crear_grid(self, obstaculos):
        # Creamos una matriz llena de ceros (libre)
        grid = np.zeros((self.tamaño_x, self.tamaño_y))
        
        # Llenamos con unos (1) las celdas donde caen los obstáculos
        for (ox, oy, radio) in obstaculos:
            r_grid = int(math.ceil(radio / self.resolucion))
            cx = int(ox / self.resolucion)
            cy = int(oy / self.resolucion)
            
            for i in range(-r_grid, r_grid + 1):
                for j in range(-r_grid, r_grid + 1):
                    # Efecto de Resolución: Si el radio cae un poco dentro del cuadrito, lo marcamos todo.
                    if math.sqrt(i**2 + j**2) <= r_grid:
                        if 0 <= cx + i < self.tamaño_x and 0 <= cy + j < self.tamaño_y:
                            grid[cx + i][cy + j] = 1 # 1 es pared
        return grid

    def calcular_heuristica(self, x, y):
        # Distancia Euclidiana (línea recta) a la meta. 
        return math.sqrt((x - self.meta[0])**2 + (y - self.meta[1])**2)

    def planear(self):

        lista_abierta = []

        nodos_visitados = dict()
        
        # Crear nodo inicial
        nodo_inicio = NodoGrid(self.inicio[0], self.inicio[1], 0.0, 0.0, None)
        heapq.heappush(lista_abierta, nodo_inicio)
        
        nodos_expandidos_x = []
        nodos_expandidos_y = []

        while len(lista_abierta) > 0:
            # 1. Sacar el nodo con el menor costo F
            nodo_actual = heapq.heappop(lista_abierta)
            id_actual = f"{nodo_actual.x},{nodo_actual.y}"

            if id_actual in nodos_visitados:
                continue
            
            nodos_visitados[id_actual] = nodo_actual
            nodos_expandidos_x.append(nodo_actual.x * self.resolucion)
            nodos_expandidos_y.append(nodo_actual.y * self.resolucion)

            # 2. ¿Llegamos a la meta?
            if nodo_actual.x == self.meta[0] and nodo_actual.y == self.meta[1]:
                print("¡Meta encontrada con A*!")
                return self.generar_trayectoria(nodo_actual), nodos_expandidos_x, nodos_expandidos_y

            # 3. Expandir vecinos
            for mov in self.movimientos:
                vecino_x = nodo_actual.x + mov[0]
                vecino_y = nodo_actual.y + mov[1]
                costo_paso = mov[2]

                # Revisar límites del mapa
                if not (0 <= vecino_x < self.tamaño_x and 0 <= vecino_y < self.tamaño_y):
                    continue
                
                # Revisar colisiones (si la celda es un 1)
                if self.mapa_grid[vecino_x][vecino_y] == 1:
                    continue
                
                id_vecino = f"{vecino_x},{vecino_y}"
                if id_vecino in nodos_visitados:
                    continue

                # g = costo acumulado de llegar al vecino
                costo_g_nuevo = nodo_actual.costo_g + costo_paso
                # h = heurística del vecino a la meta
                costo_h_nuevo = self.calcular_heuristica(vecino_x, vecino_y)
                # f = costo total
                costo_f_nuevo = costo_g_nuevo + costo_h_nuevo

                nodo_vecino = NodoGrid(vecino_x, vecino_y, costo_g_nuevo, costo_f_nuevo, nodo_actual)
                heapq.heappush(lista_abierta, nodo_vecino)

        print("No se encontró ruta en el Grid.")
        return None, nodos_expandidos_x, nodos_expandidos_y

    def generar_trayectoria(self, nodo_meta):
        rx, ry = [], []
        nodo_actual = nodo_meta
        while nodo_actual.padre is not None:
            rx.append(nodo_actual.x * self.resolucion)
            ry.append(nodo_actual.y * self.resolucion)
            nodo_actual = nodo_actual.padre
        rx.append(self.inicio[0] * self.resolucion)
        ry.append(self.inicio[1] * self.resolucion)
        # Invertimos para que la ruta vaya del inicio a la meta
        return [rx[::-1], ry[::-1]]

if __name__ == '__main__':
    # Mismos parámetros que el Sampling Method para poder compararlos
    inicio = [1.0, 4.0]
    meta = [9.0, 9.0]
    obstaculos = [
        (4, 4, 1.5), 
        (7, 7, 2), 
        (3, 8, 1.5), 
        (8, 2, 1)
    ]

    resolucion = 0.5 
    
    astar = AStarGrid(inicio, meta, obstaculos, 12.0, 12.0, resolucion)
    trayectoria, exp_x, exp_y = astar.planear()


    plt.figure(figsize=(8,8))
    
    # 1. Mostrar Mapa Discretizado (Cuadriculado)
    plt.pcolor(np.arange(0, 12, resolucion), np.arange(0, 12, resolucion), 
               astar.mapa_grid.T, cmap='Greys', vmin=0, vmax=1, alpha=0.3)
    
    # 2. Mostrar Expansión de Nodos
    plt.plot(exp_x, exp_y, 'xc', markersize=4, label='Nodos Expandidos')
    
    plt.plot(inicio[0], inicio[1], "ob", markersize=10, label="Inicio")
    plt.plot(meta[0], meta[1], "xg", markersize=10, label="Meta")
    
    # 3. Mostrar Trayectoria y Movimiento del Robot
    if trayectoria:
        # Dibujamos la trayectoria final
        plt.plot(trayectoria[0], trayectoria[1], "-r", linewidth=3, label="Trayectoria Final")
        plt.legend()
        plt.title("Simulación Grid Method (A*)")
        

        for i in range(len(trayectoria[0])):
            plt.plot(trayectoria[0][i], trayectoria[1][i], "om", markersize=8)
            plt.pause(0.4)
    
    plt.show()
