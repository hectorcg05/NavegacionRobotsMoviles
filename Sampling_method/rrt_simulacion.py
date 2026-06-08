import math
import random
import matplotlib.pyplot as plt
import numpy as np

class Nodo:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.padre = None

class RRT:
    def __init__(self, inicio, meta, obstaculos, area_mapa, paso_expansion=1.0, prob_meta=10):
        self.inicio = Nodo(inicio[0], inicio[1])
        self.meta = Nodo(meta[0], meta[1])
        self.obstaculos = obstaculos # Lista de (x, y, radio)
        self.area_mapa = area_mapa   # [min_x, max_x, min_y, max_y]
        self.paso_expansion = paso_expansion # Qué tanto avanza por cada iteración
        self.prob_meta = prob_meta   # Probabilidad de forzar la muestra hacia la meta
        self.nodos = []

    def planear(self, animacion=True):
        self.nodos = [self.inicio]
        
        for i in range(500): # Máximo de iteraciones
            # 1. Muestra aleatoria (Sampling)
            nodo_random = self.generar_nodo_aleatorio()
            
            # 2. Buscar el nodo más cercano en el árbol actual
            indice_cercano = self.obtener_nodo_mas_cercano(self.nodos, nodo_random)
            nodo_cercano = self.nodos[indice_cercano]
            
            # 3. Crecer el árbol (Steer)
            nodo_nuevo = self.crear_nuevo_nodo(nodo_cercano, nodo_random)
            
            # 4. Revisar Colisiones
            if not self.hay_colision(nodo_nuevo, self.obstaculos):
                self.nodos.append(nodo_nuevo)
                
                # Visualización en vivo
                if animacion:
                    plt.plot(nodo_random.x, nodo_random.y, "^k", markersize=3) # Muestra aleatoria
                    plt.plot([nodo_cercano.x, nodo_nuevo.x], [nodo_cercano.y, nodo_nuevo.y], "-g") # Crecimiento del árbol
                    plt.pause(0.4) # <-- Aumenta este valor para hacer la animación más lenta (ej. 0.1 o 0.5)
                
                # Revisar si ya llegamos a la meta
                distancia_meta = self.calcular_distancia(nodo_nuevo, self.meta)
                if distancia_meta <= self.paso_expansion:
                    print("¡Meta alcanzada!")
                    return self.generar_trayectoria_final(len(self.nodos) - 1)
        
        print("No se encontró ruta en el límite de iteraciones.")
        return None

    def generar_nodo_aleatorio(self):
        # A veces (prob_meta%), mandamos el punto directo a la meta para acelerar la búsqueda
        if random.randint(0, 100) > self.prob_meta:
            x = random.uniform(self.area_mapa[0], self.area_mapa[1])
            y = random.uniform(self.area_mapa[2], self.area_mapa[3])
            return Nodo(x, y)
        return Nodo(self.meta.x, self.meta.y)

    def obtener_nodo_mas_cercano(self, lista_nodos, nodo_objetivo):
        distancias = [(nodo.x - nodo_objetivo.x)**2 + (nodo.y - nodo_objetivo.y)**2 for nodo in lista_nodos]
        return distancias.index(min(distancias))

    def crear_nuevo_nodo(self, nodo_origen, nodo_destino):
        nodo_nuevo = Nodo(nodo_origen.x, nodo_origen.y)
        distancia, angulo = self.calcular_distancia_y_angulo(nodo_nuevo, nodo_destino)
        
        # Solo avanzamos un "paso_expansion" máximo
        nodo_nuevo.x += self.paso_expansion * math.cos(angulo)
        nodo_nuevo.y += self.paso_expansion * math.sin(angulo)
        nodo_nuevo.padre = nodo_origen
        return nodo_nuevo

    def hay_colision(self, nodo, obstaculos):
        # El obstaculo se define como (x, y, radio)
        for (ox, oy, radio) in obstaculos:
            distancia_al_centro = math.sqrt((nodo.x - ox)**2 + (nodo.y - oy)**2)
            if distancia_al_centro <= radio + 0.5: # + 0.5 de margen de seguridad para el robot
                # Visualizar la colisión intentada en rojo
                plt.plot(nodo.x, nodo.y, "xr") 
                return True # Hubo choque
        return False # Camino libre

    def generar_trayectoria_final(self, indice_meta):
        trayectoria = [[self.meta.x, self.meta.y]]
        nodo_actual = self.nodos[indice_meta]
        while nodo_actual.padre is not None:
            trayectoria.append([nodo_actual.x, nodo_actual.y])
            nodo_actual = nodo_actual.padre
        trayectoria.append([self.inicio.x, self.inicio.y])
        return trayectoria

    def calcular_distancia_y_angulo(self, nodo_origen, nodo_destino):
        dx = nodo_destino.x - nodo_origen.x
        dy = nodo_destino.y - nodo_origen.y
        distancia = math.sqrt(dx**2 + dy**2)
        angulo = math.atan2(dy, dx)
        return distancia, angulo

    def calcular_distancia(self, nodo_a, nodo_b):
        return math.sqrt((nodo_a.x - nodo_b.x)**2 + (nodo_a.y - nodo_b.y)**2)

# ==========================================
# Script principal para correr la simulación
# ==========================================
if __name__ == '__main__':
    # Configuración del mapa
    inicio = [0, 0]
    meta = [10, 10]
    # Lista de obstáculos: (x, y, radio)
    obstaculos = [
        (4, 4, 1.5), 
        (7, 7, 2), 
        (3, 8, 1.5), 
        (8, 2, 1)
    ]
    area_mapa = [-2, 12, -2, 12]

    print("Iniciando Simulación RRT...")
    
    # Preparar la gráfica
    plt.figure(figsize=(8,8))
    plt.xlim(area_mapa[0], area_mapa[1])
    plt.ylim(area_mapa[2], area_mapa[3])
    plt.plot(inicio[0], inicio[1], "ob", markersize=10, label="Inicio")
    plt.plot(meta[0], meta[1], "xg", markersize=10, label="Meta")
    
    # Dibujar obstáculos
    for (ox, oy, radio) in obstaculos:
        circulo = plt.Circle((ox, oy), radio, color='gray', fill=True)
        plt.gca().add_patch(circulo)
        
    plt.title("Simulación RRT (Sampling Method)")
    plt.grid(True)

    # Correr el algoritmo
    rrt = RRT(inicio=inicio, meta=meta, obstaculos=obstaculos, area_mapa=area_mapa)
    trayectoria = rrt.planear(animacion=True)

    # Dibujar trayectoria final
    if trayectoria:
        plt.plot([x for (x, y) in trayectoria], [y for (x, y) in trayectoria], '-r', linewidth=3, label="Trayectoria Final")
        plt.legend()
        plt.show()