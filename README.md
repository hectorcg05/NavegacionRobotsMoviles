# Navegación Autónoma para Robot Diferencial (Puzzlebot)

Este repositorio contiene tres estrategias diferentes para que un robot móvil (Puzzlebot con Jetson Orin Nano y ROS2) pueda navegar de un punto a otro esquivando obstáculos. 

El proyecto cubre dos simulaciones en computadora para analizar algoritmos de búsqueda, y la prueba física en el laboratorio usando navegación reactiva.

## 📁 Estructura de las Carpetas

* **`Grid_method/`**: Simulación cuadriculando el mapa (Algoritmo A*).
* **`Sampling_method/`**: Simulación buscando rutas con puntos aleatorios (Algoritmo RRT).
* **`potential_fields/`**: Paquete de ROS2 para que el robot real navegue esquivando obstáculos.

---

## 💻 1. Grid Method (Simulación con A*)
Este método divide el mapa en una cuadrícula (como un tablero de ajedrez) y evalúa cuál es el camino más corto paso a paso, guiándose por la distancia en línea recta hacia la meta.

* **Archivos:** `grid_simulacion.py`, captura del mapa inicial y video de demostración.
* **Ventajas/Desventajas:** Es muy preciso para encontrar la ruta ideal. Sin embargo, si el mapa es gigante o los cuadritos muy pequeños (resolución alta), la computadora se vuelve lenta porque tiene que analizar demasiadas opciones.
* **Para correrlo:** `python grid_simulacion.py` (Requiere `numpy` y `matplotlib`).

## 💻 2. Sampling Method (Simulación con RRT)
En lugar de cuadricular todo el espacio, este algoritmo (Rapidly-exploring Random Tree) "lanza" puntos al azar por todo el mapa y los va conectando como si fueran las ramas de un árbol hasta chocar con la meta.

* **Archivos:** `rrt_simulacion.py`, captura del mapa inicial y video de demostración.
* **Ventajas/Desventajas:** Es súper rápido en espacios abiertos y grandes porque no analiza cada centímetro del mapa. La desventaja es que la ruta que encuentra casi nunca es la más corta ni la más bonita.
* **Para correrlo:** `python rrt_simulacion.py` (Requiere `numpy` y `matplotlib`).

---

## 🤖 3. Campos Potenciales Virtuales (Robot Físico con ROS2)
Aquí pasamos al robot real. Usamos ROS2 para que el Puzzlebot llegue a varios objetivos de forma autónoma. Imagina que la meta es un imán que atrae al robot, y los obstáculos son imanes del mismo polo que lo empujan lejos.

### ¿Cómo toma decisiones el robot?
* **Fuerza Atractiva:** Jala al robot hacia la meta.
* **Fuerza Repulsiva:** Empuja al robot lejos de las cajas/paredes. Solo se activa si el robot entra en la "zona de peligro" (radio de influencia) del obstáculo.
* **Movimiento:** El código suma estas dos fuerzas. El resultado (el vector total) le dice al robot hacia dónde debe apuntar y qué tan rápido debe girar las llantas para avanzar seguro.

### Nodos de ROS2 incluidos
1. **`differential_odometry`**: Calcula dónde está el robot (x, y, ángulo) contando las vueltas de las llantas (encoders físicos).
2. **`waypoint_sequencer`**: Le va pasando al controlador los puntos a los que tiene que llegar, uno por uno, esperando a que llegue al primero para soltar el segundo.
3. **`apf_controller`**: Es el cerebro principal. Suscribe la posición actual y la meta, calcula las fuerzas (atracción + repulsión) y le manda las instrucciones de velocidad al robot (`/cmd_vel`).

### Diagrama de comunicación
```text
[Encoders] -> /VelocityEnc -> differential_odometry -> /odom -> apf_controller -> /cmd_vel -> [Ruedas]
                                                                      ^
[Lista de puntos] ----------> waypoint_sequencer -> /next_point ------|
```

### Instrucciones para correrlo en el robot
Primero compila el paquete dentro de tu espacio de trabajo de ROS2:
```bash
cd ~/ros2_ws
colcon build --packages-select potential_fields
source install/setup.bash
```

Para lanzar todo el sistema (odometría, secuenciador y controlador) con un solo comando:
```bash
ros2 launch potential_fields potential_fields.launch.py
```
*(También puedes correr los nodos individualmente usando `ros2 run potential_fields <nombre_del_nodo>`)*.