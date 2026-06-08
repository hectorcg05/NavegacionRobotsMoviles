# potential_fields

Paquete de ROS2 para navegación autónoma de un robot diferencial (Puzzlebot) usando el algoritmo de **Campos Potenciales Artificiales (APF)**. El robot navega hacia una secuencia de objetivos evitando obstáculos mediante la suma de fuerzas virtuales atractivas y repulsivas.

---

## Estructura del paquete

```
potential_fields/
├── launch/
│   └── potential_fields.launch.py      # Lanza los 3 nodos
├── potential_fields/
│   ├── differential_odometry.py        # Estimación de pose por encoders
│   ├── waypoint_sequencer.py           # Secuenciador de objetivos
│   └── apf_controller.py              # Controlador de campos potenciales
├── package.xml
└── setup.py
```

---

## Nodos

### `differential_odometry`

Estima la posición y orientación del robot integrando las velocidades de los encoders de ambas ruedas.

**Suscribe:**
| Topic | Tipo | Descripción |
|---|---|---|
| `/VelocityEncR` | `std_msgs/Float32` | Velocidad angular rueda derecha [rad/s] |
| `/VelocityEncL` | `std_msgs/Float32` | Velocidad angular rueda izquierda [rad/s] |

**Publica:**
| Topic | Tipo | Descripción |
|---|---|---|
| `/odom` | `turtlesim/Pose` | Pose estimada del robot (x, y, θ) |

**Parámetros físicos del Puzzlebot:**
- Radio de rueda: `r = 0.0505 m`
- Distancia entre ruedas: `l = 0.183 m`
- Frecuencia de actualización: `100 Hz`

---

### `waypoint_sequencer`

Envía al controlador los objetivos uno a uno. Espera confirmación de llegada antes de enviar el siguiente.

**Publica:**
| Topic | Tipo | Descripción |
|---|---|---|
| `/next_point` | `turtlesim/Pose` | Coordenadas del objetivo actual (x, y) |

**Suscribe:**
| Topic | Tipo | Descripción |
|---|---|---|
| `/arrived` | `std_msgs/Bool` | Señal de llegada al objetivo actual |

**Waypoints configurados** (espacio de trabajo 2×2 m):
```
[2.0, 0.0]  →  [0.0, 2.0]
```

---

### `apf_controller`

Controlador principal. Calcula fuerzas atractivas hacia el objetivo y repulsivas desde los obstáculos, y convierte la fuerza resultante en comandos de velocidad.

**Suscribe:**
| Topic | Tipo | Descripción |
|---|---|---|
| `/odom` | `turtlesim/Pose` | Pose actual del robot |
| `/next_point` | `turtlesim/Pose` | Objetivo actual |

**Publica:**
| Topic | Tipo | Descripción |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocidad lineal y angular al robot |
| `/arrived` | `std_msgs/Bool` | `True` cuando se alcanza el objetivo |

**Parámetros del algoritmo:**
| Parámetro | Valor | Descripción |
|---|---|---|
| `k_att` | 0.8 | Ganancia fuerza atractiva |
| `k_rep` | 0.45 | Ganancia fuerza repulsiva |
| `rep_radius` | 0.3 m | Radio de influencia de obstáculos |
| `Kv` | 0.45 | Ganancia velocidad lineal |
| `Kw` | 1.2 | Ganancia velocidad angular |
| `MAX_LIN_VEL` | 0.16 m/s | Velocidad lineal máxima |
| `MAX_ANG_VEL` | 1.1 rad/s | Velocidad angular máxima |
| `goal_tolerance` | 0.10 m | Radio de aceptación del objetivo |

**Obstáculos configurados:**
```
[0.8, 1.2]
[1.5, 0.8]
```

---

## Arquitectura de comunicación

```
[Encoders físicos]
  /VelocityEncR ──►┐
  /VelocityEncL ──►│ differential_odometry ──► /odom ──────────────┐
                                                                    │
waypoint_sequencer ──────────────────────── /next_point ───────────►│
      ▲                                                             │
      │ /arrived                                               apf_controller
      └────────────────────────────────────────────────────────────┤
                                                                    │
                                                               /cmd_vel
                                                                    │
                                                          [Ruedas del robot]
```

---

## Algoritmo: Campos Potenciales Artificiales

### Fuerza atractiva

Jala al robot hacia el objetivo con magnitud proporcional a la distancia:

```
Fx_att = k_att * (goal_x - x)
Fy_att = k_att * (goal_y - y)
```

### Fuerza repulsiva

Empuja al robot lejos de cada obstáculo. Solo actúa dentro del radio de influencia `rep_radius`:

```
magnitud = k_rep * (1/d - 1/d0) * (1/d²)
```

La fuerza apunta en dirección opuesta al obstáculo.

### Generación de comandos

La fuerza total determina el ángulo deseado. El robot primero se alinea y luego avanza:

```
desired_θ = atan2(Fy_total, Fx_total)
error_θ   = desired_θ - θ_actual

ω = Kw * error_θ                           (siempre)
v = Kv * |F_total| * cos(error_θ)          (solo si |error_θ| < 0.9 rad)
```

---

## Instalación y uso

### Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select potential_fields
source install/setup.bash
```

### Lanzar todos los nodos

```bash
ros2 launch potential_fields potential_fields.launch.py
```

### Lanzar nodos individualmente

```bash
ros2 run potential_fields differential_odometry
ros2 run potential_fields waypoint_sequencer
ros2 run potential_fields apf_controller
```

---

## Dependencias

- `rclpy`
- `geometry_msgs`
- `std_msgs`
- `turtlesim`
- `launch`
- `launch_ros`
