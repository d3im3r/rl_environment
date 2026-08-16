# Desarrollo de mundos personalizados y entorno RL para TurtleBot3 en ROS 2 Humble

## 1. Descripción general

Este documento describe el desarrollo completo realizado hasta el momento para construir un entorno de simulación en Gazebo Classic con TurtleBot3 en ROS 2 Humble, orientado al entrenamiento de agentes de Aprendizaje por Refuerzo, específicamente usando una arquitectura DQN.

El proyecto se dividió en dos paquetes principales:

```text
turtlebot3_custom_worlds
turtlebot3_rl_training
````

El primer paquete contiene los mundos personalizados, modelos visuales, launch files y herramientas de reinicio de episodios. El segundo paquete contiene la lógica del entrenamiento RL, la interfaz con ROS/Gazebo, el controlador de acciones discretas, el entorno tipo Gym, el núcleo DQN, el logger de episodios, el renderizador de videos y las herramientas para graficar métricas.

La arquitectura general es:

```text
Gazebo Classic
    ↓
TurtleBot3 Burger
    ↓
/scan, /odom, /gazebo/model_states
    ↓
rl_interface_node
    ↓
/rl_state, /rl_goal_reached
    ↓
GazeboTurtleBot3Env
    ↓
DQN
    ↓
/rl_action
    ↓
rl_motion_controller
    ↓
/cmd_vel
    ↓
TurtleBot3 en Gazebo
```

---

# 2. Paquete `turtlebot3_custom_worlds`

## 2.1 Objetivo del paquete

El paquete `turtlebot3_custom_worlds` se creó para centralizar todos los mundos personalizados de Gazebo utilizados con TurtleBot3.

Su objetivo es permitir cargar escenarios progresivos de navegación mediante un argumento corto:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1
```

Este paquete contiene:

```text
- Mundos personalizados por etapas.
- Cuadrícula métrica de referencia.
- Ejes positivos visibles.
- Meta visual movible.
- Obstáculos y paredes definidos directamente en los archivos .world.
- Launch principal para cargar Gazebo y TurtleBot3.
- Script de gestión de episodios.
```

---

## 2.2 Estructura del paquete

La estructura actual del paquete es:

```text
turtlebot3_custom_worlds/
├── CMakeLists.txt
├── config
│   └── d3im3r_stage_02_front_obstacle.yaml
├── include
│   └── turtlebot3_custom_worlds
├── launch
│   └── d3im3r_world.launch.py
├── models
│   ├── d3im3r_goal_marker
│   │   ├── model.config
│   │   └── model.sdf
│   └── d3im3r_grid
│       ├── model.config
│       └── model.sdf
├── package.xml
├── README.md
├── scripts
│   └── rl_episode_manager.py
├── src
└── worlds
    ├── d3im3r_stage_00_empty.world
    ├── d3im3r_stage_01_direct_goal.world
    ├── d3im3r_stage_02_front_obstacle.world
    ├── d3im3r_stage_03_left_right_choice.world
    ├── d3im3r_stage_04_corridor.world
    ├── d3im3r_stage_05_narrow_door.world
    ├── d3im3r_stage_06_random_obstacles.world
    └── d3im3r_stage_07_simple_maze.world
```

Una corrección importante realizada fue limpiar la carpeta `models/`, eliminando archivos sueltos `model.sdf` y `model.config` que estaban directamente dentro de `models/`.

La estructura correcta para modelos de Gazebo es:

```text
models/
├── d3im3r_grid/
│   ├── model.config
│   └── model.sdf
└── d3im3r_goal_marker/
    ├── model.config
    └── model.sdf
```

Cada modelo debe tener su propia carpeta.

---

# 3. Convención del mundo

Se definió una convención clara para todos los mundos:

```text
+X  -> dirección principal hacia adelante
+Y  -> dirección lateral izquierda
+Z  -> dirección vertical hacia arriba
yaw = 0 -> robot mirando hacia +X
```

El plano principal mide:

```text
10 m x 10 m
```

Esto significa que el mundo va aproximadamente desde:

```text
x = -5 m hasta x = 5 m
y = -5 m hasta y = 5 m
```

Cada cuadro de la cuadrícula representa:

```text
1 cuadro = 1 metro
```

La pose inicial recomendada del robot es:

```text
x = -1.5
y =  0.0
z =  0.01
yaw = 0.0
```

La meta inicial típica es:

```text
x = 1.5
y = 0.0
z = 0.03
```

---

# 4. Cuadrícula métrica personalizada

Se creó un modelo llamado:

```text
d3im3r_grid
```

Ubicado en:

```text
models/d3im3r_grid/
```

La cuadrícula tiene:

```text
Tamaño: 10 m x 10 m
Separación: 1 m
```

Se configuró visualmente para mostrar:

```text
Eje +X -> rojo
Eje +Y -> verde
Resto de líneas -> gris oscuro
```

La decisión final fue mostrar solo los ejes positivos como una especie de “L” en el origen, para no saturar visualmente el entorno.

Esto permite identificar rápidamente:

```text
Origen: (0, 0)
Dirección +X: línea roja
Dirección +Y: línea verde
```

Ejemplos de interpretación:

```text
(-1.5, 0.0) -> 1.5 m hacia el lado negativo de X
( 1.5, 0.0) -> 1.5 m hacia el lado positivo de X
( 0.0, 1.0) -> 1.0 m hacia el lado positivo de Y
( 0.0,-1.0) -> 1.0 m hacia el lado negativo de Y
```

---

# 5. Mundos personalizados por stages

Se definieron varios mundos progresivos para entrenamiento, evaluación y pruebas.

## Stage 00: mundo vacío

Archivo:

```text
worlds/d3im3r_stage_00_empty.world
```

Objetivo:

```text
Validar que Gazebo, TurtleBot3, /cmd_vel, /odom y /scan funcionen correctamente.
```

Contenido:

```text
- Plano gris.
- Cuadrícula.
- TurtleBot3.
- Sin meta.
- Sin obstáculos.
```

Ejecución:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=0
```

---

## Stage 01: meta directa

Archivo:

```text
worlds/d3im3r_stage_01_direct_goal.world
```

Objetivo:

```text
Validar navegación directa hacia una meta sin obstáculos.
```

Contenido:

```text
- Robot en (-1.5, 0.0).
- Meta en (1.5, 0.0).
- Sin obstáculos.
```

Uso recomendado:

```text
- Primer entrenamiento RL.
- Validación de estado.
- Validación de recompensa.
- Validación del cálculo de ángulo y distancia a la meta.
```

Ejecución:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1
```

---

## Stage 02: obstáculo frontal

Archivo:

```text
worlds/d3im3r_stage_02_front_obstacle.world
```

Objetivo:

```text
Evaluar evasión básica de un obstáculo ubicado entre el robot y la meta.
```

Contenido:

```text
- Meta en el eje X positivo.
- Obstáculo frontal cerca del centro.
```

Uso recomendado:

```text
- Evasión simple.
- Pruebas de política con detección frontal.
- Reglas difusas básicas.
```

Ejecución:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=2
```

---

## Stage 03: decisión izquierda/derecha

Archivo:

```text
worlds/d3im3r_stage_03_left_right_choice.world
```

Objetivo:

```text
Obligar al robot a decidir entre trayectorias laterales.
```

Contenido:

```text
- Dos obstáculos simétricos.
- Meta al frente.
```

Uso recomendado:

```text
- Evaluar acciones de giro.
- Comparar izquierda vs derecha.
- Entrenar decisiones más complejas que avanzar recto.
```

---

## Stage 04: corredor

Archivo:

```text
worlds/d3im3r_stage_04_corridor.world
```

Objetivo:

```text
Evaluar navegación dentro de un pasillo.
```

Contenido:

```text
- Dos paredes paralelas.
- Meta al final del corredor.
```

Uso recomendado:

```text
- Control de orientación.
- Penalización por cercanía a paredes.
- Navegación estable.
```

---

## Stage 05: chicane o desvío controlado

Archivo:

```text
worlds/d3im3r_stage_05_narrow_door.world
```

Inicialmente se intentó crear una puerta angosta, pero se detectó que las aperturas eran demasiado grandes o poco útiles para entrenamiento.

Se reemplazó por un escenario tipo chicane:

```text
- Dos paredes desplazadas.
- Un obstáculo lateral.
- Meta final.
```

Objetivo:

```text
Obligar al robot a realizar un desvío suave antes de llegar a la meta.
```

Uso recomendado:

```text
- Evaluar corrección de trayectoria.
- Evitar navegación puramente recta.
- Entrenar políticas más robustas.
```

---

## Stage 06: obstáculos distribuidos

Archivo:

```text
worlds/d3im3r_stage_06_random_obstacles.world
```

Objetivo:

```text
Evaluar navegación con varios obstáculos distribuidos.
```

Contenido:

```text
- Varios bloques.
- Meta desplazada del eje X.
```

Uso recomendado:

```text
- Generalización.
- Evaluación de políticas entrenadas.
- Comparación entre distintos métodos.
```

---

## Stage 07: laberinto simple

Archivo:

```text
worlds/d3im3r_stage_07_simple_maze.world
```

Objetivo:

```text
Evaluar navegación más exigente con paredes internas.
```

Contenido:

```text
- Paredes internas.
- Meta en zona superior derecha.
```

Uso recomendado:

```text
- Evaluación final.
- Validación de políticas más maduras.
- Comparación experimental.
```

---

# 6. Meta movible para RL

Para entrenamiento RL se necesita mover la meta durante los episodios.

Se detectó que si la meta se define como:

```xml
<static>true</static>
```

Gazebo permite visualizarla, pero no siempre permite moverla con:

```text
/gazebo/set_entity_state
```

Por eso se modificó la meta para que sea:

```xml
<static>false</static>
```

Bloque recomendado:

```xml
<model name="goal_marker">
    <static>false</static>

    <pose>1.5 0.0 0.03 0 0 0</pose>

    <link name="link">
        <gravity>false</gravity>
        <kinematic>true</kinematic>

        <visual name="visual">
            <geometry>
                <cylinder>
                    <radius>0.18</radius>
                    <length>0.02</length>
                </cylinder>
            </geometry>
            <material>
                <ambient>0.0 1.0 0.0 1.0</ambient>
                <diffuse>0.0 1.0 0.0 1.0</diffuse>
            </material>
        </visual>
    </link>
</model>
```

Características:

```text
<static>false</static>      -> permite mover la meta
<gravity>false</gravity>    -> evita que caiga
<kinematic>true</kinematic> -> permite controlarla por pose
sin collision               -> el robot no choca con ella
z = 0.03                    -> queda visible sobre el piso
```

---

# 7. Plugin `gazebo_ros_state`

Para poder mover entidades y leer estados de modelos se agregó en los mundos el plugin:

```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros>
        <namespace>/gazebo</namespace>
    </ros>
    <update_rate>20.0</update_rate>
</plugin>
```

Este plugin permite disponer de:

```text
/gazebo/model_states
/gazebo/link_states
/gazebo/get_entity_state
/gazebo/set_entity_state
```

Servicios importantes:

```bash
ros2 service list | grep entity
```

Resultado esperado:

```text
/gazebo/get_entity_state
/gazebo/set_entity_state
```

---

# 8. Launch principal de mundos

El archivo principal es:

```text
launch/d3im3r_world.launch.py
```

Este launch hace lo siguiente:

```text
- Define el modelo TurtleBot3 Burger.
- Carga el mundo según stage:=N.
- Configura GAZEBO_MODEL_PATH.
- Desactiva la base de datos externa de Gazebo.
- Inserta TurtleBot3 usando el modelo SDF oficial.
- Permite configurar pose inicial.
```

Ejemplo básico:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1
```

Ejemplo con pose personalizada:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py \
stage:=5 \
x_pose:=-1.5 \
y_pose:=0.0 \
z_pose:=0.01 \
yaw:=0.0
```

También se recomendó agregar ejecución sin GUI para acelerar entrenamientos:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1 gui:=false
```

La GUI se puede activar para demostraciones:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1 gui:=true
```

---

# 9. Script `rl_episode_manager.py`

Se creó un script inicial para reiniciar episodios:

```text
scripts/rl_episode_manager.py
```

Este script permite:

```text
- Detener el robot.
- Reiniciar simulación.
- Reubicar TurtleBot3.
- Reubicar la meta.
- Probar cambios dinámicos de meta.
```

Luego se instaló como ejecutable de ROS 2 mediante `CMakeLists.txt`.

Ahora puede ejecutarse con:

```bash
ros2 run turtlebot3_custom_worlds rl_episode_manager
```

En `CMakeLists.txt` se agregó:

```cmake
install(
    PROGRAMS
        scripts/rl_episode_manager.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME rl_episode_manager
)
```

---

# 10. Paquete `turtlebot3_rl_training`

## 10.1 Objetivo

El paquete `turtlebot3_rl_training` contiene toda la lógica de entrenamiento DQN y conexión con ROS/Gazebo.

Su función es separar:

```text
turtlebot3_custom_worlds -> mundos, modelos y launch
turtlebot3_rl_training   -> entrenamiento, RL, logs, videos y métricas
```

---

## 10.2 Estructura del paquete

La estructura desarrollada es:

```text
turtlebot3_rl_training/
├── config/
│   └── training_config.yaml
├── package.xml
├── resource/
│   └── turtlebot3_rl_training
├── setup.cfg
├── setup.py
├── training_runs/
└── turtlebot3_rl_training/
    ├── __init__.py
    ├── dqn_core.py
    ├── episode_logger.py
    ├── gazebo_rl_env.py
    ├── plot_training_metrics.py
    ├── rl_interface_node.py
    ├── rl_motion_controller.py
    ├── run_manager.py
    ├── test_gazebo_env.py
    ├── train_dqn_ros.py
    ├── evaluate_dqn_ros.py
    └── video_renderer.py
```

---

# 11. Núcleo DQN: `dqn_core.py`

El archivo:

```text
dqn_core.py
```

contiene los componentes principales del algoritmo DQN:

```text
- QNetwork
- ReplayBuffer
- Transition
- select_action
- train_step
- save_checkpoint
- load_q_network_from_checkpoint
```

## 11.1 QNetwork

La red recibe un estado de dimensión 5:

```text
[d_front_norm, d_left_norm, d_right_norm, theta_goal_norm, d_goal_norm]
```

y devuelve 3 valores Q:

```text
Q(s, avanzar)
Q(s, girar izquierda)
Q(s, girar derecha)
```

La arquitectura es:

```text
Linear(state_dim, 128)
ReLU
Linear(128, 128)
ReLU
Linear(128, action_dim)
```

## 11.2 ReplayBuffer

El buffer almacena transiciones:

```text
state
action
reward
next_state
done
```

Permite muestrear lotes aleatorios para romper correlación temporal durante el entrenamiento.

## 11.3 Selección de acción

Se usa política epsilon-greedy:

```text
Con probabilidad epsilon -> acción aleatoria
Con probabilidad 1-epsilon -> acción con mayor Q
```

Esto permite exploración al inicio y explotación al final.

## 11.4 train_step

Realiza la actualización DQN:

```text
Q(s,a) -> red principal
target = r + gamma * max Q_target(s', a')
loss = MSE(Q(s,a), target)
```

La red objetivo se actualiza cada cierto número de episodios.

---

# 12. Interfaz RL: `rl_interface_node.py`

Este nodo conecta Gazebo/TurtleBot3 con el estado usado por RL.

Lee:

```text
/scan
/odom
/gazebo/model_states
```

Publica:

```text
/rl_state
/rl_goal_reached
```

## 12.1 Estado RL publicado

El estado tiene 5 elementos:

```text
[
    d_front_norm,
    d_left_norm,
    d_right_norm,
    theta_goal_norm,
    d_goal_norm
]
```

Donde:

```text
d_front_norm  -> distancia frontal normalizada
d_left_norm   -> distancia a 45° izquierda normalizada
d_right_norm  -> distancia a 45° derecha normalizada
theta_goal_norm -> error angular hacia la meta dividido entre pi
d_goal_norm   -> distancia a la meta normalizada
```

El tópico es:

```text
/rl_state
```

Tipo:

```text
std_msgs/msg/Float32MultiArray
```

## 12.2 Llegada a la meta

El nodo calcula si el robot llegó a la meta usando:

```text
d_goal <= goal_tolerance
```

Publica:

```text
/rl_goal_reached
```

Tipo:

```text
std_msgs/msg/Bool
```

---

# 13. Controlador de acciones: `rl_motion_controller.py`

Este nodo convierte acciones discretas en movimientos reales del TurtleBot3.

Se suscribe a:

```text
/rl_action
```

Tipo:

```text
std_msgs/msg/Int32
```

Publica:

```text
/cmd_vel
/rl_action_done
```

## 13.1 Acciones discretas

Se definieron 3 acciones:

```text
0 -> avanzar
1 -> girar izquierda
2 -> girar derecha
```

## 13.2 Acción 0: avanzar

El robot avanza usando odometría.

Cuando recibe acción 0, guarda:

```text
x inicial
y inicial
```

Luego calcula:

```text
distancia recorrida = sqrt((x_actual - x_inicial)^2 + (y_actual - y_inicial)^2)
```

Se detiene cuando:

```text
distancia recorrida >= forward_distance
```

Valor actual recomendado:

```text
forward_distance = 0.20 m
```

## 13.3 Acción 1: girar izquierda

Guarda el yaw inicial y calcula:

```text
delta_yaw = yaw_actual - yaw_inicial
```

Se detiene cuando:

```text
delta_yaw >= turn_angle
```

Se modificó el giro de 15 grados a 10 grados:

```python
self.declare_parameter('turn_angle', math.pi / 18.0)
```

## 13.4 Acción 2: girar derecha

Similar al giro izquierdo, pero con velocidad angular negativa.

Se detiene cuando:

```text
delta_yaw <= -turn_angle
```

## 13.5 Tópico `/rl_action_done`

Se añadió:

```text
/rl_action_done
```

Tipo:

```text
std_msgs/msg/Bool
```

La lógica es:

```text
Al recibir acción válida:
    publica /rl_action_done = false

Al completar la acción:
    publica /rl_action_done = true

Si ocurre timeout:
    detiene robot
    publica /rl_action_done = true
```

Esto evita que el entorno RL mande una nueva acción mientras la anterior sigue activa.

---

# 14. Entorno ROS/Gazebo: `gazebo_rl_env.py`

Este archivo implementa una interfaz tipo Gym:

```python
state = env.reset()
next_state, reward, done, info = env.step(action)
```

## 14.1 Función `reset()`

La función `reset()` realiza:

```text
1. Detiene el robot.
2. Reinicia la simulación.
3. Reubica TurtleBot3.
4. Reubica la meta.
5. Espera /rl_state.
6. Retorna estado inicial.
```

Usa servicios:

```text
/reset_simulation
/gazebo/set_entity_state
```

## 14.2 Función `step(action)`

La función `step()` realiza:

```text
1. Guarda estado actual.
2. Publica acción en /rl_action.
3. Espera /rl_action_done = true.
4. Lee nuevo /rl_state.
5. Verifica colisión.
6. Verifica llegada a meta.
7. Calcula reward.
8. Calcula done.
9. Retorna next_state, reward, done, info.
```

## 14.3 Timeout de acción

El entorno tiene un timeout máximo:

```python
action_execution_time = 5.0
```

Este timeout no representa la duración normal de una acción, sino el tiempo máximo que el entorno esperará a que el controlador publique:

```text
/rl_action_done = true
```

El controlador tiene su propio timeout interno:

```text
action_timeout = 4.0 s
```

La configuración recomendada es:

```text
rl_motion_controller action_timeout = 4.0 s
gazebo_rl_env action_execution_time = 5.0 s
```

---

# 15. Recompensa

La recompensa usada en `gazebo_rl_env.py` combina:

```text
- Progreso hacia la meta.
- Penalización por paso.
- Penalización por error angular.
- Penalización por retroceder respecto a la meta.
- Penalización por colisión.
- Recompensa por llegar a la meta.
```

La fórmula conceptual es:

```text
reward =
    progress_gain * (d_goal_prev - d_goal_next)
    - step_penalty
    - heading_penalty * abs(theta_goal_next)
    - backward_penalty si se aleja
    + collision_penalty si choca
    + goal_reward si llega
```

Valores usados:

```text
reward_progress_gain = 8.0
reward_step_penalty = 0.05
reward_heading_penalty = 0.10
reward_backward_penalty = 0.50
reward_collision = -100.0
reward_goal = 150.0
```

---

# 16. Logger de episodios: `episode_logger.py`

Este archivo guarda trayectorias de episodios en formato JSON.

Cada episodio contiene:

```text
metadata
summary
trajectory
```

Ejemplo:

```json
{
    "metadata": {
        "training_id": "stage_1_2026_05_22_101530",
        "episode_id": 25,
        "stage": 1,
        "eval_id": 1,
        "goal": {
            "x": 1.5,
            "y": 0.0
        }
    },
    "summary": {
        "success": true,
        "collision": false,
        "timeout": false,
        "total_reward": 120.5,
        "steps": 32
    },
    "trajectory": [
        {
            "step": 0,
            "x": -1.5,
            "y": 0.0,
            "yaw": 0.0,
            "goal_x": 1.5,
            "goal_y": 0.0,
            "action": 0,
            "reward": 0.0,
            "total_reward": 0.0,
            "done": false,
            "goal_reached": false,
            "collision": false,
            "state": [1.0, 1.0, 1.0, 0.0, 0.6]
        }
    ]
}
```

Nombres estándar:

```text
episode_0025_eval_01.json
episode_0025_eval_01.mp4
episode_0025_eval_01.gif
```

---

# 17. Renderizador de episodios: `video_renderer.py`

Este script genera videos MP4 o GIF desde los archivos JSON.

Ejecutable:

```bash
ros2 run turtlebot3_rl_training render_episode_video \
--input <archivo_json> \
--output <archivo_salida> \
--format mp4 \
--fps 8
```

## 17.1 Diseño visual final

El diseño final incluye:

```text
- Obstáculos en gris oscuro.
- Meta como estrella verde.
- Robot como círculo azul.
- Flecha negra indicando orientación.
- Trayectoria azul.
- Caja de texto auto-ubicable.
- Título con stage, episode y training_id.
```

La caja de texto contiene:

```text
Step
Action
Reward total
Goal reached
Collision
Steps
Summary reward
Status
```

El estado se dejó como texto simple al final:

```text
Status: SUCCESS
Status: COLLISION
Status: TIMEOUT
```

Esto se decidió porque el intento de colorear la palabra `SUCCESS` generaba superposición visual dentro del cuadro de texto.

---

# 18. Gestión de corridas: `run_manager.py`

Se agregó una lógica para crear carpetas con fecha y hora.

Formato de `training_id`:

```text
run_YYYY_MM_DD_HHMMSS
```

Para entrenamientos por stage se usa:

```text
stage_1_YYYY_MM_DD_HHMMSS
```

Estructura generada:

```text
training_runs/
└── stage_1_2026_05_22_101530/
    ├── config.json
    ├── metrics.csv
    ├── summary.json
    ├── checkpoints/
    │   ├── best_model.pth
    │   └── last_model.pth
    ├── episodes/
    │   ├── episode_0025_eval_01.json
    │   ├── episode_0025_eval_01.mp4
    │   └── episode_0025_eval_01.gif
    ├── plots/
    └── bags/
```

---

# 19. Entrenamiento DQN: `train_dqn_ros.py`

Este es el script principal de entrenamiento.

Se ejecuta con:

```bash
ros2 run turtlebot3_rl_training train_dqn_ros
```

## 19.1 Configuración actual

La configuración actual implementada fue:

```python
stage = 1
episodes = 100
max_steps = 80
batch_size = 64
gamma = 0.99
learning_rate = 1e-3
buffer_capacity = 20000
min_buffer_size = 500
target_update_every = 25
epsilon = 1.0
epsilon_end = 0.05
epsilon_decay = 0.995
eval_every = 25
eval_episodes = 3
save_videos = True
video_fps = 8
```

Se explicó que para acelerar se recomienda ajustar a:

```python
episodes = 100
max_steps = 40
eval_every = 25
eval_episodes = 2
```

Y durante entrenamiento guardar principalmente MP4, no GIF.

---

## 19.2 Número total de episodios

Con la configuración original:

```text
episodes = 100
eval_every = 25
eval_episodes = 3
```

Se tienen:

```text
100 episodios de entrenamiento
4 evaluaciones
3 episodios por evaluación
12 episodios extra de evaluación
```

Total:

```text
112 episodios ejecutados
```

Máximo de pasos:

```text
100 * 80 = 8000 pasos de entrenamiento
12 * 80 = 960 pasos de evaluación
Total máximo = 8960 pasos
```

---

# 20. Métricas de entrenamiento

El archivo principal de métricas es:

```text
training_runs/<training_id>/metrics.csv
```

Columnas principales:

```text
episode
reward
steps
success
collision
timeout
epsilon
avg_loss
buffer_size
eval_avg_reward
eval_success_rate
eval_collision_rate
eval_avg_steps
```

---

# 21. Graficador de métricas: `plot_training_metrics.py`

Se creó un script para generar gráficas automáticamente desde `metrics.csv`.

Ejecutable:

```bash
ros2 run turtlebot3_rl_training plot_training_metrics \
--run-dir ~/ros2_ws/src/turtlebot3_rl_training/training_runs/<training_id>
```

Genera archivos en:

```text
training_runs/<training_id>/plots/
```

Se pidió añadir numeración en los archivos según el orden de revisión.

El orden final recomendado es:

```text
01_eval_success_rate.png
02_eval_avg_reward.png
03_outcome_rates.png
04_reward.png
05_steps.png
06_loss.png
07_epsilon.png
08_buffer_size.png
09_eval_collision_rate.png
10_eval_avg_steps.png
11_metrics_summary.txt
```

## 21.1 Orden de análisis recomendado

El análisis debe hacerse en este orden:

```text
1. eval_success_rate
2. eval_avg_reward
3. outcome_rates
4. reward
5. steps
6. loss
7. epsilon
8. buffer_size
9. eval_collision_rate
10. eval_avg_steps
11. metrics_summary
```

---

# 22. Qué analizar en un entrenamiento

## 22.1 Videos

Revisar:

```text
training_runs/<training_id>/episodes/
```

Preguntas importantes:

```text
¿El robot se acerca a la meta?
¿Gira sin sentido?
¿Corrige orientación?
¿Se queda oscilando?
¿Llega con menos pasos?
¿Choca?
¿El comportamiento mejora con el tiempo?
```

## 22.2 `eval_success_rate`

Debe aumentar con el tiempo.

Criterio para avanzar de stage 1 a stage 2:

```text
eval_success_rate >= 0.80
eval_collision_rate = 0.0
eval_avg_reward positivo
comportamiento visual coherente
```

## 22.3 `eval_avg_reward`

Debe aumentar progresivamente.

No tiene que crecer de forma perfecta, pero la tendencia debe ser positiva.

## 22.4 `reward`

Debe mostrar una tendencia general ascendente.

En RL es normal tener ruido.

## 22.5 `steps`

Si el agente mejora, los pasos para llegar a la meta deberían bajar o estabilizarse.

## 22.6 `epsilon`

Debe bajar desde:

```text
1.0
```

hasta:

```text
0.05
```

## 22.7 `loss`

La pérdida no siempre baja suavemente en DQN.

No debe ser el único criterio de evaluación.

---

# 23. Recomendaciones para acelerar entrenamiento

Se detectó que los entrenamientos toman bastante tiempo por:

```text
- Simulación en Gazebo.
- Ejecución física de cada acción.
- Renderizado de videos.
- Generación de GIF.
- GUI activa.
```

Recomendaciones:

## 23.1 Ejecutar sin GUI

Usar:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1 gui:=false
```

Para demostración:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1 gui:=true
```

## 23.2 No guardar GIF durante entrenamiento

El GIF es más costoso que MP4.

Durante entrenamiento se recomienda:

```text
Guardar MP4
No guardar GIF
```

El GIF se deja para episodios finales o resultados importantes.

## 23.3 Reducir `max_steps`

Para stage 1 se recomienda bajar:

```python
max_steps = 80
```

a:

```python
max_steps = 40
```

Incluso puede probarse:

```python
max_steps = 30
```

## 23.4 Aumentar velocidades del controlador

Valores iniciales:

```python
linear_speed = 0.15
angular_speed = 0.45
```

Recomendados para acelerar:

```python
linear_speed = 0.25
angular_speed = 0.75
```

Se mantiene:

```python
forward_distance = 0.20
turn_angle = math.pi / 18.0
```

Así la acción representa lo mismo, pero se ejecuta más rápido.

---

# 24. Comandos principales de ejecución

## 24.1 Compilar paquetes

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_custom_worlds turtlebot3_rl_training
source install/setup.bash
```

## 24.2 Lanzar Gazebo

Con GUI:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1 gui:=true
```

Sin GUI:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1 gui:=false
```

## 24.3 Lanzar interfaz RL

```bash
ros2 run turtlebot3_rl_training rl_interface_node
```

## 24.4 Lanzar controlador

```bash
ros2 run turtlebot3_rl_training rl_motion_controller
```

## 24.5 Probar entorno

```bash
ros2 run turtlebot3_rl_training test_gazebo_env
```

## 24.6 Entrenar DQN

```bash
ros2 run turtlebot3_rl_training train_dqn_ros
```

## 24.7 Graficar métricas

```bash
ros2 run turtlebot3_rl_training plot_training_metrics \
--run-dir ~/ros2_ws/src/turtlebot3_rl_training/training_runs/<training_id>
```

## 24.8 Renderizar episodio

```bash
ros2 run turtlebot3_rl_training render_episode_video \
--input ~/ros2_ws/src/turtlebot3_rl_training/training_runs/<training_id>/episodes/episode_0025_eval_01.json \
--output ~/ros2_ws/src/turtlebot3_rl_training/training_runs/<training_id>/episodes/episode_0025_eval_01.mp4 \
--format mp4 \
--fps 8
```

---

# 25. Troubleshooting documentado

## 25.1 La meta no se mueve

Causa:

```text
La meta está como static=true.
```

Solución:

```xml
<static>false</static>
```

## 25.2 `/gazebo/set_entity_state` no aparece

Causa:

```text
Falta plugin gazebo_ros_state.
```

Solución:

```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros>
        <namespace>/gazebo</namespace>
    </ros>
    <update_rate>20.0</update_rate>
</plugin>
```

## 25.3 El robot gira cuando debería avanzar

Causa probable:

```text
Modelo URDF/Xacro mal cargado.
```

Solución:

```text
Usar modelo SDF oficial de turtlebot3_gazebo.
```

## 25.4 Siempre carga mundo vacío

Solución:

```bash
pkill -f gzserver
pkill -f gzclient
colcon build --packages-select turtlebot3_custom_worlds
source install/setup.bash
```

## 25.5 `/rl_action_done` da timeout

Causa:

```text
El timeout del entorno es menor que el tiempo real de acción.
```

Solución:

```text
gazebo_rl_env action_execution_time = 5.0
rl_motion_controller action_timeout = 4.0
```

## 25.6 Se genera superposición en cuadro de texto del video

Solución adoptada:

```text
Eliminar texto coloreado separado.
Dejar Status como línea simple al final del cuadro.
```

---

# 26. Flujo final recomendado

## Entrenamiento rápido

Terminal 1:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1 gui:=false
```

Terminal 2:

```bash
ros2 run turtlebot3_rl_training rl_interface_node
```

Terminal 3:

```bash
ros2 run turtlebot3_rl_training rl_motion_controller
```

Terminal 4:

```bash
ros2 run turtlebot3_rl_training train_dqn_ros
```

## Análisis posterior

```bash
ros2 run turtlebot3_rl_training plot_training_metrics \
--run-dir ~/ros2_ws/src/turtlebot3_rl_training/training_runs/<training_id>
```

Luego revisar:

```text
plots/
episodes/
checkpoints/
summary.json
metrics.csv
```

---

# 27. Estado actual del proyecto

Hasta el momento se logró:

```text
- Crear mundos personalizados para TurtleBot3.
- Crear cuadrícula métrica con ejes positivos.
- Hacer meta movible.
- Cargar mundos por stage.
- Ejecutar TurtleBot3 correctamente.
- Crear interfaz RL con /rl_state y /rl_goal_reached.
- Crear controlador discreto con /rl_action.
- Añadir /rl_action_done.
- Crear entorno tipo Gym conectado a ROS/Gazebo.
- Crear núcleo DQN.
- Crear logger de episodios.
- Crear renderizador MP4/GIF desde JSON.
- Crear generación de training_id por fecha/hora.
- Crear entrenamiento DQN inicial.
- Crear script de gráficas de métricas.
- Documentar análisis y troubleshooting.
```

---

# 28. Próximos pasos recomendados

## 28.1 Consolidar configuración YAML

Actualmente varios parámetros están definidos directamente en scripts. Se recomienda mover a:

```text
config/training_config.yaml
```

Parámetros como:

```text
stage
episodes
max_steps
batch_size
learning_rate
gamma
epsilon_decay
eval_every
eval_episodes
save_videos
video_fps
goals
reward_config
```

## 28.2 Crear launch integral de entrenamiento

Crear un launch que levante:

```text
Gazebo
rl_interface_node
rl_motion_controller
```

y luego ejecutar el entrenamiento aparte.

## 28.3 Crear evaluación independiente

Terminar:

```text
evaluate_dqn_ros.py
```

para cargar:

```text
best_model.pth
```

y evaluar sin exploración.

## 28.4 Entrenar por curriculum

Propuesta:

```text
Stage 1 -> aprender navegación directa.
Stage 2 -> obstáculo frontal.
Stage 3 -> decisión lateral.
Stage 4 -> corredor.
Stage 5 -> chicane.
Stage 6 -> obstáculos distribuidos.
Stage 7 -> laberinto.
```

No pasar de stage hasta que:

```text
eval_success_rate >= 0.80
```

## 28.5 Comparación experimental

Guardar por cada entrenamiento:

```text
config.json
metrics.csv
summary.json
best_model.pth
last_model.pth
plots/
episodes/
```

Esto permitirá construir tablas tipo paper:

```text
success_rate
collision_rate
avg_reward
avg_steps
training_time
stage
algorithm
```

---

# 29. Conclusión

El sistema desarrollado establece una base completa para entrenar un agente DQN en TurtleBot3 usando ROS 2 Humble y Gazebo Classic.

La arquitectura separa correctamente:

```text
Simulación y mundos -> turtlebot3_custom_worlds
Entrenamiento y análisis -> turtlebot3_rl_training
```

El flujo actual permite:

```text
1. Cargar un escenario.
2. Publicar estados RL.
3. Ejecutar acciones discretas.
4. Esperar finalización de acción.
5. Calcular recompensa.
6. Entrenar una red DQN.
7. Guardar modelos.
8. Guardar métricas.
9. Renderizar episodios.
10. Graficar resultados.
```
