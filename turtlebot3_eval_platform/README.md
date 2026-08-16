# 🐢 TurtleBot3 Evaluation & Benchmarking Platform (ROS 2 Humble)

Plataforma modular, autocontenida y altamente optimizada para el **entrenamiento por refuerzo (DQN)** y la **evaluación comparativa (*benchmarking*)** de controladores de navegación continua en simulación robótica usando ROS 2 Humble y Gazebo Classic.

---

## 📌 Resumen General

Esta plataforma permite entrenar y evaluar objetivamente diferentes paradigmas de control (**DQN de PyTorch**, **Lógica Difusa Sugeno** y **Control Reactivo basado en Reglas**) sobre el robot TurtleBot3 Burger.

El sistema implementa un lazo de control continuo a **10 Hz** que interactúa mediante los tópicos `/cmd_vel`, `/rl_state`, `/rl_action` y efectúa el reposicionamiento físico sin inercia del robot a través de servicios directos de Gazebo (`/gazebo/set_entity_state`), garantizando pruebas episódicas automatizadas de alta velocidad.

---

## 🛠️ Estructura del Proyecto

```text
ros2_ws/src/
├── turtlebot3_eval_platform/        # Plataforma principal de benchmarking y agentes de evaluación
│   ├── launch/
│   │   └── benchmark.launch.py      # Lanzador paramétrico de evaluación comparativa
│   └── turtlebot3_eval_platform/
│       ├── gazebo_rl_env.py         # Entorno Gym continuo e interacción con Gazebo
│       ├── benchmark_runner.py      # Bucle principal de benchmarking
│       └── agents/
│           ├── dqn_agent.py         # Cargador DQN adaptable de PyTorch (sostiene 3 y 5 acciones)
│           ├── fuzzy_agent.py       # Controlador difuso Sugeno calibrado a distancias físicas
│           └── rule_based_agent.py  # Agente reactivo basado en reglas y umbrales
│
├── turtlebot3_rl_training/          # Paquete especializado en el entrenamiento RL (DQN)
│   ├── launch/
│   │   └── train_dqn_stage.launch.py # Lanzador principal de sesiones de entrenamiento
│   └── turtlebot3_rl_training/
│       ├── train_dqn_ros.py         # Algoritmo DQN, Replay Buffer y bucle de optimización
│       ├── gazebo_rl_env.py         # Entorno de entrenamiento y función de recompensa
│       ├── rl_interface_node.py     # Nodo procesador de sensores (LiDAR y Odometría)
│       └── rl_motion_controller.py # Controlador de movimiento discreto a continuo (10 Hz)
│
└── turtlebot3_custom_worlds/        # Mundos y escenarios Gazebo calibrados (Stages 0 a 7)
```

---

## 🏎️ Espacio de Estados y de Acciones

### Espacio de Estados (5D Normalizado)
El robot percibe su entorno a través de un vector de estado de 5 dimensiones normalizadas en el intervalo $[0.0, 1.0]$ o $[-1.0, 1.0]$:

| Índice | Variable | Descripción | Rango de Normalización |
| :---: | :--- | :--- | :---: |
| **0** | $d_{front}$ | Distancia al obstáculo frontal (LiDAR) | $[0.0, 1.0]$ (Máx: $3.5\text{ m}$) |
| **1** | $d_{left}$ | Distancia al obstáculo izquierdo (LiDAR) | $[0.0, 1.0]$ (Máx: $3.5\text{ m}$) |
| **2** | $d_{right}$ | Distancia al obstáculo derecho (LiDAR) | $[0.0, 1.0]$ (Máx: $3.5\text{ m}$) |
| **3** | $\theta_{goal}$ | Error angular respecto a la meta (orientación) | $[-1.0, 1.0]$ (Dividido por $\pi$) |
| **4** | $d_{goal}$ | Distancia euclidiana hacia la meta | $[0.0, 1.0]$ (Dividido por $5.0\text{ m}$) |

### Espacio de Acciones (Siempre Adelante)
Para garantizar flujo constante y evitar atascos, se definieron 5 acciones discretas mapeadas a comandos continuos de velocidad lineal ($v$ en m/s) y angular ($w$ en rad/s):

| Acción | Descripción | Velocidad Lineal ($v$) | Velocidad Angular ($w$) |
| :---: | :--- | :---: | :---: |
| **0** | Avance Recto | $0.18\text{ m/s}$ | $0.00\text{ rad/s}$ |
| **1** | Giro Leve Izquierda | $0.14\text{ m/s}$ | $+0.60\text{ rad/s}$ |
| **2** | Giro Leve Derecha | $0.14\text{ m/s}$ | $-0.60\text{ rad/s}$ |
| **3** | Giro Pronunciado Izquierda | $0.10\text{ m/s}$ | $+1.20\text{ rad/s}$ |
| **4** | Giro Pronunciado Derecha | $0.10\text{ m/s}$ | $-1.20\text{ rad/s}$ |

---

## 🧮 Función de Recompensa y Reward Shaping con LiDAR

La función de recompensa en `gazebo_rl_env.py` combina un incentivo continuo de aproximación a la meta con una penalización reactiva basada en LiDAR para evitación de obstáculos:

$$\text{Recompensa por Paso} = R_{\text{progreso}} + R_{\text{tiempo}} + R_{\text{orientación}} + R_{\text{LiDAR\_shaping}} + R_{\text{oscilación}}$$

### 1. Componentes Continuos:
*   **Progreso Físico ($R_{\text{progreso}}$)**: Premio proporcional al avance hacia la meta ($+r_{\text{gain}} \times \Delta d_{\text{meta}}$). Si se aleja, sufre penalización de retroceso.
*   **Costo por Paso ($R_{\text{tiempo}}$)**: Pequeño costo fijo por paso para incentivar rutas óptimas y cortas.
*   **Alineación Angular ($R_{\text{orientación}}$)**: Penalización proporcional al desvío angular $|\theta_{\text{meta}}|$ y bonificación por corregir el rumbo.

### 2. Reward Shaping por Proximidad de Obstáculos ($R_{\text{LiDAR\_shaping}}$):
Evaluado cuando la distancia LiDAR frontal cae por debajo del umbral de evitación ($d_{\text{front}} < 1.05\text{ m}$ / norm $0.30$):
*   **Avance Recto (`Acción 0`) cerca de pared**: Sufre una penalización directa por insistir en avanzar hacia la pared:
    $$\text{Penalización} = -1.5 \times (0.30 - d_{\text{front\_norm}})$$
*   **Giros de Evitación (`Acción 1` o `Acción 2`) cerca de pared**: Recibe un premio directo de **$+0.30$** y queda eximido de la penalización por giro innecesario.

### 3. Recompensas Terminales ($R_{\text{terminal}}$):
*   **Éxito (Llegada a Meta)**: $+\mathbf{150.0}$
*   **Colisión ($d_{\text{láser}} < 35\text{ cm}$)**: $-\mathbf{100.0}$
*   **Timeout (`max_steps` superados)**: $-\mathbf{80.0}$

---

## 📐 Distancias Físicas Calibradas

| Métrica | Valor Físico | Valor Normalizado | Comportamiento |
| :--- | :---: | :---: | :--- |
| **Rango Máximo LiDAR** | $3.50\text{ m}$ | $1.00$ | Límite superior del sensor láser. |
| **Inicio de Evitación (Shaping)**| **$1.05\text{ m}$** | **$0.30$** | El robot detecta la pared, penaliza la Acción 0 y premia giros de evitación. |
| **Umbral de Colisión (Impacto)** | **$0.35\text{ m}$ ($35\text{ cm}$)**| **$0.10$** | Límite físico de colisión. Detiene el episodio y penaliza con $-100.0$. |
| **Margen de Maniobra** | **$0.70\text{ m}$ ($70\text{ cm}$)**| — | Espacio libre para ejecutar giros ágiles antes de impactar. |

---

## 🎯 Modos de Posicionamiento de Metas (`goal_mode`)

El parámetro `goal_mode` define la estrategia de variación de metas entre episodios para evitar el sobreajuste (*overfitting*) a rutas fijas y fomentar el aprendizaje condicionado:

*   **`single`**: Meta fija frontal `(1.5, 0.0)`. Recomendado para validación inicial de convergencia en espacio libre.
*   **`soft`**: Variaciones laterales leves `[(1.5, 0.0), (1.5, 0.25), (1.5, -0.25)]`. Otorga tolerancia ante pequeñas desviaciones de rumbo.
*   **`medium`**: Variaciones laterales amplias detrás de obstáculos `[(1.5, 0.0), (1.2, 0.8), (1.2, -0.8)]`. Obliga a la red neuronal a aprender a bordear paredes por la izquierda o por la derecha según la meta asignada en el episodio.
*   **`separated`**: Matriz completa de orígenes y metas distribuidas para pruebas de generalización extrema en escenarios complejos (Stages 3-7).

---

## ⚡ Características Clave del Sistema

### 1. Detención Física Activa en Teletransporte
El entorno no solo reposiciona el robot usando `/gazebo/set_entity_state`, sino que además reinicia el vector de velocidad física (`twist`) de la entidad a `0.0`, previniendo que inercias de episodios pasados contaminen el inicio de la prueba.

### 2. Cargador Adaptable DQN (`dqn_agent.py`)
El cargador lee el `state_dict` del archivo `.pth` recibido y **reconfigura automáticamente** las capas de salida de la red neuronal (`QNetwork`):
*   **Compatibilidad de 3 Acciones**: Reestructura automáticamente la red si el modelo fue entrenado con rotaciones in-situ.
*   **Nativo de 5 Acciones**: Carga directamente las 5 acciones continuas de avance siempre adelante.

### 3. Monitoreo Verbose en Tiempo Real sin Spam
El sistema redirige los logs informativos repetitivos de los controladores a nivel `DEBUG` e imprime en la terminal únicamente el resumen relevante de cada episodio (`éxito20`, `colisión20`, `avg20`, `loss`) y los cuadros destacados de **Evaluación de Política ($\epsilon=0.0$)**.

### 4. Reinicio Inteligente de Puntaje en Transfer Learning y Cambio de Metas
Al realizar transferencia de aprendizaje entre etapas distintas (ej. Stage 1 $\to$ Stage 2) o al cambiar el modo de metas dentro de la misma etapa (ej. `single` $\to$ `medium`), el sistema **detecta el cambio de configuración y reinicia el `best_score` a $-\infty$**. Esto permite que la plataforma evalúe y guarde correctamente los mejores modelos (`best_model.pth`) del nuevo entorno sin ser bloqueado por puntajes históricos de configuraciones anteriores.

---

## 🚀 Guía de Ejecución y Parámetros Configurables

### Compilación del Workspace
Antes de ejecutar cualquier lanzador, compila los paquetes del workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_eval_platform turtlebot3_rl_training turtlebot3_custom_worlds
source install/setup.bash
```

---

### A. Entrenamiento de DQN (`train_dqn_stage.launch.py`)

#### Comandos de Ejemplo

##### 1. Entrenamiento Estándar en Stage 2 con Metas Dinámicas (`goal_mode:=medium`)
```bash
ros2 launch turtlebot3_rl_training train_dqn_stage.launch.py \
    stage:=2 \
    goal_mode:=medium \
    gui:=true \
    episodes:=200 \
    batch_size:=64 \
    learning_rate:=0.001 \
    max_steps:=80 \
    epsilon_start:=0.80 \
    epsilon_decay:=0.995
```

##### 2. Reanudación / Fine-Tuning desde un Checkpoint (`best_model.pth`)
```bash
ros2 launch turtlebot3_rl_training train_dqn_stage.launch.py \
    stage:=2 \
    goal_mode:=medium \
    gui:=true \
    episodes:=150 \
    resume_checkpoint:=/home/d3im3r/ros2_ws/src/train_runs/stage_2_medium_2026_08_16_112449/checkpoints/best_model.pth \
    epsilon_start:=0.35 \
    learning_rate:=0.0003 \
    max_steps:=80
```

##### 3. Transferencia de Aprendizaje (Domain Transfer: Stage 1 Single $\to$ Stage 2 Medium)
Aprovecha el conocimiento de un modelo entrenado en Stage 1 (`single`) e inyecta sus pesos para aprender evitación de obstáculos en Stage 2 (`medium`), reiniciando automáticamente el `best_score`:
```bash
ros2 launch turtlebot3_rl_training train_dqn_stage.launch.py \
    stage:=2 \
    goal_mode:=medium \
    gui:=true \
    episodes:=200 \
    resume_checkpoint:=/home/d3im3r/ros2_ws/src/train_runs/stage_1_single_2026_08_15_171709/checkpoints/best_model.pth \
    epsilon_start:=0.40 \
    learning_rate:=0.0005 \
    max_steps:=80
```

##### 4. Entrenamiento Rápido sin Interfaz Gráfica (Modo Headless)
```bash
ros2 launch turtlebot3_rl_training train_dqn_stage.launch.py \
    stage:=3 \
    goal_mode:=medium \
    gui:=false \
    episodes:=300 \
    batch_size:=128 \
    learning_rate:=0.0005 \
    epsilon_decay:=0.997 \
    max_steps:=100
```

#### Parámetros Configurables de Entrenamiento

| Parámetro | Tipo | Valor por Defecto | Descripción | Casos de Uso / Modificación |
| :--- | :---: | :---: | :--- | :--- |
| `stage` | `int` | `1` | Escenario/Mundo de Gazebo (0 a 7). | Seleccionar la dificultad del mapa. |
| `goal_mode` | `string`| `single` | Modo de muestreo de metas (`single`, `soft`, `medium`, `separated`). | Usar `medium` para aprender evitación condicionada por ambos lados de un obstáculo. |
| `gui` | `bool` | `true` | Muestra u oculta la ventana gráfica de Gazebo. | Usar `false` para maximizar velocidad de cómputo en CPU/GPU. |
| `episodes` | `int` | `120` | Número total de episodios de la sesión. | Aumentar a `200`-`500` para escenarios complejos. |
| `max_steps` | `int` | `40` | Pasos máximos permitidos por episodio. | Usar `80` en Stage 2 para dar tiempo a bordear obstáculos. |
| `resume_checkpoint`| `string`| `""` | Ruta a `.pth` para reanudar o transferir pesos. | Cargar `best_model.pth` para fine-tuning o transferencia entre etapas. |
| `epsilon_start` | `float`| `1.0` | Probabilidad inicial de exploración ($\epsilon$). | Reducir a `0.30`-`0.40` al reanudar entrenamientos pre-entrenados. |
| `epsilon_decay` | `float`| `0.995` | Decaimiento geométrico por episodio para $\epsilon$. | Usar `0.997` en sesiones de más de 300 episodios. |
| `learning_rate` | `float`| `0.001` | Tasa de aprendizaje del optimizador Adam. | Reducir a `0.0003` durante fine-tuning. |
| `batch_size` | `int` | `64` | Muestras extraídas del Replay Buffer por actualización. | Usar `64` o `128` para mayor estabilidad de gradiente. |
| `base_dir` | `string`| `/home/d3im3r/ros2_ws/src/train_runs` | Directorio de salida para guardar ejecuciones. | Ubicación automática de logs y checkpoints. |

---

### B. Evaluación y Benchmarking (`benchmark.launch.py`)

Permite medir de forma determinista ($\epsilon = 0.0$) el desempeño real de los agentes en metas predefinidas.

#### Comandos de Ejemplo

```bash
# 1. Evaluar agente DQN en Stage 2 apuntando a un modelo entrenado
ros2 launch turtlebot3_eval_platform benchmark.launch.py \
    agent:=dqn \
    stage:=2 \
    episodes:=5 \
    launch_gazebo:=true \
    gui:=true \
    model_path:=/home/d3im3r/ros2_ws/src/train_runs/stage_2_medium_2026_08_16_112449/checkpoints/best_model.pth

# 2. Evaluar agente Lógica Difusa (Fuzzy) en Stage 1
ros2 launch turtlebot3_eval_platform benchmark.launch.py \
    agent:=fuzzy \
    stage:=1 \
    episodes:=5 \
    launch_gazebo:=true \
    gui:=true

# 3. Evaluar agente Reactivo basado en Reglas en Stage 2 sin relanzar Gazebo
ros2 launch turtlebot3_eval_platform benchmark.launch.py \
    agent:=rule_based \
    stage:=2 \
    episodes:=3 \
    launch_gazebo:=false
```

#### Parámetros Configurables de Evaluación

| Parámetro | Tipo | Valor por Defecto | Descripción | Casos de Uso / Modificación |
| :--- | :---: | :---: | :--- | :--- |
| `agent` | `string`| `fuzzy` | Algoritmo a evaluar (`dqn`, `fuzzy`, `rule_based`). | Permite comparar distintos paradigmas de control. |
| `stage` | `int` | `1` | Escenario/Mundo de evaluación (0 a 7). | Seleccionar la etapa de prueba. |
| `episodes` | `int` | `1` | Episodios de prueba por meta. | Aumentar a `5` o `10` para significancia estadística. |
| `launch_gazebo`| `bool` | `true` | Auto-lanza el servidor Gazebo. | Usar `false` si Gazebo ya está corriendo. |
| `gui` | `bool` | `true` | Muestra la ventana gráfica de Gazebo. | Cambiar a `false` para pruebas automatizadas rápidas. |
| `model_path` | `string`| `""` | Ruta al archivo `.pth` del modelo DQN. | Requerido cuando `agent:=dqn`. |
| `output_dir` | `string`| `/home/d3im3r/ros2_ws/src/eval_runs` | Directorio de registros episódicos. | Ruta de guardado del informe detallado. |
| `csv_file` | `string`| `/home/d3im3r/ros2_ws/src/eval_history.csv` | Archivo CSV acumulativo central. | Mantiene el resumen comparativo entre agentes. |

---

## 📊 Sistema de Registro de Datos (Logs)

La plataforma guarda los resultados en dos niveles de resolución:

1. **Registro Resumido Centralizado (`eval_history.csv`)**:
   * Ubicación: `/home/d3im3r/ros2_ws/src/eval_history.csv`
   * Columnas: `timestamp, agent, checkpoint, world, n_obstacles, init_pos, goal_pos, episodes, avg_reward, success_rate, collision_rate, min_steps, avg_steps, max_steps, std_steps, run_dir`

2. **Registro Detallado por Episodio (`episode_history.csv`)**:
   * Ubicación: `/home/d3im3r/ros2_ws/src/train_runs/<run_id>/episode_history.csv` o `/home/d3im3r/ros2_ws/src/eval_runs/<eval_id>/episode_history.csv`
   * Columnas: `episode, goal_x, goal_y, success, collision, timeout, reward, steps, path_length, avg_speed`

---

## ⚙️ Requisitos del Sistema
* **ROS 2 Humble Hawksbill**
* **Gazebo Classic** (con paquetes `turtlebot3_gazebo` e `turtlebot3_custom_worlds`)
* **PyTorch** (para inferencia y entrenamiento DQN en GPU/CPU)
* **Numpy** y **Pandas** (para procesamiento estadístico de métricas)
