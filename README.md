# 🐢 TurtleBot3 Autonomous Navigation & Benchmarking Platform (ROS 2 Humble + Gazebo)

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange.svg)](http://gazebosim.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.0+-red.svg)](https://pytorch.org/)
[![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)](https://www.python.org/)

Plataforma modular, autocontenida y de alto rendimiento desarrollada en **ROS 2 Humble** y **Gazebo Classic** para la investigación, entrenamiento por refuerzo profundo (**Deep Q-Network - DQN**) y evaluación comparativa estandarizada (*benchmarking*) de algoritmos de navegación autónoma sobre el robot móvil **TurtleBot3 Burger**.

El sistema integra un entorno de aprendizaje estilo OpenAI Gym acoplado directamente al motor de física de Gazebo, soporte para metas dinámicas condicionadas (*goal-conditioned RL*), reward shaping reactivo guiado por LiDAR y controladores comparativos de **Lógica Difusa (Fuzzy)** y **Control Reactivo basado en Reglas**.

---

## 📋 Tabla de Contenidos
1. [Resumen General del Proyecto](#-resumen-general-del-proyecto)
2. [Estructura Completa del Repositorio](#-estructura-completa-del-repositorio)
3. [Espacio de Estados y de Acciones](#-espacio-de-estados-y-de-acciones)
4. [Formulación Matemática de la Recompensa (Reward Shaping)](#-formulación-matemática-de-la-recompensa-reward-shaping)
5. [Distancias Físicas y Umbrales Calibrados](#-distancias-físicas-y-umbrales-calibrados)
6. [Modos de Metas Dinámicas (`goal_mode`)](#-modos-de-metas-dinámicas-goal_mode)
7. [Mecanismo de Reinicio de Score en Transfer Learning](#-mecanismo-de-reinicio-de-score-en-transfer-learning)
8. [Requisitos e Instalación](#-requisitos-e-instalación)
9. [Guía Exhaustiva de Entrenamiento (`train_dqn_stage.launch.py`)](#-guía-exhaustiva-de-entrenamiento-train_dqn_stagelaunchpy)
10. [Guía Exhaustiva de Benchmarking (`benchmark.launch.py`)](#-guía-exhaustiva-de-benchmarking-benchmarklaunchpy)
11. [Sistema de Registro de Datos e Historiales (Logs)](#-sistema-de-registro-de-datos-e-historiales-logs)
12. [Diagnóstico y Solución de Problemas](#-diagnóstico-y-solución-de-problemas)

---

## 📌 Resumen General del Proyecto

Esta plataforma fue construida para resolver el problema de atascos y colisiones deterministas en tareas de navegación robótica mediante un flujo de control continuo a **10 Hz**. 

El sistema ejecuta el lazo cerrado de control conectando nodos ROS 2 nativos con PyTorch:
* **Procesamiento Sensorial**: Convierte escaneos de LiDAR ($360^\circ$) y odometría ($/odom$) en un vector de estado de 5 dimensiones normalizadas.
* **Controlador de Movimiento**: Mapea las 5 acciones discretas elegidas por la red a comandos continuos de velocidad lineal y angular publicados en `/cmd_vel`.
* **Teletransporte y Reset Limpio**: Ejecuta el reposicionamiento del robot usando `/gazebo/set_entity_state` y reinicia el vector de velocidades física (`twist`) a cero absoluto, eliminando las inercias residuales entre episodios.

---

## 🛠️ Estructura Completa del Repositorio

```text
rl_environment/
├── README.md                           # Documentación maestra del repositorio
├── .gitignore                          # Exclusión de pesos .pth, logs y temporales de ROS 2
│
├── turtlebot3_rl_training/             # Paquete de Entrenamiento Reinforcement Learning (DQN)
│   ├── launch/
│   │   ├── train_dqn_stage.launch.py   # Launch file principal paramétrico para entrenamiento
│   │   └── rl_env_stage.launch.py      # Launch file secundario para el entorno de simulación
│   └── turtlebot3_rl_training/
│       ├── train_dqn_ros.py            # Bucle de entrenamiento DQN, Replay Buffer y optimizador
│       ├── gazebo_rl_env.py            # Entorno Gym ROS 2 y formulación de recompensa (Reward Shaping)
│       ├── rl_interface_node.py        # Nodo suscriptor de LiDAR/Odom y publicador del estado 5D
│       ├── rl_motion_controller.py     # Nodo ejecutor de velocidades en /cmd_vel (10 Hz)
│       ├── dqn_core.py                 # Arquitectura de la red neuronal PyTorch (QNetwork)
│       └── run_manager.py              # Gestión de directorios, metadatos y guardado de modelos
│
├── turtlebot3_eval_platform/           # Paforma de Evaluación Comparativa (Benchmarking)
│   ├── launch/
│   │   └── benchmark.launch.py         # Launch file principal para benchmarking
│   └── turtlebot3_eval_platform/
│       ├── gazebo_rl_env.py            # Clase de entorno optimizada para evaluación continua
│       ├── benchmark_runner.py         # Ejecutor principal de pruebas automatizadas
│       └── agents/
│           ├── dqn_agent.py            # Cargador adaptable DQN (soporta modelos de 3 y 5 acciones)
│           ├── fuzzy_agent.py          # Controlador por Lógica Difusa Sugeno
│           └── rule_based_agent.py     # Agente reactivo basado en reglas y umbrales
│
└── turtlebot3_custom_worlds/           # Paquete de Escenarios y Mundos de Gazebo Classic
    ├── launch/                         # Launchers de mundos Gazebo
    ├── models/                         # Modelos 3D y obstáculos
    └── worlds/                         # Archivos .world calibrados (Stage 1 a Stage 7)
        ├── d3im3r_stage_01.world       # Stage 1: Cuadrado libre sin obstáculos
        ├── d3im3r_stage_02_front_obstacle.world # Stage 2: Obstáculo central frontal
        └── ...                         # Stages 3 a 7: Múltiples obstáculos y laberintos
```

---

## 🏎️ Espacio de Estados y de Acciones

### 1. Espacio de Estados (Vector 5D Normalizado)
El estado $S_t$ percibido por el agente se compone de 5 variables escalarmente acotadas:

$$\mathbf{S}_t = \left[ d_{\text{front}}, d_{\text{left}}, d_{\text{right}}, \theta_{\text{goal}}, d_{\text{goal}} \right]$$

| Índice | Variable | Descripción Técnica | Rango de Entrada | Rango Normalizado |
| :---: | :--- | :--- | :---: | :---: |
| **0** | $d_{\text{front}}$ | Distancia mínima del sector frontal del LiDAR ($[-15^\circ, 15^\circ]$) | $[0.0\text{ m}, 3.5\text{ m}]$ | $[0.0, 1.0]$ |
| **1** | $d_{\text{left}}$ | Distancia mínima del sector izquierdo del LiDAR ($[15^\circ, 75^\circ]$) | $[0.0\text{ m}, 3.5\text{ m}]$ | $[0.0, 1.0]$ |
| **2** | $d_{\text{right}}$ | Distancia mínima del sector derecho del LiDAR ($[-75^\circ, -15^\circ]$) | $[0.0\text{ m}, 3.5\text{ m}]$ | $[0.0, 1.0]$ |
| **3** | $\theta_{\text{goal}}$ | Error angular entre la orientación del robot y la meta | $[-\pi, \pi]\text{ rad}$ | $[-1.0, 1.0]$ |
| **4** | $d_{\text{goal}}$ | Distancia euclidiana desde la posición actual a la meta | $[0.0\text{ m}, 5.0\text{ m}]$ | $[0.0, 1.0]$ |

### 2. Espacio de Acciones Discretas a Comandos Continuos
Para garantizar un flujo constante y evitar que el robot se detenga frente a paredes, las 5 acciones discretas se traducen en velocidades continuas publicadas a 10 Hz:

| Acción ($A_t$) | Denominación | Velocidad Lineal ($v$) | Velocidad Angular ($w$) | Propósito Operativo |
| :---: | :--- | :---: | :---: | :--- |
| **0** | Avance Recto | $0.18\text{ m/s}$ | $0.00\text{ rad/s}$ | Desplazamiento ágil en corredores libres. |
| **1** | Giro Leve Izquierda | $0.14\text{ m/s}$ | $+0.60\text{ rad/s}$ | Corrección suave de orientación. |
| **2** | Giro Leve Derecha | $0.14\text{ m/s}$ | $-0.60\text{ rad/s}$ | Corrección suave de orientación. |
| **3** | Giro Pronunciado Izquierda | $0.10\text{ m/s}$ | $+1.20\text{ rad/s}$ | Evitación rápida de obstáculos cercanos. |
| **4** | Giro Pronunciado Derecha | $0.10\text{ m/s}$ | $-1.20\text{ rad/s}$ | Evitación rápida de obstáculos cercanos. |

---

## 🧮 Formulación Matemática de la Recompensa (Reward Shaping)

El retorno por paso de tiempo $R_t$ está definido por la suma de componentes continuos y penalizaciones reactivas de evitación:

$$R_t = R_{\text{progreso}} + R_{\text{tiempo}} + R_{\text{orientacion}} + R_{\text{shaping}} + R_{\text{terminal}}$$

### 1. Componentes Continuos de Navegación:
* **Progreso a la Meta**: Premio proporcional a la distancia recorrida en dirección a la meta:
  $$R_{\text{progreso}} = 12.0 \times (d_{\text{goal,prev}} - d_{\text{goal,current}})$$
* **Penalización por Tiempo (Step Penalty)**: Costo fijo por paso para favorecer rutas cortas:
  $$R_{\text{tiempo}} = -0.40$$
* **Alineación Angular**: Recompensa por mantener el frente orientado hacia la meta:
  $$R_{\text{orientacion}} = -0.30 \times |\theta_{\text{goal}}|$$

### 2. Reward Shaping Reactivo por LiDAR ($R_{\text{shaping}}$):
Se activa cuando el obstáculo frontal se detecta a una distancia normalizada $d_{\text{front}} < 0.30$ ($1.05\text{ metros}$ en métrica real):
* **Si el agente ejecuta Avance Recto (`Acción 0`) hacia el obstáculo**:
  $$R_{\text{shaping}} = -1.5 \times (0.30 - d_{\text{front}})$$
* **Si el agente ejecuta Giros de Evitación (`Acción 1` o `Acción 2`)**:
  $$R_{\text{shaping}} = +0.30$$

### 3. Recompensas Terminales ($R_{\text{terminal}}$):
* **Éxito (Llegada a Meta, $d_{\text{goal}} \le 0.25\text{ m}$)**: $+\mathbf{150.0}$
* **Colisión ($d_{\text{laser}} \le 0.35\text{ m}$)**: $-\mathbf{100.0}$
* **Timeout ($t \ge t_{\text{max}}$)**: $-\mathbf{80.0}$

---

## 📏 Distancias Físicas y Umbrales Calibrados

| Métrica del Sistema | Distancia Real en Metros | Valor Normalizado | Interpretación Físico-Robótica |
| :--- | :---: | :---: | :--- |
| **Alcance Máximo LiDAR** | $3.50\text{ m}$ | $1.00$ | Límite superior de lectura del sensor láser. |
| **Inicio de Evitación (Shaping)** | **$1.05\text{ m}$** | **$0.30$** | El sensor activa penalizaciones por avance recto y premia el giro evasivo. |
| **Umbral de Colisión (Impacto)** | **$0.35\text{ m}$ ($35\text{ cm}$)** | **$0.10$** | Límite de contacto físico. Detiene el episodio y liquida con $-100.0$. |
| **Margen de Maniobra Libre** | **$0.70\text{ m}$ ($70\text{ cm}$)** | — | Espacio neto para desacelerar y virar antes de colisionar. |

---

## 🎯 Modos de Metas Dinámicas (`goal_mode`)

El argumento `goal_mode` controla cómo se seleccionan las coordenadas de la meta $(x_{\text{goal}}, y_{\text{goal}})$ al inicio de cada episodio:

```text
                     [ MODO MEDIUM EN STAGE 2 ]
                        Meta 2: (1.2, 0.8)
                              \
   [Robot: -1.5, 0.0] ---> [ OBSTÁCULO ] ---> Meta 1: (1.5, 0.0)
                              /
                        Meta 3: (1.2, -0.8)
```

1. **`single`**: Meta fija frontal `(1.5, 0.0)`. Recomendado únicamente para verificar la convergencia inicial del gradiente en línea recta.
2. **`soft`**: Variaciones laterales moderadas `[(1.5, 0.0), (1.5, 0.25), (1.5, -0.25)]`. Enseña a la red a tolerar desviaciones leves.
3. **`medium`**: Metas distribuidas a la izquierda y derecha detrás de obstáculos `[(1.5, 0.0), (1.2, 0.8), (1.2, -0.8)]`. **Obliga al robot a aprender a esquivar por la izquierda o por la derecha según la ubicación de la meta.**
4. **`separated`**: Matriz multizona de orígenes y metas para entrenamiento generalizado y benchmarking riguroso.

---

## 🔄 Mecanismo de Reinicio de Score en Transfer Learning

Cuando se utiliza `resume_checkpoint` para transferir un modelo pre-entrenado:
* **El Problema**: Un modelo entrenado en Stage 1 (`single`) alcanza puntajes de **`+260.42`**. En Stage 2 (`medium`), debido a las maniobras de evitación, el puntaje máximo teórico es **`+145.0`**. Si el sistema mantuviera el score histórico, nunca reconocería un "Nuevo Mejor Modelo" en Stage 2.
* **La Solución**: `train_dqn_ros.py` compara la etapa (`stage`) y el modo de meta (`goal_mode`) del checkpoint con la sesión actual. **Si detecta cualquier cambio de configuración, reinicia automáticamente `best_score = -inf`**, permitiendo evaluar y guardar adecuadamente los nuevos pesos en `best_model.pth`.

---

## 💻 Requisitos e Instalación

### Requisitos del Sistema
* **Sistema Operativo**: Linux Ubuntu 22.04 LTS
* **ROS 2**: Humble Hawksbill (Desktop Install)
* **Simulador**: Gazebo Classic 11
* **Librerías Python**: PyTorch, NumPy, Pandas, Matplotlib

### Pasos de Compilación del Workspace
```bash
# 1. Crear e ingresar al workspace de ROS 2
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clonar el repositorio
git clone https://github.com/d3im3r/rl_environment.git .

# 3. Compilar los paquetes con colcon
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_custom_worlds turtlebot3_rl_training turtlebot3_eval_platform

# 4. Cargar el entorno
source install/setup.bash
```

---

## 🚀 Guía Exhaustiva de Entrenamiento (`train_dqn_stage.launch.py`)

El script `train_dqn_stage.launch.py` es el punto de entrada principal para entrenar modelos DQN.

### Comandos de Ejemplo por Caso de Uso

#### 1. Entrenamiento Estándar desde Cero (Stage 2 con Metas Dinámicas)
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

#### 2. Fine-Tuning en la Misma Etapa (Cargando `best_model.pth`)
```bash
ros2 launch turtlebot3_rl_training train_dqn_stage.launch.py \
    stage:=2 \
    goal_mode:=medium \
    gui:=true \
    episodes:=150 \
    resume_checkpoint:=/home/d3im3r/ros2_ws/src/train_runs/stage_2_medium_2026_08_16_112449/checkpoints/best_model.pth \
    epsilon_start:=0.30 \
    learning_rate:=0.0003 \
    max_steps:=80
```

#### 3. Transferencia de Aprendizaje (Stage 1 Single $\to$ Stage 2 Medium)
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

#### 4. Entrenamiento Rápido de Alta Velocidad (Sin Ventana Gráfica / Headless)
```bash
ros2 launch turtlebot3_rl_training train_dqn_stage.launch.py \
    stage:=3 \
    goal_mode:=medium \
    gui:=false \
    episodes:=400 \
    batch_size:=128 \
    learning_rate:=0.0005 \
    epsilon_decay:=0.997 \
    max_steps:=100
```

---

### Tabla Completa de Argumentos de Entrenamiento

| Argumento | Tipo | Valor Defecto | Valores Recomendados | Descripción Detallada |
| :--- | :---: | :---: | :---: | :--- |
| `stage` | `int` | `1` | `1`, `2`, `3`, ..., `7` | Escenario/Mundo de Gazebo a cargar (`d3im3r_stage_XX.world`). |
| `goal_mode` | `string` | `single` | `single`, `soft`, `medium`, `separated` | Modo de muestreo de metas entre episodios para evitar overfitting. |
| `gui` | `bool` | `true` | `true` (debug visual), `false` (rápido) | Activa u oculta la interfaz gráfica de Gazebo Classic. |
| `episodes` | `int` | `120` | `200` a `500` | Cantidad total de episodios a entrenar en la sesión. |
| `max_steps` | `int` | `40` | `80` (Stage 2), `120` (Stage 3+) | Límite máximo de pasos de control antes de declarar TIMEOUT. |
| `resume_checkpoint` | `string` | `""` | Ruta a `.pth` | Ruta absoluta al modelo `.pth` para reanudar o transferir pesos. |
| `epsilon_start` | `float` | `1.0` | `0.80` (scratch), `0.30` (fine-tuning) | Tasa inicial de exploración aleatoria ($\epsilon$). |
| `epsilon_decay` | `float` | `0.995` | `0.995` (200 eps), `0.997` (500 eps) | Factor de decaimiento geométrico por episodio para $\epsilon$. |
| `epsilon_end` | `float` | `0.05` | `0.05` | Valor mínimo absoluto al que decae la exploración $\epsilon$. |
| `learning_rate` | `float` | `0.001` | `0.001` (scratch), `0.0003` (fine-tuning) | Tasa de aprendizaje para el optimizador Adam de PyTorch. |
| `batch_size` | `int` | `64` | `64`, `128` | Tamaño del lote de transiciones extraídas del Replay Buffer. |
| `eval_every` | `int` | `25` | `25` | Frecuencia (en episodios) para evaluar la política con $\epsilon = 0.0$. |
| `base_dir` | `string` | `.../train_runs` | Ruta absoluta | Directorio raíz para guardar logs, métricas y checkpoints. |

---

## 📊 Guía Exhaustiva de Benchmarking (`benchmark.launch.py`)

El script `benchmark.launch.py` ejecuta pruebas de desempeño deterministas ($\epsilon = 0.0$) para evaluar comparativamente controladores.

### Comandos de Ejemplo por Agente

#### 1. Evaluación de Agente DQN (Apuntando al Mejor Modelo)
```bash
ros2 launch turtlebot3_eval_platform benchmark.launch.py \
    agent:=dqn \
    stage:=2 \
    episodes:=5 \
    launch_gazebo:=true \
    gui:=true \
    model_path:=/home/d3im3r/ros2_ws/src/train_runs/stage_2_medium_2026_08_16_112449/checkpoints/best_model.pth
```

#### 2. Evaluación de Agente de Lógica Difusa (Fuzzy Agent)
```bash
ros2 launch turtlebot3_eval_platform benchmark.launch.py \
    agent:=fuzzy \
    stage:=2 \
    episodes:=5 \
    launch_gazebo:=true \
    gui:=true
```

#### 3. Evaluación de Agente Reactivo Basado en Reglas (Rule-Based)
```bash
ros2 launch turtlebot3_eval_platform benchmark.launch.py \
    agent:=rule_based \
    stage:=2 \
    episodes:=5 \
    launch_gazebo:=true \
    gui:=true
```

---

### Tabla Completa de Argumentos de Benchmarking

| Argumento | Tipo | Valor Defecto | Opciones Válidas | Descripción Detallada |
| :--- | :---: | :---: | :---: | :--- |
| `agent` | `string` | `fuzzy` | `dqn`, `fuzzy`, `rule_based` | Tipo de agente o paradigma de control a evaluar. |
| `stage` | `int` | `1` | `1`, `2`, `3`, ..., `7` | Escenario/Mundo de Gazebo en el que se correrá el benchmark. |
| `episodes` | `int` | `1` | `5` a `10` | Cantidad de episodios de prueba por cada posición de meta evaluada. |
| `launch_gazebo` | `bool` | `true` | `true`, `false` | Lanza un nuevo proceso de Gazebo o se acopla a uno existente. |
| `gui` | `bool` | `true` | `true`, `false` | Muestra u oculta la interfaz gráfica durante el benchmark. |
| `model_path` | `string` | `""` | Ruta a `.pth` | Requerido para `agent:=dqn`. Apunta al archivo `best_model.pth`. |
| `output_dir` | `string` | `.../eval_runs` | Ruta absoluta | Directorio donde se guardan los logs detallados por episodio. |
| `csv_file` | `string` | `.../eval_history.csv` | Ruta a CSV | Archivo de registro centralizado para comparar corridas de distintas fechas. |

---

## 📈 Sistema de Registro de Datos e Historiales (Logs)

La plataforma utiliza una arquitectura de almacenamiento estructurada en dos niveles:

### 1. Registro Resumido Centralizado (`eval_history.csv`)
Almacena una fila consolidada tras completar cada sesión de benchmarking en `/home/d3im3r/ros2_ws/src/eval_history.csv`:

```text
timestamp,agent,checkpoint,world,n_obstacles,init_pos,goal_pos,episodes,avg_reward,success_rate,collision_rate,min_steps,avg_steps,max_steps,std_steps,run_dir
```

### 2. Registros Episódicos Detallados (`episode_history.csv`)
Almacena el detalle paso a paso de cada episodio dentro del directorio de la ejecución:
```text
episode,goal_x,goal_y,success,collision,timeout,reward,steps,path_length,avg_speed
```

---

## 🛠️ Diagnóstico y Solución de Problemas

### 1. Advertencias de Gazebo: `Negative sim time difference detected`
* **Causa**: Ocurre debido al reset instantáneo del reloj de simulación de Gazebo cuando el robot se teletransporta al inicio del episodio.
* **Solución**: Es un comportamiento esperado en ROS 2 / Gazebo Classic. **No afecta la lógica de entrenamiento ni el cálculo de gradientes**.

### 2. Liberación de Procesos Atascados en Fondo
Si una sesión de entrenamiento se interrumpe abruptamente y Gazebo no cierra correctamente, ejecuta:
```bash
killall -9 gzserver gzclient ros2 2>/dev/null || true
```

### 3. Verificación del Uso de GPU (PyTorch CUDA)
Para confirmar que el entrenamiento DQN está utilizando aceleración por GPU:
```bash
python3 -c "import torch; print('CUDA Disponible:', torch.cuda.is_available()); print('Dispositivo:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```
