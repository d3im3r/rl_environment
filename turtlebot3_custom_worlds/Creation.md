# turtlebot3_custom_worlds

Paquete ROS 2 para crear, ejecutar y probar mundos personalizados de Gazebo Classic con TurtleBot3 en ROS 2 Humble.

Este paquete fue construido para trabajar con navegación autónoma, lógica difusa y aprendizaje por refuerzo usando TurtleBot3 Burger. Incluye mundos progresivos por etapas, cuadrícula de referencia métrica, ejes positivos visibles, meta visual movible y un gestor de episodios ejecutable con `ros2 run`.

---

## 1. Objetivo del paquete

El objetivo principal de `turtlebot3_custom_worlds` es permitir cargar diferentes escenarios de entrenamiento para TurtleBot3 sin modificar los paquetes oficiales de TurtleBot3.

El paquete permite:

- Cargar mundos personalizados mediante argumento corto `stage:=N`.
- Insertar TurtleBot3 Burger en una pose inicial definida.
- Visualizar una cuadrícula de 10 m x 10 m.
- Identificar fácilmente los ejes positivos:
  - Eje `+X`: rojo.
  - Eje `+Y`: verde.
- Visualizar una meta circular verde.
- Reiniciar episodios de entrenamiento RL sin cerrar Gazebo.
- Cambiar dinámicamente la posición de la meta.
- Ejecutar el gestor de episodios con `ros2 run`.
- Probar escenarios progresivos desde navegación directa hasta laberintos simples.

---

## 2. Estructura actual del paquete

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

Nota importante sobre `models/`:

```text
Cada modelo de Gazebo debe tener su propia carpeta.
No deben quedar archivos sueltos model.config o model.sdf directamente dentro de models/.
```

Estructura correcta:

```text
models/
├── d3im3r_goal_marker/
│   ├── model.config
│   └── model.sdf
└── d3im3r_grid/
    ├── model.config
    └── model.sdf
```

---

## 3. Convención del mundo

La convención usada en todos los mundos es:

```text
Sistema de referencia del mundo:

x positivo  -> hacia adelante del escenario
y positivo  -> hacia la izquierda del escenario
z positivo  -> hacia arriba
yaw = 0     -> robot mirando hacia +X
```

El plano principal tiene tamaño:

```text
10 m x 10 m
```

Por tanto, el mundo cubre aproximadamente:

```text
x = -5 m hasta x = 5 m
y = -5 m hasta y = 5 m
```

Cada cuadro de la cuadrícula equivale a:

```text
1 cuadro = 1 metro
```

Pose inicial recomendada del robot:

```text
x = -1.5
y =  0.0
z =  0.01
yaw = 0.0
```

Meta inicial típica:

```text
x = 1.5
y = 0.0
z = 0.03
```

---

## 4. Dependencias

Este paquete asume que ya tienes instalado ROS 2 Humble, Gazebo Classic y TurtleBot3.

Instalar dependencias principales:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-gazebo-msgs
sudo apt install python3-colcon-common-extensions
```

---

## 5. `package.xml`

El archivo `package.xml` debe incluir las dependencias necesarias para los mundos, el launch y el gestor de episodios RL:

```xml
<?xml version="1.0"?>
<package format="3">
    <name>turtlebot3_custom_worlds</name>
    <version>0.0.1</version>
    <description>Custom Gazebo worlds for TurtleBot3 navigation and reinforcement learning experiments.</description>

    <maintainer email="d3im3r@gmail.com">Deymer Miranda</maintainer>
    <license>MIT</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <exec_depend>launch</exec_depend>
    <exec_depend>launch_ros</exec_depend>

    <exec_depend>gazebo_ros</exec_depend>
    <exec_depend>gazebo_msgs</exec_depend>

    <exec_depend>rclpy</exec_depend>
    <exec_depend>geometry_msgs</exec_depend>
    <exec_depend>std_srvs</exec_depend>

    <exec_depend>turtlebot3</exec_depend>
    <exec_depend>turtlebot3_gazebo</exec_depend>
    <exec_depend>turtlebot3_description</exec_depend>

    <exec_depend>robot_state_publisher</exec_depend>
    <exec_depend>xacro</exec_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

## 6. `CMakeLists.txt`

El archivo `CMakeLists.txt` instala los mundos, modelos, archivos launch, configuraciones y el script `rl_episode_manager.py` como ejecutable ROS 2.

```cmake
cmake_minimum_required(VERSION 3.8)
project(turtlebot3_custom_worlds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files, world files, model files, and config files
install(
    DIRECTORY launch worlds models config
    DESTINATION share/${PROJECT_NAME}
)

# Install the episode manager script
install(
    PROGRAMS
        scripts/rl_episode_manager.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME rl_episode_manager
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```

Con esta configuración, el archivo físico sigue siendo:

```text
scripts/rl_episode_manager.py
```

pero se puede ejecutar como:

```bash
ros2 run turtlebot3_custom_worlds rl_episode_manager
```

---

## 7. Permisos del script `rl_episode_manager.py`

El archivo debe iniciar con:

```python
#!/usr/bin/env python3
```

Dar permisos de ejecución:

```bash
cd ~/ros2_ws/src/turtlebot3_custom_worlds

chmod +x scripts/rl_episode_manager.py
```

Verificar permisos:

```bash
ls -l scripts/rl_episode_manager.py
```

Debe verse algo similar a:

```text
-rwxr-xr-x
```

---

## 8. Launch principal

El archivo principal es:

```text
launch/d3im3r_world.launch.py
```

Este launch permite cargar mundos con argumento corto:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1
```

Relación entre `stage` y mundo:

```text
stage:=0 -> d3im3r_stage_00_empty.world
stage:=1 -> d3im3r_stage_01_direct_goal.world
stage:=2 -> d3im3r_stage_02_front_obstacle.world
stage:=3 -> d3im3r_stage_03_left_right_choice.world
stage:=4 -> d3im3r_stage_04_corridor.world
stage:=5 -> d3im3r_stage_05_narrow_door.world
stage:=6 -> d3im3r_stage_06_random_obstacles.world
stage:=7 -> d3im3r_stage_07_simple_maze.world
```

El launch también:

- Configura `TURTLEBOT3_MODEL=burger`.
- Define `GAZEBO_MODEL_PATH`.
- Desactiva la base de datos externa de Gazebo con `GAZEBO_MODEL_DATABASE_URI=''`.
- Carga el mundo.
- Inserta TurtleBot3 desde el modelo SDF oficial.
- Permite configurar la pose inicial del robot.

Argumentos disponibles:

```text
stage   -> etapa o mundo a cargar
model   -> burger, waffle o waffle_pi
x_pose  -> posición inicial X del robot
y_pose  -> posición inicial Y del robot
z_pose  -> posición inicial Z del robot
yaw     -> orientación inicial del robot
```

Ejemplo:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py \
stage:=5 \
x_pose:=-1.5 \
y_pose:=0.0 \
z_pose:=0.01 \
yaw:=0.0
```

---

## 9. Cuadrícula del mundo

La cuadrícula está definida como modelo reutilizable:

```text
models/d3im3r_grid/model.sdf
```

Características:

```text
Tamaño total: 10 m x 10 m
Separación: 1 m
Eje +X: rojo
Eje +Y: verde
Lados negativos: líneas normales de cuadrícula
```

Interpretación:

```text
La esquina o nacimiento de la L roja-verde representa el origen (0, 0).

Línea roja  -> dirección positiva de X
Línea verde -> dirección positiva de Y
```

Ejemplos:

```text
(-1.5, 0.0) -> 1.5 m hacia el lado negativo de X
( 1.5, 0.0) -> 1.5 m hacia el lado positivo de X
( 0.0, 1.0) -> 1.0 m hacia el lado positivo de Y
( 0.0,-1.0) -> 1.0 m hacia el lado negativo de Y
```

---

## 10. Meta visual movible

Para entrenamiento RL, la meta debe poder moverse durante los episodios. Por eso debe estar definida con:

```xml
<static>false</static>
```

Bloque recomendado para `goal_marker` dentro de los `.world`:

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

Puntos clave:

```text
<static>false</static>      -> permite mover la meta
<gravity>false</gravity>    -> evita que caiga
<kinematic>true</kinematic> -> se comporta como objeto controlado por pose
sin collision               -> el robot no choca con la meta
z = 0.03                    -> queda visible sobre la cuadrícula
```

---

## 11. Plugin para mover entidades en Gazebo

Para mover el robot y la meta durante los episodios, los mundos deben incluir el plugin:

```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros>
        <namespace>/gazebo</namespace>
    </ros>
    <update_rate>20.0</update_rate>
</plugin>
```

Debe ir dentro del bloque:

```xml
<world name="...">
    ...
</world>
```

Ejemplo:

```xml
<world name="d3im3r_stage_05_narrow_door">

    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
        <ros>
            <namespace>/gazebo</namespace>
        </ros>
        <update_rate>20.0</update_rate>
    </plugin>

    ...
</world>
```

Verificar servicios:

```bash
ros2 service list | grep entity
```

Resultado esperado:

```text
/gazebo/get_entity_state
/gazebo/set_entity_state
```

---

## 12. Descripción de los stages

### Stage 00: mundo vacío

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=0
```

Objetivo:

```text
Verificar que Gazebo, TurtleBot3, /cmd_vel, /odom y /scan funcionan correctamente.
```

Contenido:

```text
- Plano gris
- Cuadrícula
- TurtleBot3
- Sin meta
- Sin obstáculos
```

---

### Stage 01: meta directa

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1
```

Objetivo:

```text
Validar navegación directa hacia una meta sin obstáculos.
```

Contenido:

```text
- Robot en (-1.5, 0.0)
- Meta en (1.5, 0.0)
- Sin obstáculos
```

---

### Stage 02: obstáculo frontal

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=2
```

Objetivo:

```text
Evaluar evasión básica de un obstáculo ubicado entre el robot y la meta.
```

Contenido:

```text
- Robot en (-1.5, 0.0)
- Meta en (1.5, 0.0)
- Obstáculo cerca del centro del mundo
```

---

### Stage 03: decisión izquierda/derecha

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=3
```

Objetivo:

```text
Forzar al robot a interpretar obstáculos laterales y decidir una trayectoria.
```

Contenido:

```text
- Dos obstáculos simétricos
- Meta en el eje X positivo
```

---

### Stage 04: corredor

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=4
```

Objetivo:

```text
Evaluar movimiento estable dentro de un pasillo.
```

Contenido:

```text
- Dos paredes paralelas
- Meta al final del corredor
```

---

### Stage 05: chicane o desvío controlado

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=5
```

Objetivo:

```text
Obligar al robot a realizar un desvío suave antes de llegar a la meta.
```

Contenido:

```text
- Dos paredes desplazadas
- Un bloque lateral
- Meta final
```

Este mundo reemplaza la idea inicial de una puerta angosta.

---

### Stage 06: obstáculos distribuidos

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=6
```

Objetivo:

```text
Evaluar navegación en un entorno con varios obstáculos.
```

Contenido:

```text
- Varios bloques distribuidos
- Meta desplazada del eje X
```

---

### Stage 07: laberinto simple

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=7
```

Objetivo:

```text
Evaluar navegación más exigente con paredes internas.
```

Contenido:

```text
- Laberinto simple
- Meta en zona superior derecha
```

---

## 13. Compilación

Compilación normal:

```bash
cd ~/ros2_ws

colcon build --packages-select turtlebot3_custom_worlds

source install/setup.bash
```

Compilación limpia:

```bash
cd ~/ros2_ws

rm -rf build/turtlebot3_custom_worlds
rm -rf install/turtlebot3_custom_worlds

colcon build --packages-select turtlebot3_custom_worlds

source install/setup.bash
```

---

## 14. Verificar ejecutables del paquete

Después de instalar `rl_episode_manager.py` desde `CMakeLists.txt`, verificar:

```bash
ros2 pkg executables turtlebot3_custom_worlds
```

Resultado esperado:

```text
turtlebot3_custom_worlds rl_episode_manager
```

---

## 15. Ejecución básica de mundos

Antes de lanzar Gazebo, cerrar procesos anteriores:

```bash
pkill -f gzserver
pkill -f gzclient
```

Lanzar stage 1:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1
```

Lanzar stage 5:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=5
```

Lanzar stage 7:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=7
```

---

## 16. Probar movimiento del TurtleBot3

En otra terminal:

```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Avanzar:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Girar:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Detener:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 17. Tópicos importantes

Ver tópicos disponibles:

```bash
ros2 topic list
```

Tópicos esperados:

```text
/cmd_vel
/odom
/scan
/tf
/tf_static
/joint_states
/clock
```

Ver odometría:

```bash
ros2 topic echo /odom --once
```

Ver LiDAR:

```bash
ros2 topic echo /scan --once
```

---

## 18. Servicios importantes

Ver servicio de spawn:

```bash
ros2 service list | grep spawn
```

Resultado esperado:

```text
/spawn_entity
```

Ver servicios de entidades:

```bash
ros2 service list | grep entity
```

Resultado esperado si el plugin está cargado:

```text
/gazebo/get_entity_state
/gazebo/set_entity_state
```

---

## 19. Probar movimiento manual de la meta

Con Gazebo abierto:

```bash
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState "{
state: {
name: 'goal_marker',
pose: {
position: {x: 1.5, y: 1.0, z: 0.03},
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
},
twist: {
linear: {x: 0.0, y: 0.0, z: 0.0},
angular: {x: 0.0, y: 0.0, z: 0.0}
},
reference_frame: 'world'
}
}"
```

Si la meta cambia de posición, el sistema está listo para reinicios de episodios RL.

---

## 20. Reinicio de episodios para RL

Para entrenamiento RL no se recomienda cerrar y abrir Gazebo en cada episodio.

Flujo recomendado:

```text
1. Cargar el stage una sola vez.
2. Detener el robot.
3. Reiniciar la simulación.
4. Reubicar el robot.
5. Reubicar la meta.
6. Esperar una pequeña pausa.
7. Iniciar el episodio.
```

El script encargado es:

```text
scripts/rl_episode_manager.py
```

Ahora también se puede ejecutar con:

```bash
ros2 run turtlebot3_custom_worlds rl_episode_manager
```

---

## 21. Ejecutar `rl_episode_manager`

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash

pkill -f gzserver
pkill -f gzclient

ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=5
```

Terminal 2:

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 run turtlebot3_custom_worlds rl_episode_manager
```

Resultado esperado:

```text
Waiting for /reset_simulation...
Waiting for /gazebo/set_entity_state...
RL Episode Manager ready.
Test episode 1
Resetting RL episode...
turtlebot3 moved to x=-1.50, y=0.00, yaw=0.00
goal_marker moved to x=1.50, y=0.00, yaw=0.00
Episode reset complete.
```

En Gazebo debe observarse:

```text
- El robot vuelve a la pose inicial.
- La meta cambia de posición.
- El stage no se cierra.
- Los obstáculos permanecen.
```

---

## 22. Recomendación de curriculum learning

Para entrenar con RL, se recomienda avanzar progresivamente:

```text
Stage 01 -> Meta directa sin obstáculos
Stage 02 -> Obstáculo frontal
Stage 03 -> Decisión izquierda/derecha
Stage 04 -> Corredor
Stage 05 -> Chicane
Stage 06 -> Obstáculos distribuidos
Stage 07 -> Laberinto simple
```

Estrategia inicial:

```text
1000 episodios en stage 1
2000 episodios en stage 2
3000 episodios en stage 3
5000 episodios en stage 5
Luego mezclar stages para evaluar generalización
```

Para empezar, mantener fijo el robot:

```text
robot_start = (-1.5, 0.0, 0.0)
```

Y variar solo la meta:

```text
goal = (x_goal, y_goal)
```

---

## 23. Troubleshooting

### Problema 1: `ros2 run` no encuentra `rl_episode_manager`

Síntoma:

```bash
ros2 run turtlebot3_custom_worlds rl_episode_manager
```

Error:

```text
No executable found
```

Soluciones:

1. Revisar permisos:

```bash
ls -l ~/ros2_ws/src/turtlebot3_custom_worlds/scripts/rl_episode_manager.py
```

Debe tener permisos de ejecución:

```text
-rwxr-xr-x
```

Si no:

```bash
chmod +x ~/ros2_ws/src/turtlebot3_custom_worlds/scripts/rl_episode_manager.py
```

2. Verificar `CMakeLists.txt`:

```cmake
install(
    PROGRAMS
        scripts/rl_episode_manager.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME rl_episode_manager
)
```

3. Compilar limpio:

```bash
cd ~/ros2_ws

rm -rf build/turtlebot3_custom_worlds
rm -rf install/turtlebot3_custom_worlds

colcon build --packages-select turtlebot3_custom_worlds

source install/setup.bash
```

4. Confirmar ejecutable:

```bash
ros2 pkg executables turtlebot3_custom_worlds
```

Debe aparecer:

```text
turtlebot3_custom_worlds rl_episode_manager
```

---

### Problema 2: siempre carga el mundo vacío

Síntoma:

```text
Cambio stage:=1, stage:=2 o stage:=5, pero Gazebo siempre muestra el plano vacío.
```

Soluciones:

```bash
pkill -f gzserver
pkill -f gzclient
```

Compilar limpio:

```bash
cd ~/ros2_ws

rm -rf build/turtlebot3_custom_worlds
rm -rf install/turtlebot3_custom_worlds

colcon build --packages-select turtlebot3_custom_worlds

source install/setup.bash
```

Verificar en consola:

```text
Loading custom world: .../d3im3r_stage_05_narrow_door.world
```

---

### Problema 3: la meta no aparece

Causas comunes:

```text
- El modelo goal_marker no está dentro del .world.
- La meta tiene z muy bajo y queda oculta por la cuadrícula.
- Gazebo está cargando una versión antigua del mundo.
- No se recompiló después de editar.
```

Solución:

Usar:

```xml
<pose>1.5 0.0 0.03 0 0 0</pose>
```

Compilar y relanzar:

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_custom_worlds
source install/setup.bash

pkill -f gzserver
pkill -f gzclient

ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1
```

---

### Problema 4: el robot gira cuando debería avanzar

Síntoma:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.15}, angular: {z: 0.0}}"
```

El robot gira sobre su eje.

Causa probable:

```text
El robot fue insertado manualmente con un URDF/Xacro incorrecto o incompleto.
```

Solución:

```text
Usar el modelo SDF oficial desde turtlebot3_gazebo/models/turtlebot3_burger/model.sdf.
```

El launch actual ya hace esto.

---

### Problema 5: el robot se ve gris o sin texturas

Causa probable:

```text
Gazebo no encuentra correctamente los modelos o materiales de TurtleBot3.
```

Solución:

Verificar que `GAZEBO_MODEL_PATH` incluya:

```text
turtlebot3_custom_worlds/models
turtlebot3_gazebo/models
```

Limpiar variables contaminadas:

```bash
unset GAZEBO_MODEL_PATH
unset GAZEBO_RESOURCE_PATH
unset GAZEBO_PLUGIN_PATH
```

---

### Problema 6: muchos errores `Missing model.config`

Síntoma:

```text
[Err] [InsertModelWidget.cc:403] Missing model.config
```

Causa:

```text
GAZEBO_MODEL_PATH está apuntando a carpetas que contienen paquetes ROS, no modelos de Gazebo.
```

Solución:

No concatenar rutas anteriores de `GAZEBO_MODEL_PATH` si están contaminadas.

Limpiar antes de lanzar:

```bash
unset GAZEBO_MODEL_PATH
unset GAZEBO_RESOURCE_PATH
unset GAZEBO_PLUGIN_PATH
```

---

### Problema 7: Gazebo se queda en `Getting models from http://models.gazebosim.org`

Causa:

```text
Gazebo intenta descargar modelos externos como sun o ground_plane.
```

Solución:

Definir la luz y el plano directamente dentro del `.world`.

También usar:

```python
SetEnvironmentVariable(
    name='GAZEBO_MODEL_DATABASE_URI',
    value=''
)
```

---

### Problema 8: `/gazebo/model_states` no aparece

Síntoma:

```bash
ros2 topic echo /gazebo/model_states --once
```

Salida:

```text
WARNING: topic [/gazebo/model_states] does not appear to be published yet
```

Solución recomendada:

Usar `/odom` para verificar pose del robot:

```bash
ros2 topic echo /odom --once
```

Para mover modelos, usar los servicios:

```bash
ros2 service list | grep entity
```

---

### Problema 9: `/gazebo/set_entity_state` no aparece

Solución:

Agregar el plugin:

```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros>
        <namespace>/gazebo</namespace>
    </ros>
    <update_rate>20.0</update_rate>
</plugin>
```

Luego compilar y relanzar.

---

### Problema 10: el robot se resetea pero la meta no se mueve

Causa:

```text
La meta está definida como static=true.
```

Solución:

Usar:

```xml
<model name="goal_marker">
    <static>false</static>
    ...
</model>
```

Y mover la meta con `z=0.03`.

---

### Problema 11: la cuadrícula no se actualiza

Solución:

```bash
pkill -f gzserver
pkill -f gzclient

cd ~/ros2_ws

rm -rf build/turtlebot3_custom_worlds
rm -rf install/turtlebot3_custom_worlds

colcon build --packages-select turtlebot3_custom_worlds

source install/setup.bash
```

---

## 24. Flujo recomendado de trabajo

Cada vez que edites mundos, modelos, launch o scripts:

```bash
cd ~/ros2_ws

colcon build --packages-select turtlebot3_custom_worlds

source install/setup.bash

pkill -f gzserver
pkill -f gzclient

ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1
```

Si no se reflejan los cambios:

```bash
cd ~/ros2_ws

rm -rf build/turtlebot3_custom_worlds
rm -rf install/turtlebot3_custom_worlds

colcon build --packages-select turtlebot3_custom_worlds

source install/setup.bash
```

---

## 25. Prueba rápida completa

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash

pkill -f gzserver
pkill -f gzclient

ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=5
```

Terminal 2:

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Terminal 3:

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 run turtlebot3_custom_worlds rl_episode_manager
```

---

## 26. Próximos pasos sugeridos

Después de validar los mundos, los siguientes pasos naturales son:

```text
1. Crear un nodo rl_interface_node.py.
2. Publicar estado normalizado en /rl_state.
3. Leer /scan, /odom y tf.
4. Calcular distancia frontal, izquierda y derecha.
5. Calcular distancia y ángulo hacia la meta.
6. Publicar /rl_goal_reached.
7. Crear un entorno Gymnasium para DQN.
8. Usar reset_episode() al inicio de cada episodio.
```

Estado recomendado para RL:

```text
[d_front_norm, d_left_norm, d_right_norm, theta_goal_norm, d_goal_norm]
```

Acciones discretas recomendadas:

```text
0 -> avanzar
1 -> girar izquierda
2 -> girar derecha
```

---

## 27. Resumen

Este paquete permite trabajar con TurtleBot3 en Gazebo de forma controlada y progresiva.

Características principales:

```text
- Mundos por stages.
- Argumento corto stage:=N.
- Cuadrícula métrica de 1 m.
- Ejes positivos visibles.
- Meta visual verde.
- Meta movible para entrenamiento RL.
- Reinicio de episodios sin cerrar Gazebo.
- Gestor de episodios ejecutable con ros2 run.
- Escenarios simples, intermedios y avanzados.
```

Este entorno queda preparado para navegación clásica, lógica difusa, aprendizaje por refuerzo y comparación experimental.
