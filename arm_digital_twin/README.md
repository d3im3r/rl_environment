
# 🤖 Arm Digital Twin (ROS 2 + RViz)

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![Build](https://img.shields.io/badge/Build-colcon-blue)
![Status](https://img.shields.io/badge/Status-Academic%20Use-success)
![License](https://img.shields.io/badge/License-Educational-lightgrey)



Este repositorio contiene un **gemelo digital de un brazo robótico de 3 grados de libertad (3 GDL) con gripper**, desarrollado en ROS 2.

El objetivo es permitir la **visualización en RViz del robot y su movimiento en tiempo real**, facilitando la integración con sistemas físicos (ESP32 + micro-ROS) sin que el estudiante tenga que construir el modelo desde cero.

---

## 🎯 Propósito

Este paquete ha sido diseñado para:

- Visualizar un manipulador robótico en RViz.
- Controlar el robot mediante comandos articulares.
- Reflejar el movimiento de un robot físico en un entorno virtual.
- Servir como base para integración con micro-ROS y hardware real.

---

## 🧱 Estructura del Proyecto

```

arm_digital_twin_ws/
├── src/
│   └── arm_digital_twin/
│       ├── arm_digital_twin/
│       ├── launch/
│       ├── rviz/
│       ├── urdf/
│       ├── config/
│       ├── resource/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       └── README.md

````

---

## ⚙️ Requisitos

### Sistema operativo
- Ubuntu 22.04

### ROS 2
- ROS 2 Humble Hawksbill

### Dependencias necesarias

```bash
sudo apt update
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui
````

---

## 📥 Instalación

### 1. Clonar el repositorio

```bash
cd ~
git clone <URL_DEL_REPOSITORIO>
```

### 2. Entrar al workspace

```bash
cd arm_digital_twin_ws
```

### 3. Compilar

```bash
colcon build --symlink-install
```

### 4. Cargar el entorno

```bash
source install/setup.bash
```

---

## 🚀 Ejecución

Lanzar el gemelo digital:

```bash
ros2 launch arm_digital_twin display.launch.py
```

Esto abrirá automáticamente:

* RViz
* Modelo del brazo robótico
* Configuración lista para visualización

---

## 🎮 Control del Robot

El robot se controla mediante el tópico:

```
/arm_cmd
```

### Tipo de mensaje

```
sensor_msgs/msg/JointState
```

---

## 🧪 Ejemplo de control completo

```bash
ros2 topic pub --once /arm_cmd sensor_msgs/msg/JointState \
"{name: ['joint1','joint2','joint3','gripper_joint'], position: [0.6, 0.4, 1.0, 0.03]}"
```

---

## 📌 Orden de articulaciones

```
joint1 → base  
joint2 → hombro  
joint3 → codo  
gripper_joint → pinza
```

---

## 🎯 Control individual de articulaciones

También es posible controlar una sola articulación:

### Mover solo el hombro

```bash
ros2 topic pub /arm_cmd sensor_msgs/msg/JointState \
"{name: ['joint2'], position: [0.8]}" --once
```

### Mover solo el gripper

```bash
ros2 topic pub /arm_cmd sensor_msgs/msg/JointState \
"{name: ['gripper_joint'], position: [0.02]}" --once
```

---

## ⚠️ Consideraciones importantes

* Las posiciones están en **radianes**.
* El orden de `name` y `position` debe coincidir.
* Puedes enviar uno o varios joints en el mismo mensaje.
* Si un joint no se envía, mantiene su última posición.
* Los nombres de articulaciones deben coincidir exactamente.

---

## 🔄 Flujo del sistema

```
Comando ROS 2 → /arm_cmd → cmd_to_joint_states → JointStates → RViz
```

---

## 🔗 Integración con Hardware

Este gemelo digital está diseñado para integrarse con:

* ESP32 + micro-ROS
* Control por potenciómetros
* Nodos ROS 2 personalizados

Ejemplo de flujo completo:

```
ESP32 / Nodo ROS → /arm_cmd → ROS 2 → Gemelo digital (RViz)
```

---

## 🧠 Recomendaciones

1. Ejecutar primero el gemelo digital.
2. En otra terminal, enviar comandos manuales.
3. Verificar movimiento en RViz.
4. Luego integrar con hardware.

---

## 👨‍🏫 Uso Académico

Este proyecto está diseñado para apoyar el aprendizaje en:

* Robótica industrial
* Cinemática directa
* ROS 2
* micro-ROS
* Integración hardware-software

---

## 📄 Licencia

Uso académico y educativo.

---

## ✨ Autor

Desarrollado por
**Deimer Miranda Montoya**

---

## 🚀 Frase del proyecto

> *“El robot físico ejecuta… el gemelo digital lo representa.”*

---


