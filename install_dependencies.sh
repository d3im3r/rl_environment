#!/usr/bin/env bash
# ==============================================================================
# Script de Instalación Autónoma de Dependencias para TurtleBot3 RL Platform
# Sistema Operativo: Ubuntu 22.04 LTS / ROS 2 Humble Hawksbill
# ==============================================================================

set -e

# Colores para mensajes en la terminal
RED='\031[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # Sin color

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}      Instalación Autónoma: TurtleBot3 RL Navigation Platform        ${NC}"
echo -e "${BLUE}======================================================================${NC}"

# 1. Actualizar repositorio de paquetes apt
echo -e "\n${YELLOW}[1/5] Actualizando lista de paquetes del sistema (apt update)...${NC}"
sudo apt update

# 2. Instalación de paquetes de ROS 2 Humble y Gazebo Classic
echo -e "\n${YELLOW}[2/5] Instalando dependencias de ROS 2 Humble y TurtleBot3 Gazebo...${NC}"
sudo apt install -y \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-description \
    ros-humble-gazebo-ros-pkgs \
    python3-pip \
    ffmpeg

# 3. Instalación y actualización de herramientas de compilación Python
echo -e "\n${YELLOW}[3/5] Actualizando herramientas de compilación de Python (setuptools, packaging)...${NC}"
pip3 install --upgrade setuptools packaging

# 4. Instalación de librerías de Inteligencia Artificial y Ciencia de Datos (numpy<2)
echo -e "\n${YELLOW}[4/5] Instalando PyTorch, NumPy (<2.0), Pandas y Matplotlib...${NC}"
pip3 install "numpy<2" torch torchvision pandas matplotlib

# 5. Configurar la variable de entorno TURTLEBOT3_MODEL=burger
echo -e "\n${YELLOW}[5/5] Configurando variable de entorno TURTLEBOT3_MODEL=burger...${NC}"
if ! grep -q "export TURTLEBOT3_MODEL=burger" ~/.bashrc; then
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    echo -e "${GREEN}✓ Agregado 'export TURTLEBOT3_MODEL=burger' a ~/.bashrc${NC}"
else
    echo -e "${GREEN}✓ La variable TURTLEBOT3_MODEL ya está configurada en ~/.bashrc${NC}"
fi
export TURTLEBOT3_MODEL=burger

echo -e "\n${BLUE}======================================================================${NC}"
echo -e "${GREEN} ¡Instalación de dependencias completada exitosamente! 🎉 ${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo -e "${YELLOW}Para compilar y cargar el workspace ejecuta:${NC}"
echo -e "  ${GREEN}cd ~/ros2_ws${NC}"
echo -e "  ${GREEN}source /opt/ros/humble/setup.bash${NC}"
echo -e "  ${GREEN}colcon build${NC}"
echo -e "  ${GREEN}source install/setup.bash${NC}\n"
