#!/bin/bash
set -e

# Ambiente ROS + Gazebo
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh || true

# Overlay do workspace (só se existir)
if [ -f /home/autodrive_devkit/install/setup.bash ]; then
  source /home/autodrive_devkit/install/setup.bash
fi

# Exporta caminhos do f1tenth_dynsim (se o pacote já estiver instalado)
PKG_PREFIX="$(ros2 pkg prefix f1tenth_dynsim 2>/dev/null || echo /home/autodrive_devkit/install/f1tenth_dynsim)"
export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH}:${PKG_PREFIX}/share/f1tenth_dynsim"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${PKG_PREFIX}/share/f1tenth_dynsim/models"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}:${PKG_PREFIX}/lib"

cd /home/autodrive_devkit

# NÃO lance nada automaticamente aqui. Mantém o container vivo:
sleep infinity
