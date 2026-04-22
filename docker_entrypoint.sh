#!/bin/bash
set -e

# Симлинк для совместимости с hardcoded путями в URDF (напр. /home/alexey/ros2_ws)
mkdir -p /home/alexey
ln -sfn /ros2_ws /home/alexey/ros2_ws 2>/dev/null || true

source /opt/ros/jazzy/setup.bash

# Подключить overlay если есть сборка
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
