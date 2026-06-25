#!/bin/bash
set -e

echo "Starting socat inside container..."
socat -d -d pty,raw,echo=0,link=/dev/ttyVCOM1,waitlock=/tmp/ttyVCOM1.lock,nonblock=1 tcp:192.168.5.42:81 &
SOCAT_PID=$!

echo "=========================================="
echo "Starting AgriRobot Production Container"
echo "=========================================="

# Проверка minimalmodbus
if python3 -c "import minimalmodbus" 2>/dev/null; then
    echo "✓ minimalmodbus installed"
else
    echo "⚠ minimalmodbus not found, installing..."
    pip3 install minimalmodbus pynput --break-system-packages || true
fi

# Проверка устройства
if [ -e /dev/ttyVCOM1 ]; then
    echo "✓ /dev/ttyVCOM1 exists"
    ls -la /dev/ttyVCOM1
    
    # Исправляем права
    echo "Fixing permissions..."
    chmod 666 /dev/ttyVCOM1
    chown root:dialout /dev/ttyVCOM1
    
    # Проверяем, открыт ли порт
    echo "Testing port..."
    if python3 -c "import serial; s=serial.Serial('/dev/ttyVCOM1', 38400, timeout=1); s.close(); print('OK')" 2>/dev/null; then
        echo "✓ Port is accessible"
    else
        echo "⚠ Port is not accessible"
    fi
else
    echo "⚠ /dev/ttyVCOM1 not found!"
    # Создаем порт через socat внутри контейнера
    echo "Starting socat inside container..."
    socat -d -d pty,raw,echo=0,link=/dev/ttyVCOM1,waitlock=/tmp/ttyVCOM1.lock tcp:192.168.5.42:81 &
    sleep 3
    chmod 666 /dev/ttyVCOM1
fi
# Симлинк для совместимости с hardcoded путями в URDF (напр. /home/alexey/ros2_ws)
mkdir -p /home/alexey
ln -sfn /ros2_ws /home/alexey/ros2_ws 2>/dev/null || true

source /opt/ros/jazzy/setup.bash

# Подключить overlay если есть сборка
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
