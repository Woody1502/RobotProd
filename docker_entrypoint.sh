#!/bin/bash
set -e

# Адрес WiFi-RS485 адаптера. По умолчанию — настоящее железо; для теста
# против эмулятора (modbus_emulator) переопределить VIM_HOST/VIM_PORT
# (см. docker-compose.yml).
VIM_HOST="${VIM_HOST:-192.168.5.42}"
VIM_PORT="${VIM_PORT:-81}"

# lock-файл от предыдущего socat может остаться при крэше контейнера
# (restart: unless-stopped сохраняет /tmp) — удаляем, иначе новый socat
# зависнет на waitlock и /dev/ttyVCOM1 никогда не создастся
rm -f /tmp/ttyVCOM1.lock

echo "Starting socat inside container (target: ${VIM_HOST}:${VIM_PORT})..."
socat -d -d pty,raw,echo=0,link=/dev/ttyVCOM1,waitlock=/tmp/ttyVCOM1.lock,nonblock=1 tcp:${VIM_HOST}:${VIM_PORT} &
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
    socat -d -d pty,raw,echo=0,link=/dev/ttyVCOM1,waitlock=/tmp/ttyVCOM1.lock tcp:${VIM_HOST}:${VIM_PORT} &
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
