#!/bin/bash
set -e

# Поднимает все три контейнера для теста против эмулятора вместо реального робота:
#   1. modbus_emulator — эмулятор WiFi-RS485 адаптера
#   2. robot           — основной стек (rs485_bridge и т.д.), подключаем к эмулятору
#   3. agro_gui        — GUI (отдельный docker-compose.yml в AgroGUI)
# build перед up — иначе запущенный контейнер тихо продолжит работать со старым
# образом, если исходники менялись после последней пересборки.

echo "=== modbus_emulator ==="
docker compose --profile sim build modbus_emulator
docker compose --profile sim up -d modbus_emulator

echo "=== robot (VIM_HOST=127.0.0.1 -> эмулятор, не настоящее железо) ==="
docker compose build robot
VIM_HOST=127.0.0.1 VIM_PORT=81 docker compose up -d robot

echo "=== agro_gui ==="
(cd /home/alex/AgroGUI && docker compose build agro_gui && docker compose up -d agro_gui)

echo "=== готово ==="
docker ps --filter name=modbus_emulator --filter name=agrorobot_prod --filter name=agro_gui --format '{{.Names}}\t{{.Status}}'