#!/bin/bash
# Останавливает все три контейнера, поднятые emu.sh: robot, modbus_emulator, agro_gui.
# Не удаляет контейнеры (docker compose stop, не down) — emu.sh потом просто
# поднимет их обратно без пересборки, если код не менялся.

echo "=== robot ==="
docker compose stop robot

echo "=== modbus_emulator ==="
docker compose --profile sim stop modbus_emulator

echo "=== agro_gui ==="
(cd /home/alex/AgroGUI && docker compose stop agro_gui)

echo "=== готово ==="
docker ps -a --filter name=modbus_emulator --filter name=agrorobot_prod --filter name=agro_gui --format '{{.Names}}\t{{.Status}}'
