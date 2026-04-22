# Ros2AgrobotProd — документация

Прод-версия для реального робота. Без симуляции. Запускается на мини-ПК (Thunderrobot RTX 4060).

---

## Структура проекта

```
Ros2AgrobotProd/
├── Dockerfile                          базовый образ без Gazebo, с pyserial
├── docker-compose.yml                  host network, проброс /dev/ttyUSB*
├── docker_entrypoint.sh
└── src/
    ├── my_robot/
    │   ├── config/
    │   │   ├── ekf.yaml                параметры Extended Kalman Filter (локализация)
    │   │   └── field_params.yaml       геометрия поля, скорости, параметры миссии
    │   ├── launch/
    │   │   └── robot.launch.py         единственный launch для прода
    │   ├── meshes/                     STL-модели (нужны robot_state_publisher → TF)
    │   ├── urdf/
    │   │   └── fito.urdf               описание робота
    │   └── my_robot/
    │       ├── rs485_bridge.py         ← главный узел, шлёт команды на RS-485
    │       ├── acker_odom.py           одометрия (Акерман)
    │       ├── row_driver.py           поддержание скорости при автопилоте
    │       └── field_mission_node.py   исполнение миссии (развороты, смена рядов)
    └── visual_multi_crop_row_navigation/
        └── ...                         визуальная сервировка (следование по ряду)
```

---

## Граф данных

```
AgroGUI (ПК оператора, DDS) ──► /velocity_controller/commands ──────┐
                             ──► /position_controller/commands ──────┤
                             ──► /camera_tilt_controller/commands ───┤
                                                                      │
Камера                                                                │
  └─► vs_navigation ──► /position_controller/commands ───────────────┤
                                                                     │
field_mission ──► /mission/vs_active                                  │
             ──► /autopilot/enable                                    │
             ──► /velocity_controller/commands ───────────────────────┤
                                                                      │
row_driver ──► /velocity_controller/commands ─────────────────────────┤
                                                                      ▼
                                                              rs485_bridge
                                                               │        │
                                                    /joint_states     RS-485
                                                         │           /dev/ttyUSB0
                                               robot_state_publisher     │
                                                    TF-дерево         Контроллер
                                                                       двигателей
                                                  acker_odom
                                                  ekf_filter_node_odom  ──► /odometry/local
                                                  ekf_filter_node_map   ──► /odometry/global
```

---

## Ноды

| Нода | Исполняемый | Назначение |
|---|---|---|
| `rs485_bridge` | `rs485_bridge` | Отправляет команды на RS-485, публикует `/joint_states` |
| `robot_state_publisher` | ros пакет | Публикует TF-дерево из URDF + `/joint_states` |
| `acker_odom` | `acker_odom` | Считает одометрию по кинематике Акермана |
| `ekf_filter_node_odom` | robot_localization | EKF: odom + IMU → `/odometry/local` |
| `ekf_filter_node_map` | robot_localization | EKF: odom + GPS + IMU → `/odometry/global` |
| `row_driver` | `row_driver` | Поддерживает постоянную скорость пока активен автопилот |
| `field_mission` | `field_mission` | Управляет миссией (ряды, развороты, headland) |
| `vs_navigation` | `vs_navigation` | Визуальная сервировка, держит робот в ряду |

### Порядок старта (из `robot.launch.py`)

| Время | Ноды |
|---|---|
| 0 с | robot_state_publisher, rs485_bridge, ekf_odom, ekf_map |
| +2 с | acker_odom |
| +4 с | row_driver |
| +6 с | field_mission, vs_navigation |

---

## Протокол RS-485

### Физический уровень

| Параметр | Значение |
|---|---|
| Порт | `/dev/ttyUSB0` (настраивается параметром `port`) |
| Скорость | 115200 бод |
| Биты данных | 8 |
| Чётность | нет |
| Стоп-бит | 1 |
| Тайм-аут чтения | 100 мс |
| Частота отправки | 20 Гц (каждые 50 мс) |

### Структура фрейма

Каждые 50 мс `rs485_bridge` отправляет один фрейм из **28 байт**:

```
Байт(ы)     Размер   Тип        Описание
──────────────────────────────────────────────────────────────────────
0           1        uint8      Старт-байт 1: 0xAA
1           1        uint8      Старт-байт 2: 0x55
2–5         4        float32 LE Скорость переднего правого колеса (рад/с)
6–9         4        float32 LE Скорость переднего левого колеса  (рад/с)
10–13       4        float32 LE Скорость заднего правого колеса   (рад/с)
14–17       4        float32 LE Скорость заднего левого колеса    (рад/с)
18–21       4        float32 LE Угол руля                         (рад)
22–25       4        float32 LE Угол наклона камеры               (рад)
26          1        uint8      CRC-8 (XOR байт 2–25, т.е. payload)
27          1        uint8      Стоп-байт: 0xFF
```

> **LE** = Little-Endian (младший байт первый). Стандарт `struct.pack('<6f', ...)` в Python.

### Диапазоны значений

| Поле | Диапазон | Ноль | Примечание |
|---|---|---|---|
| Скорость колёс | −∞ … +∞ рад/с | 0.0 | Отрицательное = движение назад |
| Угол руля | −0.30 … +0.30 рад | 0.0 | Механический предел joint'а `base_link_to_wheeling_mech` |
| Наклон камеры | 0.0 … +2.09 рад | 0.0 | По URDF: limit lower=0.0, upper=2.09, joint `front_wheels_base_to_depth_camera` |

### CRC-8

Контрольная сумма — XOR всех байт **payload** (байты 2–25, 24 байта):

```python
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc
```

**Проверка на стороне контроллера:**
```c
uint8_t crc = 0;
for (int i = 2; i <= 25; i++) crc ^= frame[i];
if (crc == frame[26]) { /* фрейм корректен */ }
```

### Пример фрейма (Python)

```python
import struct

vel_fr = 1.5   # рад/с — переднее правое
vel_fl = 1.5   # рад/с — переднее левое
vel_br = 1.5   # рад/с — заднее правое
vel_bl = 1.5   # рад/с — заднее левое
steer  = 0.2   # рад  — поворот руля
tilt   = 0.5   # рад  — наклон камеры

payload = struct.pack('<6f', vel_fr, vel_fl, vel_br, vel_bl, steer, tilt)
crc     = 0
for b in payload: crc ^= b

frame = b'\xAA\x55' + payload + bytes([crc]) + b'\xFF'
# len(frame) == 28
```

### Пример приёма на микроконтроллере (C, псевдокод)

```c
#define FRAME_LEN 28

typedef struct {
    float vel_fr, vel_fl, vel_br, vel_bl;
    float steer;
    float tilt;
} RobotCmd;

bool parse_frame(uint8_t *buf, RobotCmd *cmd) {
    if (buf[0] != 0xAA || buf[1] != 0x55 || buf[27] != 0xFF) return false;

    uint8_t crc = 0;
    for (int i = 2; i <= 25; i++) crc ^= buf[i];
    if (crc != buf[26]) return false;

    memcpy(&cmd->vel_fr, &buf[2],  4);
    memcpy(&cmd->vel_fl, &buf[6],  4);
    memcpy(&cmd->vel_br, &buf[10], 4);
    memcpy(&cmd->vel_bl, &buf[14], 4);
    memcpy(&cmd->steer,  &buf[18], 4);
    memcpy(&cmd->tilt,   &buf[22], 4);
    return true;
}
```

> Архитектура процессора на мини-ПК (x86) и типичных STM32/ESP32 — Little-Endian, поэтому `memcpy` в float работает напрямую без перестановки байт.

---

## Топики ROS2

### Входные (rs485_bridge читает)

| Топик | Тип | Откуда | Что содержит |
|---|---|---|---|
| `/velocity_controller/commands` | `Float64MultiArray` | `joy_control`, `row_driver` | `data[0..3]` — скорости 4 колёс (рад/с) в порядке: fr, fl, br, bl |
| `/position_controller/commands` | `Float64MultiArray` | `joy_control`, `vs_navigation`, `field_mission` | `data[0]` — угол руля (рад) |
| `/camera_tilt_controller/commands` | `Float64MultiArray` | GUI (`AgroGUI`), ручное управление | `data[0]` — угол наклона камеры (рад) |

### Выходные (rs485_bridge публикует)

| Топик | Тип | Что содержит |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Приблизительные позиции/скорости joint'ов на основе последних команд. Нужно для TF (robot_state_publisher). |

**Joint'ы в `/joint_states`:**

| Индекс | Имя joint'а | Поле `position` | Поле `velocity` |
|---|---|---|---|
| 0 | `front_right_base_to_front_right_wheel` | 0.0 (нет энкодера) | vel_fr (рад/с) |
| 1 | `front_left_base_to_front_left_wheel`  | 0.0 | vel_fl (рад/с) |
| 2 | `back_right_base_to_back_right_wheel`  | 0.0 | vel_br (рад/с) |
| 3 | `back_left_base_to_back_left_wheel`    | 0.0 | vel_bl (рад/с) |
| 4 | `base_link_to_wheeling_mech`           | steer (рад) | 0.0 |
| 5 | `front_wheels_base_to_depth_camera`    | tilt (рад) | 0.0 |

> Position колёс не накапливается (всегда 0.0) — реальная позиция из энкодеров не доступна до интеграции hardware feedback по RS-485.

---

## Параметры rs485_bridge

| Параметр | Тип | По умолчанию | Описание |
|---|---|---|---|
| `port` | string | `/dev/ttyUSB0` | Устройство RS-485 адаптера |
| `baudrate` | int | `115200` | Скорость порта (должна совпадать с контроллером) |
| `publish_rate` | double | `20.0` | Частота отправки фреймов и публикации joint_states (Гц) |

Переопределить в launch:
```python
rs485_bridge = Node(
    package='my_robot',
    executable='rs485_bridge',
    parameters=[{
        'port':         '/dev/ttyUSB1',  # другой порт
        'baudrate':     57600,
        'publish_rate': 30.0,
    }],
)
```

---

## Как адаптировать протокол под свой контроллер

Весь протокол изолирован в методе `_send_frame()` файла [rs485_bridge.py](src/my_robot/my_robot/rs485_bridge.py). Остальной код (подписки, joint_states, таймер) трогать не нужно.

**Пример: замена на текстовый протокол**

```python
def _send_frame(self):
    if self._ser is None or not self._ser.is_open:
        return
    line = f'V {self._vel[0]:.3f} {self._vel[2]:.3f} S {self._steer:.3f} T {self._tilt:.3f}\n'
    self._ser.write(line.encode())
```

**Пример: Modbus RTU (два регистра = одно float32)**

```python
# требует библиотеку minimalmodbus или pymodbus
def _send_frame(self):
    ...
```

**Пример: отправка скорости как RPM (целые числа)**

```python
def _send_frame(self):
    WHEEL_RADIUS = 0.37   # м
    RAD_TO_RPM   = 60 / (2 * math.pi)
    rpm = [int(v * RAD_TO_RPM) for v in self._vel]
    steer_deg = int(math.degrees(self._steer))
    payload = struct.pack('<4hh', *rpm, steer_deg)
    ...
```

---

## Запуск

```bash
# На мини-ПК робота:
cd /home/.../Ros2AgrobotProd
docker compose up robot

# Проверить что RS-485 отправляется:
ros2 topic echo /joint_states --once

# Проверить команды вручную:
ros2 topic pub --once /velocity_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0, 1.0, 1.0]}"

ros2 topic pub --once /position_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.2]}"
```

### Запуск GUI на ПК оператора

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42          # тот же что в docker-compose.yml робота
cd /home/Alexey/AgroGUI
./run.sh
```

---

## Переменные окружения (docker-compose.yml)

| Переменная | Значение | Назначение |
|---|---|---|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS-реализация для ROS2 |
| `ROS_DOMAIN_ID` | `42` | Изоляция сети — должна совпадать с GUI-ПК |
