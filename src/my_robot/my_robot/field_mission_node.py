#!/usr/bin/env python3
"""
field_mission_node.py — автономный обход поля бустрофедоном.

Роль в системе:
  - Диспетчер: решает кто управляет роботом в каждый момент времени.
    * Во время движения по ряду — включает vs_navigation и row_driver,
      сам в контроллеры не пишет.
    * Во время разворота — выключает vs_navigation и row_driver,
      сам публикует команды руля и скорости напрямую.
  - Исполнитель разворота: реализует headland-манёвр (4 шага).
  - Счётчик рядов: отслеживает какой ряд сейчас и в каком направлении.

Запуск миссии:
  ros2 topic pub --once /mission/start std_msgs/msg/Bool '{data: true}'

Экстренная остановка:
  ros2 topic pub --once /mission/stop std_msgs/msg/Bool '{data: true}'

[ОТЛАДКА] Подъезд к точке (не нужен для полевой миссии, только для теста):
  ros2 topic pub --once /mission/goto geometry_msgs/msg/Point '{x: 5.0, y: 2.0, z: 0.0}'
"""

import math
import rclpy
from enum import IntEnum
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


# ══════════════════════════════════════════════════════════════════════════════
# Состояния конечного автомата (FSM)
# ══════════════════════════════════════════════════════════════════════════════

class State(IntEnum):
    IDLE             = 0   # ждём команды старта
    GOTO_TEST        = 1   # [ОТЛАДКА] подъезд к произвольной точке по одометрии
    ROW_FOLLOWING    = 2   # движение по ряду (VS активен, row_driver активен)
    END_BRAKING      = 3   # торможение в конце ряда перед headland
    HEADLAND_0       = 4   # headland шаг 1: прямо headland_depth м (выезжаем за ряды)
    HEADLAND_1       = 5   # headland шаг 2: поворот 90° поперёк рядов
    HEADLAND_2       = 6   # headland шаг 3: прямо row_spacing м (смещаемся к след. ряду)
    HEADLAND_3       = 7   # headland шаг 4: поворот 90° вдоль рядов (в обратном направл.)
    ROW_ENTRY        = 8   # медленно въезжаем в ряд, ждём захвата VS
    MISSION_COMPLETE = 9   # все ряды пройдены
    EMERGENCY_STOP   = 10  # экстренная остановка


# ══════════════════════════════════════════════════════════════════════════════
# Нода
# ══════════════════════════════════════════════════════════════════════════════

class FieldMissionNode(Node):

    def __init__(self):
        super().__init__('field_mission')

        # ── параметры поля ────────────────────────────────────────────────────
        # Все значения берутся из config/field_params.yaml (передаётся в launch).

        # Количество рядов на поле
        self.declare_parameter('num_rows',          5)
        # Расстояние между центрами соседних рядов, м
        self.declare_parameter('row_spacing',       2.0)
        # Длина ряда от точки старта до конца, м
        self.declare_parameter('row_length',        26.5)
        # За сколько метров до конца ряда начинать торможение
        self.declare_parameter('end_margin',        3.0)
        # Расстояние, которое робот проезжает прямо в headland перед разворотом.
        # Должно быть больше минимального радиуса разворота (≈6.8 м при max_steer=0.28).
        self.declare_parameter('headland_depth',    9.0)
        # Таймаут ожидания захвата ряда визуальной сервировкой при ROW_ENTRY, с
        self.declare_parameter('row_entry_timeout', 8.0)

        # ── скорости (рад/с на колёсах) ───────────────────────────────────────
        # Радиус колеса 0.37 м → 1 рад/с ≈ 0.37 м/с линейной скорости.
        self.declare_parameter('nav_speed',   1.0)   # движение прямо в headland
        self.declare_parameter('turn_speed',  0.6)   # движение во время поворота
        self.declare_parameter('entry_speed', 0.5)   # медленно при входе в ряд (VS должен успеть захватить)

        # ── рулевые параметры ─────────────────────────────────────────────────
        self.declare_parameter('max_steer',       0.28)  # рад, механический предел joint'а
        # Знак команды руля: +1 = без инверсии, -1 = инвертировать.
        # Если после первого теста разворот идёт в неправильную сторону — поменять на -1.
        self.declare_parameter('steer_sign_flip', 1)

        # ── [ОТЛАДКА] параметры GOTO_TEST ─────────────────────────────────────
        # Используются только в режиме GOTO_TEST, для полевой миссии не нужны.
        self.declare_parameter('goto_steer_gain', 1.0)  # коэффициент пропорционального регулятора
        self.declare_parameter('goto_tolerance',  0.5)  # м — считаем точку достигнутой
        self.declare_parameter('goto_speed',      0.8)  # рад/с колёса

        # ── загрузка значений ─────────────────────────────────────────────────
        self.num_rows          = self.get_parameter('num_rows').value
        self.row_spacing       = self.get_parameter('row_spacing').value
        self.row_length        = self.get_parameter('row_length').value
        self.end_margin        = self.get_parameter('end_margin').value
        self.headland_depth    = self.get_parameter('headland_depth').value
        self.row_entry_timeout = self.get_parameter('row_entry_timeout').value
        self.nav_speed         = self.get_parameter('nav_speed').value
        self.turn_speed        = self.get_parameter('turn_speed').value
        self.entry_speed       = self.get_parameter('entry_speed').value
        self.max_steer         = self.get_parameter('max_steer').value
        self.steer_sign_flip   = self.get_parameter('steer_sign_flip').value
        self.goto_steer_gain   = self.get_parameter('goto_steer_gain').value
        self.goto_tolerance    = self.get_parameter('goto_tolerance').value
        self.goto_speed        = self.get_parameter('goto_speed').value

        # ── состояние FSM ─────────────────────────────────────────────────────
        self.state       = State.IDLE
        self.current_row = 0     # 0-based, номер текущего ряда
        # Направление движения вдоль ряда: +1 = по +x (чётные ряды), -1 = по -x (нечётные).
        # Чередуется после каждого разворота — бустрофедон.
        self.current_dir  = 1
        # Сколько секунд провели в текущем состоянии (сбрасывается при _transition)
        self.state_timer  = 0.0

        # ── одометрия (из /odom, публикует acker_odom) ───────────────────────
        self.x     = 0.0   # м, позиция в odom-фрейме
        self.y     = 0.0
        self.theta = 0.0   # рад, курс (yaw)

        # Позиция и курс в начале текущего шага FSM.
        # Используются для вычисления пройденного расстояния и угла поворота
        # внутри каждого шага headland без накопленной погрешности.
        self.step_x0     = 0.0
        self.step_y0     = 0.0
        self.step_theta0 = 0.0

        # ── детекция конца ряда по сигналу VS ────────────────────────────────
        # vs_navigation публикует True в /vs_nav/row_end когда видит конец ряда.
        # Требуем ROW_END_CONFIRM кадров подряд чтобы избежать ложных срабатываний.
        self.row_end_frames  = 0
        self.ROW_END_CONFIRM = 5

        # ── [ОТЛАДКА] цель для GOTO_TEST ─────────────────────────────────────
        self.goto_target = None  # geometry_msgs/Point

        # ── подписки ──────────────────────────────────────────────────────────
        # Одометрия: позиция + курс для отслеживания прогресса по ряду и headland
        self.create_subscription(Odometry, '/odom',           self._odom_cb,    10)
        # Сигнал конца ряда от визуальной сервировки
        self.create_subscription(Bool,     '/vs_nav/row_end', self._row_end_cb, 10)
        # Команда старта полной миссии (True = начать)
        self.create_subscription(Bool,     '/mission/start',  self._start_cb,   10)
        # Экстренная остановка (True = стоп)
        self.create_subscription(Bool,     '/mission/stop',   self._stop_cb,    10)
        # [ОТЛАДКА] подъезд к точке — только для ручного тестирования позиционирования
        self.create_subscription(Point,    '/mission/goto',   self._goto_cb,    10)

        # ── публикации ────────────────────────────────────────────────────────
        # Команды руля → rs485_bridge → RS-485 (используется только во время headland)
        self.steer_pub  = self.create_publisher(Float64MultiArray,
                                                '/position_controller/commands', 10)
        # Команды скорости → rs485_bridge → RS-485 (используется только во время headland)
        self.vel_pub    = self.create_publisher(Float64MultiArray,
                                                '/velocity_controller/commands', 10)
        # Включение/выключение row_driver (поддержание скорости при следовании по ряду)
        self.enable_pub = self.create_publisher(Bool,   '/autopilot/enable',   10)
        # Включение/выключение vs_navigation (визуальная сервировка)
        self.vs_pub     = self.create_publisher(Bool,   '/mission/vs_active',  10)
        # Статус миссии для GUI и логирования
        self.status_pub = self.create_publisher(String, '/mission/status',     10)

        # Главный цикл 10 Гц: достаточно для управления по одометрии (обновляется ~30 Гц)
        self.DT = 0.1
        self.create_timer(self.DT, self._update)

        self.get_logger().info(
            '[MISSION] Ready.\n'
            '  Start mission : ros2 topic pub --once /mission/start std_msgs/msg/Bool \'{data: true}\'\n'
            '  Emergency stop: ros2 topic pub --once /mission/stop  std_msgs/msg/Bool \'{data: true}\'\n'
            '  [DEBUG] Goto  : ros2 topic pub --once /mission/goto  geometry_msgs/msg/Point \'{x: 5.0, y: 0.0, z: 0.0}\''
        )

    # ══════════════════════════════════════════════════════════════════════════
    # Колбэки подписок
    # ══════════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        """Обновляем текущую позицию и курс из одометрии acker_odom."""
        self.x    = msg.pose.pose.position.x
        self.y    = msg.pose.pose.position.y
        q         = msg.pose.pose.orientation
        # Извлекаем yaw из кватерниона
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def _row_end_cb(self, msg: Bool):
        """
        Считаем подряд идущие кадры с сигналом конца ряда от vs_navigation.
        Сброс счётчика при первом False — защита от кратковременных ложных срабатываний.
        """
        if msg.data:
            self.row_end_frames += 1
        else:
            self.row_end_frames = 0

    def _start_cb(self, msg: Bool):
        """Запускаем миссию только из IDLE и только при True."""
        if msg.data and self.state == State.IDLE:
            self.get_logger().info('[MISSION] Starting full field coverage.')
            self._start_row()

    def _stop_cb(self, msg: Bool):
        if msg.data:
            self._emergency_stop()

    def _goto_cb(self, msg: Point):
        """
        [ОТЛАДКА] Переходим в режим GOTO_TEST — подъезжаем к указанной точке.
        Полезно для проверки одометрии и рулевого управления на реальном поле
        без запуска полной миссии. Не используется во время штатной работы.
        """
        if self.state not in (State.IDLE, State.GOTO_TEST):
            self.get_logger().warn('[MISSION] Cannot accept goto — not in IDLE.')
            return
        self.goto_target = msg
        self._transition(State.GOTO_TEST)
        self.get_logger().info(
            f'[DEBUG GOTO] Target: odom ({msg.x:.2f}, {msg.y:.2f})')

    # ══════════════════════════════════════════════════════════════════════════
    # Главный цикл FSM (10 Гц)
    # ══════════════════════════════════════════════════════════════════════════

    def _update(self):
        self.state_timer += self.DT

        if self.state == State.IDLE:
            pass  # ждём /mission/start

        elif self.state == State.GOTO_TEST:
            self._tick_goto_test()

        elif self.state == State.ROW_FOLLOWING:
            self._tick_row_following()

        elif self.state == State.END_BRAKING:
            self._tick_end_braking()

        elif self.state in (State.HEADLAND_0, State.HEADLAND_1,
                            State.HEADLAND_2, State.HEADLAND_3):
            self._tick_headland()

        elif self.state == State.ROW_ENTRY:
            self._tick_row_entry()

        elif self.state == State.MISSION_COMPLETE:
            pass  # команды не шлём, робот стоит

    # ══════════════════════════════════════════════════════════════════════════
    # [ОТЛАДКА] GOTO_TEST
    # ══════════════════════════════════════════════════════════════════════════

    def _tick_goto_test(self):
        """
        [ОТЛАДКА] Простой пропорциональный регулятор: едем к goto_target.

        Используется только для ручного тестирования позиционирования робота
        на реальном поле (например, проверить точность одометрии на 10 м).
        При штатной работе этот метод не вызывается.

        Алгоритм:
          1. Считаем вектор от текущей позиции до цели.
          2. Вычисляем желаемый курс target_heading = atan2(dy, dx).
          3. Угловая ошибка = разница между желаемым и текущим курсом.
          4. Руль = clip(gain * angle_err, ±max_steer).
          5. Когда dist < tolerance — стоп, возврат в IDLE.
        """
        if self.goto_target is None:
            return

        tx, ty = self.goto_target.x, self.goto_target.y
        dx   = tx - self.x
        dy   = ty - self.y
        dist = math.hypot(dx, dy)

        # Логируем раз в секунду чтобы не засорять консоль
        if int(self.state_timer * 10) % 10 == 0:
            self.get_logger().info(
                f'[DEBUG GOTO] pos=({self.x:.2f},{self.y:.2f}) '
                f'target=({tx:.2f},{ty:.2f}) dist={dist:.2f}m '
                f'theta={math.degrees(self.theta):.1f}°')

        if dist < self.goto_tolerance:
            self._cmd_wheels(0.0)
            self._cmd_steer(0.0)
            self._publish_status('GOTO_REACHED')
            self.get_logger().info(
                f'[DEBUG GOTO] Reached ({tx:.2f},{ty:.2f}). '
                f'Final pos: ({self.x:.2f},{self.y:.2f})')
            self._transition(State.IDLE)
            return

        target_heading = math.atan2(dy, dx)
        angle_err = self._norm_angle(target_heading - self.theta)
        steer = self.steer_sign_flip * max(-self.max_steer,
                                           min(self.max_steer,
                                               self.goto_steer_gain * angle_err))
        self._cmd_steer(steer)
        self._cmd_wheels(self.goto_speed)

    # ══════════════════════════════════════════════════════════════════════════
    # ROW_FOLLOWING
    # ══════════════════════════════════════════════════════════════════════════

    # Игнорируем сигнал конца ряда от VS пока не проехали хотя бы MIN_ROW_DIST м.
    # Защита от ложного срабатывания сразу после входа в ряд.
    MIN_ROW_DIST = 1.5  # м

    def _tick_row_following(self):
        """
        Следуем по ряду. Сами ничего не публикуем — за нас работают:
          - vs_navigation → /position_controller/commands (руль, держит ряд)
          - row_driver    → /velocity_controller/commands (постоянная скорость)

        Наша задача — определить конец ряда двумя независимыми способами:
          1. По одометрии: проехали row_length - end_margin метров.
          2. По сигналу VS: vs_navigation видит конец ряда ROW_END_CONFIRM кадров подряд.
        Любое из двух условий → переходим в END_BRAKING.
        """
        dist_in_row = self._dist_from_step()

        row_end_by_odom = dist_in_row >= (self.row_length - self.end_margin)
        row_end_by_vs   = (self.row_end_frames >= self.ROW_END_CONFIRM
                           and dist_in_row >= self.MIN_ROW_DIST)

        if row_end_by_odom or row_end_by_vs:
            reason = 'odom' if row_end_by_odom else 'VS'
            self.get_logger().info(
                f'[MISSION] Row {self.current_row} end ({reason}), '
                f'dist={dist_in_row:.1f}m')
            self._transition(State.END_BRAKING)

    # ══════════════════════════════════════════════════════════════════════════
    # END_BRAKING
    # ══════════════════════════════════════════════════════════════════════════

    def _tick_end_braking(self):
        """
        Останавливаем всё перед headland:
          - Шлём нулевую скорость и нулевой руль напрямую (перебиваем row_driver и VS).
          - Выключаем vs_navigation и row_driver.
          - Ждём 1 с чтобы робот физически остановился.
          - Переходим в HEADLAND_0.
        """
        self._cmd_wheels(0.0)
        self._cmd_steer(0.0)
        self._set_vs(False)
        self._set_drive(False)

        if self.state_timer >= 1.0:
            self._transition(State.HEADLAND_0)
            self.get_logger().info('[MISSION] Starting headland sequence.')

    # ══════════════════════════════════════════════════════════════════════════
    # HEADLAND (шаги 0–3)
    # ══════════════════════════════════════════════════════════════════════════

    def _tick_headland(self):
        """
        Четырёхшаговый square-turn разворот (бустрофедон):

          HEADLAND_0: прямо headland_depth м
                      — выезжаем за концы рядов чтобы было место для разворота
          HEADLAND_1: поворот 90° в сторону следующего ряда
                      — руль max_steer, измеряем угол поворота по одометрии
          HEADLAND_2: прямо row_spacing м
                      — смещаемся на один межрядный интервал
          HEADLAND_3: поворот 90° вдоль рядов (в обратном направлении)
                      — после этого шага смотрим точно вдоль следующего ряда

        Знак руля при повороте:
          Движемся по +x (dir=+1): следующий ряд слева (+y) → поворот влево → turn_steer > 0
          Движемся по -x (dir=-1): следующий ряд справа (-y) → поворот вправо → turn_steer < 0
          Формула: turn_steer = max_steer * current_dir * steer_sign_flip

        Прогресс поворота измеряется как |изменение курса с начала шага| по одометрии.
        Останавливаемся при 82° (не 90°) чтобы компенсировать выкат после отпускания руля.
        """
        turn_steer  = self.max_steer * self.current_dir * self.steer_sign_flip
        TURN_DONE   = math.radians(82.0)  # чуть меньше 90° — компенсация выката

        if self.state == State.HEADLAND_0:
            # Едем прямо в headland пока не проехали headland_depth м
            dist = self._dist_from_step()
            if dist >= self.headland_depth:
                self._transition(State.HEADLAND_1)
            else:
                self._cmd_steer(0.0)
                self._cmd_wheels(self.nav_speed)

        elif self.state == State.HEADLAND_1:
            # Поворачиваем пока курс не изменился на 82°
            turned = abs(self._heading_change_from_step())
            if turned >= TURN_DONE:
                self._transition(State.HEADLAND_2)
            else:
                self._cmd_steer(turn_steer)
                self._cmd_wheels(self.turn_speed)

        elif self.state == State.HEADLAND_2:
            # Едем прямо на row_spacing м (переходим к следующему ряду)
            dist = self._dist_from_step()
            if dist >= self.row_spacing:
                self._transition(State.HEADLAND_3)
            else:
                self._cmd_steer(0.0)
                self._cmd_wheels(self.nav_speed)

        elif self.state == State.HEADLAND_3:
            # Второй поворот 90° — теперь смотрим вдоль следующего ряда
            turned = abs(self._heading_change_from_step())
            if turned >= TURN_DONE:
                self._finish_headland()
            else:
                self._cmd_steer(turn_steer)
                self._cmd_wheels(self.turn_speed)

    def _finish_headland(self):
        """
        Разворот завершён. Обновляем счётчики и переходим к следующему ряду.
        Если ряды закончились — завершаем миссию.
        """
        self._cmd_wheels(0.0)
        self._cmd_steer(0.0)

        self.current_row += 1
        self.current_dir *= -1   # инвертируем направление движения (бустрофедон)
        self.row_end_frames = 0  # сбрасываем счётчик конца ряда

        if self.current_row >= self.num_rows:
            self._transition(State.MISSION_COMPLETE)
            self.get_logger().info('[MISSION] All rows completed!')
            self._publish_status('MISSION_COMPLETE')
            return

        self.get_logger().info(
            f'[MISSION] Starting row {self.current_row} (dir={self.current_dir:+d})')
        self._transition(State.ROW_ENTRY)

    # ══════════════════════════════════════════════════════════════════════════
    # ROW_ENTRY
    # ══════════════════════════════════════════════════════════════════════════

    def _tick_row_entry(self):
        """
        Въезжаем в ряд на малой скорости, ждём захвата VS.

        Включаем vs_navigation и едем медленно (entry_speed).
        VS должен найти ряд и начать публиковать команды руля.
        Если за row_entry_timeout секунд VS не захватил ряд — продолжаем всё равно
        (предупреждение в лог, робот будет ехать без коррекции курса).
        """
        self._set_vs(True)
        self._cmd_wheels(self.entry_speed)

        if self.state_timer >= self.row_entry_timeout:
            self.get_logger().warn(
                f'[MISSION] ROW_ENTRY timeout on row {self.current_row}. '
                'Proceeding without VS lock.')
            self._start_row()

    # ══════════════════════════════════════════════════════════════════════════
    # Вспомогательные методы
    # ══════════════════════════════════════════════════════════════════════════

    def _start_row(self):
        """
        Запустить следующий ряд: активировать vs_navigation и row_driver.
        После этого field_mission только наблюдает, не публикует в контроллеры.
        """
        self._set_step_origin()
        self._set_vs(True)
        self._set_drive(True)
        self._transition(State.ROW_FOLLOWING)
        self._publish_status(f'ROW_FOLLOWING row={self.current_row}')
        self.get_logger().info(
            f'[MISSION] ROW_FOLLOWING row={self.current_row} dir={self.current_dir:+d}')

    def _emergency_stop(self):
        """Немедленная остановка: нулевые команды + выключить VS и row_driver."""
        self._cmd_wheels(0.0)
        self._cmd_steer(0.0)
        self._set_vs(False)
        self._set_drive(False)
        self._transition(State.EMERGENCY_STOP)
        self.get_logger().error('[MISSION] EMERGENCY STOP')

    def _transition(self, new_state: State):
        """Сменить состояние FSM, сбросить таймер и запомнить начало шага."""
        old = State(self.state).name
        self.state       = new_state
        self.state_timer = 0.0
        self._set_step_origin()
        self._publish_status(new_state.name)
        self.get_logger().info(f'[MISSION] {old} → {new_state.name}')

    def _set_step_origin(self):
        """Запомнить текущую позицию и курс как начало нового шага."""
        self.step_x0     = self.x
        self.step_y0     = self.y
        self.step_theta0 = self.theta

    def _dist_from_step(self) -> float:
        """Евклидово расстояние от начала текущего шага до текущей позиции."""
        return math.hypot(self.x - self.step_x0, self.y - self.step_y0)

    def _heading_change_from_step(self) -> float:
        """Изменение курса с начала текущего шага (со знаком, нормализовано ±π)."""
        return self._norm_angle(self.theta - self.step_theta0)

    @staticmethod
    def _norm_angle(a: float) -> float:
        """Нормализовать угол в диапазон (-π, +π]."""
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── низкоуровневые команды ────────────────────────────────────────────────

    def _cmd_steer(self, angle: float):
        """Опубликовать угол руля → rs485_bridge → RS-485."""
        msg = Float64MultiArray()
        msg.data = [float(angle)]
        self.steer_pub.publish(msg)

    def _cmd_wheels(self, speed: float):
        """Опубликовать скорость всех 4 колёс (одинаковую) → rs485_bridge → RS-485."""
        msg = Float64MultiArray()
        msg.data = [float(speed)] * 4
        self.vel_pub.publish(msg)

    def _set_drive(self, enabled: bool):
        """Включить / выключить row_driver (поддержание скорости при следовании по ряду)."""
        msg = Bool()
        msg.data = enabled
        self.enable_pub.publish(msg)

    def _set_vs(self, active: bool):
        """Включить / выключить vs_navigation (визуальная сервировка)."""
        msg = Bool()
        msg.data = active
        self.vs_pub.publish(msg)

    def _publish_status(self, text: str):
        """Публиковать текущий статус миссии для GUI и логирования."""
        msg = String()
        msg.data = f'[row={self.current_row} dir={self.current_dir:+d}] {text}'
        self.status_pub.publish(msg)


# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = FieldMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
