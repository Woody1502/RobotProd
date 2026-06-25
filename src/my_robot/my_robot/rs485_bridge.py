#!/usr/bin/env python3
"""
rs485_bridge.py — Modbus RTU bridge to VIM hardware via virtual COM port.

Uses minimalmodbus to send Modbus RTU frames through a virtual serial port
(created by socat: pty -> TCP:192.168.5.42:81) to the WiFi-RS485 bridge,
which forwards them to BLDC motor boards (addr 8-11) and a steering relay
board (addr 7).

Wheel velocities received as rad/s are converted to power 0-255 proportionally.
Actual velocities are read back from Hall sensors and published to /joint_states.

Parameters:
  modbus_port   : virtual COM port path           (default /dev/ttyVCOM1)
  modbus_baud   : Modbus baud rate                (default 38400)
  publish_rate  : control loop Hz                 (default 20.0)
  max_speed     : rad/s that maps to power 255    (default 10.0 — tune on robot)
  hall_to_rads  : Hall sensor Hz → rad/s          (default 1.0  — tune on robot)
  mag_port      : magnetometer serial device      (default /dev/ttyUSB1)
  mag_baudrate  : magnetometer baud rate          (default 9600)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32, Int8
from sensor_msgs.msg import JointState

try:
    import minimalmodbus
    _HAS_MINIMALMODBUS = True
except ImportError:
    _HAS_MINIMALMODBUS = False

try:
    import serial
    _HAS_SERIAL = True
except ImportError:
    _HAS_SERIAL = False


# ── Modbus board addresses ────────────────────────────────────────────────────

_BLDC_FR    = 9    # ПП — front-right
_BLDC_FL    = 8    # ПЛ — front-left
_BLDC_BR    = 11   # ЗП — rear-right
_BLDC_BL    = 10   # ЗЛ — rear-left
_RELAY_STEER = 7   # Рулевое управление

_BLDC_ADDRS = (_BLDC_FR, _BLDC_FL, _BLDC_BR, _BLDC_BL)

# ── VIM relay actuator map ────────────────────────────────────────────────────
# (modbus_addr, {cmd: (ch0_val, ch1_val)})
# ch values: 0=off, 1=forward polarity, 2=reverse polarity
_VIM_MAP = {
    'manipulator': (1, {0:(0,0), 1:(1,0), 2:(2,0), 3:(0,1), 4:(0,2)}),
    'bucket':      (2, {0:(0,0), 1:(1,1), 2:(2,2)}),
    'frame':       (3, {0:(0,0), 1:(1,1), 2:(2,2)}),
    'bunker':      (4, {0:(0,0), 1:(1,1), 2:(2,2)}),
    'flaps':       (5, {0:(0,0), 1:(1,1)}),
}

_SEPARATOR_ADDR = 6  # BLDC board

_WHEEL_JOINTS = [
    'front_right_base_to_front_right_wheel',
    'front_left_base_to_front_left_wheel',
    'back_right_base_to_back_right_wheel',
    'back_left_base_to_back_left_wheel',
]
_STEER_JOINT = 'base_link_to_wheeling_mech'


class RS485Bridge(Node):
    def __init__(self):
        super().__init__('rs485_bridge')

        if not _HAS_MINIMALMODBUS:
            self.get_logger().fatal('minimalmodbus is not installed. Exiting.')
            raise SystemExit(1)

        self.declare_parameter('modbus_port',  '/dev/ttyVCOM1')
        self.declare_parameter('modbus_baud',   38400)
        self.declare_parameter('publish_rate',  20.0)
        self.declare_parameter('max_speed',     10.0)
        self.declare_parameter('hall_to_rads',  1.0)
        self.declare_parameter('mag_port',      '/dev/ttyUSB1')
        self.declare_parameter('mag_baudrate',  9600)
        self.declare_parameter('bldc_min_power', 200.0)  # 0-255 floor once moving — cpptest found ~100 just clicks, 200+ actually spins the wheel

        modbus_port    = self.get_parameter('modbus_port').value
        modbus_baud    = self.get_parameter('modbus_baud').value
        rate           = self.get_parameter('publish_rate').value
        self._max_speed    = self.get_parameter('max_speed').value
        self._hall_to_rads = self.get_parameter('hall_to_rads').value
        mag_port       = self.get_parameter('mag_port').value
        mag_baud       = self.get_parameter('mag_baudrate').value
        self._min_power    = self.get_parameter('bldc_min_power').value

        # last commanded values — stop is explicit (vel == 0.0) only; sources
        # that drive continuously must publish a final 0.0 when they stop
        # (see row_driver's enable_cb), rather than relying on a timeout here
        self._vel   = [0.0, 0.0, 0.0, 0.0]   # fr, fl, br, bl  (rad/s)
        self._steer = 0.0                      # rad

        # per-wheel BLDC sequencing, matching the proven working reference at
        # /home/alex/modbus_robot_controller: Enable is asserted once for all
        # wheels at startup (_enable_all_bldc) and never touched again for
        # ordinary driving. Each tick only re-sends Direction when it changes
        # (one tick to settle before Speed follows, same as that reference's
        # 50ms gap) and Speed when its value changes — a plain vel==0 just
        # sends Speed=0 and leaves Enable/Direction latched, it does not do a
        # full disable (only the old VIM-GUI style needed full re-enable).
        # None = not yet set, forcing an initial write the first time the
        # wheel is commanded to move.
        self._bldc_dir = [None, None, None, None]
        self._bldc_sent_power = [None, None, None, None]

        # actual velocities from Hall sensors
        self._hall_vel  = [0.0, 0.0, 0.0, 0.0]
        self._hall_tick = 0

        # VIM actuator state
        self._vim_cmds      = {name: 0 for name in _VIM_MAP}
        self._separator_cmd = 0   # 0=stop, 1=forward(up), 2=reverse(down)
        self._motor_enables = [True, True, True, True]

        # ── Modbus instruments (all share the same virtual port) ──────────────
        self._devs = {}          # addr -> Instrument
        self._modbus_lock = threading.Lock()
        self._modbus_ok = False

        # circuit breaker: a dead/disconnected address (e.g. separator addr=6
        # with nothing wired up) takes far longer to fail than its configured
        # 0.1s serial timeout — observed ~2.3s per failed transaction — and
        # since the whole 20Hz tick is single-threaded, hammering it every
        # tick collapses the real control-loop rate to ~1-2Hz, starving the
        # BLDC ramp of the frequency it needs. Trip on the FIRST failure (a
        # threshold>1 just means eating N slow timeouts before backing off,
        # which barely helps) and back off with exponential cooldown, since
        # an address that's dead tends to stay dead.
        self._FAIL_THRESHOLD     = 1
        self._FAIL_COOLDOWN      = 5.0
        self._FAIL_COOLDOWN_MAX  = 60.0
        self._addr_fail_count  = {}
        self._addr_skip_until  = {}
        self._addr_trip_count  = {}

        self.get_logger().info(f'Connecting to Modbus on {modbus_port} @ {modbus_baud} baud...')
        try:
            self._init_modbus(modbus_port, modbus_baud)
            self._modbus_ok = True
            self._enable_all_bldc()
            self._probe_devices()
        except Exception as e:
            self.get_logger().error(f'Modbus init failed on {modbus_port}: {e}')
            self.get_logger().warn('Running without hardware — all Modbus calls will be no-ops.')

        # ── Magnetometer serial ───────────────────────────────────────────────
        self._mag_ser = None
        if _HAS_SERIAL:
            try:
                self._mag_ser = serial.Serial(mag_port, mag_baud, timeout=1.0)
                self.get_logger().info(f'Magnetometer open: {mag_port} @ {mag_baud}')
                threading.Thread(target=self._mag_reader, daemon=True).start()
            except serial.SerialException as e:
                self.get_logger().warn(f'Magnetometer port {mag_port} unavailable: {e}')

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, '/velocity_controller/commands',    self._vel_cb,   10)
        self.create_subscription(
            Float64MultiArray, '/position_controller/commands',    self._steer_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/vim/motor_enable', self._motor_enable_cb, 10)
        for name in _VIM_MAP:
            self.create_subscription(
                Int8, f'/vim/{name}',
                lambda msg, n=name: self._vim_cb(n, msg), 10)
        self.create_subscription(
            Int8, '/vim/separator', self._separator_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._js_pub  = self.create_publisher(JointState, '/joint_states',        10)
        self._mag_pub = self.create_publisher(Float32,    '/magnetometer/heading', 10)

        self.create_timer(1.0 / rate, self._tick)

    # ── Modbus init ───────────────────────────────────────────────────────────

    def _init_modbus(self, port: str, baud: int):
        """Create minimalmodbus Instrument for each Modbus address."""
        all_addrs = list(_BLDC_ADDRS) + [_SEPARATOR_ADDR, _RELAY_STEER] + \
                    [v[0] for v in _VIM_MAP.values()]
        for addr in all_addrs:
            dev = minimalmodbus.Instrument(port, addr)
            dev.serial.baudrate = baud
            dev.serial.timeout = 0.1
            self._devs[addr] = dev
        self.get_logger().info(
            f'Serial port {port} opened @ {baud} baud — {len(self._devs)} Modbus addresses registered')

    def _enable_all_bldc(self):
        """Enable all BLDC boards (wheels + separator) once at startup (coil 0 = Enable)."""
        for addr in _BLDC_ADDRS:
            self._write_bit(addr, 0, True)
        self._write_bit(_SEPARATOR_ADDR, 0, True)
        self.get_logger().info('Enable coil sent to all BLDC boards (addr 6, 8-11)')

    def _probe_devices(self):
        """Probe each board with FC03 read; log which ones respond."""
        _NAMES = {
            1: 'manipulator', 2: 'bucket', 3: 'frame', 4: 'bunker',
            5: 'flaps', 6: 'separator(BLDC)', 7: 'steering',
            8: 'motor-FL', 9: 'motor-FR', 10: 'motor-RL', 11: 'motor-RR',
        }
        ok, fail = [], []
        for addr, name in _NAMES.items():
            try:
                with self._modbus_lock:
                    self._devs[addr].read_register(0, 0, 3, False)
                ok.append(f'{addr}:{name}')
            except Exception:
                fail.append(f'{addr}:{name}')
        if ok:
            self.get_logger().info(f'Modbus OK:   {", ".join(ok)}')
        if fail:
            self.get_logger().warn(f'Modbus FAIL: {", ".join(fail)}')

    # ── minimalmodbus wrappers ────────────────────────────────────────────────

    def _breaker_blocked(self, addr: int) -> bool:
        until = self._addr_skip_until.get(addr, 0.0)
        if until and time.monotonic() < until:
            return True
        return False

    def _breaker_on_success(self, addr: int):
        self._addr_fail_count[addr] = 0
        self._addr_trip_count[addr] = 0

    def _breaker_on_failure(self, addr: int):
        n = self._addr_fail_count.get(addr, 0) + 1
        self._addr_fail_count[addr] = n
        if n >= self._FAIL_THRESHOLD:
            self._addr_fail_count[addr] = 0
            trips = self._addr_trip_count.get(addr, 0) + 1
            self._addr_trip_count[addr] = trips
            cooldown = min(self._FAIL_COOLDOWN_MAX, self._FAIL_COOLDOWN * (2 ** (trips - 1)))
            self._addr_skip_until[addr] = time.monotonic() + cooldown
            self.get_logger().warn(
                f'[BREAKER] addr={addr} failed — backing off {cooldown:.0f}s (trip #{trips})',
                throttle_duration_sec=1.0)

    def _write_bit(self, addr: int, coil: int, value: bool):
        """Write Single Coil (FC05) through minimalmodbus."""
        if not self._modbus_ok or addr not in self._devs:
            self.get_logger().warn(
                f'[TX SKIP] addr={addr} FC05 coil={coil} val={int(value)} — modbus_ok={self._modbus_ok}, known_addr={addr in self._devs}',
                throttle_duration_sec=2.0)
            return
        if self._breaker_blocked(addr):
            return
        with self._modbus_lock:
            try:
                self._devs[addr].write_bit(coil, int(value), 5)
                self.get_logger().info(f'[TX OK] addr={addr} FC05 coil={coil} val={int(value)}')
                self._breaker_on_success(addr)
            # broad on purpose: minimalmodbus's exception hierarchy varies by
            # version (ModbusException vs MasterReportedException/IOError),
            # and a raw serial timeout/OSError must not vanish silently either
            except Exception as e:
                self.get_logger().warn(f'[TX FAIL] addr={addr} FC05 coil={coil} val={int(value)}: {type(e).__name__}: {e}',
                                       throttle_duration_sec=2.0)
                self._breaker_on_failure(addr)

    def _write_register(self, addr: int, reg: int, value: int):
        """Write Single Register (FC06) through minimalmodbus."""
        if not self._modbus_ok or addr not in self._devs:
            self.get_logger().warn(
                f'[TX SKIP] addr={addr} FC06 reg={reg} val={value} — modbus_ok={self._modbus_ok}, known_addr={addr in self._devs}',
                throttle_duration_sec=2.0)
            return
        if self._breaker_blocked(addr):
            return
        with self._modbus_lock:
            try:
                self._devs[addr].write_register(reg, value, 0, 6, False)
                self.get_logger().info(f'[TX OK] addr={addr} FC06 reg={reg} val={value}',
                                       throttle_duration_sec=0.5)
                self._breaker_on_success(addr)
            except Exception as e:
                self.get_logger().warn(f'[TX FAIL] addr={addr} FC06 reg={reg} val={value}: {type(e).__name__}: {e}',
                                       throttle_duration_sec=2.0)
                self._breaker_on_failure(addr)

    def _read_register(self, addr: int, reg: int) -> int:
        """Read Input Register (FC04) through minimalmodbus and return raw 16-bit value."""
        if not self._modbus_ok or addr not in self._devs:
            return 0
        if self._breaker_blocked(addr):
            return 0
        with self._modbus_lock:
            try:
                value = self._devs[addr].read_register(reg, 0, 4, False)
                self._breaker_on_success(addr)
                return value
            except Exception as e:
                self.get_logger().warn(f'[RX FAIL] addr={addr} FC04 reg={reg}: {type(e).__name__}: {e}',
                                       throttle_duration_sec=2.0)
                self._breaker_on_failure(addr)
                return 0

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _vel_cb(self, msg: Float64MultiArray):
        d = list(msg.data)
        if len(d) >= 4:
            new_vel = d[:4]
        elif len(d) >= 1:
            new_vel = [d[0]] * 4
        else:
            return
        if new_vel != self._vel:
            self.get_logger().info(f'[VEL] /velocity_controller/commands: {self._vel} -> {new_vel}')
        self._vel = new_vel

    def _steer_cb(self, msg: Float64MultiArray):
        if msg.data:
            self._steer = msg.data[0]

    def _vim_cb(self, name: str, msg: Int8):
        self._vim_cmds[name] = int(msg.data)

    def _separator_cb(self, msg: Int8):
        self._separator_cmd = int(msg.data)

    def _motor_enable_cb(self, msg: Float64MultiArray):
        d = list(msg.data)
        if len(d) >= 4:
            self._motor_enables = [bool(v) for v in d[:4]]
            for addr, en in zip(_BLDC_ADDRS, self._motor_enables):
                self._write_bit(addr, 0, en)

    # ── Main loop (20 Hz) ─────────────────────────────────────────────────────

    def _tick(self):
        if not self._modbus_ok:
            return
        self._send_bldc()
        self._send_steering()
        self._send_actuators()
        self._hall_tick += 1
        if self._hall_tick >= 4:
            self._read_hall()
            self._hall_tick = 0
        self._publish_joint_states()

    def _send_bldc(self):
        """Direction -> Speed on every change, matching the proven reference
        at /home/alex/modbus_robot_controller (processDriveCommand): Enable is
        set once for all wheels at startup, never touched again here. A plain
        vel==0 just sends Speed=0 and leaves Enable/Direction latched — no
        per-stop disable, no multi-tick settle handshake, so a short tap is
        enough to actually reach the board instead of being undone before the
        old start sequence could finish.

        Direction coil 1: False=forward, True=reverse (matches DirectionControlDevN
        in the original — Contor_System_VIM_reconstructed.py:389-391)."""
        status = []
        for i, (addr, vel) in enumerate(zip(_BLDC_ADDRS, self._vel)):
            reverse = vel < 0.0

            if self._bldc_dir[i] != reverse:
                self._write_bit(addr, 1, reverse)
                self._bldc_dir[i] = reverse
                status.append(f'{addr}:DIR={"REV" if reverse else "FWD"}')
                continue   # one tick to settle before sending speed (reference's 50ms gap)

            if vel == 0.0:
                speed = 0
            else:
                # below ~200/255 the board just clicks without actually
                # spinning the wheel (confirmed empirically) — never ask for
                # less than that once we actually want to move
                speed = int(min(255.0, max(abs(vel) / self._max_speed * 255, self._min_power)))

            if speed != self._bldc_sent_power[i]:
                self._write_register(addr, 0, speed)
                self._bldc_sent_power[i] = speed
                status.append(f'{addr}:vel={vel:.2f},pwr={speed}')
            else:
                status.append(f'{addr}:vel={vel:.2f},pwr={speed}(held)')

        self.get_logger().info(f'[BLDC tick] {" | ".join(status)}', throttle_duration_sec=1.0)

    def _send_steering(self):
        steer = self._steer
        if steer > 0.05:
            val = 1
        elif steer < -0.05:
            val = 2
        else:
            val = 0
        self._write_register(_RELAY_STEER, 0, val)

    def _send_actuators(self):
        """Write relay actuator boards (FC06 ch0/ch1) and separator BLDC (FC05+FC06)."""
        for name, (addr, cmds) in _VIM_MAP.items():
            ch0, ch1 = cmds.get(self._vim_cmds[name], (0, 0))
            self._write_register(addr, 0, ch0)
            self._write_register(addr, 1, ch1)

        cmd = self._separator_cmd
        if cmd == 0:
            self._write_register(_SEPARATOR_ADDR, 0, 0)
        elif cmd == 1:
            self._write_bit(_SEPARATOR_ADDR, 1, False)
            self._write_register(_SEPARATOR_ADDR, 0, 255)
        elif cmd == 2:
            self._write_bit(_SEPARATOR_ADDR, 1, True)
            self._write_register(_SEPARATOR_ADDR, 0, 255)

    def _read_hall(self):
        """Read Hall sensor frequency from each BLDC board (FC04, reg 0)."""
        for i, addr in enumerate(_BLDC_ADDRS):
            hz = self._read_register(addr, 0)
            sign = 1.0 if self._vel[i] >= 0.0 else -1.0
            self._hall_vel[i] = sign * hz * self._hall_to_rads

    def _publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = _WHEEL_JOINTS + [_STEER_JOINT]
        js.velocity = list(self._hall_vel) + [0.0]
        js.position = [0.0] * 4            + [self._steer]
        self._js_pub.publish(js)

    # ── Magnetometer reader (background thread) ───────────────────────────────

    def _mag_reader(self):
        buf = b''
        while rclpy.ok() and self._mag_ser and self._mag_ser.is_open:
            try:
                chunk = self._mag_ser.read(64)
            except serial.SerialException as e:
                self.get_logger().warn(f'Magnetometer read error: {e}',
                                       throttle_duration_sec=5.0)
                break
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._parse_mag_line(line.strip())

    def _parse_mag_line(self, line: bytes):
        try:
            heading = float(line.decode('ascii'))
        except (ValueError, UnicodeDecodeError):
            return
        msg = Float32()
        msg.data = heading
        self._mag_pub.publish(msg)

    def destroy_node(self):
        """Zero all actuators and disable BLDC boards before shutdown."""
        self.get_logger().info('Shutting down: zeroing all actuators and disabling motors...')
        for addr in list(_BLDC_ADDRS) + [_SEPARATOR_ADDR]:
            self._write_register(addr, 0, 0)
            self._write_bit(addr, 1, False)
            self._write_bit(addr, 0, False)
        for name, (addr, _) in _VIM_MAP.items():
            self._write_register(addr, 0, 0)
            self._write_register(addr, 1, 0)
        self._write_register(_RELAY_STEER, 0, 0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RS485Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()