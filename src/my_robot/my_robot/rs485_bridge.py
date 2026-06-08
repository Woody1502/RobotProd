#!/usr/bin/env python3
"""
rs485_bridge.py — Modbus RTU bridge to VIM hardware over TCP (USR-W610 WiFi module).

Sends raw Modbus RTU frames over a TCP socket to the WiFi-RS485 bridge, which
forwards them to BLDC motor boards (addr 8-11) and a steering relay board (addr 7).

Wheel velocities received as rad/s are converted to power 0-255 proportionally.
Actual velocities are read back from Hall sensors and published to /joint_states.

Parameters:
  vim_host      : WiFi bridge IP               (default 192.168.5.42)
  vim_port      : WiFi bridge TCP port         (default 81)
  publish_rate  : control loop Hz              (default 20.0)
  max_speed     : rad/s that maps to power 255 (default 10.0 — tune on robot)
  hall_to_rads  : Hall sensor Hz → rad/s       (default 1.0  — tune on robot)
  mag_port      : magnetometer serial device   (default /dev/ttyUSB1)
  mag_baudrate  : magnetometer baud rate       (default 9600)
"""

import socket
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32, Int8
from sensor_msgs.msg import JointState

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
# cmd: 0=stop, 1=A, 2=B, 3=C, 4=home
_VIM_MAP = {
    'manipulator': (1, {0:(0,0), 1:(1,0), 2:(0,1), 3:(0,2), 4:(0,0)}),
    'bucket':      (2, {0:(0,0), 1:(1,0), 2:(2,0), 3:(0,0)}),
    'frame':       (3, {0:(0,0), 1:(1,0), 2:(2,0), 3:(0,0)}),
    'bunker':      (4, {0:(0,0), 1:(1,0), 2:(2,0)}),
    'flaps':       (5, {0:(0,0), 1:(1,0)}),
    'separator':   (6, {0:(0,0), 1:(1,0), 2:(2,0)}),
}

_WHEEL_JOINTS = [
    'front_right_base_to_front_right_wheel',
    'front_left_base_to_front_left_wheel',
    'back_right_base_to_back_right_wheel',
    'back_left_base_to_back_left_wheel',
]
_STEER_JOINT = 'base_link_to_wheeling_mech'
_TILT_JOINT  = 'front_wheels_base_to_depth_camera'

# ── Modbus RTU frame builders ─────────────────────────────────────────────────

def _crc16(data: bytes) -> int:
    """Compute Modbus CRC-16 (polynomial 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def _frame(*args: int) -> bytes:
    """Build a Modbus RTU frame: raw bytes + CRC-16 appended little-endian."""
    raw = bytes(args)
    crc = _crc16(raw)
    return raw + bytes([crc & 0xFF, crc >> 8])

def _fc05(addr: int, coil: int, on: bool) -> bytes:
    """Write Single Coil."""
    v = 0xFF00 if on else 0x0000
    return _frame(addr, 0x05, 0, coil, v >> 8, v & 0xFF)

def _fc06(addr: int, reg: int, value: int) -> bytes:
    """Write Single Register."""
    return _frame(addr, 0x06, 0, reg, value >> 8, value & 0xFF)

def _fc04(addr: int, reg: int, count: int = 1) -> bytes:
    """Read Input Registers."""
    return _frame(addr, 0x04, 0, reg, 0, count)


class RS485Bridge(Node):
    def __init__(self):
        super().__init__('rs485_bridge')

        self.declare_parameter('vim_host',     '192.168.5.42')
        self.declare_parameter('vim_port',     81)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_speed',    10.0)
        self.declare_parameter('hall_to_rads', 1.0)
        self.declare_parameter('mag_port',     '/dev/ttyUSB1')
        self.declare_parameter('mag_baudrate', 9600)

        vim_host        = self.get_parameter('vim_host').value
        vim_port        = self.get_parameter('vim_port').value
        rate            = self.get_parameter('publish_rate').value
        self._max_speed    = self.get_parameter('max_speed').value
        self._hall_to_rads = self.get_parameter('hall_to_rads').value
        mag_port        = self.get_parameter('mag_port').value
        mag_baud        = self.get_parameter('mag_baudrate').value

        # last commanded values
        self._vel   = [0.0, 0.0, 0.0, 0.0]   # fr, fl, br, bl  (rad/s)
        self._steer = 0.0                      # rad
        self._tilt  = 0.0                      # rad (stored for joint_states)

        # actual velocities from Hall sensors
        self._hall_vel  = [0.0, 0.0, 0.0, 0.0]
        self._hall_tick = 0

        # VIM actuator state
        self._vim_cmds     = {name: 0 for name in _VIM_MAP}
        self._motor_enables = [True, True, True, True]  # fr, fl, br, bl

        # ── TCP socket to WiFi bridge ─────────────────────────────────────────
        self._sock      = None
        self._sock_lock = threading.Lock()
        self._connect_vim(vim_host, vim_port)
        self._enable_all_bldc()

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
            Float64MultiArray, '/camera_tilt_controller/commands', self._tilt_cb,  10)
        self.create_subscription(
            Float64MultiArray, '/vim/motor_enable', self._motor_enable_cb, 10)
        for name in _VIM_MAP:
            self.create_subscription(
                Int8, f'/vim/{name}',
                lambda msg, n=name: self._vim_cb(n, msg), 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._js_pub  = self.create_publisher(JointState, '/joint_states',        10)
        self._mag_pub = self.create_publisher(Float32,    '/magnetometer/heading', 10)

        self.create_timer(1.0 / rate, self._tick)

    # ── TCP connection ────────────────────────────────────────────────────────

    def _connect_vim(self, host: str, port: int):
        """Open TCP connection to the USR-W610 WiFi-RS485 bridge. Logs error on failure."""
        try:
            self._sock = socket.create_connection((host, port), timeout=3.0)
            self._sock.settimeout(0.1)
            self.get_logger().info(f'VIM WiFi bridge connected: {host}:{port}')
        except OSError as e:
            self.get_logger().error(f'VIM connect failed ({host}:{port}): {e}')

    def _send(self, frame: bytes, read_len: int = 0) -> bytes:
        """Send Modbus frame, optionally read response. Thread-safe."""
        with self._sock_lock:
            if self._sock is None:
                return b''
            try:
                self._sock.sendall(frame)
                if read_len > 0:
                    return self._sock.recv(read_len)
            except OSError as e:
                self.get_logger().warn(f'VIM socket error: {e}',
                                       throttle_duration_sec=5.0)
                self._sock = None
        return b''

    def _enable_all_bldc(self):
        """Enable all BLDC boards once at startup (coil 0 = Enable)."""
        for addr in _BLDC_ADDRS:
            self._send(_fc05(addr, 0, True))
        self.get_logger().info('BLDC boards enabled')

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _vel_cb(self, msg: Float64MultiArray):
        """Store commanded wheel velocities (rad/s). Accepts 4-element or broadcast 1-element array."""
        d = list(msg.data)
        if len(d) >= 4:
            self._vel = d[:4]
        elif len(d) >= 1:
            self._vel = [d[0]] * 4

    def _steer_cb(self, msg: Float64MultiArray):
        """Store commanded steering angle (rad). Positive = left, negative = right."""
        if msg.data:
            self._steer = msg.data[0]

    def _tilt_cb(self, msg: Float64MultiArray):
        """Store camera tilt angle (rad) for joint_states publishing (no hardware output)."""
        if msg.data:
            self._tilt = msg.data[0]

    def _vim_cb(self, name: str, msg: Int8):
        """Store latest command integer for a named VIM actuator (0 = stop)."""
        self._vim_cmds[name] = int(msg.data)

    def _motor_enable_cb(self, msg: Float64MultiArray):
        """Enable or disable individual BLDC boards. Sends FC05 coil-0 per board immediately."""
        d = list(msg.data)
        if len(d) >= 4:
            self._motor_enables = [bool(v) for v in d[:4]]
            for addr, en in zip(_BLDC_ADDRS, self._motor_enables):
                self._send(_fc05(addr, 0, en))

    # ── Main loop (20 Hz) ─────────────────────────────────────────────────────

    def _tick(self):
        """Main control loop called at publish_rate Hz. Drives motors, steering, actuators; reads Hall sensors every 4th tick."""
        self._send_bldc()
        self._send_steering()
        self._send_actuators()
        self._hall_tick += 1
        if self._hall_tick >= 4:
            self._read_hall()
            self._hall_tick = 0
        self._publish_joint_states()

    def _send_bldc(self):
        """Convert rad/s velocities to 0-255 power and send direction + power to each BLDC board via FC05/FC06."""
        for addr, vel in zip(_BLDC_ADDRS, self._vel):
            power   = min(255, int(abs(vel) / self._max_speed * 255))
            forward = vel >= 0.0
            self._send(_fc05(addr, 1, forward))   # coil 1 = Direction
            self._send(_fc06(addr, 0, power))      # reg  0 = Power

    def _send_steering(self):
        """Write steering relay register: 1 = left, 2 = right, 0 = stop (deadband ±0.05 rad)."""
        steer = self._steer
        if steer > 0.05:
            val = 1   # прямая полярность → налево
        elif steer < -0.05:
            val = 2   # обратная полярность → направо
        else:
            val = 0   # стоп
        self._send(_fc06(_RELAY_STEER, 0, val))

    def _send_actuators(self):
        """Write ch0/ch1 relay register values for each VIM actuator board according to _VIM_MAP."""
        for name, (addr, cmds) in _VIM_MAP.items():
            ch0, ch1 = cmds.get(self._vim_cmds[name], (0, 0))
            self._send(_fc06(addr, 0, ch0))
            self._send(_fc06(addr, 1, ch1))

    def _read_hall(self):
        """Read Hall sensor frequency from each BLDC board (FC04, reg 0)."""
        for i, addr in enumerate(_BLDC_ADDRS):
            resp = self._send(_fc04(addr, 0, 1), read_len=7)
            if len(resp) == 7 and resp[1] == 0x04:
                hz   = (resp[3] << 8) | resp[4]
                sign = 1.0 if self._vel[i] >= 0.0 else -1.0
                self._hall_vel[i] = sign * hz * self._hall_to_rads

    def _publish_joint_states(self):
        """Publish wheel Hall velocities + steering/tilt positions to /joint_states."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = _WHEEL_JOINTS + [_STEER_JOINT, _TILT_JOINT]
        js.velocity = list(self._hall_vel) + [0.0,         0.0]
        js.position = [0.0] * 4            + [self._steer, self._tilt]
        self._js_pub.publish(js)

    # ── Magnetometer reader (background thread) ───────────────────────────────

    def _mag_reader(self):
        """Background thread: reads ASCII heading lines from magnetometer serial port and publishes them."""
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
        """Parse a single ASCII float line from the magnetometer and publish to /magnetometer/heading."""
        try:
            heading = float(line.decode('ascii'))
        except (ValueError, UnicodeDecodeError):
            return
        msg = Float32()
        msg.data = heading
        self._mag_pub.publish(msg)

    def destroy_node(self):
        with self._sock_lock:
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
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
