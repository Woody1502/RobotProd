#!/usr/bin/env python3
"""
rs485_bridge.py — forwards ROS2 control commands to motor controllers via RS-485.

Frame format (28 bytes):
  [0xAA][0x55] | float32 vel_fr | float32 vel_fl | float32 vel_br | float32 vel_bl
               | float32 steer  | float32 tilt   | crc8 | [0xFF]

Parameters (ROS2):
  port         : serial device  (default /dev/ttyUSB0)
  baudrate     : baud rate      (default 115200)
  publish_rate : send rate, Hz  (default 20.0)
"""

import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

try:
    import serial
    _HAS_SERIAL = True
except ImportError:
    _HAS_SERIAL = False


_WHEEL_JOINTS = [
    'front_right_base_to_front_right_wheel',
    'front_left_base_to_front_left_wheel',
    'back_right_base_to_back_right_wheel',
    'back_left_base_to_back_left_wheel',
]
_STEER_JOINT = 'base_link_to_wheeling_mech'
_TILT_JOINT  = 'front_wheels_base_to_depth_camera'


def _crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


class RS485Bridge(Node):
    def __init__(self):
        super().__init__('rs485_bridge')

        self.declare_parameter('port',         '/dev/ttyUSB0')
        self.declare_parameter('baudrate',     115200)
        self.declare_parameter('publish_rate', 20.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        rate = self.get_parameter('publish_rate').value

        # last commanded values
        self._vel   = [0.0, 0.0, 0.0, 0.0]  # fr, fl, br, bl  (rad/s)
        self._steer = 0.0                     # rad
        self._tilt  = 0.0                     # rad

        # serial
        self._ser = None
        if not _HAS_SERIAL:
            self.get_logger().warn('pyserial not installed — running without RS-485 output')
        else:
            try:
                self._ser = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f'RS-485 open: {port} @ {baud}')
            except serial.SerialException as e:
                self.get_logger().error(f'Cannot open RS-485 port {port}: {e}')

        # subscribers
        self.create_subscription(
            Float64MultiArray, '/velocity_controller/commands',    self._vel_cb,   10)
        self.create_subscription(
            Float64MultiArray, '/position_controller/commands',    self._steer_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/camera_tilt_controller/commands', self._tilt_cb,  10)

        # publishers
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.create_timer(1.0 / rate, self._tick)

    # ── subscribers ──────────────────────────────────────────────────────────

    def _vel_cb(self, msg: Float64MultiArray):
        d = list(msg.data)
        if len(d) >= 4:
            self._vel = d[:4]
        elif len(d) >= 1:
            self._vel = [d[0]] * 4

    def _steer_cb(self, msg: Float64MultiArray):
        if msg.data:
            self._steer = msg.data[0]

    def _tilt_cb(self, msg: Float64MultiArray):
        if msg.data:
            self._tilt = msg.data[0]

    # ── main loop ─────────────────────────────────────────────────────────────

    def _tick(self):
        self._send_frame()
        self._publish_joint_states()

    def _send_frame(self):
        if self._ser is None or not self._ser.is_open:
            return
        payload = struct.pack('<6f',
                              self._vel[0], self._vel[1],
                              self._vel[2], self._vel[3],
                              self._steer,  self._tilt)
        frame = b'\xAA\x55' + payload + bytes([_crc8(payload)]) + b'\xFF'
        try:
            self._ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().warn(f'RS-485 write error: {e}', throttle_duration_sec=5.0)

    def _publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = _WHEEL_JOINTS + [_STEER_JOINT, _TILT_JOINT]
        js.velocity = list(self._vel)   + [0.0,          0.0]
        js.position = [0.0] * 4         + [self._steer,  self._tilt]
        self._js_pub.publish(js)


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
