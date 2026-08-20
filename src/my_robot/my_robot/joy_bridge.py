#!/usr/bin/env python3
"""
joy_bridge.py — читает геймпад через Linux joystick API (/dev/input/js0)
и публикует команды управления роботом напрямую в ROS2 топики.

Маппинг осей (Xbox Wireless Controller):
  Ось 0 — Left Stick X     → руль (/position_controller/commands)
  Ось 2 — Left Trigger     → скорость назад
  Ось 5 — Right Trigger    → скорость вперёд
  (speed = RT - LT → /velocity_controller/commands)

Параметры:
  js_device     : путь к устройству  (default /dev/input/js0)
  max_speed     : рад/с при полном триггере (default 10.0)
  max_steer     : рад при полном стике (default 0.4)
  publish_rate  : Гц публикации команд (default 20.0)
"""

import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80
_JS_RT_AXIS    = 5
_JS_LT_AXIS    = 2
_JS_LSX_AXIS   = 0
_JS_TRIGGER_DZ = 500
_JS_STICK_DZ   = 2000
_JS_MAX        = 32767.0


class JoyBridge(Node):
    def __init__(self):
        super().__init__('joy_bridge')

        self.declare_parameter('js_device',    '/dev/input/js0')
        self.declare_parameter('max_speed',    10.0)
        self.declare_parameter('max_steer',    0.4)
        self.declare_parameter('publish_rate', 20.0)

        self._dev       = self.get_parameter('js_device').value
        self._max_speed = self.get_parameter('max_speed').value
        self._max_steer = self.get_parameter('max_steer').value
        rate            = self.get_parameter('publish_rate').value

        self._vel_pub   = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self._steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands',  10)

        self._speed     = 0.0
        self._steer     = 0.0
        self._connected = False
        self._lock      = threading.Lock()

        self.create_timer(1.0 / rate, self._publish)
        threading.Thread(target=self._js_reader, daemon=True).start()

    def _publish(self):
        with self._lock:
            if not self._connected:
                return
            speed = self._speed
            steer = self._steer

        vel_msg = Float64MultiArray()
        vel_msg.data = [speed, speed, speed, speed]
        self._vel_pub.publish(vel_msg)

        steer_msg = Float64MultiArray()
        steer_msg.data = [steer]
        self._steer_pub.publish(steer_msg)

    def _js_reader(self):
        fmt  = 'IhBB'
        size = struct.calcsize(fmt)

        def _trigger(raw):
            v = max(0, raw)
            return 0.0 if v < _JS_TRIGGER_DZ else v / _JS_MAX * self._max_speed

        def _stick(raw):
            return 0.0 if abs(raw) < _JS_STICK_DZ else -raw / _JS_MAX * self._max_steer

        while rclpy.ok():
            rt = 0.0
            lt = 0.0
            steer = 0.0
            try:
                with open(self._dev, 'rb') as f:
                    self.get_logger().info(f'Gamepad opened: {self._dev}')
                    with self._lock:
                        self._connected = True
                    while rclpy.ok():
                        data = f.read(size)
                        if len(data) < size:
                            break
                        _, value, etype, number = struct.unpack(fmt, data)
                        if (etype & ~_JS_EVENT_INIT) != _JS_EVENT_AXIS:
                            continue
                        if number == _JS_RT_AXIS:
                            rt = _trigger(value)
                        elif number == _JS_LT_AXIS:
                            lt = _trigger(value)
                        elif number == _JS_LSX_AXIS:
                            steer = _stick(value)
                        with self._lock:
                            self._speed = rt - lt
                            self._steer = steer

                with self._lock:
                    self._speed     = 0.0
                    self._steer     = 0.0
                    self._connected = False
                # publish one final stop so rs485_bridge doesn't hold last speed
                stop = Float64MultiArray(); stop.data = [0.0, 0.0, 0.0, 0.0]
                self._vel_pub.publish(stop)
                steer_z = Float64MultiArray(); steer_z.data = [0.0]
                self._steer_pub.publish(steer_z)
                self.get_logger().warn('Gamepad disconnected, retry in 3s...')
            except OSError:
                self.get_logger().warn(f'Gamepad unavailable ({self._dev}), retry in 3s...')
            time.sleep(3.0)


def main(args=None):
    rclpy.init(args=args)
    node = JoyBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
