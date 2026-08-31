#!/usr/bin/env python3
"""
joy_bridge.py — reads gamepad via Linux joystick API (/dev/input/js0)
and publishes robot control commands to ROS2 topics.

Xbox Wireless Controller mapping:
  Axis 0  — Left Stick X   → steering
  Axis 4  — LT             → drive backward
  Axis 5  — RT             → drive forward
  Axis 6  — D-pad X        → gripper ←/→ (manipulator) | [X held] bunker ↑/↓
  Axis 7  — D-pad Y        → arm ↑/↓ (manipulator)     | [X held] frame ↑/↓
  Btn A   — hold           → separator forward (up)
  Btn B   — press          → separator stop
  Btn X   — hold           → secondary mode modifier
  Btn Y   — hold           → separator reverse (down)
  Btn LB  — hold           → bucket up   | [X held] flaps up
  Btn RB  — hold           → bucket down | [X held] flaps down
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, Int8

_JS_EVENT_AXIS   = 0x02
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_INIT   = 0x80

_JS_RT_AXIS  = 5
_JS_LT_AXIS  = 4
_JS_LSX_AXIS = 0
_JS_DPAD_X   = 6   # left=-32767, right=+32767
_JS_DPAD_Y   = 7   # up=-32767,   down=+32767

_JS_BTN_A  = 0    # separator forward (hold)
_JS_BTN_B  = 1    # separator stop
_JS_BTN_X  = 2    # secondary mode modifier (hold)
_JS_BTN_Y  = 3    # separator reverse (hold)
_JS_BTN_LB = 4    # bucket up  / [X] flaps up
_JS_BTN_RB = 5    # bucket down / [X] flaps down

_JS_TRIGGER_DZ = 500
_JS_STICK_DZ   = 2000
_JS_DPAD_DZ    = 16000
_JS_MAX        = 32767.0


def _dpad_to_manip_cmd(dx: int, dy: int) -> int:
    if dy < -_JS_DPAD_DZ: return 1   # arm up
    if dy > _JS_DPAD_DZ:  return 2   # arm down
    if dx < -_JS_DPAD_DZ: return 3   # gripper left
    if dx > _JS_DPAD_DZ:  return 4   # gripper right
    return 0


def _dpad_to_frame_bunker(dx: int, dy: int):
    """Secondary mode: D-pad Y → frame, D-pad X → bunker. Returns (frame_cmd, bunker_cmd)."""
    frame  = 1 if dy < -_JS_DPAD_DZ else (2 if dy > _JS_DPAD_DZ else 0)
    bunker = 1 if dx < -_JS_DPAD_DZ else (2 if dx > _JS_DPAD_DZ else 0)
    return frame, bunker


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

        # Drive
        self._vel_pub   = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self._steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands',  10)

        # Attachments
        self._manip_pub  = self.create_publisher(Int8, '/vim/manipulator', 10)
        self._bucket_pub = self.create_publisher(Int8, '/vim/bucket',      10)
        self._frame_pub  = self.create_publisher(Int8, '/vim/frame',       10)
        self._bunker_pub = self.create_publisher(Int8, '/vim/bunker',      10)
        self._flaps_pub  = self.create_publisher(Int8, '/vim/flaps',       10)
        self._sep_pub    = self.create_publisher(Int8, '/vim/separator',   10)

        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._conn_pub  = self.create_publisher(Bool, '/joy_bridge/connected', _latched)

        self._lock      = threading.Lock()
        self._speed     = 0.0
        self._steer     = 0.0
        self._connected = False

        # Input state (protected by _lock)
        self._dpad_x = 0
        self._dpad_y = 0
        self._btn_x  = False   # modifier
        self._btn_lb = False
        self._btn_rb = False

        self.create_timer(1.0 / rate, self._publish)
        threading.Thread(target=self._js_reader, daemon=True).start()

    # ── periodic drive publisher ─────────────────────────────────────────────

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

    # ── event handlers ───────────────────────────────────────────────────────

    def _on_dpad(self, axis: int, value: int):
        with self._lock:
            if axis == _JS_DPAD_X:
                self._dpad_x = value
            else:
                self._dpad_y = value
            dx, dy  = self._dpad_x, self._dpad_y
            x_held  = self._btn_x

        if x_held:
            frame, bunker = _dpad_to_frame_bunker(dx, dy)
            self._frame_pub.publish(Int8(data=frame))
            self._bunker_pub.publish(Int8(data=bunker))
        else:
            self._manip_pub.publish(Int8(data=_dpad_to_manip_cmd(dx, dy)))

    def _on_button(self, btn: int, pressed: bool):
        with self._lock:
            if btn == _JS_BTN_X:
                self._btn_x = pressed
            elif btn == _JS_BTN_LB:
                self._btn_lb = pressed
            elif btn == _JS_BTN_RB:
                self._btn_rb = pressed
            dx, dy = self._dpad_x, self._dpad_y
            x_held = self._btn_x
            lb     = self._btn_lb
            rb     = self._btn_rb

        if btn == _JS_BTN_X:
            if pressed:
                # Primary → Secondary: stop manipulator/bucket, start frame/bunker/flaps
                self._manip_pub.publish(Int8(data=0))
                self._bucket_pub.publish(Int8(data=0))
                frame, bunker = _dpad_to_frame_bunker(dx, dy)
                self._frame_pub.publish(Int8(data=frame))
                self._bunker_pub.publish(Int8(data=bunker))
                if lb:
                    self._flaps_pub.publish(Int8(data=1))
                elif rb:
                    self._flaps_pub.publish(Int8(data=2))
            else:
                # Secondary → Primary: stop frame/bunker/flaps, start manipulator/bucket
                self._frame_pub.publish(Int8(data=0))
                self._bunker_pub.publish(Int8(data=0))
                self._flaps_pub.publish(Int8(data=0))
                self._manip_pub.publish(Int8(data=_dpad_to_manip_cmd(dx, dy)))
                if lb:
                    self._bucket_pub.publish(Int8(data=1))
                elif rb:
                    self._bucket_pub.publish(Int8(data=2))

        elif btn == _JS_BTN_LB:
            if x_held:
                self._flaps_pub.publish(Int8(data=1 if pressed else 0))
            else:
                self._bucket_pub.publish(Int8(data=1 if pressed else 0))

        elif btn == _JS_BTN_RB:
            if x_held:
                self._flaps_pub.publish(Int8(data=2 if pressed else 0))
            else:
                self._bucket_pub.publish(Int8(data=2 if pressed else 0))

        elif btn == _JS_BTN_A:
            self._sep_pub.publish(Int8(data=1 if pressed else 0))   # forward

        elif btn == _JS_BTN_Y:
            self._sep_pub.publish(Int8(data=2 if pressed else 0))   # reverse

        elif btn == _JS_BTN_B and pressed:
            self._sep_pub.publish(Int8(data=0))                     # stop

    def _stop_all_attachments(self):
        for pub in (self._manip_pub, self._bucket_pub, self._frame_pub,
                    self._bunker_pub, self._flaps_pub, self._sep_pub):
            pub.publish(Int8(data=0))

    # ── gamepad reader thread ────────────────────────────────────────────────

    def _js_reader(self):
        fmt  = 'IhBB'
        size = struct.calcsize(fmt)

        def _trigger(raw):
            v = max(0, raw)
            return 0.0 if v < _JS_TRIGGER_DZ else v / _JS_MAX * self._max_speed

        def _stick(raw):
            return 0.0 if abs(raw) < _JS_STICK_DZ else -raw / _JS_MAX * self._max_steer

        while rclpy.ok():
            rt = lt = steer = 0.0
            try:
                with open(self._dev, 'rb') as f:
                    self.get_logger().info(f'Gamepad opened: {self._dev}')
                    with self._lock:
                        self._connected = True
                    self._conn_pub.publish(Bool(data=True))

                    while rclpy.ok():
                        data = f.read(size)
                        if len(data) < size:
                            break
                        _, value, etype, number = struct.unpack(fmt, data)
                        is_init   = bool(etype & _JS_EVENT_INIT)
                        etype_raw = etype & ~_JS_EVENT_INIT

                        if not is_init:
                            self.get_logger().info(
                                f'[JOY] type={etype_raw} num={number} val={value}',
                                throttle_duration_sec=0.5)

                        if etype_raw == _JS_EVENT_AXIS:
                            if number == _JS_RT_AXIS:
                                rt = _trigger(value)
                            elif number == _JS_LT_AXIS:
                                lt = _trigger(value)
                            elif number == _JS_LSX_AXIS:
                                steer = _stick(value)
                            elif number in (_JS_DPAD_X, _JS_DPAD_Y):
                                self._on_dpad(number, value)
                                continue
                            with self._lock:
                                self._speed = lt - rt
                                self._steer = steer

                        elif etype_raw == _JS_EVENT_BUTTON and not is_init:
                            self._on_button(number, bool(value))

                self._conn_pub.publish(Bool(data=False))
                with self._lock:
                    self._speed  = 0.0
                    self._steer  = 0.0
                    self._connected = False
                    self._dpad_x = 0
                    self._dpad_y = 0
                    self._btn_x  = False
                    self._btn_lb = False
                    self._btn_rb = False
                stop = Float64MultiArray(); stop.data = [0.0, 0.0, 0.0, 0.0]
                self._vel_pub.publish(stop)
                steer_z = Float64MultiArray(); steer_z.data = [0.0]
                self._steer_pub.publish(steer_z)
                self._stop_all_attachments()
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
