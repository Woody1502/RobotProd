"""ROS2 node: camera, status tracking, command publishers."""

import asyncio
import json
import threading
from typing import Set

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from fastapi import WebSocket


class RobotNode(Node):
    def __init__(self):
        super().__init__('web_server')

        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._pubs = {
            name: self.create_publisher(Int8, f'/vim/{name}', 10)
            for name in ('bucket', 'frame', 'bunker', 'flaps', 'manipulator', 'separator')
        }
        self._vel_pub   = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self._steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands',  10)
        self._gp_en_pub = self.create_publisher(Bool, '/joy_bridge/enabled', _latched)

        self.create_subscription(Bool, '/joy_bridge/connected', self._on_gp_conn, _latched)
        self.create_subscription(
            Float64MultiArray, '/velocity_controller/commands', self._on_vel, 10)
        self.create_subscription(Image, '/camera/depth/pure_image', self._on_image, 10)

        self._autopilot_pub = self.create_publisher(Bool, '/autopilot/enable', 10)
        self._vs_reset_pub = self.create_publisher(Bool, '/vs_nav/reset_request', 10)

        self.create_subscription(Image, '/vs_nav/graphic', self._on_graphic, 10)

        self._bridge      = CvBridge()
        self._frame_lock  = threading.Lock()
        self._latest_jpeg: bytes | None = None

        self._graphic_lock = threading.Lock()
        self._latest_graphic: bytes | None = None

        self._status: dict = {'gp_connected': False, 'speed': 0.0, 'autopilot': False}
        self._ws_clients: Set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop | None = None

    # ── ROS callbacks ────────────────────────────────────────────────────────

    def _on_gp_conn(self, msg: Bool):
        self._status['gp_connected'] = msg.data
        self._push_status()

    def _on_vel(self, msg: Float64MultiArray):
        if msg.data:
            self._status['speed'] = abs(float(msg.data[0]))
            self._push_status()

    def _on_graphic(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        with self._graphic_lock:
            self._latest_graphic = buf.tobytes()

    def _on_image(self, msg: Image):
        try:
            if msg.encoding in ('16UC1', '32FC1'):
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            else:
                frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        with self._frame_lock:
            self._latest_jpeg = buf.tobytes()

    # ── WebSocket broadcast ──────────────────────────────────────────────────

    def _push_status(self):
        if self._loop is None:
            return
        payload = json.dumps(self._status)
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    async def _broadcast(self, payload: str):
        dead: Set[WebSocket] = set()
        for ws in list(self._ws_clients):
            try:
                await ws.send_text(payload)
            except Exception:
                dead.add(ws)
        self._ws_clients -= dead

    # ── Commands ─────────────────────────────────────────────────────────────

    def cmd(self, device: str, state: int):
        if device in self._pubs:
            self._pubs[device].publish(Int8(data=state))

    def estop(self):
        for pub in self._pubs.values():
            pub.publish(Int8(data=0))
        self._vel_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0]))
        self._steer_pub.publish(Float64MultiArray(data=[0.0]))

    def set_gamepad_enabled(self, enabled: bool):
        self._gp_en_pub.publish(Bool(data=enabled))

    def set_autopilot_enabled(self, enabled: bool):
        self._autopilot_pub.publish(Bool(data=enabled))
        self._status['autopilot'] = enabled
        self._push_status()

    def reset_crop_lane(self):
        self._vs_reset_pub.publish(Bool(data=True))
