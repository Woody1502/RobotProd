#!/usr/bin/env python3
"""
camera_node.py — captures frames from a USB camera and publishes them
                 to the topics expected by visual_servoing_node.

Published topics:
  /camera/depth/pure_image  (sensor_msgs/Image,      bgr8)
  /camera/depth/image_depth (sensor_msgs/Image,      32FC1 — zeros, no real depth)
  /camera/depth/camera_info (sensor_msgs/CameraInfo)

Parameters:
  device_id : V4L2 device index (default 0  → /dev/video0)
  width     : capture width,  px (default 1280)
  height    : capture height, px (default 720)
  fps       : capture rate,   Hz (default 30.0)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('width',     1280)
        self.declare_parameter('height',    720)
        self.declare_parameter('fps',       30.0)

        device_id    = self.get_parameter('device_id').value
        self._width  = self.get_parameter('width').value
        self._height = self.get_parameter('height').value
        fps          = self.get_parameter('fps').value

        self._bridge = CvBridge()

        self._cap = cv2.VideoCapture(device_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS,          fps)

        if not self._cap.isOpened():
            self.get_logger().error(f'Cannot open camera device {device_id}')
        else:
            self.get_logger().info(
                f'Camera device {device_id} opened ({self._width}x{self._height} @ {fps} fps)')

        self._img_pub   = self.create_publisher(Image,      '/camera/depth/pure_image',  10)
        self._depth_pub = self.create_publisher(Image,      '/camera/depth/image_depth', 10)
        self._info_pub  = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        self._camera_info = self._make_camera_info()
        self._zero_depth  = np.zeros((self._height, self._width), dtype=np.float32)

        self.create_timer(1.0 / fps, self._tick)

    def _make_camera_info(self) -> CameraInfo:
        info = CameraInfo()
        info.width  = self._width
        info.height = self._height
        info.distortion_model = 'plumb_bob'
        fx = float(self._width)  # rough estimate; replace with calibrated values if needed
        fy = float(self._width)
        cx = self._width  / 2.0
        cy = self._height / 2.0
        info.k = [fx, 0.0, cx,  0.0, fy, cy,  0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,  0.0, fy, cy, 0.0,  0.0, 0.0, 1.0, 0.0]
        return info

    def _tick(self):
        if not self._cap.isOpened():
            return

        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('Camera read failed', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()

        img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp    = now
        img_msg.header.frame_id = 'camera_link'
        self._img_pub.publish(img_msg)

        depth_msg = self._bridge.cv2_to_imgmsg(self._zero_depth, encoding='32FC1')
        depth_msg.header.stamp    = now
        depth_msg.header.frame_id = 'camera_link'
        self._depth_pub.publish(depth_msg)

        self._camera_info.header.stamp    = now
        self._camera_info.header.frame_id = 'camera_link'
        self._info_pub.publish(self._camera_info)

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
