#!/usr/bin/env python3
"""Entry point: initialise ROS2 node, start uvicorn on port 8080."""

import threading

import rclpy
import uvicorn

from .web.node import RobotNode
from .web.app import create_app


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()

    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    app = create_app(node)
    uvicorn.run(app, host='0.0.0.0', port=8080, log_level='warning')
