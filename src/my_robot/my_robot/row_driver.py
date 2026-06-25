#!/usr/bin/env python3
"""
Row driver node: drives wheels forward at a fixed speed for autopilot mode.
Subscribes to /autopilot/enable (Bool) to start/stop.
Also subscribes to /position_controller/commands to detect autopilot activity.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool


class RowDriverNode(Node):
    def __init__(self):
        super().__init__('row_driver')

        self.declare_parameter('forward_speed', 1.0)   # rad/s at wheels
        self.declare_parameter('publish_rate', 20.0)   # Hz

        self.forward_speed = self.get_parameter('forward_speed').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.enabled = False

        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        self.enable_sub = self.create_subscription(
            Bool, '/autopilot/enable', self.enable_cb, 10)

        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_velocity)

        self.get_logger().info('Row driver ready. Send True to /autopilot/enable to start.')

    def enable_cb(self, msg: Bool):
        was_enabled = self.enabled
        self.enabled = msg.data
        state = 'ENABLED' if self.enabled else 'DISABLED'
        self.get_logger().info(f'Autopilot drive: {state}')
        if was_enabled and not self.enabled:
            # rs485_bridge only stops on an explicit 0.0 — going silent here
            # would leave the wheels holding the last forward_speed forever
            cmd = Float64MultiArray()
            cmd.data = [0.0] * 4
            self.velocity_pub.publish(cmd)

    def publish_velocity(self):
        if not self.enabled:
            return
        cmd = Float64MultiArray()
        cmd.data = [self.forward_speed] * 4
        self.velocity_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RowDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
