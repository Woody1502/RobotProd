#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
from math import sin, cos, tan

class AckermanOdometry(Node):
    def __init__(self):
        super().__init__('acker_odom')
        
        # Параметры из URDF
        self.declare_parameter('wheel_radius', 0.37)  # Радиус колеса (в метрах)
        # Геометрия рулевого механизма: весь передний блок поворачивается как единое целое
        # вокруг шарнира base_link_to_wheeling_mech.
        # rear_to_pivot  = 0.812 (задняя ось) + 1.051 (шарнир) = 1.863 м
        # pivot_to_axle  = 0.585 м (шарнир → передняя ось)
        # Точная формула: R = (pivot_to_axle + rear_to_pivot * cos δ) / sin δ
        self.declare_parameter('rear_to_pivot',  1.863)  # задняя ось → шарнир, м
        self.declare_parameter('pivot_to_axle',  0.585)  # шарнир → передняя ось, м
        self.declare_parameter('track_width',    1.28)   # ширина колеи (не используется в расчёте)

        self.wheel_radius   = self.get_parameter('wheel_radius').value
        self.rear_to_pivot  = self.get_parameter('rear_to_pivot').value
        self.pivot_to_axle  = self.get_parameter('pivot_to_axle').value
        self.track_width    = self.get_parameter('track_width').value
        
        # Текущее состояние
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.steering_angle = 0.0
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.last_time = self.get_clock().now()
        
        # Подписки
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        # Публикации
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info("Ackerman Odometry node initialized")

    def joint_state_callback(self, msg):
        # Получаем текущее время
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Парсим данные из joint_states
        for i, name in enumerate(msg.name):
            if name == "base_link_to_wheeling_mech":
                self.steering_angle = msg.position[i]
            elif name == "back_left_base_to_back_left_wheel":
                self.left_wheel_velocity = msg.velocity[i] * self.wheel_radius
            elif name == "back_right_base_to_back_right_wheel":
                self.right_wheel_velocity = msg.velocity[i] * self.wheel_radius
        
        # Рассчитываем среднюю скорость
        linear_velocity = (self.left_wheel_velocity + self.right_wheel_velocity) *0.5
        
        # Кинематика: передний блок поворачивается как единое целое вокруг шарнира
        # R = (pivot_to_axle + rear_to_pivot * cos δ) / sin δ
        if abs(self.steering_angle) > 0.001:
            turn_radius = (self.pivot_to_axle + self.rear_to_pivot * cos(self.steering_angle)) \
                          / sin(self.steering_angle)
            angular_velocity = linear_velocity / turn_radius
        else:
            angular_velocity = 0.0
        
        # Интегрирование положения
        self.theta -= angular_velocity * dt
        self.x -= linear_velocity * cos(self.theta) * dt
        self.y -= linear_velocity * sin(self.theta) * dt

        # Нормализация угла
        self.theta = math.atan2(sin(self.theta), cos(self.theta))
        
        # Публикация Odometry
        self.publish_odometry(current_time, linear_velocity, angular_velocity)

    def publish_odometry(self, current_time, linear_velocity, angular_velocity):
        # Создаем сообщение Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Заполняем pose
        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Преобразуем угол в кватернион
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = sin(self.theta / 2)
        q.w = cos(self.theta / 2)
        odom_msg.pose.pose.orientation = q
        
        # Заполняем twist
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity
        
        # Публикуем
        self.odom_pub.publish(odom_msg)
        
        # Публикуем трансформацию
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = q
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = AckermanOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()