"""
robot.launch.py — запуск на реальном роботе.

Управляющие сигналы уходят на RS-485 через rs485_bridge.
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg      = get_package_share_directory('my_robot')
    pkg_vs   = get_package_share_directory('visual_multi_crop_row_navigation')

    urdf_file  = os.path.join(pkg,    'urdf',    'fito.urdf')
    ekf_yaml   = os.path.join(pkg,    'config',  'ekf.yaml')
    field_yaml = os.path.join(pkg,    'config',  'field_params.yaml')
    vs_yaml    = os.path.join(pkg_vs, 'configs', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
        output='screen',
    )

    rs485_bridge = Node(
        package='my_robot',
        executable='rs485_bridge',
        parameters=[{
            'vim_host':      '192.168.5.42',
            'vim_port':      81,
            'publish_rate':  20.0,
            'max_speed':     10.0,   # рад/с → мощность 255, подобрать на роботе
            'hall_to_rads':  1.0,    # Гц Холла → рад/с, подобрать на роботе
        }],
        output='screen',
    )

    acker_odom = Node(
        package='my_robot',
        executable='acker_odom',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local')],
        output='screen',
    )

    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global')],
        output='screen',
    )

    row_driver = Node(
        package='my_robot',
        executable='row_driver',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'forward_speed': 1.5,
            'publish_rate':  20.0,
        }],
        output='screen',
    )

    field_mission = Node(
        package='my_robot',
        executable='field_mission',
        parameters=[field_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    vs_node = Node(
        package='visual_multi_crop_row_navigation',
        executable='vs_navigation',
        parameters=[vs_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    usb_camera = Node(
        package='my_robot',
        executable='usb_camera',
        parameters=[{
            'device_id': 0,
            'width':     1280,
            'height':    720,
            'fps':       30.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        rs485_bridge,
        usb_camera,
        ekf_odom,
        ekf_map,
        TimerAction(period=2.0, actions=[acker_odom]),
        TimerAction(period=4.0, actions=[row_driver]),
        TimerAction(period=6.0, actions=[field_mission, vs_node]),
    ])
