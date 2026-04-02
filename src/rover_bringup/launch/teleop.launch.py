"""
Teleoperation launch file — run alongside rover_simulation.launch.py.

Usage:
  ros2 launch rover_bringup teleop.launch.py

Controls (PS5 DualSense):
  Hold L1            → enable movement
  Hold L1 + R1       → turbo (2x speed)
  Left stick Y       → forward / backward
  Left stick X       → strafe left / right  (swerve mode only)
  Right stick X      → rotate left / right

  Triangle (△)       → switch to swerve mode
  Square   (□)       → switch to differential mode
  Circle   (○)       → switch to ackermann mode
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    teleop_config = os.path.join(
        get_package_share_directory('rover_bringup'),
        'config', 'ps5_teleop.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'device_id': 0}],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[teleop_config],
    )

    mode_switcher_node = Node(
        package='rover_controller',
        executable='joy_mode_switcher',
        name='joy_mode_switcher',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        mode_switcher_node,
    ])
