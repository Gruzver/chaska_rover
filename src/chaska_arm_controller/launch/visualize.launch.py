"""
Visualización estática del brazo en RViz (sin Gazebo, sin joystick).
Útil para revisar URDF, meshes y rangos de joints.

Para la visualización del sistema unificado rover+brazo usar:
  ros2 launch chaska_bringup chaska_display.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_arm = get_package_share_directory('chaska_arm_description')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(pkg_arm, 'urdf', 'chaska_arm.urdf.xacro'),
            ' is_simulation:=false',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_arm, 'rviz', 'chaska_arm.rviz')],
            output='screen',
        ),
    ])
