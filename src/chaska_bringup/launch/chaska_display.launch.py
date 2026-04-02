"""
Visualización estática del robot unificado (rover + brazo) en RViz.
No lanza Gazebo. Útil para ajustar posición del arm_mount_joint,
revisar el árbol TF y verificar meshes sin necesidad de simulación.

Lanzamiento:
  ros2 launch chaska_bringup chaska_display.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    chaska_bringup_dir    = get_package_share_directory('chaska_bringup')
    rover_description_dir = get_package_share_directory('rover_description')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(chaska_bringup_dir, 'urdf', 'chaska_robot.urdf.xacro'),
            ' is_simulation:=false',
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # joint_state_publisher_gui: permite mover los joints manualmente en RViz
    # para verificar rangos y cinemática del brazo
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            rover_description_dir, 'rviz', 'display.rviz'
        )],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
