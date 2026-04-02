import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    pkg_rover_description = get_package_share_directory('rover_description')

    # --- Robot State Publisher (real hardware) ---
    rover_description = ParameterValue(
        Command(['xacro ', os.path.join(
            pkg_rover_description, 'urdf', 'rover.urdf.xacro'
        ), ' is_simulation:=false']),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': rover_description,
            'use_sim_time': False,
        }]
    )

    # --- ros2_control: controller manager ---
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': rover_description},
            os.path.join(pkg_rover_description, 'config', 'rover_controllers.yaml'),
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    yaw_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['yaw_position_controller'],
    )

    wheel_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller'],
    )

    # --- RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            pkg_rover_description, 'rviz', 'display.rviz'
        )],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        TimerAction(period=2.0, actions=[
            LogInfo(msg='--- Spawning controllers ---'),
            joint_state_broadcaster_spawner,
        ]),
        TimerAction(period=3.0, actions=[yaw_position_controller_spawner]),
        TimerAction(period=3.0, actions=[wheel_velocity_controller_spawner]),
        TimerAction(period=3.0, actions=[rviz]),
    ])
