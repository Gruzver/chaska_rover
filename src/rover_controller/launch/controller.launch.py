import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── ros2_control: spawn controllers ──────────────────────────────
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )

    yaw_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['yaw_position_controller',
                   '--controller-manager', '/controller_manager'],
    )

    wheel_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller',
                   '--controller-manager', '/controller_manager'],
    )

    # ── Swerve kinematics node ────────────────────────────────────────
    swerve_node = Node(
        package='rover_controller',
        executable='swerve_node',
        name='swerve_drive_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        joint_state_broadcaster,
        TimerAction(period=1.0, actions=[yaw_position_controller]),
        TimerAction(period=1.0, actions=[wheel_velocity_controller]),
        TimerAction(period=2.0, actions=[swerve_node]),
    ])
