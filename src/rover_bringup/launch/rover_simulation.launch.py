import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='World to load: empty, rubicon, ...'
    )

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='7.0', description='Spawn X position (m)'
    )

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='7.0', description='Spawn Y position (m)'
    )

    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z', default_value='3.0', description='Spawn Z height (m)'
    )

    # ── Gazebo + robot spawn + ros_gz_bridge ──────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_description'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'spawn_x':    LaunchConfiguration('spawn_x'),
            'spawn_y':    LaunchConfiguration('spawn_y'),
            'spawn_z':    LaunchConfiguration('spawn_z'),
        }.items()
    )

    # ── ros2_control controllers + swerve node ────────────────────────
    # Delay: robot spawns at t=3s inside gazebo.launch.py, then
    # ign_ros2_control needs ~3-5s to initialise → start controllers at t=8s
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_controller'),
                'launch', 'controller.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('rover_description'),
            'rviz', 'display.rviz'
        )],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        gazebo,
        TimerAction(period=8.0, actions=[controller]),
        TimerAction(period=9.0, actions=[rviz]),
    ])
