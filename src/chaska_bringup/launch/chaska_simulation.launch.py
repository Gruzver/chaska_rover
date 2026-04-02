import os
from pathlib import Path
from os import pathsep
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    chaska_bringup_dir      = get_package_share_directory('chaska_bringup')
    rover_description_dir   = get_package_share_directory('rover_description')
    arm_description_dir     = get_package_share_directory('chaska_arm_description')

    # ── Args ─────────────────────────────────────────────────────────────────
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='empty',
        description='World to load (sin extensión): empty, rubicon, ...'
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='10.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='10.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='3')

    # ── Paths ─────────────────────────────────────────────────────────────────
    world_path = PathJoinSubstitution([
        rover_description_dir,
        'worlds',
        PythonExpression(["'", LaunchConfiguration('world_name'), "'", " + '.world'"])
    ])

    # Permite que Gazebo encuentre los meshes de rover_description y chaska_arm_description
    model_path = str(Path(rover_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(rover_description_dir, 'models')
    model_path += pathsep + str(Path(arm_description_dir).parent.resolve())

    gazebo_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path)

    # ── Robot description (URDF unificado) ───────────────────────────────────
    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(chaska_bringup_dir, 'urdf', 'chaska_robot.urdf.xacro'),
            ' is_simulation:=true',
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ── Gazebo Ignition Fortress ──────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items(),
    )

    # ── Spawn robot ───────────────────────────────────────────────────────────
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'chaska_robot',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
        ]
    )

    # ── ROS-Gazebo bridge ─────────────────────────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Livox 3D LiDAR
            '/livox/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Cámara RGBD del rover
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Cámara RealSense D435 del brazo
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
    )

    # ── Controladores ─────────────────────────────────────────────────────────
    # Broadcaster único para rover + brazo
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    # Rover
    yaw_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['yaw_position_controller', '--controller-manager', '/controller_manager'],
    )
    wheel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
    )
    # Brazo
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
    )

    # ── Nodos de aplicación ───────────────────────────────────────────────────
    swerve_node = Node(
        package='rover_controller',
        executable='swerve_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    arm_velocity_node = Node(
        package='chaska_arm_controller',
        executable='joint_velocity_node',
        parameters=[{
            'use_sim_time':   True,
            'control_rate':   50.0,
            'damping':        1e-3,
            'velocity_scale': 1.0,
        }],
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'use_sim_time': True}],
    )

    joy_mode_switcher = Node(
        package='rover_controller',
        executable='joy_mode_switcher',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            rover_description_dir, 'rviz', 'display.rviz'
        )],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Secuencia de arranque ─────────────────────────────────────────────
    # t=0s  : Gazebo + RSP + bridge + joy_node
    # t=3s  : spawn robot
    # t=8s  : joint_state_broadcaster
    # t=9s  : yaw + wheel (rover)
    # t=10s : arm + gripper
    # t=11s : swerve_node + joy_mode_switcher + arm_velocity_node (IK→JointTrajectory)
    # t=12s : RViz

    return LaunchDescription([
        world_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_bridge,
        joy_node,
        TimerAction(period=3.0,  actions=[gz_spawn]),
        TimerAction(period=8.0,  actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=9.0,  actions=[yaw_spawner, wheel_spawner]),
        TimerAction(period=10.0, actions=[arm_spawner, gripper_spawner]),
        TimerAction(period=11.0, actions=[swerve_node, joy_mode_switcher, arm_velocity_node]),
        TimerAction(period=12.0, actions=[rviz]),
    ])
