import os
from pathlib import Path
from os import pathsep
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    rover_description_dir = get_package_share_directory('rover_description')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(
            rover_description_dir, 'urdf', 'rover.urdf.xacro'
        ),
        description='Path to the URDF/xacro model file'
    )

    world_name_arg = DeclareLaunchArgument(
        name='world_name',
        default_value='empty',
        description='World to load (without .world extension)'
    )

    spawn_x_arg = DeclareLaunchArgument(
        name='spawn_x',
        default_value='7.0',
        description='Spawn X position (m)'
    )

    spawn_y_arg = DeclareLaunchArgument(
        name='spawn_y',
        default_value='7.0',
        description='Spawn Y position (m)'
    )

    spawn_z_arg = DeclareLaunchArgument(
        name='spawn_z',
        default_value='3.0',
        description='Spawn Z height (m). Increase for worlds with elevated terrain.'
    )

    world_path = PathJoinSubstitution([
        rover_description_dir,
        'worlds',
        PythonExpression(["'", LaunchConfiguration('world_name'), "'", " + '.world'"])
    ])

    model_path = str(Path(rover_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(rover_description_dir, 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        model_path
    )

    rover_description = ParameterValue(
        Command([
            'xacro ',
            LaunchConfiguration('model'),
            ' is_simulation:=true',
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': rover_description,
            'use_sim_time': True,
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items(),
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'chaska_rover',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
        ]
    )

    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Livox 3D LiDAR — gpu_lidar publica PointCloud en {topic}/points
            '/livox/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Cámara RGBD — rgbd_camera publica image, depth_image, camera_info, points
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        TimerAction(period=3.0, actions=[gz_spawn_entity]),
        gz_ros2_bridge,
    ])
