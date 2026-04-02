import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('chaska_arm_description')

    # Ignition convierte package:// → model:// al parsear el SDF.
    # IGN_GAZEBO_RESOURCE_PATH debe apuntar al directorio PADRE del share del paquete
    # para que model://chaska_arm_description/meshes/... se resuelva correctamente.
    pkg_share_parent = os.path.dirname(pkg_share)
    amiibot_models = os.path.join(
        get_package_share_directory('amiibot_description'), 'models'
    )
    chaska_models = os.path.join(pkg_share, 'models')
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=pkg_share_parent + ':' + amiibot_models + ':' + chaska_models,
    )

    # ── Argumentos ───────────────────────────────────────────────────────────

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(pkg_share, 'urdf', 'chaska_arm.urdf.xacro'),
        description='Ruta al archivo xacro del robot',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Usar reloj de simulación (Gazebo)',
    )

    # ── Procesar xacro → URDF ────────────────────────────────────────────────

    robot_description = ParameterValue(
        Command([
            'xacro ',
            LaunchConfiguration('model'),
            ' is_simulation:=true',
        ]),
        value_type=str,
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ── Ignition Gazebo (mundo vacío) ─────────────────────────────────────────

    world_file = os.path.join(pkg_share, 'worlds', 'small_warehouse.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ]),
        launch_arguments={'gz_args': f'-v 4 -r {world_file}'}.items(),
    )

    # ── Bridge ROS 2 ↔ Ignition: solo /clock ─────────────────────────────────

    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros2_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
    )

    # ── Spawn robot (espera 3s a que Gazebo inicialice) ───────────────────────

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_chaska_arm',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'chaska_arm',
                    '-z', '0.0',
                ],
            )
        ],
    )

    # ── Cargar controllers ────────────────────────────────────────────────────

    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            )
        ],
    )

    arm_controller_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            )
        ],
    )

    gripper_controller_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
            )
        ],
    )

    return LaunchDescription([
        set_ign_resource_path,
        model_arg,
        use_sim_time_arg,
        robot_state_publisher,
        gazebo,
        gz_ros2_bridge,
        spawn_robot,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
