# chaska_arm_controller/launch/joystick_control.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ─────────────────────────────────────────────────────────────────────────
    #  Robot description desde xacro (is_simulation=false → mock hardware)
    # ─────────────────────────────────────────────────────────────────────────
    from launch_ros.parameter_descriptions import ParameterValue
    from launch.substitutions import Command
    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(
                get_package_share_directory('chaska_arm_description'),
                'urdf', 'chaska_arm.urdf.xacro'
            ),
            ' is_simulation:=false',
        ]),
        value_type=str,
    )

    # ─────────────────────────────────────────────────────────────────────────
    #  Argumentos configurables desde CLI
    #  Ejemplo: ros2 launch chaska_arm_controller joystick_control.launch.py
    #           damping:=0.005 max_ee_velocity:=0.03
    # ─────────────────────────────────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument('damping',            default_value='0.001'),
        DeclareLaunchArgument('control_rate',       default_value='50.0'),
        DeclareLaunchArgument('velocity_scale',     default_value='1.0'),
        DeclareLaunchArgument('max_ee_velocity',    default_value='0.05'),
        DeclareLaunchArgument('max_joint_velocity', default_value='0.5'),
        DeclareLaunchArgument('slow_multiplier',    default_value='0.3'),
        DeclareLaunchArgument('deadzone',           default_value='0.05'),
        DeclareLaunchArgument('deadman_required',   default_value='false'),
        DeclareLaunchArgument('joy_device_id',      default_value='0'),
    ]

    # ─────────────────────────────────────────────────────────────────────────
    #  1. Driver del joystick
    #     Publica en: /joy  (sensor_msgs/Joy)
    # ─────────────────────────────────────────────────────────────────────────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id':        LaunchConfiguration('joy_device_id'),
            'deadzone':         0.05,
            'autorepeat_rate':  20.0,    # Hz — publica aunque no haya cambios
            'coalesce_interval_ms': 4,   # ms — reduce jitter
        }],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────────────
    #  2. robot_state_publisher
    #     Lee /joint_states  →  publica /tf y /tf_static  →  RViz lo consume
    # ─────────────────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,   # Hz — debe ser >= control_rate
        }],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────────────
    #  3. Nodo de cinemática con Pinocchio
    #     Suscribe:  /ee_velocity_target   (geometry_msgs/Vector3)
    #                /wrist_velocity_command (sensor_msgs/JointState)
    #                /arm_estop             (std_msgs/Bool)
    #                /joint_states_feedback (sensor_msgs/JointState)  [opcional]
    #     Publica:   /joint_states          (sensor_msgs/JointState)  → RSP → RViz
    #                /joint_velocity_command (sensor_msgs/JointState) → controlador real
    #                /ee_velocity_actual    (geometry_msgs/Vector3)
    #                /ee_position           (geometry_msgs/Vector3)
    # ─────────────────────────────────────────────────────────────────────────
    joint_velocity_node = Node(
        package='chaska_arm_controller',
        executable='joint_velocity_node',
        name='joint_velocity_node',
        parameters=[{
            'damping':        LaunchConfiguration('damping'),
            'control_rate':   LaunchConfiguration('control_rate'),
            'velocity_scale': LaunchConfiguration('velocity_scale'),
        }],
        output='screen',
        # Sin remappings — publica directo en /joint_states
    )

    # ─────────────────────────────────────────────────────────────────────────
    #  4. Controlador de joystick
    #     Suscribe:  /joy                  (sensor_msgs/Joy)
    #     Publica:   /ee_velocity_target   (geometry_msgs/Vector3)
    #                /wrist_velocity_command (sensor_msgs/JointState)
    #                /arm_estop             (std_msgs/Bool)
    # ─────────────────────────────────────────────────────────────────────────
    joystick_controller = Node(
        package='chaska_arm_controller',
        executable='joystick_controller',
        name='joystick_controller_node',
        parameters=[{
            'max_ee_velocity':    LaunchConfiguration('max_ee_velocity'),
            'max_joint_velocity': LaunchConfiguration('max_joint_velocity'),
            'slow_multiplier':    LaunchConfiguration('slow_multiplier'),
            'deadzone':           LaunchConfiguration('deadzone'),
            'deadman_required':   LaunchConfiguration('deadman_required'),
            'publish_rate':       LaunchConfiguration('control_rate'),
        }],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────────────
    #  5. RViz2
    # ─────────────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('chaska_arm_description'),
            'rviz', 'chaska_arm.rviz'
        )],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────────────
    #  Orden de arranque:
    #   robot_state_publisher  (necesita /joint_states para publicar /tf)
    #   joint_velocity_node    (empieza a publicar /joint_states desde home)
    #   joystick_controller    (empieza a mandar comandos)
    #   joy_node               (fuente de /joy)
    #   rviz2                  (consume /tf)
    # ─────────────────────────────────────────────────────────────────────────
    return LaunchDescription(
        declared_args + [
            LogInfo(msg='Iniciando chaska_arm joystick control...'),
            robot_state_publisher,
            joint_velocity_node,
            joystick_controller,
            joy_node,
            rviz_node,
        ]
    )