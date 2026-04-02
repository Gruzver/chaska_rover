"""
Teleoperation launch — usar junto con rover_simulation.launch.py (solo rover)
o rover_bringup.launch.py (hardware real).

NO usar junto con chaska_simulation.launch.py — ese launch ya incluye
joy_node + joy_mode_switcher internamente.

joy_mode_switcher publica /cmd_vel directamente (no se necesita teleop_twist_joy).

Controles PS5 DualSense:
  Mantener L1       → habilita movimiento
  L1 + R1           → turbo (2× velocidad)
  Stick izq. Y      → adelante / atrás
  Stick izq. X      → strafe (solo modo swerve)
  Stick der. X      → rotar
  △ Triángulo       → modo swerve
  □ Cuadrado        → modo differential
  ○ Círculo         → modo ackermann
  ✕ Cruz            → modo brazo (arm_controller en hardware real)
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'device_id': 0}],
    )

    mode_switcher_node = Node(
        package='rover_controller',
        executable='joy_mode_switcher',
        name='joy_mode_switcher',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        mode_switcher_node,
    ])
