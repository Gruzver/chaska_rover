"""
Joy Mode Switcher — switches the rover drive mode and arm control via gamepad.

Subscribes:
  /joy  (sensor_msgs/Joy)

Publishes:
  /drive_mode        (std_msgs/String)       — rover drive mode
  /ee_velocity_target (geometry_msgs/Vector3) — EE velocity when in arm mode
  /wrist_velocity_command (sensor_msgs/JointState) — wrist when in arm mode

PS5 DualSense button mapping:
  Triangle (2) → swerve
  Square   (3) → differential
  Circle   (1) → ackermann
  Cross    (0) → arm mode (detiene rover, activa control de brazo)

Arm mode (stick mapping):
  Left  stick X (axis 0) → EE velocity X (lateral)
  Left  stick Y (axis 1) → EE velocity Z (arriba/abajo, invertido)
  Right stick X (axis 2) → EE velocity Y (adelante/atrás)
  R2    (axis 5)         → gripper (joint_5 wrist)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

# Button indices on PS5 DualSense (Linux evdev)
BTN_CROSS    = 0   # arm mode
BTN_CIRCLE   = 1   # ackermann
BTN_TRIANGLE = 2   # swerve
BTN_SQUARE   = 3   # differential

ROVER_BUTTON_MODE_MAP = {
    BTN_TRIANGLE: 'swerve',
    BTN_SQUARE:   'differential',
    BTN_CIRCLE:   'ackermann',
}

# Axis indices
AXIS_LEFT_X  = 0
AXIS_LEFT_Y  = 1
AXIS_RIGHT_X = 2
AXIS_RIGHT_Y = 3
AXIS_R2      = 5   # analog trigger: -1=released, +1=fully pressed

EE_SPEED = 0.05   # m/s máximo de velocidad del efector


class JoyModeSwitcher(Node):

    def __init__(self):
        super().__init__('joy_mode_switcher')

        self._drive_pub = self.create_publisher(String, '/drive_mode', 10)
        self._ee_pub    = self.create_publisher(Vector3, '/ee_velocity_target', 10)
        self._wrist_pub = self.create_publisher(JointState, '/wrist_velocity_command', 10)

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self._prev_buttons: list[int] = []
        self._arm_mode = False

        self.get_logger().info(
            'Joy mode switcher ready — '
            'Triangle=swerve  Square=differential  Circle=ackermann  Cross=arm'
        )

    def _joy_cb(self, msg: Joy):
        buttons = msg.buttons
        axes    = msg.axes

        if not self._prev_buttons:
            self._prev_buttons = list(buttons)
            return

        # ── Detección de rising edge en botones ───────────────────────────
        def rising(idx):
            prev = self._prev_buttons[idx] if idx < len(self._prev_buttons) else 0
            return idx < len(buttons) and buttons[idx] == 1 and prev == 0

        # Cross → alternar modo brazo
        if rising(BTN_CROSS):
            self._arm_mode = not self._arm_mode
            label = 'arm' if self._arm_mode else 'exit-arm'
            self.get_logger().info(f'Control mode → {label}')
            if not self._arm_mode:
                # Al salir del modo brazo publicar velocidad cero
                self._ee_pub.publish(Vector3())

        if self._arm_mode:
            # ── Modo brazo: sticks → velocidad del efector ────────────────
            def ax(idx):
                return axes[idx] if idx < len(axes) else 0.0

            ee = Vector3()
            ee.x = ax(AXIS_LEFT_X)  * EE_SPEED   # lateral
            ee.z = -ax(AXIS_LEFT_Y) * EE_SPEED   # arriba/abajo (eje Y invertido)
            ee.y = ax(AXIS_RIGHT_X) * EE_SPEED   # adelante/atrás
            self._ee_pub.publish(ee)

            # R2 → wrist (joint_5)
            r2 = (ax(AXIS_R2) + 1.0) / 2.0   # normalizar 0..1
            if r2 > 0.05:
                wrist = JointState()
                wrist.name     = ['joint_5']
                wrist.velocity = [r2 * 0.5]
                self._wrist_pub.publish(wrist)
        else:
            # ── Modo rover: botones de forma ─────────────────────────────
            for btn_idx, mode in ROVER_BUTTON_MODE_MAP.items():
                if rising(btn_idx):
                    out = String()
                    out.data = mode
                    self._drive_pub.publish(out)
                    self.get_logger().info(f'Drive mode → {mode}')

        self._prev_buttons = list(buttons)


def main(args=None):
    rclpy.init(args=args)
    node = JoyModeSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
