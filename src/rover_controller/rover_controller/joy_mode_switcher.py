"""
Joy Mode Switcher — control unificado rover + brazo con mando PS5.

Un único nodo maneja TODA la entrada del joystick:
  - Rover mode (default): sticks + L1 deadman → /cmd_vel; △□○ → /drive_mode
  - Arm mode (Cruz toggle): sticks → /ee_velocity_target; R2 → /wrist_velocity_command

NO se necesita teleop_twist_joy cuando se usa este nodo.

Suscribe:
  /joy  (sensor_msgs/Joy)

Publica:
  /cmd_vel               (geometry_msgs/Twist)   — siempre; cero en modo brazo
  /drive_mode            (std_msgs/String)        — swerve | differential | ackermann
  /ee_velocity_target    (geometry_msgs/Vector3)  — velocidad EE en modo brazo
  /wrist_velocity_command (sensor_msgs/JointState) — muñeca en modo brazo

PS5 DualSense — mapeo Linux evdev (verificado con ps5_teleop.yaml):
  Botones:
    0=Cross  1=Circle  2=Square  3=Triangle
    4=L1     5=R1      6=L2      7=R2
  Ejes:
    0=Left  stick X   (izq=-1, der=+1)
    1=Left  stick Y   (arr=-1, abj=+1) ← invertido para avanzar
    2=L2    analógico (-1=suelto, +1=presionado)
    3=Right stick X   (izq=-1, der=+1)
    4=Right stick Y
    5=R2    analógico (-1=suelto, +1=presionado)

Modo rover:
  Mantener L1       → habilita movimiento (deadman)
  L1 + R1           → turbo (2× velocidad)
  Stick izq. Y      → linear.x  (adelante/atrás)
  Stick izq. X      → linear.y  (strafe, solo swerve)
  Stick der. X      → angular.z (rotación)
  △ Triángulo       → swerve
  □ Cuadrado        → differential
  ○ Círculo         → ackermann
  ✕ Cruz            → activar modo brazo

Modo brazo:
  Stick izq. X      → EE velocidad X (lateral)
  Stick izq. Y      → EE velocidad Z (arriba/abajo, invertido)
  Stick der. X      → EE velocidad Y (adelante/atrás)
  R2 analógico      → muñeca joint_5
  ✕ Cruz            → volver a modo rover
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

# ── Botones ───────────────────────────────────────────────────────────────────
BTN_CROSS    = 0
BTN_CIRCLE   = 1
BTN_SQUARE   = 2
BTN_TRIANGLE = 3
BTN_L1       = 4
BTN_R1       = 5

# ── Ejes ─────────────────────────────────────────────────────────────────────
AXIS_LEFT_X  = 0
AXIS_LEFT_Y  = 1   # invertido: arriba = -1 → negar para avanzar
AXIS_RIGHT_X = 3   # stick derecho X (eje 2 es L2 analógico, no stick)
AXIS_R2      = 5   # R2 analógico: -1=suelto → normalizar a 0..1

# ── Velocidades rover ─────────────────────────────────────────────────────────
LINEAR_NORMAL  = 0.5    # m/s
LINEAR_TURBO   = 1.0
ANGULAR_NORMAL = 1.0    # rad/s
ANGULAR_TURBO  = 2.0

# ── Velocidad brazo ───────────────────────────────────────────────────────────
EE_SPEED    = 0.05   # m/s máximo efector
WRIST_SPEED = 0.5    # rad/s muñeca

ROVER_BUTTON_MODE_MAP = {
    BTN_TRIANGLE: 'swerve',
    BTN_SQUARE:   'differential',
    BTN_CIRCLE:   'ackermann',
}


class JoyModeSwitcher(Node):

    def __init__(self):
        super().__init__('joy_mode_switcher')

        self._cmd_vel_pub = self.create_publisher(Twist,      '/cmd_vel',                10)
        self._drive_pub   = self.create_publisher(String,     '/drive_mode',             10)
        self._ee_pub      = self.create_publisher(Vector3,    '/ee_velocity_target',     10)
        self._wrist_pub   = self.create_publisher(JointState, '/wrist_velocity_command', 10)

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self._prev_buttons: list[int] = []
        self._arm_mode = False

        self.get_logger().info(
            'Joy mode switcher listo\n'
            '  Rover: L1+sticks=mover  △=swerve  □=diff  ○=ackermann  ✕=modo brazo\n'
            '  Brazo: stick-izq=EE XZ  stick-der=EE Y  R2=muñeca  ✕=volver rover'
        )

    # ─────────────────────────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        buttons = msg.buttons
        axes    = msg.axes

        # Primera recepción: inicializar estado previo sin actuar
        if not self._prev_buttons:
            self._prev_buttons = list(buttons)
            return

        def rising(idx: int) -> bool:
            prev = self._prev_buttons[idx] if idx < len(self._prev_buttons) else 0
            return idx < len(buttons) and buttons[idx] == 1 and prev == 0

        def btn(idx: int) -> int:
            return buttons[idx] if idx < len(buttons) else 0

        def ax(idx: int) -> float:
            return float(axes[idx]) if idx < len(axes) else 0.0

        # ── Toggle modo brazo ─────────────────────────────────────────────────
        if rising(BTN_CROSS):
            self._arm_mode = not self._arm_mode
            mode_str = 'BRAZO' if self._arm_mode else 'ROVER'
            self.get_logger().info(f'Modo → {mode_str}')
            if not self._arm_mode:
                self._ee_pub.publish(Vector3())   # detener EE al salir de modo brazo

        # ── Modo brazo ────────────────────────────────────────────────────────
        if self._arm_mode:
            # Detener rover explícitamente en cada ciclo
            self._cmd_vel_pub.publish(Twist())

            ee = Vector3()
            ee.x =  ax(AXIS_LEFT_X)  * EE_SPEED   # lateral
            ee.z = -ax(AXIS_LEFT_Y)  * EE_SPEED   # arriba/abajo (eje Y invertido)
            ee.y =  ax(AXIS_RIGHT_X) * EE_SPEED   # adelante/atrás
            self._ee_pub.publish(ee)

            # R2: normalizar de [-1, +1] → [0, 1]
            r2 = (ax(AXIS_R2) + 1.0) / 2.0
            if r2 > 0.05:
                wrist = JointState()
                wrist.name     = ['joint_5']
                wrist.velocity = [r2 * WRIST_SPEED]
                self._wrist_pub.publish(wrist)

        # ── Modo rover ────────────────────────────────────────────────────────
        else:
            # Cambio de modo de conducción (flanco de subida)
            for btn_idx, drive_mode in ROVER_BUTTON_MODE_MAP.items():
                if rising(btn_idx):
                    out = String()
                    out.data = drive_mode
                    self._drive_pub.publish(out)
                    self.get_logger().info(f'Drive mode → {drive_mode}')

            # cmd_vel con deadman L1
            if btn(BTN_L1):
                turbo = btn(BTN_R1)
                scale_lin = LINEAR_TURBO   if turbo else LINEAR_NORMAL
                scale_ang = ANGULAR_TURBO  if turbo else ANGULAR_NORMAL

                twist = Twist()
                twist.linear.x  = -ax(AXIS_LEFT_Y)  * scale_lin   # invertir eje Y
                twist.linear.y  =  ax(AXIS_LEFT_X)  * scale_lin
                twist.angular.z =  ax(AXIS_RIGHT_X) * scale_ang
                self._cmd_vel_pub.publish(twist)
            else:
                # Sin L1: detener rover
                self._cmd_vel_pub.publish(Twist())

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
