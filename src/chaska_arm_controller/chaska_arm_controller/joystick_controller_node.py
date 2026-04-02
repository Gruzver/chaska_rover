# chaska_arm_controller/joystick_controller_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool


# ─────────────────────────────────────────────
#  MAPEO DEL MANDO  (ajusta según tu mando)
#  Ejecuta: ros2 topic echo /joy  para verificar
# ─────────────────────────────────────────────
class GamepadMap:
    # ── Ejes ──────────────────────────────────
    AXIS_LX     = 0   # Joystick izq. horizontal → vx
    AXIS_LY     = 1   # Joystick izq. vertical   → vy
    AXIS_RX     = 3   # Joystick der. horizontal → joint_4
    AXIS_RY     = 4   # Joystick der. vertical   → joint_5
    AXIS_L2     = 2   # Gatillo izquierdo        → slow mode
    AXIS_R2     = 5   # Gatillo derecho           → dead man switch

    # ── Botones ───────────────────────────────
    BTN_VZ_UP   = 3   # □ / Y  → vz positivo
    BTN_VZ_DOWN = 0   # × / A  → vz negativo
    BTN_HOME    = 7   # START  → posición home
    BTN_ESTOP   = 6   # SELECT → E-STOP / reanudar


# ─────────────────────────────────────────────
#  NOMBRES DE JOINTS (mismo orden que Pinocchio)
# ─────────────────────────────────────────────
JOINT_NAMES   = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
IDX_JOINT_4   = 3   # índice de joint_4 en el vector
IDX_JOINT_5   = 4   # índice de joint_5 en el vector


class JoystickControllerNode(Node):
    def __init__(self):
        super().__init__('joystick_controller_node')

        # ── Parámetros ────────────────────────
        self.declare_parameter('max_ee_velocity',    0.05)   # m/s  → vx, vy, vz
        self.declare_parameter('max_joint_velocity', 0.5)    # rad/s → joint_4, joint_5
        self.declare_parameter('slow_multiplier',    0.3)    # factor modo lento
        self.declare_parameter('deadzone',           0.05)   # zona muerta joystick
        self.declare_parameter('deadman_required',   False)  # True = requiere R2
        self.declare_parameter('publish_rate',       50.0)   # Hz

        self.max_ee_vel    = self.get_parameter('max_ee_velocity').value
        self.max_joint_vel = self.get_parameter('max_joint_velocity').value
        self.slow_mult     = self.get_parameter('slow_multiplier').value
        self.deadzone      = self.get_parameter('deadzone').value
        self.deadman_req   = self.get_parameter('deadman_required').value
        rate               = self.get_parameter('publish_rate').value
        self.dt            = 1.0 / rate

        # ── Estado interno ────────────────────
        self.axes    = []
        self.buttons = []
        self.estop   = False          # E-STOP activo
        self.joy_ok  = False          # Se recibió al menos un mensaje Joy
        self._prev_estop_btn = 0      # Para toggle del E-STOP

        # Velocidades de salida
        self.v_ee        = np.zeros(3)   # [vx, vy, vz]
        self.dq_wrist    = np.zeros(2)   # [dq4, dq5]

        # ── QoS ──────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ── Suscriptores ─────────────────────
        self.create_subscription(Joy, '/joy', self.joy_callback, qos)

        # ── Publicadores ─────────────────────
        self.ee_vel_pub    = self.create_publisher(Vector3,   '/ee_velocity_target',   qos)
        self.wrist_vel_pub = self.create_publisher(JointState, '/wrist_velocity_command', qos)
        self.estop_pub     = self.create_publisher(Bool,       '/arm_estop',            qos)

        # ── Timer ─────────────────────────────
        self.create_timer(self.dt, self.publish_loop)

        self.get_logger().info('─' * 50)
        self.get_logger().info('  JoystickControllerNode iniciado')
        self.get_logger().info('─' * 50)
        self.get_logger().info('  JS izq (↔↕) → vx / vy')
        self.get_logger().info('  □ / Y       → vz +')
        self.get_logger().info('  × / A       → vz -')
        self.get_logger().info('  JS der (↔↕) → joint_4 / joint_5')
        self.get_logger().info('  SELECT      → E-STOP toggle')
        self.get_logger().info('  START       → HOME')
        self.get_logger().info('─' * 50)

    # ──────────────────────────────────────────
    #  Callback del joystick
    # ──────────────────────────────────────────
    def joy_callback(self, msg: Joy):
        self.axes    = list(msg.axes)
        self.buttons = list(msg.buttons)
        self.joy_ok  = True

        # Toggle E-STOP con botón SELECT
        estop_btn = self._get_button(GamepadMap.BTN_ESTOP)
        if estop_btn == 1 and self._prev_estop_btn == 0:
            self.estop = not self.estop
            state = 'ACTIVADO 🔴' if self.estop else 'DESACTIVADO 🟢'
            self.get_logger().warn(f'E-STOP {state}')
        self._prev_estop_btn = estop_btn

    # ──────────────────────────────────────────
    #  Loop de publicación a frecuencia fija
    # ──────────────────────────────────────────
    def publish_loop(self):
        if not self.joy_ok:
            return

        # Publicar estado E-STOP siempre
        self.estop_pub.publish(Bool(data=self.estop))

        # Si E-STOP → publicar ceros y salir
        if self.estop:
            self._publish_zero()
            return

        # Verificar dead man switch (R2 gatillo derecho)
        if self.deadman_req:
            r2_val = self._get_axis(GamepadMap.AXIS_R2)
            # R2 en reposo = 1.0, presionado = -1.0
            deadman_active = r2_val < 0.0
            if not deadman_active:
                self._publish_zero()
                return

        # ── Factor de velocidad (modo lento) ──
        l2_val = self._get_axis(GamepadMap.AXIS_L2)
        slow_mode = l2_val < 0.0   # L2 presionado activa modo lento
        speed_factor = self.slow_mult if slow_mode else 1.0

        # ── Velocidad EE ──────────────────────
        vx = self._apply_deadzone(self._get_axis(GamepadMap.AXIS_LX)) * self.max_ee_vel * speed_factor
        vy = self._apply_deadzone(self._get_axis(GamepadMap.AXIS_LY)) * self.max_ee_vel * speed_factor

        # vz con botones (mayor prioridad que joystick)
        vz = 0.0
        if self._get_button(GamepadMap.BTN_VZ_UP):
            vz =  self.max_ee_vel * speed_factor
        elif self._get_button(GamepadMap.BTN_VZ_DOWN):
            vz = -self.max_ee_vel * speed_factor

        # ── Velocidad de muñeca (joint_4, joint_5) ──
        dq4 = self._apply_deadzone(self._get_axis(GamepadMap.AXIS_RX)) * self.max_joint_vel * speed_factor
        dq5 = self._apply_deadzone(self._get_axis(GamepadMap.AXIS_RY)) * self.max_joint_vel * speed_factor

        # ── Publicar velocidad EE ─────────────
        ee_msg = Vector3(x=vx, y=vy, z=vz)
        self.ee_vel_pub.publish(ee_msg)

        # ── Publicar velocidad de muñeca ──────
        wrist_msg = JointState()
        wrist_msg.header.stamp = self.get_clock().now().to_msg()
        wrist_msg.name         = ['joint_4', 'joint_5']
        wrist_msg.velocity     = [dq4, dq5]
        self.wrist_vel_pub.publish(wrist_msg)

        # Log solo cuando hay movimiento
        moving = abs(vx) + abs(vy) + abs(vz) + abs(dq4) + abs(dq5) > 0.001
        if moving:
            mode = '🐢 LENTO' if slow_mode else '🚀 NORMAL'
            self.get_logger().info(
                f'{mode} | EE=[{vx:+.3f}, {vy:+.3f}, {vz:+.3f}] m/s '
                f'| j4={dq4:+.3f} j5={dq5:+.3f} rad/s'
            )

    # ──────────────────────────────────────────
    #  Helpers
    # ──────────────────────────────────────────
    def _get_axis(self, idx: int) -> float:
        if idx < len(self.axes):
            return float(self.axes[idx])
        return 0.0

    def _get_button(self, idx: int) -> int:
        if idx < len(self.buttons):
            return int(self.buttons[idx])
        return 0

    def _apply_deadzone(self, value: float) -> float:
        """Elimina ruido en el centro del joystick y rescala el rango."""
        if abs(value) < self.deadzone:
            return 0.0
        # Rescalar para que arranque desde 0 al salir de la zona muerta
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _publish_zero(self):
        """Publica velocidad cero en todos los tópicos."""
        self.ee_vel_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))
        zero_wrist = JointState()
        zero_wrist.header.stamp = self.get_clock().now().to_msg()
        zero_wrist.name         = ['joint_4', 'joint_5']
        zero_wrist.velocity     = [0.0, 0.0]
        self.wrist_vel_pub.publish(zero_wrist)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()