#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import myactuator_rmd_py as rmd
import math

# ─── CONFIGURACIÓN ───────────────────────────────────────────
CAN_INTERFACE  = "slcan0"
MOTOR_ID       = 3
MOTOR_SPEED    = 100.0   # deg/s
ANGLE_OFFSET   = 612.0   # grados de desfase
JOINT_NAME     = "joint_2"
# ─────────────────────────────────────────────────────────────

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.get_logger().info(f"Conectando a motor ID {MOTOR_ID} en {CAN_INTERFACE}...")
        self.driver   = rmd.CanDriver(CAN_INTERFACE)
        self.actuator = rmd.ActuatorInterface(self.driver, MOTOR_ID)
        self.get_logger().info("Motor conectado OK")

        self.last_target_deg = self.actuator.getMultiTurnAngle()
        self.get_logger().info(f"Posición inicial del motor: {self.last_target_deg:.2f}°")

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.publish_motor_status)
        self.get_logger().info(f"Escuchando '{JOINT_NAME}' en /joint_states")

    def joint_states_callback(self, msg: JointState):
        if JOINT_NAME not in msg.name:
            return

        idx = msg.name.index(JOINT_NAME)
        angle_rad = -msg.position[idx]                        # radianes raw de joint_states
        angle_deg = math.degrees(angle_rad) + ANGLE_OFFSET  # grados absolutos para el motor
        target_rad = math.radians(angle_deg)                 # lo mismo pero en radianes para mostrar

        if abs(angle_deg - self.last_target_deg) > 0.1:
            self.actuator.sendPositionAbsoluteSetpoint(angle_deg, MOTOR_SPEED)
            self.last_target_deg = angle_deg

            self.get_logger().info(
                f"[joint_states] {JOINT_NAME}: {angle_rad:.4f} rad  →  "
                f"[motor target] {target_rad:.4f} rad ({angle_deg:.2f}°)  "
                f"[offset] {math.radians(ANGLE_OFFSET):.4f} rad ({ANGLE_OFFSET}°)"
            )

    def publish_motor_status(self):
        try:
            status    = self.actuator.getMotorStatus2()
            real_deg  = status.shaft_angle
            real_rad  = math.radians(real_deg)
            target_rad = math.radians(self.last_target_deg)

            self.get_logger().info(
                f"[motor]  Real: {real_rad:.4f} rad ({real_deg:.2f}°)  |  "
                f"Target: {target_rad:.4f} rad ({self.last_target_deg:.2f}°)  |  "
                f"Speed: {status.shaft_speed:.1f} RPM"
            )
        except Exception as e:
            self.get_logger().warn(f"Error leyendo motor: {e}")

    def destroy_node(self):
        self.get_logger().info("Apagando nodo, manteniendo última posición del motor...")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C detectado, manteniendo posición.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()