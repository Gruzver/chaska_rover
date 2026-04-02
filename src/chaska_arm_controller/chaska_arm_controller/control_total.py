#!/usr/bin/env python3
"""
robot_control.py
================
Nodo ROS2 Humble unificado que lee /joy y controla:
  [ESP32 - Rover]  servo1, servo2, motor DC          → /dev/ttyESP32_rover
  [Arduino - NEMA] motor NEMA desplazamiento + ángulo → /dev/ttyArduino_nema
  [CAN - RMD]      motor RMD ángulo                   → slcan0 (manual)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time
import math

try:
    import myactuator_rmd_py as rmd
    RMD_AVAILABLE = True
except ImportError:
    RMD_AVAILABLE = False

# ─── CONFIGURACIÓN PUERTOS ───────────────────────────────────────────────────

ROVER_PORT    = '/dev/ttyESP32_rover'
ROVER_BAUD    = 115200

NEMA_PORT     = '/dev/ttyArduino_nema'
NEMA_BAUD     = 115200

CAN_INTERFACE = 'slcan0'

# ─── CONFIGURACIÓN RMD ───────────────────────────────────────────────────────

MOTOR_ID      = 3
MOTOR_SPEED   = 100.0
ANGLE_OFFSET  = 612.0

# ─── CONFIGURACIÓN ROVER ─────────────────────────────────────────────────────

SERVO_STEP        = 2.0
MOTOR_SPEED_ROVER = 70

# ─── MAPEO JOYSTICK ───────────────────────────────────────────────────────────

AXIS_MOTOR_DC   = 4
AXIS_NEMA_DESP  = 0
AXIS_RMD        = 1
AXIS_NEMA_ANG   = 2

BTN_SERVO2_UP   = 4
BTN_SERVO1_UP   = 5
BTN_SERVO2_DOWN = 6
BTN_SERVO1_DOWN = 7

# ─── ESCALAS Y LÍMITES ────────────────────────────────────────────────────────

SCALE_DESP_MM  = 1.0
SCALE_RAD      = 0.01
DEAD_ZONE      = 0.05

MAX_DESP_MM    = 220.0
MIN_DESP_MM    = -220.0
MAX_RAD_NEMA   = math.pi
MAX_RAD_RMD    = 4 * math.pi

UMBRAL_DESP_MM = 0.1
UMBRAL_RAD     = 0.001

CONTROL_PERIOD = 0.1

# ─────────────────────────────────────────────────────────────────────────────

def apply_dead_zone(value: float, dead: float) -> float:
    if abs(value) < dead:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dead) / (1.0 - dead)


def open_serial(port: str, baud: int, logger, name: str):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        logger.info(f'[{name}] Serial conectado: {port}')
        return ser
    except Exception as e:
        logger.error(f'[{name}] No se pudo abrir {port}: {e}')
        return None


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control')

        # ── Estado rover ──────────────────────────────────────────────────
        self.ang1          = 90.0
        self.ang2          = 90.0
        self.pwm_rover     = 0

        # ── Estado NEMA / RMD ─────────────────────────────────────────────
        self.pos_nema_desp_mm = 0.0
        self.pos_nema_ang_rad = 0.0
        self.pos_rmd_rad      = 0.0

        self.sent_nema_desp   = None
        self.sent_nema_ang    = None
        self.sent_rmd_deg     = None

        # ── Joystick ──────────────────────────────────────────────────────
        self.joy_axes    = []
        self.joy_buttons = []

        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        # ── Seriales ──────────────────────────────────────────────────────
        self.ser_rover = open_serial(ROVER_PORT, ROVER_BAUD, self.get_logger(), 'ROVER')
        self.ser_nema  = open_serial(NEMA_PORT,  NEMA_BAUD,  self.get_logger(), 'NEMA')

        # ── CAN / RMD ─────────────────────────────────────────────────────
        self.actuator = None
        if RMD_AVAILABLE:
            try:
                driver = rmd.CanDriver(CAN_INTERFACE)
                self.actuator = rmd.ActuatorInterface(driver, MOTOR_ID)
                initial_deg = self.actuator.getMultiTurnAngle()
                self.pos_rmd_rad  = math.radians(initial_deg - ANGLE_OFFSET)
                self.sent_rmd_deg = initial_deg
                self.get_logger().info(f'[RMD] Conectado. Posición inicial: {initial_deg:.2f}°')
            except Exception as e:
                self.get_logger().error(f'[RMD] Error al conectar: {e}')
        else:
            self.get_logger().warn('[RMD] myactuator_rmd_py no disponible. CAN deshabilitado.')

        # ── Timer 10 Hz ───────────────────────────────────────────────────
        self.timer = self.create_timer(CONTROL_PERIOD, self.control_loop)
        self.get_logger().info('Nodo robot_control iniciado.')

    # ─────────────────────────────────────────────────────────────────────────

    def joy_callback(self, msg: Joy):
        self.joy_axes    = list(msg.axes)
        self.joy_buttons = list(msg.buttons)

    def control_loop(self):
        self._update_rover()
        self._update_nema_rmd()

    # ── Rover ─────────────────────────────────────────────────────────────────

    def _update_rover(self):
        axes    = self.joy_axes
        buttons = self.joy_buttons

        # Motor DC
        axis_val = axes[AXIS_MOTOR_DC] if len(axes) > AXIS_MOTOR_DC else 0.0
        if axis_val > 0.5:
            self.pwm_rover = MOTOR_SPEED_ROVER
        elif axis_val < -0.5:
            self.pwm_rover = -MOTOR_SPEED_ROVER
        else:
            self.pwm_rover = 0

        # Servos
        if len(buttons) > BTN_SERVO1_DOWN:
            if buttons[BTN_SERVO1_UP]:
                self.ang1 = min(180.0, self.ang1 + SERVO_STEP)
            if buttons[BTN_SERVO1_DOWN]:
                self.ang1 = max(0.0,   self.ang1 - SERVO_STEP)
            if buttons[BTN_SERVO2_UP]:
                self.ang2 = min(180.0, self.ang2 + SERVO_STEP)
            if buttons[BTN_SERVO2_DOWN]:
                self.ang2 = max(0.0,   self.ang2 - SERVO_STEP)

        if self.ser_rover and self.ser_rover.is_open:
            cmd = f'{self.ang1:.1f},{self.ang2:.1f},{self.pwm_rover}\n'
            try:
                self.ser_rover.write(cmd.encode())
                self.get_logger().debug(f'[ROVER] → {cmd.strip()}')
            except Exception as e:
                self.get_logger().error(f'[ROVER] Error serial: {e}')

    # ── NEMA + RMD ────────────────────────────────────────────────────────────

    def _update_nema_rmd(self):
        axes = self.joy_axes

        raw_desp = apply_dead_zone(axes[AXIS_NEMA_DESP] if len(axes) > AXIS_NEMA_DESP else 0.0, DEAD_ZONE)
        raw_rmd  = apply_dead_zone(axes[AXIS_RMD]       if len(axes) > AXIS_RMD       else 0.0, DEAD_ZONE)
        raw_ang  = apply_dead_zone(axes[AXIS_NEMA_ANG]  if len(axes) > AXIS_NEMA_ANG  else 0.0, DEAD_ZONE)

        self.pos_nema_desp_mm += raw_desp * SCALE_DESP_MM
        self.pos_nema_ang_rad += raw_ang  * SCALE_RAD
        self.pos_rmd_rad      += raw_rmd  * SCALE_RAD

        self.pos_nema_desp_mm = max(MIN_DESP_MM,   min(MAX_DESP_MM,   self.pos_nema_desp_mm))
        self.pos_nema_ang_rad = max(-MAX_RAD_NEMA, min(MAX_RAD_NEMA,  self.pos_nema_ang_rad))
        self.pos_rmd_rad      = max(-MAX_RAD_RMD,  min(MAX_RAD_RMD,   self.pos_rmd_rad))

        # Serial NEMA
        desp_changed = self.sent_nema_desp is None or abs(self.pos_nema_desp_mm - self.sent_nema_desp) >= UMBRAL_DESP_MM
        ang_changed  = self.sent_nema_ang  is None or abs(self.pos_nema_ang_rad  - self.sent_nema_ang)  >= UMBRAL_RAD

        if (desp_changed or ang_changed) and self.ser_nema and self.ser_nema.is_open:
            cmd = f'{-self.pos_nema_ang_rad:.4f},{self.pos_nema_desp_mm:.4f}\n'
            try:
                self.ser_nema.write(cmd.encode())
                self.sent_nema_desp = self.pos_nema_desp_mm
                self.sent_nema_ang  = self.pos_nema_ang_rad
                self.get_logger().info(
                    f'[NEMA] ángulo: {math.degrees(self.pos_nema_ang_rad):.2f}°  '
                    f'desp: {self.pos_nema_desp_mm:.2f} mm  →  "{cmd.strip()}"'
                )
                if self.ser_nema.in_waiting > 0:
                    resp = self.ser_nema.readline().decode(errors='replace').strip()
                    if resp:
                        self.get_logger().info(f'[NEMA] Arduino: {resp}')
            except Exception as e:
                self.get_logger().error(f'[NEMA] Error serial: {e}')

        # CAN RMD
        target_deg  = math.degrees(self.pos_rmd_rad) + ANGLE_OFFSET
        rmd_changed = self.sent_rmd_deg is None or abs(target_deg - self.sent_rmd_deg) >= math.degrees(UMBRAL_RAD)

        if rmd_changed and self.actuator is not None:
            try:
                self.actuator.sendPositionAbsoluteSetpoint(target_deg, MOTOR_SPEED)
                self.sent_rmd_deg = target_deg
                self.get_logger().info(
                    f'[RMD] {self.pos_rmd_rad:.4f} rad ({math.degrees(self.pos_rmd_rad):.2f}°) → {target_deg:.2f}°'
                )
            except Exception as e:
                self.get_logger().error(f'[RMD] Error CAN: {e}')

    # ─────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.ser_rover and self.ser_rover.is_open:
            try:
                self.ser_rover.write(f'{self.ang1:.1f},{self.ang2:.1f},0\n'.encode())
            except Exception:
                pass
            self.ser_rover.close()
        if self.ser_nema and self.ser_nema.is_open:
            self.ser_nema.close()
        self.get_logger().info('Nodo apagado.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()