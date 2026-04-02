#!/usr/bin/env python3
"""
joy_motor_control.py
====================
Nodo ROS2 Humble que lee /joy y controla:
  - Motor NEMA (desplazamiento lineal, mm) + Motor NEMA (ángulo, rad)  → Arduino vía USB serial
  - Motor RMD  (ángulo, rad → grados + offset)                         → CAN bus

Mapeo de ejes del joystick:
  axes[0]  →  NEMA desplazamiento  (prismatic, mm)
  axes[1]  →  Motor RMD            (revolute,  rad)
  axes[2]  →  NEMA ángulo          (revolute,  rad)

Cada 0.1 s se acumula la posición y se envía a los actuadores.

Ejecución:
  ros2 run joy joy_node &
  python3 joy_motor_control.py
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

# ─── CONFIGURACIÓN ──────────────────────────────────────────────────────────

# Serial (NEMA steppers vía Arduino)
SERIAL_PORT   = '/dev/ttyUSB0'
SERIAL_BAUD   = 115200

# CAN (RMD motor)
CAN_INTERFACE = 'slcan0'
MOTOR_ID      = 3
MOTOR_SPEED   = 100.0   # deg/s
ANGLE_OFFSET  = 612.0   # grados de desfase absoluto para el RMD

# Ejes del joystick (índices en axes[])
AXIS_NEMA_DESP  = 0   # desplazamiento lineal
AXIS_RMD        = 1   # ángulo motor RMD
AXIS_NEMA_ANG   = 2   # ángulo motor NEMA

# Escala por ciclo (cada 0.1 s)
#   Joystick full  → incremento máximo por ciclo
SCALE_DESP_MM   = 1.0    # mm  por ciclo  (±20 mm/s a full stick)
SCALE_RAD       = 0.01   # rad por ciclo  (±0.2 rad/s a full stick)
DEAD_ZONE       = 0.05   # zona muerta del joystick

# Límites de posición (seguridad)
MAX_DESP_MM     = 220.0  # mm
MIN_DESP_MM     = -220.0    # mm
MAX_RAD_NEMA    = math.pi        # ±180°
MAX_RAD_RMD     = 4 * math.pi   # ±720°

# Umbral mínimo de cambio para enviar al hardware
UMBRAL_DESP_MM  = 0.1    # mm
UMBRAL_RAD      = 0.001  # rad

# Período del timer de control
CONTROL_PERIOD  = 0.1    # segundos

# ────────────────────────────────────────────────────────────────────────────


def apply_dead_zone(value: float, dead: float) -> float:
    if abs(value) < dead:
        return 0.0
    # Rescalar para que sea suave al cruzar la zona muerta
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dead) / (1.0 - dead)


class JoyMotorControlNode(Node):
    def __init__(self):
        super().__init__('joy_motor_control')

        # ── Posiciones acumuladas ──────────────────────────────────────────
        self.pos_nema_desp_mm   = 0.0   # mm
        self.pos_nema_ang_rad   = 0.0   # rad
        self.pos_rmd_rad        = 0.0   # rad (relativo a cero)

        # Última posición enviada al hardware
        self.sent_nema_desp     = None
        self.sent_nema_ang      = None
        self.sent_rmd_deg       = None

        # ── Joystick ─────────────────────────────────────────────────────
        self.joy_axes = [0.0] * 6

        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        # ── Serial (Arduino / NEMA) ───────────────────────────────────────
        self.ser = None
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            time.sleep(2)
            self.get_logger().info(f'Serial conectado: {SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir serial: {e}')

        # ── CAN (RMD motor) ───────────────────────────────────────────────
        self.actuator = None
        if RMD_AVAILABLE:
            try:
                driver = rmd.CanDriver(CAN_INTERFACE)
                self.actuator = rmd.ActuatorInterface(driver, MOTOR_ID)
                # Leer posición inicial real del motor
                initial_deg = self.actuator.getMultiTurnAngle()
                # Ajustar pos_rmd para que el "cero" del joystick corresponda a la posición actual
                self.pos_rmd_rad = math.radians(initial_deg - ANGLE_OFFSET)
                self.sent_rmd_deg = initial_deg
                self.get_logger().info(f'RMD conectado. Posición inicial: {initial_deg:.2f}°')
            except Exception as e:
                self.get_logger().error(f'No se pudo conectar al RMD: {e}')
        else:
            self.get_logger().warn('myactuator_rmd_py no disponible. RMD deshabilitado.')

        # ── Timer de control (10 Hz) ──────────────────────────────────────
        self.timer = self.create_timer(CONTROL_PERIOD, self.control_loop)
        self.get_logger().info('Nodo joy_motor_control iniciado. Joystick listo.')

    # ─────────────────────────────────────────────────────────────────────────

    def joy_callback(self, msg: Joy):
        self.joy_axes = list(msg.axes) + [0.0] * max(0, 6 - len(msg.axes))

    def control_loop(self):
        """Se ejecuta cada 0.1 s: acumula posición y envía al hardware."""

        # Leer ejes con zona muerta
        raw_desp = apply_dead_zone(
            self.joy_axes[AXIS_NEMA_DESP] if len(self.joy_axes) > AXIS_NEMA_DESP else 0.0,
            DEAD_ZONE
        )
        raw_rmd = apply_dead_zone(
            self.joy_axes[AXIS_RMD] if len(self.joy_axes) > AXIS_RMD else 0.0,
            DEAD_ZONE
        )
        raw_ang = apply_dead_zone(
            self.joy_axes[AXIS_NEMA_ANG] if len(self.joy_axes) > AXIS_NEMA_ANG else 0.0,
            DEAD_ZONE
        )

        # Acumular posiciones
        self.pos_nema_desp_mm += raw_desp * SCALE_DESP_MM
        self.pos_nema_ang_rad += raw_ang  * SCALE_RAD
        self.pos_rmd_rad      += raw_rmd  * SCALE_RAD

        # Aplicar límites
        self.pos_nema_desp_mm = max(MIN_DESP_MM, min(MAX_DESP_MM, self.pos_nema_desp_mm))
        self.pos_nema_ang_rad = max(-MAX_RAD_NEMA, min(MAX_RAD_NEMA, self.pos_nema_ang_rad))
        self.pos_rmd_rad      = max(-MAX_RAD_RMD,  min(MAX_RAD_RMD,  self.pos_rmd_rad))

        # ── Enviar al Arduino (NEMA) ──────────────────────────────────────
        desp_changed = (self.sent_nema_desp is None or
                        abs(self.pos_nema_desp_mm - self.sent_nema_desp) >= UMBRAL_DESP_MM)
        ang_changed  = (self.sent_nema_ang is None or
                        abs(self.pos_nema_ang_rad - self.sent_nema_ang) >= UMBRAL_RAD)

        if (desp_changed or ang_changed) and self.ser and self.ser.is_open:
            # Formato idéntico al código original: "angulo,desplazamiento\n"
            # El código original niega el ángulo antes de enviar
            comando = f"{-self.pos_nema_ang_rad:.4f},{self.pos_nema_desp_mm:.4f}\n"
            try:
                self.ser.write(comando.encode())
                self.sent_nema_desp = self.pos_nema_desp_mm
                self.sent_nema_ang  = self.pos_nema_ang_rad
                self.get_logger().info(
                    f'[NEMA] ángulo: {math.degrees(self.pos_nema_ang_rad):.2f}°  '
                    f'desp: {self.pos_nema_desp_mm:.2f} mm  →  "{comando.strip()}"'
                )
                # Respuesta no bloqueante
                if self.ser.in_waiting > 0:
                    resp = self.ser.readline().decode(errors='replace').strip()
                    if resp:
                        self.get_logger().info(f'Arduino: {resp}')
            except Exception as e:
                self.get_logger().error(f'Error serial: {e}')

        # ── Enviar al RMD (CAN) ───────────────────────────────────────────
        target_deg = math.degrees(self.pos_rmd_rad) + ANGLE_OFFSET

        rmd_changed = (self.sent_rmd_deg is None or
                       abs(target_deg - self.sent_rmd_deg) >= math.degrees(UMBRAL_RAD))

        if rmd_changed and self.actuator is not None:
            try:
                self.actuator.sendPositionAbsoluteSetpoint(target_deg, MOTOR_SPEED)
                self.sent_rmd_deg = target_deg
                self.get_logger().info(
                    f'[RMD]  ángulo: {self.pos_rmd_rad:.4f} rad ({math.degrees(self.pos_rmd_rad):.2f}°)  '
                    f'→  target motor: {target_deg:.2f}°'
                )
            except Exception as e:
                self.get_logger().error(f'Error CAN/RMD: {e}')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Puerto serial cerrado.')
        self.get_logger().info('Nodo apagado. Motores mantienen última posición.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyMotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()