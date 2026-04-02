#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

class JoyToESP32(Node):
    def __init__(self):
        super().__init__('joy_to_esp32')
        
        # Parámetros configurables
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('servo_step', 2.0)      # grados por pulsación
        self.declare_parameter('motor_speed', 70)       # PWM del motor (0-100)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.servo_step = self.get_parameter('servo_step').value
        self.motor_speed = self.get_parameter('motor_speed').value

        # Estado interno
        self.ang1 = 90.0   # ángulo inicial servo 1
        self.ang2 = 90.0   # ángulo inicial servo 2
        self.pwm  = 0      # velocidad motor

        # Conexión serial
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # esperar reset de ESP32
            self.get_logger().info(f'Serial conectado en {port} a {baud} bps')
        except serial.SerialException as e:
            self.get_logger().error(f'Error al abrir serial: {e}')
            raise SystemExit(1)

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Timer para enviar comandos a 10 Hz
        self.timer = self.create_timer(0.1, self.send_command)
        self.get_logger().info('Nodo joy_to_esp32 iniciado')

    def joy_callback(self, msg: Joy):
        axes    = msg.axes
        buttons = msg.buttons

        # ── Motor: axes[4] → 1=avanza, -1=retrocede ──
        axis_motor = axes[4] if len(axes) > 4 else 0.0
        if axis_motor > 0.5:
            self.pwm = self.motor_speed
        elif axis_motor < -0.5:
            self.pwm = -self.motor_speed
        else:
            self.pwm = 0

        # ── Servos: botones 4-7 ──
        # buttons[4] → aumenta servo 2
        # buttons[5] → aumenta servo 1
        # buttons[6] → disminuye servo 2
        # buttons[7] → disminuye servo 1
        if len(buttons) > 7:
            if buttons[5] == 1:
                self.ang1 = min(180.0, self.ang1 + self.servo_step)
            if buttons[7] == 1:
                self.ang1 = max(0.0,   self.ang1 - self.servo_step)
            if buttons[4] == 1:
                self.ang2 = min(180.0, self.ang2 + self.servo_step)
            if buttons[6] == 1:
                self.ang2 = max(0.0,   self.ang2 - self.servo_step)

    def send_command(self):
        cmd = f'{self.ang1:.1f},{self.ang2:.1f},{self.pwm}\n'
        try:
            self.ser.write(cmd.encode())
            self.get_logger().debug(f'Enviado: {cmd.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error serial: {e}')

    def destroy_node(self):
        self.pwm = 0
        self.send_command()   # parar motor al salir
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyToESP32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()