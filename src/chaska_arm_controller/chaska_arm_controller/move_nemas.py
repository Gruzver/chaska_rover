#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import math

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Parámetros configurables
        self.declare_parameter('puerto', '/dev/ttyUSB0')
        self.declare_parameter('baudios', 115200)
        self.declare_parameter('joint_desplazamiento', 'joint_1')  # prismatic -> mm
        self.declare_parameter('joint_angulo', 'joint_3')           # revolute -> radianes

        puerto  = self.get_parameter('puerto').get_parameter_value().string_value
        baudios = self.get_parameter('baudios').get_parameter_value().integer_value

        self.joint_desp  = self.get_parameter('joint_desplazamiento').get_parameter_value().string_value
        self.joint_ang   = self.get_parameter('joint_angulo').get_parameter_value().string_value

        # Conexión serial
        try:
            self.ser = serial.Serial(puerto, baudios, timeout=1)
            time.sleep(2)
            self.get_logger().info(f'Conectado al Arduino en {puerto}')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir el puerto serial: {e}')
            self.ser = None

        # Suscripción a joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # Últimos valores enviados (para evitar spam innecesario)
        self.ultimo_angulo        = None
        self.ultimo_desplazamiento = None
        self.umbral_angulo        = 0.001   # rad
        self.umbral_desplazamiento = 0.1    # mm

        self.get_logger().info('Nodo Arduino Bridge iniciado.')

    def joint_states_callback(self, msg: JointState):
        if self.ser is None or not self.ser.is_open:
            return

        nombres = msg.name
        posiciones = msg.position

        angulo        = None
        desplazamiento = None

        for i, nombre in enumerate(nombres):
            if nombre == self.joint_ang:
                # joint revolute: ya viene en radianes
                angulo = posiciones[i]
            elif nombre == self.joint_desp:
                # joint prismático: MoveIt lo publica en metros → convertir a mm
                desplazamiento = posiciones[i] * 1000.0

        if angulo is None or desplazamiento is None:
            return  # Todavía no llegaron los joints que necesitamos

        # Filtro de cambio mínimo para no saturar el serial
        if (self.ultimo_angulo is not None and
                abs(angulo - self.ultimo_angulo) < self.umbral_angulo and
                abs(desplazamiento - self.ultimo_desplazamiento) < self.umbral_desplazamiento):
            return

        self.ultimo_angulo        = angulo
        self.ultimo_desplazamiento = desplazamiento

        # Formato: "angulo,desplazamiento\n"  (igual que tu código original)
        comando = f"{-angulo:.4f},{desplazamiento:.4f}\n"

        try:
            self.ser.write(comando.encode())
            self.get_logger().info(f'Enviado → ángulo: {math.degrees(angulo):.2f}°  desplazamiento: {desplazamiento:.2f} mm')

            # Leer respuesta no bloqueante
            if self.ser.in_waiting > 0:
                respuesta = self.ser.readline().decode(errors='replace').strip()
                if respuesta:
                    self.get_logger().info(f'Arduino: {respuesta}')

        except Exception as e:
            self.get_logger().error(f'Error serial: {e}')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Puerto serial cerrado.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()