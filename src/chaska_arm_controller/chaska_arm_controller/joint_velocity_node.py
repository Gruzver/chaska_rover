# chaska_arm_controller/joint_velocity_node.py

import os
import subprocess
import tempfile

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory


def _xacro_to_urdf(xacro_path: str) -> str:
    """Run xacro and return path to a temp URDF file."""
    result = subprocess.run(
        ['xacro', xacro_path, 'is_simulation:=false'],
        capture_output=True, text=True, check=True,
    )
    tmp = tempfile.NamedTemporaryFile(suffix='.urdf', delete=False, mode='w')
    tmp.write(result.stdout)
    tmp.close()
    return tmp.name

from .arm_kinematics import ArmKinematics


# ─────────────────────────────────────────────────────────────────────────────
#  CONSTANTES DEL ROBOT
# ─────────────────────────────────────────────────────────────────────────────

# Nombres de joints activos en el MISMO ORDEN que Pinocchio los carga.
# Pinocchio ignora joints fijos (virtual_joint, joint_6), por lo que
# el vector q tendrá 5 elementos:  [joint_1, joint_2, joint_3, joint_4, joint_5]
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

# Índices dentro del vector q / dq  (0-based)
IDX_JOINT_1 = 0   # prismatic X
IDX_JOINT_2 = 1   # revolute  X
IDX_JOINT_3 = 2   # revolute  X
IDX_JOINT_4 = 3   # revolute  X  ← controlado por joystick der.
IDX_JOINT_5 = 4   # revolute  Y  ← controlado por joystick der.

# Posición inicial (home = centro de rango)
HOME_POSITION = np.zeros(5)

# Límites de posición articular
Q_MIN = np.array([-0.125, -1.5708, -1.5708, -1.5708, -1.5708])
Q_MAX = np.array([ 0.125,  1.5708,  1.5708,  1.5708,  1.5708])


# ─────────────────────────────────────────────────────────────────────────────
#  NODO PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────

class JointVelocityNode(Node):
    def __init__(self):
        super().__init__('joint_velocity_node')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('damping',        1e-3)
        self.declare_parameter('control_rate',   50.0)   # Hz
        self.declare_parameter('velocity_scale',  1.0)

        damping            = self.get_parameter('damping').value
        control_rate       = self.get_parameter('control_rate').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.dt            = 1.0 / control_rate
        self.damping       = damping

        # ── Cargar modelo con Pinocchio ───────────────────────────────────────
        pkg_share = get_package_share_directory('chaska_arm_description')
        xacro_path = os.path.join(pkg_share, 'urdf', 'chaska_arm.urdf.xacro')
        self.get_logger().info(f'Generando URDF desde: {xacro_path}')
        urdf_path = _xacro_to_urdf(xacro_path)

        self.kin = ArmKinematics(urdf_path)

        # Verificar que nv coincide con JOINT_NAMES
        if self.kin.nv != len(JOINT_NAMES):
            self.get_logger().warn(
                f'ADVERTENCIA: Pinocchio reporta nv={self.kin.nv}, '
                f'pero JOINT_NAMES tiene {len(JOINT_NAMES)} elementos. '
                f'Revisa el URDF o JOINT_NAMES.'
            )

        # ── Estado articular ──────────────────────────────────────────────────
        self.q  = HOME_POSITION.copy()          # posición actual (nq,)
        self.dq = np.zeros(self.kin.nv)         # velocidad actual (nv,)

        # ── Comandos recibidos ────────────────────────────────────────────────
        self.v_ee_target   = np.zeros(3)         # [vx, vy, vz] deseado
        self.dq_wrist      = np.zeros(2)         # [dq_joint4, dq_joint5] directo
        self.ee_cmd_active = False               # hay comando EE activo?
        self.estop         = False               # E-STOP activo?

        # ── QoS ───────────────────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ── Suscriptores ──────────────────────────────────────────────────────
        # Velocidad del efector final [vx, vy, vz] desde joystick izq.
        self.create_subscription(
            Vector3,
            '/ee_velocity_target',
            self.ee_velocity_callback,
            qos
        )
        # Velocidades directas de joint_4 y joint_5 desde joystick der.
        self.create_subscription(
            JointState,
            '/wrist_velocity_command',
            self.wrist_velocity_callback,
            qos
        )
        # Estado articular real (si hay robot físico o simulador externo)
        self.create_subscription(
            JointState,
            '/joint_states_feedback',            # topic separado para no hacer loop
            self.joint_state_feedback_callback,
            qos
        )
        # E-STOP desde joystick
        self.create_subscription(
            Bool,
            '/arm_estop',
            self.estop_callback,
            qos
        )

        # ── Publicadores ──────────────────────────────────────────────────────
        # Topic principal que robot_state_publisher escucha → RViz
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            qos
        )
        # Comando de velocidad separado (para un controlador real)
        self.joint_vel_cmd_pub = self.create_publisher(
            JointState,
            '/joint_velocity_command',
            qos
        )
        # Velocidad real del EE (verificación)
        self.ee_vel_actual_pub = self.create_publisher(
            Vector3,
            '/ee_velocity_actual',
            qos
        )
        # Posición cartesiana del EE
        self.ee_pos_pub = self.create_publisher(
            Vector3,
            '/ee_position',
            qos
        )

        # ── Timer de control ──────────────────────────────────────────────────
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'JointVelocityNode listo | '
            f'rate={control_rate:.0f} Hz | dt={self.dt:.4f} s | '
            f'nq={self.kin.nq} | nv={self.kin.nv}'
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def ee_velocity_callback(self, msg: Vector3):
        """
        Recibe la velocidad deseada del efector final [vx, vy, vz] en m/s.
        Publicado por el nodo joystick desde el joystick izquierdo + botones.
        """
        self.v_ee_target   = np.array([msg.x, msg.y, msg.z])
        self.ee_cmd_active = np.linalg.norm(self.v_ee_target) > 1e-6

    def wrist_velocity_callback(self, msg: JointState):
        """
        Recibe velocidades directas para joint_4 y joint_5 en rad/s.
        Publicado por el nodo joystick desde el joystick derecho.
        Usa mapeo por nombre para robustez ante cambios de orden.
        """
        name_to_vel = dict(zip(msg.name, msg.velocity))
        self.dq_wrist[0] = name_to_vel.get('joint_4', 0.0)   # índice IDX_JOINT_4
        self.dq_wrist[1] = name_to_vel.get('joint_5', 0.0)   # índice IDX_JOINT_5

    def joint_state_feedback_callback(self, msg: JointState):
        """
        Sincroniza la posición q con el robot real (opcional).
        Usa /joint_states_feedback para evitar loop con /joint_states.
        Mapea por nombre para tolerar cualquier orden en el mensaje.
        """
        name_to_pos = dict(zip(msg.name, msg.position))
        for i, joint_name in enumerate(JOINT_NAMES):
            if joint_name in name_to_pos:
                self.q[i] = float(name_to_pos[joint_name])

    def estop_callback(self, msg: Bool):
        """
        Recibe el estado del E-STOP desde el nodo joystick.
        True  → detener todo movimiento inmediatamente.
        False → reanudar operación normal.
        """
        if msg.data and not self.estop:
            self.get_logger().warn('⛔ E-STOP ACTIVADO → deteniendo brazo')
        elif not msg.data and self.estop:
            self.get_logger().info('✅ E-STOP desactivado → reanudando')
        self.estop = msg.data

    # ─────────────────────────────────────────────────────────────────────────
    #  LOOP DE CONTROL
    # ─────────────────────────────────────────────────────────────────────────

    def control_loop(self):
        """
        Ejecutado a frecuencia fija (control_rate Hz).

        Lógica:
          1. Si E-STOP → publicar ceros y salir.
          2. Calcular dq para joints 1-3 a partir de v_ee con Jacobiano.
          3. Sobreescribir dq[IDX_JOINT_4] y dq[IDX_JOINT_5] con comando directo.
          4. Integrar posición: q += dq * dt
          5. Saturar q dentro de límites articulares.
          6. Publicar /joint_states y telemetría.
        """

        # ── 1. E-STOP ─────────────────────────────────────────────────────────
        if self.estop:
            self.dq = np.zeros(self.kin.nv)
            self._publish_joint_state()
            return

        # ── 2. Cinemática inversa de velocidad (joints 1-3) ───────────────────
        dq_full = np.zeros(self.kin.nv)

        if self.ee_cmd_active:
            try:
                dq_full = self.kin.joint_velocities_from_ee_velocity(
                    q       = self.q,
                    v_ee    = self.v_ee_target,
                    damping = self.damping,
                    scale   = self.velocity_scale
                )
            except Exception as exc:
                self.get_logger().error(f'Error en cinemática: {exc}')
                dq_full = np.zeros(self.kin.nv)

        # ── 3. Control directo de muñeca (joints 4 y 5) ───────────────────────
        #       Sobreescribe lo que el Jacobiano hubiera asignado a estos joints.
        dq_full[IDX_JOINT_4] = self.dq_wrist[0]
        dq_full[IDX_JOINT_5] = self.dq_wrist[1]

        self.dq = dq_full

        # ── 4. Integrar posición ──────────────────────────────────────────────
        q_new = self.q + self.dq * self.dt

        # ── 5. Saturar dentro de límites ──────────────────────────────────────
        self.q = np.clip(q_new, Q_MIN, Q_MAX)

        # Si un joint llegó al límite, anular su velocidad para no acumular error
        at_min = self.q == Q_MIN
        at_max = self.q == Q_MAX
        self.dq[at_min & (self.dq < 0)] = 0.0
        self.dq[at_max & (self.dq > 0)] = 0.0

        # ── 6. Publicar ───────────────────────────────────────────────────────
        self._publish_joint_state()
        self._publish_ee_telemetry()

    # ─────────────────────────────────────────────────────────────────────────
    #  PUBLICADORES INTERNOS
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_joint_state(self):
        """
        Publica posición Y velocidad articular.
        robot_state_publisher necesita posiciones para actualizar /tf → RViz.
        """
        now = self.get_clock().now().to_msg()

        # /joint_states → robot_state_publisher → RViz
        js = JointState()
        js.header.stamp = now
        js.name         = JOINT_NAMES
        js.position     = self.q.tolist()
        js.velocity     = self.dq.tolist()
        js.effort       = [0.0] * len(JOINT_NAMES)
        self.joint_state_pub.publish(js)

        # /joint_velocity_command → controlador real (si existe)
        cmd = JointState()
        cmd.header.stamp = now
        cmd.name         = JOINT_NAMES
        cmd.velocity     = self.dq.tolist()
        self.joint_vel_cmd_pub.publish(cmd)

    def _publish_ee_telemetry(self):
        """
        Publica la velocidad y posición real del EE para monitoreo.
        """
        try:
            # Velocidad real del EE: v = J · dq
            J        = self.kin.compute_jacobian(self.q)
            v_actual = J @ self.dq                         # (3,)

            self.ee_vel_actual_pub.publish(
                Vector3(x=float(v_actual[0]),
                        y=float(v_actual[1]),
                        z=float(v_actual[2]))
            )

            # Posición cartesiana del EE
            pos = self.kin.ee_position(self.q)             # (3,)
            self.ee_pos_pub.publish(
                Vector3(x=float(pos[0]),
                        y=float(pos[1]),
                        z=float(pos[2]))
            )

            # Log de debug (solo cuando hay movimiento)
            moving = np.linalg.norm(self.dq) > 1e-4
            if moving:
                self.get_logger().debug(
                    f'q     = {np.round(self.q,      4)}\n'
                    f'dq    = {np.round(self.dq,     4)}\n'
                    f'EE_p  = {np.round(pos,         4)}\n'
                    f'EE_v  = {np.round(v_actual,    4)}'
                )

        except Exception as exc:
            self.get_logger().error(f'Error en telemetría EE: {exc}')


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = JointVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()