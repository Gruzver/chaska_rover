# chaska_arm_controller/joint_velocity_node.py
#
# Modo simulación: lee /joint_states (broadcaster), hace IK con Pinocchio,
# y envía JointTrajectory a /arm_controller/joint_trajectory.

import os
import subprocess
import tempfile

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory

from .arm_kinematics import ArmKinematics


def _xacro_to_urdf(xacro_path: str) -> str:
    result = subprocess.run(
        ['xacro', xacro_path, 'is_simulation:=false'],
        capture_output=True, text=True, check=True,
    )
    tmp = tempfile.NamedTemporaryFile(suffix='.urdf', delete=False, mode='w')
    tmp.write(result.stdout)
    tmp.close()
    return tmp.name


# ── Constantes ────────────────────────────────────────────────────────────────
# joint_6 (gripper) es prismatic → Pinocchio lo incluye en su modelo (nv=6).
# Usamos todos los 6 internamente para Pinocchio; solo los 5 primeros van al
# arm_controller (joint_6 lo gestiona gripper_controller por separado).
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
ARM_CTRL_JOINTS = JOINT_NAMES[:5]   # los que arm_controller acepta

IDX_JOINT_4 = 3
IDX_JOINT_5 = 4
IDX_JOINT_6 = 5

# Joints 1-3: controlados por IK de posición EE
# Joints 4-5: velocidad directa (muñeca)
# Joint  6  : velocidad directa (gripper)
Q_MIN = np.array([-0.125, -1.5708, -1.5708, -1.5708, -1.5708, -0.065])
Q_MAX = np.array([ 0.125,  1.5708,  1.5708,  1.5708,  1.5708,  0.020])


class JointVelocityNode(Node):

    def __init__(self):
        super().__init__('joint_velocity_node')

        self.declare_parameter('damping',       1e-3)
        self.declare_parameter('control_rate',  50.0)
        self.declare_parameter('velocity_scale', 1.0)

        damping             = self.get_parameter('damping').value
        control_rate        = self.get_parameter('control_rate').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.dt             = 1.0 / control_rate
        self.damping        = damping

        # ── Cargar modelo Pinocchio ───────────────────────────────────────────
        pkg_share  = get_package_share_directory('chaska_arm_description')
        xacro_path = os.path.join(pkg_share, 'urdf', 'chaska_arm.urdf.xacro')
        self.get_logger().info(f'Generando URDF para Pinocchio: {xacro_path}')
        urdf_path  = _xacro_to_urdf(xacro_path)
        self.kin   = ArmKinematics(urdf_path)
        self.get_logger().info(
            f'Pinocchio cargado: nq={self.kin.nq} nv={self.kin.nv} '
            f'joints={self.kin.model.names[1:]}'
        )

        # ── Estado articular ──────────────────────────────────────────────────
        # self.q   : posición REAL, sincronizada desde /joint_states (para IK)
        # self.q_cmd : posición COMANDADA, acumulada por integración (para JTC)
        # Se usan por separado para evitar que el JTC considere cada micro-delta
        # como "goal reached" y no avance.
        self.q            = np.zeros(self.kin.nv)
        self.dq           = np.zeros(self.kin.nv)
        self.q_cmd        = None   # se inicializa en el primer /joint_states
        self.q_ready      = False   # esperar primer /joint_states antes de actuar

        # ── Comandos recibidos ────────────────────────────────────────────────
        self.v_ee_target   = np.zeros(3)    # IK joints 1-3
        self.dq_wrist      = np.zeros(3)    # directo: joint_4, joint_5, joint_6
        self.ee_cmd_active = False
        self.estop         = False

        # ── QoS ───────────────────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Suscriptores ──────────────────────────────────────────────────────
        self.create_subscription(Vector3,    '/ee_velocity_target',   self._ee_vel_cb,    qos)
        self.create_subscription(JointState, '/wrist_velocity_command', self._wrist_cb,   qos)
        self.create_subscription(Bool,       '/arm_estop',             self._estop_cb,    qos)
        # /joint_states del broadcaster (QoS Best Effort en sim)
        self.create_subscription(JointState, '/joint_states',          self._joint_states_cb, qos_be)

        # ── Publicadores ──────────────────────────────────────────────────────
        self._traj_pub     = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory',     qos)
        self._gripper_pub  = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', qos)
        self._ee_vel_pub   = self.create_publisher(Vector3, '/ee_velocity_actual', qos)
        self._ee_pos_pub   = self.create_publisher(Vector3, '/ee_position',        qos)

        self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'JointVelocityNode (sim) listo | '
            f'{control_rate:.0f} Hz | nv={self.kin.nv}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _joint_states_cb(self, msg: JointState):
        """Sincroniza q desde joint_state_broadcaster (todos los joints del robot)."""
        name_to_pos = dict(zip(msg.name, msg.position))
        found = 0
        for i, name in enumerate(JOINT_NAMES):
            if name in name_to_pos:
                self.q[i] = float(name_to_pos[name])
                found += 1
        if found == len(JOINT_NAMES) and not self.q_ready:
            self.q_ready = True
            self.q_cmd   = self.q.copy()   # inicializar acumulador desde posición real
            self.get_logger().info(f'joint_states sincronizado: q={np.round(self.q, 4)}')

    def _ee_vel_cb(self, msg: Vector3):
        self.v_ee_target   = np.array([msg.x, msg.y, msg.z])
        self.ee_cmd_active = np.linalg.norm(self.v_ee_target) > 1e-6

    def _wrist_cb(self, msg: JointState):
        name_to_vel = dict(zip(msg.name, msg.velocity))
        self.dq_wrist[0] = name_to_vel.get('joint_4', 0.0)
        self.dq_wrist[1] = name_to_vel.get('joint_5', 0.0)
        self.dq_wrist[2] = name_to_vel.get('joint_6', 0.0)

    def _estop_cb(self, msg: Bool):
        if msg.data and not self.estop:
            self.get_logger().warn('E-STOP activado')
        elif not msg.data and self.estop:
            self.get_logger().info('E-STOP desactivado')
        self.estop = msg.data

    # ── Loop de control ───────────────────────────────────────────────────────

    def _control_loop(self):
        if not self.q_ready:
            return

        if self.estop:
            self._send_trajectory(self.q_cmd)   # mantener última posición comandada
            return

        dq = np.zeros(self.kin.nv)

        # ── IK posición EE → solo joints 1-3 (Jacobiano reducido) ───────────
        # Joints 4-5 se controlan directo; excluirlos del IK evita conflictos.
        if self.ee_cmd_active:
            try:
                J_full = self.kin.compute_jacobian(self.q)   # (3, 6)
                J_red  = J_full[:, :3]                        # solo joints 1-3
                lam    = self.damping ** 2
                J_dls  = J_red.T @ np.linalg.inv(J_red @ J_red.T + lam * np.eye(3))
                dq[:3] = J_dls @ self.v_ee_target * self.velocity_scale
            except Exception as e:
                self.get_logger().error(f'IK error: {e}')

        # ── Control directo: muñeca (joint_4, joint_5) y gripper (joint_6) ──
        dq[IDX_JOINT_4] = self.dq_wrist[0]
        dq[IDX_JOINT_5] = self.dq_wrist[1]
        dq[IDX_JOINT_6] = self.dq_wrist[2]

        self.dq = dq

        # Acumular q_cmd solo cuando hay algún comando activo
        has_command = self.ee_cmd_active or np.any(np.abs(self.dq_wrist) > 1e-6)
        if has_command:
            self.q_cmd = np.clip(self.q_cmd + dq * self.dt, Q_MIN, Q_MAX)

        # Enviar joints 1-5 al arm_controller y joint_6 al gripper_controller
        self._send_trajectory(self.q_cmd)
        self._publish_telemetry()

    def _send_trajectory(self, q_target: np.ndarray):
        """Envía trayectorias a arm_controller (joints 1-5) y gripper_controller (joint_6)."""
        lookahead = Duration(sec=0, nanosec=100_000_000)  # 0.1 s

        # arm_controller: joint_1..joint_5
        arm_traj = JointTrajectory()
        arm_traj.joint_names = ARM_CTRL_JOINTS
        arm_pt = JointTrajectoryPoint()
        arm_pt.positions       = q_target[:len(ARM_CTRL_JOINTS)].tolist()
        arm_pt.time_from_start = lookahead
        arm_traj.points = [arm_pt]
        self._traj_pub.publish(arm_traj)

        # gripper_controller: joint_6
        grip_traj = JointTrajectory()
        grip_traj.joint_names = ['joint_6']
        grip_pt = JointTrajectoryPoint()
        grip_pt.positions       = [float(q_target[IDX_JOINT_6])]
        grip_pt.time_from_start = lookahead
        grip_traj.points = [grip_pt]
        self._gripper_pub.publish(grip_traj)

    def _publish_telemetry(self):
        try:
            J   = self.kin.compute_jacobian(self.q)
            v_a = J @ self.dq
            self._ee_vel_pub.publish(
                Vector3(x=float(v_a[0]), y=float(v_a[1]), z=float(v_a[2])))
            pos = self.kin.ee_position(self.q)
            self._ee_pos_pub.publish(
                Vector3(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])))
        except Exception as e:
            self.get_logger().error(f'Telemetry error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JointVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
