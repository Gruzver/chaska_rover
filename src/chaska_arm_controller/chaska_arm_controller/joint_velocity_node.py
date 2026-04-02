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
# Pinocchio omite joints fijos (joint_7/end_effector, depth_camera_joint, joint_6 fijo)
# → el vector q tiene 5 elementos en el mismo orden que aparecen en el URDF.
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

IDX_JOINT_4 = 3
IDX_JOINT_5 = 4

Q_MIN = np.array([-0.125, -1.5708, -1.5708, -1.5708, -1.5708])
Q_MAX = np.array([ 0.125,  1.5708,  1.5708,  1.5708,  1.5708])


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

        # ── Estado articular (sincronizado desde /joint_states) ───────────────
        self.q            = np.zeros(len(JOINT_NAMES))
        self.dq           = np.zeros(len(JOINT_NAMES))
        self.q_ready      = False   # esperar primer /joint_states antes de actuar

        # ── Comandos recibidos ────────────────────────────────────────────────
        self.v_ee_target   = np.zeros(3)
        self.dq_wrist      = np.zeros(2)
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
        self._traj_pub  = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', qos)
        self._ee_vel_pub = self.create_publisher(Vector3, '/ee_velocity_actual', qos)
        self._ee_pos_pub = self.create_publisher(Vector3, '/ee_position',        qos)

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
        if found == len(JOINT_NAMES):
            self.q_ready = True

    def _ee_vel_cb(self, msg: Vector3):
        self.v_ee_target   = np.array([msg.x, msg.y, msg.z])
        self.ee_cmd_active = np.linalg.norm(self.v_ee_target) > 1e-6

    def _wrist_cb(self, msg: JointState):
        name_to_vel = dict(zip(msg.name, msg.velocity))
        self.dq_wrist[0] = name_to_vel.get('joint_4', 0.0)
        self.dq_wrist[1] = name_to_vel.get('joint_5', 0.0)

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
            self._send_trajectory(self.q)   # mantener posición actual
            return

        # IK de velocidad (joints 1-3 desde EE target)
        dq = np.zeros(self.kin.nv)
        if self.ee_cmd_active:
            try:
                dq = self.kin.joint_velocities_from_ee_velocity(
                    q=self.q, v_ee=self.v_ee_target,
                    damping=self.damping, scale=self.velocity_scale,
                )
            except Exception as e:
                self.get_logger().error(f'IK error: {e}')

        # Control directo de muñeca (joint_4, joint_5)
        dq[IDX_JOINT_4] = self.dq_wrist[0]
        dq[IDX_JOINT_5] = self.dq_wrist[1]

        self.dq = dq

        # Posición objetivo = posición actual + velocidad * dt, saturada
        q_target = np.clip(self.q + dq * self.dt, Q_MIN, Q_MAX)

        self._send_trajectory(q_target)
        self._publish_telemetry()

    def _send_trajectory(self, q_target: np.ndarray):
        """Envía un JointTrajectoryPoint al arm_controller."""
        traj = JointTrajectory()
        traj.header.stamp  = self.get_clock().now().to_msg()
        traj.joint_names   = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions       = q_target.tolist()
        pt.velocities      = self.dq.tolist()
        # Tiempo de llegada: 2 períodos de control (~40 ms a 50 Hz)
        lookahead_ns = int(2 * self.dt * 1e9)
        pt.time_from_start = Duration(sec=0, nanosec=lookahead_ns)

        traj.points = [pt]
        self._traj_pub.publish(traj)

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
