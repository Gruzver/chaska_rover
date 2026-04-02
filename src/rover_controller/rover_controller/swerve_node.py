"""
Swerve drive kinematics node for Chaska rover.

Subscribes:
  /cmd_vel          (geometry_msgs/Twist)   — linear.x, linear.y, angular.z
  /drive_mode       (std_msgs/String)       — "swerve" | "differential" | "ackermann"
  /joint_states     (sensor_msgs/JointState)— current yaw joint positions

Publishes:
  /yaw_position_controller/commands   (std_msgs/Float64MultiArray)
  /wheel_velocity_controller/commands (std_msgs/Float64MultiArray)

Joint order (same in YAML and here):
  yaw:   [front_left, front_right, rear_left, rear_right]
  wheel: [front_left, front_right, rear_left, rear_right]

Steering optimisations applied in all modes:
  1. Flip optimisation — if the shortest path to target angle is >90°,
     flip to the supplementary angle and negate wheel speed instead.
     This means wheels never need to rotate more than 90°.
  2. Cos-scaling — wheel speed is multiplied by cos(angle_error) so that
     a misaligned wheel contributes proportionally to the actual desired
     direction and does not push the robot sideways during transitions.
  3. Zero-velocity deadband — when cmd_vel is essentially zero the wheel
     angles are held in place and only the speeds are set to zero,
     avoiding unnecessary yaw-joint movement.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


# Wheel steering joint positions in base_link frame (metres).
# Derived from rover_base.xacro:
#   support_y_offset=0.255, yaw_y_offset=0.169 → ly = 0.255+0.169 = 0.424
#   yaw_x_front=0.419, yaw_x_rear=-0.419
WHEEL_POSITIONS = {
    'front_left':  ( 0.419,  0.424),
    'front_right': ( 0.419, -0.424),
    'rear_left':   (-0.419,  0.424),
    'rear_right':  (-0.419, -0.424),
}
WHEEL_ORDER = ['front_left', 'front_right', 'rear_left', 'rear_right']

WHEEL_RADIUS = 0.111   # metres
MAX_WHEEL_SPEED = 50.0  # rad/s — normalisation ceiling
CMD_VEL_DEADBAND = 1e-3  # m/s or rad/s — below this treat as zero


class SwerveDriveNode(Node):

    def __init__(self):
        super().__init__('swerve_drive_node')

        self._mode = 'swerve'

        # Last known yaw joint positions (radians), keyed by short wheel name.
        self._current_yaw: dict[str, float] = {name: 0.0 for name in WHEEL_ORDER}

        self._pub_yaw = self.create_publisher(
            Float64MultiArray,
            '/yaw_position_controller/commands',
            10
        )
        self._pub_wheel = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )

        self.create_subscription(Twist,      '/cmd_vel',      self._cmd_vel_cb,      10)
        self.create_subscription(String,     '/drive_mode',   self._drive_mode_cb,   10)
        self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)

        self.get_logger().info(f'Swerve drive node started — mode: {self._mode}')

    # ──────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────

    def _drive_mode_cb(self, msg: String):
        mode = msg.data.strip().lower()
        if mode not in ('swerve', 'differential', 'ackermann'):
            self.get_logger().warn(f'Unknown drive mode: {mode!r}')
            return
        self._mode = mode
        self.get_logger().info(f'Drive mode → {self._mode}')

    def _joint_states_cb(self, msg: JointState):
        """Track current yaw joint positions from joint_state_broadcaster."""
        for name, position in zip(msg.name, msg.position):
            # joint names are e.g. "front_left_yaw_joint" → key "front_left"
            if name.endswith('_yaw_joint'):
                key = name[: -len('_yaw_joint')]
                if key in self._current_yaw:
                    self._current_yaw[key] = position

    def _cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        if self._mode == 'swerve':
            angles, speeds = self._swerve_ik(vx, vy, wz)
        elif self._mode == 'differential':
            angles, speeds = self._differential_ik(vx, wz)
        else:  # ackermann
            angles, speeds = self._ackermann_ik(vx, wz)

        yaw_msg = Float64MultiArray()
        yaw_msg.data = angles

        wheel_msg = Float64MultiArray()
        wheel_msg.data = speeds

        self._pub_yaw.publish(yaw_msg)
        self._pub_wheel.publish(wheel_msg)

    # ──────────────────────────────────────────
    # Steering helpers
    # ──────────────────────────────────────────

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to [-π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _optimize_steer(
        self, target_angle: float, current_angle: float, speed: float
    ) -> tuple[float, float, float]:
        """
        Flip optimisation: if rotating to target_angle from current_angle
        requires more than 90°, use the supplementary angle and negate speed.

        Returns (optimised_angle, optimised_speed, residual_delta).
        residual_delta is used by the caller for cos-scaling.
        """
        delta = self._normalize_angle(target_angle - current_angle)
        if abs(delta) > math.pi / 2:
            target_angle = self._normalize_angle(target_angle + math.pi)
            speed = -speed
            delta = self._normalize_angle(target_angle - current_angle)
        return target_angle, speed, delta

    # ──────────────────────────────────────────
    # Kinematics
    # ──────────────────────────────────────────

    def _swerve_ik(self, vx: float, vy: float, wz: float):
        """
        Full swerve (holonomic): each wheel steers and drives independently.

        Steps per wheel:
          1. Inverse kinematics → raw target angle + raw speed
          2. Flip optimisation  → never rotate yaw more than 90°
          3. Cos-scaling        → speed ∝ cos(angle error) during transitions
          4. Global normalisation → clamp max wheel speed to MAX_WHEEL_SPEED
        """
        # Zero-velocity deadband: keep current angles, stop wheels.
        if abs(vx) < CMD_VEL_DEADBAND and abs(vy) < CMD_VEL_DEADBAND and abs(wz) < CMD_VEL_DEADBAND:
            return [self._current_yaw[w] for w in WHEEL_ORDER], [0.0] * 4

        opt_angles = []
        raw_speeds = []

        for name in WHEEL_ORDER:
            lx, ly = WHEEL_POSITIONS[name]
            # Velocity vector at this wheel's contact point
            a = vx - wz * ly
            b = vy + wz * lx
            raw_angle = math.atan2(b, a)
            raw_speed  = math.hypot(a, b) / WHEEL_RADIUS

            opt_angle, opt_speed, delta = self._optimize_steer(
                raw_angle, self._current_yaw[name], raw_speed
            )
            # Scale speed by cos of remaining angular error
            eff_speed = opt_speed * math.cos(delta)

            opt_angles.append(opt_angle)
            raw_speeds.append(eff_speed)

        # Normalise so no wheel exceeds MAX_WHEEL_SPEED
        max_speed = max(abs(s) for s in raw_speeds) if raw_speeds else 0.0
        if max_speed > 1e-6:
            scale = min(1.0, MAX_WHEEL_SPEED / max_speed)
        else:
            scale = 1.0

        speeds = [s * scale for s in raw_speeds]
        return opt_angles, speeds

    def _differential_ik(self, vx: float, wz: float):
        """
        Differential / skid-steer: yaw joints locked to 0°, left/right sides
        get different speeds.

        Flip optimisation still applies: if a yaw joint has drifted near ±180°
        from a previous mode, it will be kept there and the speed sign flipped
        rather than forcing a large rotation back to 0°.
        """
        # Zero-velocity deadband
        if abs(vx) < CMD_VEL_DEADBAND and abs(wz) < CMD_VEL_DEADBAND:
            return [self._current_yaw[w] for w in WHEEL_ORDER], [0.0] * 4

        left_speed  = (vx - wz * 0.424) / WHEEL_RADIUS
        right_speed = (vx + wz * 0.424) / WHEEL_RADIUS

        target_angles = {
            'front_left':  0.0, 'rear_left':  0.0,
            'front_right': 0.0, 'rear_right': 0.0,
        }
        raw_speeds = {
            'front_left':  left_speed,  'rear_left':  left_speed,
            'front_right': right_speed, 'rear_right': right_speed,
        }

        angles = []
        speeds = []
        for name in WHEEL_ORDER:
            opt_angle, opt_speed, delta = self._optimize_steer(
                target_angles[name], self._current_yaw[name], raw_speeds[name]
            )
            angles.append(opt_angle)
            speeds.append(opt_speed * math.cos(delta))

        return angles, speeds

    def _ackermann_ik(self, vx: float, wz: float):
        """
        Ackermann steering: front wheels steer, rear wheels drive straight.
        Uses bicycle model. vy is ignored.

        Unlike swerve, flip optimisation is NOT applied here — Ackermann steer
        angles are always within ±MAX_STEER so they never need a 180° flip.
        Applying flip optimisation would invert one wheel when R is small,
        causing the front wheels to point in opposite directions.

        Cos-scaling is still applied for smooth steering transitions.
        """
        track      = 0.848          # total track width (2 × 0.424 m)
        wb         = 0.838          # wheelbase
        half_track = track / 2.0
        MAX_STEER  = math.radians(40)   # physical steer limit

        # Zero-velocity deadband
        if abs(vx) < CMD_VEL_DEADBAND and abs(wz) < CMD_VEL_DEADBAND:
            return [self._current_yaw[w] for w in WHEEL_ORDER], [0.0] * 4

        if abs(wz) < 1e-6:
            # Straight line
            speed = vx / WHEEL_RADIUS
            return [0.0, 0.0, 0.0, 0.0], [speed] * 4

        # Use absolute turning radius and handle direction with turn_sign.
        # Clamp to minimum radius so the inner wheel never needs >MAX_STEER.
        R_min = wb / math.tan(MAX_STEER) + half_track
        R_abs = max(abs(vx / wz), R_min)

        # Both steer angle magnitudes are now safely < MAX_STEER
        steer_inner = math.atan2(wb, R_abs - half_track)  # larger angle
        steer_outer = math.atan2(wb, R_abs + half_track)  # smaller angle

        # Turn sign: +1 = left (CCW), -1 = right (CW)
        turn_sign = math.copysign(1.0, wz)

        if wz > 0:  # left turn: front-left is inner (larger steer)
            fl_angle = turn_sign * steer_inner
            fr_angle = turn_sign * steer_outer
            d_fl = math.hypot(wb, R_abs - half_track)   # inner front arc radius
            d_fr = math.hypot(wb, R_abs + half_track)   # outer front arc radius
            d_rl = R_abs - half_track                    # inner rear arc radius
            d_rr = R_abs + half_track                    # outer rear arc radius
        else:        # right turn: front-right is inner (larger steer)
            fl_angle = turn_sign * steer_outer
            fr_angle = turn_sign * steer_inner
            d_fl = math.hypot(wb, R_abs + half_track)
            d_fr = math.hypot(wb, R_abs - half_track)
            d_rl = R_abs + half_track
            d_rr = R_abs - half_track

        fwd   = math.copysign(1.0, vx)   # +1 forward, -1 backward
        omega = abs(vx) / R_abs           # rotation speed [rad/s]

        raw_pairs = [
            ('front_left',  fl_angle, fwd * omega * d_fl / WHEEL_RADIUS),
            ('front_right', fr_angle, fwd * omega * d_fr / WHEEL_RADIUS),
            ('rear_left',   0.0,      fwd * omega * d_rl / WHEEL_RADIUS),
            ('rear_right',  0.0,      fwd * omega * d_rr / WHEEL_RADIUS),
        ]

        # Cos-scaling only (no flip optimisation)
        angles = []
        speeds = []
        for name, target_angle, raw_speed in raw_pairs:
            delta = self._normalize_angle(target_angle - self._current_yaw[name])
            angles.append(target_angle)
            speeds.append(raw_speed * math.cos(delta))

        return angles, speeds


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
