# Chaska Rover

A 4-wheel swerve-drive rover with a 6-DOF robotic arm, simulated in Gazebo Ignition Fortress under ROS 2 Humble. The system supports autonomous navigation hardware and teleoperation via a PS5 DualSense controller, with switchable drive modes and arm control.

---

## System Overview

```
┌─────────────────────────────────────────────────────────┐
│                     chaska_robot                        │
│                                                         │
│   base_footprint                                        │
│       └── base_link (chassis)                           │
│             ├── [FL/FR/RL/RR] yaw → wheel (×4)         │
│             ├── laser  (Livox 3D LiDAR, 16ch)           │
│             ├── camera (RGBD)                           │
│             ├── imu                                     │
│             └── arm_base_link (fixed mount)             │
│                   └── link_1 → link_2 → link_3          │
│                         → link_4 → link_5               │
│                               ├── link_6 (gripper)      │
│                               ├── end_effector          │
│                               └── depth_camera (D435)   │
└─────────────────────────────────────────────────────────┘
```

**Drive modes:** Swerve (holonomic) · Differential · Ackermann  
**Arm control:** Velocity-based IK via Pinocchio (Damped Least Squares)  
**Simulation:** Gazebo Ignition Fortress · single `controller_manager` · 5 ros2_control controllers

---

## Packages

| Package | Description |
|---------|-------------|
| `rover_description` | Rover URDF (xacro), sensors, Gazebo worlds |
| `rover_controller` | Swerve kinematics node, PS5 joy mode switcher |
| `rover_bringup` | Launch files for rover-only simulation / hardware |
| `chaska_arm_description` | 6-DOF arm URDF (xacro), meshes, RViz config |
| `chaska_arm_controller` | Joint velocity node (Pinocchio IK), joystick arm controller |
| `chaska_vision` | YOLOv8-based object detection (RealSense D435) |
| `chaska_bringup` | **Unified system** — launch, URDF, and controllers for rover + arm |

---

## Hardware

| Component | Model |
|-----------|-------|
| Rover drive | 4-wheel swerve (holonomic) |
| Arm | 6-DOF: 1 prismatic + 4 revolute + 1 prismatic gripper |
| 3D LiDAR | Livox (simulated: 16-channel, 360°, 100 m range) |
| RGB-D rover | Forward-facing RGBD camera |
| RGB-D arm | Intel RealSense D435 on end-effector |
| IMU | On chassis |
| Gamepad | PS5 DualSense (USB or Bluetooth) |

---

## Requirements

- ROS 2 Humble
- Gazebo Ignition Fortress
- `ros_gz_sim`, `ros_gz_bridge`, `ign_ros2_control`
- `joint_trajectory_controller`, `joint_state_broadcaster`
- `position_controllers`, `velocity_controllers`
- `robot_state_publisher`, `joint_state_publisher_gui`
- `pinocchio` (for arm IK)
- `joy` (PS5 controller driver)

---

## Build

```bash
cd ~/chaska_rover
colcon build
source install/setup.bash
```

---

## Usage

### Unified simulation (rover + arm)

```bash
ros2 launch chaska_bringup chaska_simulation.launch.py
```

With a specific world:

```bash
ros2 launch chaska_bringup chaska_simulation.launch.py world_name:=rubicon
```

Available worlds: `empty` (default), `rubicon`

### URDF visualization (no Gazebo)

```bash
# Rover + arm — adjust arm mount position
ros2 launch chaska_bringup chaska_display.launch.py

# Arm only
ros2 launch chaska_arm_controller visualize.launch.py
```

### Rover only

```bash
ros2 launch rover_bringup rover_simulation.launch.py
```

### Kill simulation

```bash
pkill -f "ign gazebo"; pkill -f "ros2"
```

---

## PS5 Controller

### Rover mode (default)

| Button | Action |
|--------|--------|
| △ Triangle | Swerve mode (holonomic: vx + vy + ωz) |
| □ Square | Differential mode (skid-steer) |
| ○ Circle | Ackermann mode (car-like steering) |
| ✕ Cross | Switch to arm mode |

### Arm mode (after pressing Cross)

| Input | Action |
|-------|--------|
| Left stick X | End-effector velocity X (lateral) |
| Left stick Y | End-effector velocity Z (up/down) |
| Right stick X | End-effector velocity Y (forward/back) |
| R2 trigger | Wrist rotation (joint 5) |
| ✕ Cross | Return to rover mode |

---

## Sensor Topics

| Topic | Type | Source |
|-------|------|--------|
| `/livox/points` | `PointCloud2` | Livox 3D LiDAR |
| `/camera/image` | `Image` | Rover RGBD |
| `/camera/depth_image` | `Image` | Rover RGBD |
| `/camera/points` | `PointCloud2` | Rover RGBD |
| `/depth_camera/image` | `Image` | Arm RealSense D435 |
| `/depth_camera/points` | `PointCloud2` | Arm RealSense D435 |
| `/imu` | `Imu` | Chassis IMU |

> RViz displays for these topics require **Reliability Policy: Best Effort** (ros_gz_bridge publishes Best Effort).

---

## Controllers

```bash
ros2 control list_controllers
```

| Controller | Type | Joints |
|-----------|------|--------|
| `joint_state_broadcaster` | JointStateBroadcaster | all |
| `yaw_position_controller` | JointGroupPositionController | FL/FR/RL/RR yaw (×4) |
| `wheel_velocity_controller` | JointGroupVelocityController | FL/FR/RL/RR wheel (×4) |
| `arm_controller` | JointTrajectoryController | joint_1..joint_5 |
| `gripper_controller` | JointTrajectoryController | joint_6 |

---

## Diagnostics

```bash
# Controller status
ros2 control list_controllers

# TF tree
ros2 run tf2_tools view_frames && evince frames.pdf

# Joint states (rover + arm in one topic)
ros2 topic echo /joint_states --once

# Gamepad detection
ros2 run joy joy_enumerate_devices
```

---

## Roadmap

- [ ] Arm joystick → `arm_controller` bridge in simulation (IK → JointTrajectory)
- [ ] Odometry — EKF fusing GPS + IMU via `robot_localization`
- [ ] Nav2 go-to-goal navigation
- [ ] Real hardware drivers (replace `mock_components/GenericSystem`)
- [ ] Elevation mapping for traversability estimation
