# Chaska Rover

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo Ignition Fortress](https://img.shields.io/badge/Gazebo-Ignition_Fortress-orange)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10-blue?logo=python)](https://www.python.org/)
[![Pinocchio](https://img.shields.io/badge/Pinocchio-IK-green)](https://github.com/stack-of-tasks/pinocchio)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Detection-purple)](https://github.com/ultralytics/ultralytics)
[![URC](https://img.shields.io/badge/Competition-URC-red)](https://urc.marssociety.org/)
[![ERC](https://img.shields.io/badge/Competition-ERC-blue)](https://roverchallenge.eu/)

A full-stack ROS 2 platform for rover competitions, combining a **4-wheel swerve-drive chassis** with a **6-DOF robotic arm** and a sensor suite including 3D LiDAR, depth cameras, and IMU. The system features three switchable drive modes (swerve, differential, Ackermann), velocity-based arm IK via Pinocchio, and full teleoperation through a PS5 DualSense controller. Designed for the **University Rover Challenge (URC)** and **European Rover Challenge (ERC)**.

<p align="center">
  <img src="media/chaska_sim.gif" alt="Chaska Rover simulation" width="720"/>
</p>

---

## Packages

| Package | Description |
|---------|-------------|
| `rover_description` | Rover URDF (xacro), sensors, Gazebo worlds |
| `rover_controller` | Swerve kinematics node, PS5 joy mode switcher |
| `rover_bringup` | Launch files for rover-only simulation / hardware |
| `chaska_arm_description` | 6-DOF arm URDF (xacro), meshes, RViz config |
| `chaska_arm_controller` | Joint velocity node (Pinocchio IK) |
| `chaska_vision` | YOLOv8-based object detection (RealSense D435) |
| `chaska_bringup` | **Unified system** — launch, URDF, and controllers for rover + arm |

---

## Hardware

<p align="center">
  <img src="media/chaska_real.gif" alt="Chaska Rover real hardware" width="720"/>
</p>

| Component | Model |
|-----------|-------|
| Rover drive | 4-wheel swerve (holonomic) |
| Arm | 6-DOF: 1 prismatic + 4 revolute + 1 prismatic gripper |
| 3D LiDAR | Livox (16-channel, 360°, 100 m range) |
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
- `pinocchio` (arm IK)
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

```bash
# With Rubicon world
ros2 launch chaska_bringup chaska_simulation.launch.py world_name:=rubicon
```

### URDF visualization (no Gazebo)

```bash
ros2 launch chaska_bringup chaska_display.launch.py
```

### Rover only

```bash
ros2 launch rover_bringup rover_simulation.launch.py
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
| L1 + sticks | Move (deadman switch) |
| L1 + R1 + sticks | Turbo (2× speed) |

### Arm mode (after pressing Cross)

| Input | Action |
|-------|--------|
| Left stick X/Y | End-effector lateral / forward-back (IK) |
| Right stick Y | End-effector height (IK) |
| D-pad ↑/↓ | Wrist pitch (joint 4) |
| D-pad ←/→ | Wrist roll (joint 5) |
| R1 / L1 | Gripper open / close |
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

> RViz displays require **Reliability Policy: Best Effort** (ros_gz_bridge QoS).
