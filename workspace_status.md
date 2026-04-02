# Workspace Status — chaska_rover
Última actualización: 2026-04-01

---

## Estructura de paquetes

```
src/
├── rover_description/        ← URDF rover, sensores, mundos Gazebo
├── rover_controller/         ← swerve_node + joy_mode_switcher
├── rover_bringup/            ← launch solo rover (hardware/sim)
├── chaska_arm_description/   ← URDF brazo, meshes, RViz config
├── chaska_arm_controller/    ← joint_velocity_node (IK Pinocchio), joystick arm
├── chaska_vision/            ← YOLO detector (RealSense)
└── chaska_bringup/           ← sistema unificado rover+brazo
    ├── urdf/chaska_robot.urdf.xacro   ← URDF unificado
    ├── config/chaska_controllers.yaml ← 5 controladores
    └── launch/
        ├── chaska_simulation.launch.py ← Gazebo unificado
        └── chaska_display.launch.py    ← RViz sin Gazebo
```

---

## Estado del sistema unificado

### Simulación — `chaska_simulation.launch.py`

| Componente | Estado |
|-----------|--------|
| Gazebo Ignition Fortress | ✓ |
| URDF unificado (rover + brazo) | ✓ |
| TF tree completo | ✓ `base_footprint → base_link → [rover] + [arm_base_link → links → EE]` |
| joint_state_broadcaster | ✓ cubre todos los joints (rover + brazo) |
| yaw_position_controller | ✓ 4 joints yaw |
| wheel_velocity_controller | ✓ 4 joints wheel |
| arm_controller | ✓ joint_1..joint_5 (JointTrajectoryController) |
| gripper_controller | ✓ joint_6 (JointTrajectoryController) |
| swerve_node | ✓ |
| joy_mode_switcher | ✓ rover + modo brazo (Cruz PS5) |
| LiDAR 3D Livox `/livox/points` | ✓ |
| Cámara RGBD rover `/camera/*` | ✓ |
| RealSense brazo `/depth_camera/*` | ✓ |
| IMU `/imu` | ✓ |

### Secuencia de arranque

| t | Evento |
|---|--------|
| 0s | Gazebo + RSP + bridge + joy_node |
| 3s | spawn robot (x=7, y=7, z=0.5) |
| 8s | joint_state_broadcaster |
| 9s | yaw + wheel controllers |
| 10s | arm + gripper controllers |
| 11s | swerve_node + joy_mode_switcher |
| 12s | RViz |

> Spawners escalonados (1s entre grupos) para evitar timeouts del controller_manager.

### Mando PS5 — modos

**Modo rover** (default):
- △ Triángulo → swerve (holonómico)
- □ Cuadrado → differential
- ○ Círculo → ackermann
- ✕ Cruz → activar modo brazo

**Modo brazo** (tras pulsar Cruz):
- Stick izq. X/Y → EE velocidad X/Z → `/ee_velocity_target`
- Stick der. X → EE velocidad Y
- R2 → wrist joint_5 → `/wrist_velocity_command`
- ✕ Cruz → volver a modo rover

> En simulación, los comandos del brazo via joy llegan a `joint_velocity_node` (IK Pinocchio) que publica en `/joint_states`. Esto **no está conectado** al `arm_controller` (JointTrajectoryController) — ver pendientes.

---

## Comandos de uso rápido

```bash
# Matar sesión anterior
pkill -f "ign gazebo"; pkill -f "ros2"; sleep 2

# Build + source
cd ~/chaska_rover && colcon build && source install/setup.bash

# Sistema completo rover+brazo
ros2 launch chaska_bringup chaska_simulation.launch.py

# Mundo Rubicon
ros2 launch chaska_bringup chaska_simulation.launch.py world_name:=rubicon

# Visualización sin Gazebo (ajuste posición brazo)
ros2 launch chaska_bringup chaska_display.launch.py

# Solo visualizar brazo
ros2 launch chaska_arm_controller visualize.launch.py

# Solo rover
ros2 launch rover_bringup rover_simulation.launch.py

# Diagnóstico controllers
ros2 control list_controllers

# Ver árbol TF
ros2 run tf2_tools view_frames && evince frames.pdf
```

---

## Aspectos técnicos importantes

### Mundos Gazebo
Ambos mundos (`empty.world`, `rubicon.world`) requieren los 5 plugins:
`Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors` (ogre2), `IMU`
Sin ellos, **ningún sensor publica**.

### URDF unificado
- `arm_mount_joint` (fixed): `base_link → arm_base_link`, `xyz="0.10 0.0 0.20"` — **pendiente ajuste visual**
- Rover: `with_control_plugin:=false` suprime su propio plugin `ign_ros2_control`
- Un solo plugin en `chaska_robot.urdf.xacro` gestiona los 14 joints
- LiDAR: `rpy="0 0.1745 0"` en `laser_joint` (pitch +10° = nose down)

### ros_gz_bridge y RViz
Topics publicados con QoS `Best Effort`. Los displays de RViz deben usar `Reliability Policy: Best Effort`.

### chaska_arm_controller — uso en hardware real
`joint_velocity_node` hace IK (Pinocchio) y publica a `/joint_states`. Se lanza solo para hardware real, **no en simulación** (el broadcaster ya cubre ese topic).
URDF para Pinocchio se genera automáticamente desde xacro en tiempo de ejecución.

---

## Pendientes

| Prioridad | Tarea |
|-----------|-------|
| Alta | **Ajustar `arm_mount_joint` xyz** — usar `chaska_display.launch.py` para verificar posición física del brazo sobre el chasis |
| Alta | **Conectar joystick brazo → arm_controller en simulación** — actualmente joy → `joint_velocity_node` → `/joint_states` (no pasa por JointTrajectoryController). Necesita un bridge que convierta velocidades EE → `JointTrajectory` hacia `arm_controller/joint_trajectory` |
| Alta | **Verificar frame raíz en `arm_kinematics.py`** — Pinocchio carga el URDF del brazo standalone; la posición del EE está referenciada a `arm_base_link`, no a `base_link` del rover. Evaluar si importa para el caso de uso |
| Media | **Odometría** — EKF con robot_localization fusionando GPS + IMU → `/odom` |
| Media | **Nav2 / go-to-goal** — requiere odometría |
| Baja | **Driver hardware real** — reemplazar `mock_components/GenericSystem` en ambos URDF |

---

## Depuración frecuente

| Síntoma | Causa | Fix |
|---------|-------|-----|
| Spawner "Failed to configure controller" | Spawners simultáneos → timeout CM | Ya corregido: escalonados 1s entre grupos |
| Sin topics de sensores en Gazebo | Plugins del mundo faltantes | Verificar plugins en `.world` |
| Topics en Gazebo pero no en RViz | QoS mismatch | `Reliability Policy: Best Effort` en displays |
| Brazo en posición incorrecta | `arm_mount_joint xyz` | Ajustar en `chaska_robot.urdf.xacro`, verificar con `chaska_display.launch.py` |
| Meshes del brazo no aparecen | `GZ_SIM_RESOURCE_PATH` incompleto | Launch ya incluye parent dirs de ambos paquetes |
| joint_velocity_node falla al iniciar | No encuentra URDF | Ya corregido: genera URDF desde xacro en runtime |
