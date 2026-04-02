# chaska_arm_controller/arm_kinematics.py

import pinocchio as pin
import numpy as np

class ArmKinematics:

    # Límites de tu robot
    JOINT_LIMITS = {
        'joint_1': {'type': 'prismatic', 'min': -0.125, 'max':  0.125},  # metros
        'joint_2': {'type': 'revolute',  'min': -1.5708, 'max': 1.5708}, # ±90°
        'joint_3': {'type': 'revolute',  'min': -1.5708, 'max': 1.5708},
        'joint_4': {'type': 'revolute',  'min': -1.5708, 'max': 1.5708},
        'joint_5': {'type': 'revolute',  'min': -1.5708, 'max': 1.5708},
    }

    def __init__(self, urdf_path: str):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()

        # Pinocchio ignora joints fijos → nq = 5
        self.nq = self.model.nq
        self.nv = self.model.nv

        # Imprimir info del modelo para verificación
        self._print_model_info()

        # End effector: joint_6 es fijo, Pinocchio lo convierte en frame
        # El nombre del frame será "end_effector" (el child link de joint_6)
        self.ee_frame_id = self._get_ee_frame()

        # Límites de velocidad del modelo
        self.v_max = self.model.velocityLimit  # shape: (nv,)

    def _print_model_info(self):
        print(f"\n{'='*50}")
        print(f"  Modelo cargado: {self.model.name}")
        print(f"  nq (config):    {self.model.nq}")
        print(f"  nv (vel):       {self.model.nv}")
        print(f"\n  Joints activos:")
        for i, name in enumerate(self.model.names):
            if name != 'universe':
                print(f"    [{i}] {name}")
        print(f"\n  Frames disponibles:")
        for i in range(self.model.nframes):
            print(f"    [{i}] {self.model.frames[i].name}")
        print(f"{'='*50}\n")

    def _get_ee_frame(self) -> int:
        """Busca el frame del efector final con fallback."""
        candidates = ['end_effector', 'link_5', 'tool0', 'ee_link']
        for name in candidates:
            if self.model.existFrame(name):
                print(f"  ✓ Frame EE encontrado: '{name}'")
                return self.model.getFrameId(name)
        raise ValueError(
            "No se encontró frame del efector final. "
            f"Frames disponibles: {[self.model.frames[i].name for i in range(self.model.nframes)]}"
        )

    # ------------------------------------------------------------------
    def forward_kinematics(self, q: np.ndarray) -> pin.SE3:
        """Cinemática directa → pose del EE."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame_id]

    def compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Jacobiano traslacional del EE (3 x nv)."""
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        J_full = pin.getFrameJacobian(
            self.model, self.data,
            self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )  # (6, nv)

        return J_full[:3, :]  # Solo traslación (3, nv)

    def joint_velocities_from_ee_velocity(
        self,
        q:          np.ndarray,
        v_ee:       np.ndarray,
        damping:    float = 1e-3,
        scale:      float = 1.0
    ) -> np.ndarray:
        """
        Calcula dq dado v_ee = [vx, vy, vz] usando pseudoinversa amortiguada.

        Parámetros:
            q       : configuración articular actual (nq,)
            v_ee    : velocidad deseada del EE en m/s (3,)
            damping : factor de amortiguación λ para singularidades
            scale   : escala global de velocidad (0.0 - 1.0)

        Retorna:
            dq      : velocidades articulares (nv,)  [m/s, rad/s]
        """
        J   = self.compute_jacobian(q)
        lam = damping ** 2

        # DLS: dq = J^T (J J^T + λI)^{-1} v_ee
        JJT   = J @ J.T
        J_dls = J.T @ np.linalg.inv(JJT + lam * np.eye(3))
        dq    = J_dls @ v_ee * scale

        # Saturar a límites de velocidad del modelo
        dq = self._saturate_velocity(dq)
        return dq

    def _saturate_velocity(self, dq: np.ndarray) -> np.ndarray:
        """Satura dq respetando los límites del URDF."""
        if np.any(self.v_max > 0):
            dq = np.clip(dq, -self.v_max, self.v_max)
        return dq

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        """Posición cartesiana del EE [x, y, z]."""
        pose = self.forward_kinematics(q)
        return pose.translation