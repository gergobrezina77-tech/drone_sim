"""Quadrotor parameters and vectorised access to a group of drones inside one MjData."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

GRAVITY = 9.81


@dataclass(frozen=True)
class DroneParams:
    """Physical parameters of an X-configuration quadrotor (single source of truth for
    both the MJCF model and the controller's allocation matrix)."""

    mass: float = 0.5  # kg
    inertia: tuple[float, float, float] = (2.3e-3, 2.3e-3, 4.0e-3)  # kg m^2
    arm: float = 0.12  # centre-to-motor distance, m
    rotor_radius: float = 0.055  # m; rotor tips reach arm + rotor_radius = 0.175 m
    max_thrust: float = 3.5  # per motor, N  (thrust/weight ~ 2.85)
    yaw_coef: float = 0.016  # rotor drag torque / thrust, m
    drag_coef: float = 0.2  # linear aerodynamic drag, N / (m/s)
    radius: float = 0.18  # footprint radius (rotor tips) used by swarm logic and rewards, m

    @property
    def motor_angles(self) -> np.ndarray:
        # front-left, rear-left, rear-right, front-right (x forward, y left)
        return np.deg2rad([45.0, 135.0, 225.0, 315.0])

    @property
    def motor_xy(self) -> np.ndarray:
        a = self.motor_angles
        return self.arm * np.stack([np.cos(a), np.sin(a)], axis=1)

    @property
    def spins(self) -> np.ndarray:
        # diagonal motors spin the same way; sign of the reaction torque about body z
        return np.array([1.0, -1.0, 1.0, -1.0])

    @property
    def allocation(self) -> np.ndarray:
        """A such that [T, tau_x, tau_y, tau_z] = A @ f for motor thrusts f."""
        x, y = self.motor_xy.T
        return np.stack([np.ones(4), y, -x, self.spins * self.yaw_coef])

    @property
    def hover_thrust(self) -> float:
        return self.mass * GRAVITY / 4.0


@dataclass
class DroneState:
    pos: np.ndarray  # (n, 3) world position
    quat: np.ndarray  # (n, 4) orientation, w x y z
    R: np.ndarray  # (n, 3, 3) body-to-world rotation
    vel: np.ndarray  # (n, 3) world linear velocity
    omega: np.ndarray  # (n, 3) body-frame angular velocity

    @property
    def n(self) -> int:
        return len(self.pos)


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """Batch (n, 4) wxyz quaternions -> (n, 3, 3) rotation matrices."""
    w, x, y, z = np.moveaxis(q, -1, 0)
    # fmt: off
    return np.stack(
        [
            1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
        ],
        axis=-1,
    ).reshape(*q.shape[:-1], 3, 3)
    # fmt: on


class DroneGroup:
    """Reads state from and writes commands to drones named ``{prefix}{i}`` in a model.

    MuJoCo free joints store the linear velocity in the world frame and the angular
    velocity in the body frame, which is exactly what the controller wants.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, n: int, prefix: str = "drone"):
        self.model, self.data, self.n = model, data, n
        obj = mujoco.mjtObj
        self.body_ids = np.array([mujoco.mj_name2id(model, obj.mjOBJ_BODY, f"{prefix}{i}") for i in range(n)])
        joints = model.body_jntadr[self.body_ids]
        self._qpos_idx = model.jnt_qposadr[joints][:, None] + np.arange(7)
        self._qvel_idx = model.jnt_dofadr[joints][:, None] + np.arange(6)
        self.actuator_ids = np.array(
            [[mujoco.mj_name2id(model, obj.mjOBJ_ACTUATOR, f"{prefix}{i}_m{j}") for j in range(4)] for i in range(n)]
        )

    def state(self) -> DroneState:
        qpos = self.data.qpos[self._qpos_idx]
        qvel = self.data.qvel[self._qvel_idx]
        quat = qpos[:, 3:7]
        return DroneState(qpos[:, :3], quat, quat_to_mat(quat), qvel[:, :3], qvel[:, 3:])

    def linear_velocity(self) -> np.ndarray:
        return self.data.qvel[self._qvel_idx[:, :3]]

    def set_thrusts(self, f: np.ndarray) -> None:
        self.data.ctrl[self.actuator_ids] = f

    def set_external_forces(self, force: np.ndarray) -> None:
        self.data.xfrc_applied[self.body_ids, :3] = force

    def set_pose(self, pos: np.ndarray, quat: np.ndarray | None = None, vel: np.ndarray | None = None) -> None:
        qpos = self.data.qpos
        qpos[self._qpos_idx[:, :3]] = pos
        qpos[self._qpos_idx[:, 3:]] = [1.0, 0.0, 0.0, 0.0] if quat is None else quat
        self.data.qvel[self._qvel_idx] = 0.0
        if vel is not None:
            self.data.qvel[self._qvel_idx[:, :3]] = vel
