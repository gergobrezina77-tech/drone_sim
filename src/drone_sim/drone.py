"""Quadrotor parameters and vectorised access to a group of drones inside one MjData."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from drone_sim.core.rotations import quat_to_mat
from drone_sim.vehicle import GRAVITY, DroneParams

__all__ = ["GRAVITY", "DroneGroup", "DroneParams", "DroneState", "quat_to_mat"]


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
