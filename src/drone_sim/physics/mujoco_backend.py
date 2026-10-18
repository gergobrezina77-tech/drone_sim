"""MuJoCo implementation of PhysicsBackend.

- The compiled model is reused while ``world.geometry_hash()`` is unchanged: a restart is ``mj_resetData`` plus
  writing start poses, no recompile.
- Motor thrusts become a body wrench each substep, via each vehicle's allocation matrix ([T, tau] = A f) and the
  current attitude: equivalent to rotor-site motors, but linear in the number of drones (see ``mjcf.py``).
- Collisions are geometric (see ``mjcf.py`` and ``collision.py``): drone footprints against obstacle shapes and
  against each other, with spatial prefilters so exact distances are only computed for nearby pairs.
- Frozen drones are pinned back to their held pose after every physics step.
"""

from __future__ import annotations

import mujoco
import numpy as np

from drone_sim.core.rotations import quat_to_mat
from drone_sim.core.state import FleetState
from drone_sim.physics.collision import aabb, touching_obstacles, touching_pairs
from drone_sim.physics.mjcf import build_mjcf, drone_body_name
from drone_sim.vehicle import get_vehicle
from drone_sim.world import WorldSpec


class MujocoBackend:
    def __init__(self, timestep: float = 0.002):
        self.timestep = timestep
        self.compile_count = 0
        self._hash: str | None = None

    # --- build / reset -------------------------------------------------------------------------------------------

    def build(self, world: WorldSpec) -> None:
        geometry = world.geometry_hash()
        if geometry != self._hash:
            self._compile(world)
            self._hash = geometry
        self._start_pos = np.array([d.position for d in world.drones], dtype=float).reshape(-1, 3)
        yaw = np.array([d.yaw for d in world.drones], dtype=float)
        self._start_quat = np.stack([np.cos(yaw / 2), 0 * yaw, 0 * yaw, np.sin(yaw / 2)], axis=1)
        self.reset()

    def _compile(self, world: WorldSpec) -> None:
        m = mujoco.MjModel.from_xml_string(build_mjcf(world, self.timestep))
        self.model, self.data = m, mujoco.MjData(m)
        self.compile_count += 1
        n = len(world.drones)
        self._body = np.array(
            [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, drone_body_name(i)) for i in range(n)], int
        )
        joints = m.body_jntadr[self._body]
        self._qpos = (m.jnt_qposadr[joints][:, None] + np.arange(7)).reshape(n, 7)
        self._qvel = (m.jnt_dofadr[joints][:, None] + np.arange(6)).reshape(n, 6)
        params = [get_vehicle(d.vehicle) for d in world.drones]
        self._alloc = np.array([p.allocation for p in params]).reshape(n, 4, 4)
        self._max_thrust = np.array([p.max_thrust for p in params])
        self._radius = np.array([p.radius for p in params])
        self._shapes = [o.shape for o in world.obstacles]
        bounds = [aabb(s) for s in self._shapes]
        self._shape_lo = np.array([lo for lo, _ in bounds]).reshape(-1, 3)
        self._shape_hi = np.array([hi for _, hi in bounds]).reshape(-1, 3)

    def reset(self) -> None:
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        d.qpos[self._qpos[:, :3]] = self._start_pos
        d.qpos[self._qpos[:, 3:]] = self._start_quat
        mujoco.mj_forward(m, d)
        self._frozen = np.zeros(self.n, dtype=bool)
        self._pinned = d.qpos[self._qpos].copy()
        self._body_wrench = np.zeros((self.n, 4))  # [T, tau_x, tau_y, tau_z] in the body frame
        self._external = np.zeros((self.n, 3))

    # --- inputs / stepping ---------------------------------------------------------------------------------------

    @property
    def n(self) -> int:
        return len(self._body)

    @property
    def time(self) -> float:
        return self.data.time

    def set_thrusts(self, thrusts: np.ndarray) -> None:
        f = np.clip(thrusts, 0.0, self._max_thrust[:, None])
        f[self._frozen] = 0.0
        self._body_wrench = np.einsum("nij,nj->ni", self._alloc, f)

    def set_external_forces(self, forces: np.ndarray) -> None:
        self._external = np.where(self._frozen[:, None], 0.0, forces)

    def step(self, substeps: int = 1) -> None:
        d, frozen = self.data, self._frozen.any()
        for _ in range(substeps):
            R = quat_to_mat(d.qpos[self._qpos[:, 3:]])
            d.xfrc_applied[self._body, :3] = R[:, :, 2] * self._body_wrench[:, :1] + self._external
            d.xfrc_applied[self._body, 3:] = np.einsum("nij,nj->ni", R, self._body_wrench[:, 1:])
            mujoco.mj_step(self.model, d)
            if frozen:
                d.qpos[self._qpos[self._frozen]] = self._pinned[self._frozen]
                d.qvel[self._qvel[self._frozen]] = 0.0

    def freeze(self, mask: np.ndarray) -> None:
        new = np.asarray(mask, dtype=bool) & ~self._frozen
        self._pinned[new] = self.data.qpos[self._qpos[new]]
        self.data.qvel[self._qvel[new]] = 0.0
        self._body_wrench[new] = 0.0
        self._external[new] = 0.0
        self._frozen |= new

    # --- outputs -------------------------------------------------------------------------------------------------

    def state(self) -> FleetState:
        qpos, qvel = self.data.qpos[self._qpos], self.data.qvel[self._qvel]
        quat = qpos[:, 3:7]
        return FleetState(qpos[:, :3].copy(), quat.copy(), quat_to_mat(quat), qvel[:, :3].copy(), qvel[:, 3:].copy())

    def collisions(self) -> np.ndarray:
        live = ~self._frozen
        pos = self.data.qpos[self._qpos[:, :3]]
        hit = touching_obstacles(pos, self._radius, self._shapes, self._shape_lo, self._shape_hi)
        i, j = touching_pairs(pos, self._radius)
        both_live = live[i] & live[j]
        hit[i[both_live]] = True
        hit[j[both_live]] = True
        return hit & live
