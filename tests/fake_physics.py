"""A deliberately simple in-memory PhysicsBackend for tests: point masses, thrust along body z, flat floor.

Registered as ``test_kinematic`` so the backend contract tests (and later engine tests) can run without MuJoCo.
"""

import numpy as np

from drone_sim.core.geometry import distance
from drone_sim.core.rotations import quat_to_mat
from drone_sim.core.state import FleetState
from drone_sim.physics import PHYSICS_BACKENDS
from drone_sim.vehicle import GRAVITY, get_vehicle
from drone_sim.world import WorldSpec


class KinematicBackend:
    timestep = 0.002

    def build(self, world: WorldSpec) -> None:
        params = [get_vehicle(d.vehicle) for d in world.drones]
        self._mass = np.array([p.mass for p in params])
        self._max_thrust = np.array([p.max_thrust for p in params])
        self._radius = np.array([p.radius for p in params])
        self._start = np.array([d.position for d in world.drones], dtype=float).reshape(-1, 3)
        yaw = np.array([d.yaw for d in world.drones])
        self._quat = np.stack([np.cos(yaw / 2), 0 * yaw, 0 * yaw, np.sin(yaw / 2)], axis=1)
        self._shapes = [o.shape for o in world.obstacles]
        self.reset()

    def reset(self) -> None:
        n = len(self._start)
        self.time = 0.0
        self._pos, self._vel = self._start.copy(), np.zeros((n, 3))
        self._thrust, self._force = np.zeros((n, 4)), np.zeros((n, 3))
        self._frozen = np.zeros(n, dtype=bool)

    @property
    def n(self) -> int:
        return len(self._pos)

    def state(self) -> FleetState:
        return FleetState(
            self._pos.copy(), self._quat.copy(), quat_to_mat(self._quat), self._vel.copy(), np.zeros((self.n, 3))
        )

    def set_thrusts(self, thrusts: np.ndarray) -> None:
        self._thrust = np.clip(thrusts, 0.0, self._max_thrust[:, None])

    def set_external_forces(self, forces: np.ndarray) -> None:
        self._force = np.asarray(forces, dtype=float)

    def step(self, substeps: int = 1) -> None:
        up = quat_to_mat(self._quat)[:, :, 2]
        for _ in range(substeps):
            acc = (self._thrust.sum(axis=1) / self._mass)[:, None] * up + self._force / self._mass[:, None]
            acc[:, 2] -= GRAVITY
            self._vel += acc * self.timestep
            self._vel[self._frozen] = 0.0
            self._pos += self._vel * self.timestep
            on_floor = self._pos[:, 2] < self._start[:, 2]  # the floor is where each drone started resting
            self._pos[on_floor, 2] = self._start[on_floor, 2]
            self._vel[on_floor, 2] = np.maximum(self._vel[on_floor, 2], 0.0)
            self.time += self.timestep

    def collisions(self) -> np.ndarray:
        hit = np.zeros(self.n, dtype=bool)
        if self._shapes:
            hit |= (distance(self._pos, self._shapes)[0] < self._radius[:, None]).any(axis=1)
        gap = np.linalg.norm(self._pos[:, None] - self._pos[None], axis=-1) - (
            self._radius[:, None] + self._radius[None]
        )
        np.fill_diagonal(gap, np.inf)
        gap[self._frozen] = gap[:, self._frozen] = np.inf
        hit |= (gap < 0).any(axis=1)
        return hit & ~self._frozen

    def freeze(self, mask: np.ndarray) -> None:
        self._frozen |= np.asarray(mask, dtype=bool)
        self._vel[self._frozen] = 0.0


if "test_kinematic" not in PHYSICS_BACKENDS:
    PHYSICS_BACKENDS.register("test_kinematic", KinematicBackend)
