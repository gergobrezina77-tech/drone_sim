"""Drone software: the FlightSoftware interface and the first implementation, a naive goto autopilot.

Flight software only sees an ``Observation`` (today: ground-truth own state; a sensor model can replace it) and
its ``Goals``, and only outputs per-motor thrusts. It never touches the physics engine.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

from drone_sim.autonomy.control import GeometricController
from drone_sim.core.registry import Registry
from drone_sim.core.state import FleetState
from drone_sim.vehicle import get_vehicle
from drone_sim.world import WorldSpec


@dataclass
class Observation:
    state: FleetState
    time: float


@dataclass
class Goals:
    target: np.ndarray  # (n, 3) world position to reach
    active: np.ndarray  # (n,) bool; drones without an active goal stay idle (motors off)


@runtime_checkable
class FlightSoftware(Protocol):
    def reset(self, world: WorldSpec) -> None: ...

    def set_goals(self, goals: Goals) -> None: ...

    def step(self, obs: Observation, dt: float) -> np.ndarray:
        """Per-motor thrusts, (n, 4), in N."""
        ...


FLIGHT_SOFTWARE = Registry[Callable[[], FlightSoftware]]("flight software")


def _subset(s: FleetState, idx: np.ndarray) -> FleetState:
    return FleetState(s.pos[idx], s.quat[idx], s.R[idx], s.vel[idx], s.omega[idx])


class GotoAutopilot:
    """Fly straight to the goal at up to ``max_speed`` and hover there. No obstacle avoidance.

    Velocity command = saturate(gain * position error, max_speed), tracked by the geometric controller's velocity
    loop. Drones are grouped by vehicle type, one vectorised controller per group.
    """

    def __init__(self, max_speed: float = 3.0, gain: float = 1.2):
        self.max_speed, self.gain = max_speed, gain

    def reset(self, world: WorldSpec) -> None:
        self.n = len(world.drones)
        vehicles = np.array([d.vehicle for d in world.drones], dtype=object)
        self._groups = [
            (idx, GeometricController(get_vehicle(v), len(idx)))
            for v in dict.fromkeys(vehicles)
            for idx in [np.flatnonzero(vehicles == v)]
        ]
        self._yaw = np.array([d.yaw for d in world.drones], dtype=float)
        self._goals = Goals(np.zeros((self.n, 3)), np.zeros(self.n, dtype=bool))

    def set_goals(self, goals: Goals) -> None:
        self._goals = goals

    def step(self, obs: Observation, dt: float) -> np.ndarray:
        s = obs.state
        err = self._goals.target - s.pos
        dist = np.linalg.norm(err, axis=1, keepdims=True)
        vel_cmd = err * np.minimum(self.gain, self.max_speed / np.maximum(dist, 1e-9))
        thrusts = np.zeros((self.n, 4))
        for idx, controller in self._groups:
            thrusts[idx] = controller.velocity_control(_subset(s, idx), vel_cmd[idx], self._yaw[idx], dt)
        thrusts[~self._goals.active] = 0.0
        return thrusts
