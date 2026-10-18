"""TaskBoard: every task of a run, its status, and the failure checks shared by all task types.

One task per drone for now (sequencing is DS-025). Tasks start ACTIVE at time 0. Each tick, failures are checked
first (collision > crash > out of bounds > timeout), then each task type's success condition. Terminal statuses
are sticky. The only Python loop per tick is over task types.
"""

from __future__ import annotations

from collections import Counter

import numpy as np

from drone_sim.core.state import FleetState
from drone_sim.tasks.base import TASK_TYPES, FailureReason, TaskStatus, outside_arena
from drone_sim.world import WorldSpec

DEFAULT_TIMEOUT = 60.0  # s; any task can override it with a "timeout" param


class TaskBoard:
    def __init__(self, world: WorldSpec, crash_tilt_deg: float = 80.0):
        drone_index = {d.id: i for i, d in enumerate(world.drones)}
        if extra := [d for d, k in Counter(t.drone_id for t in world.tasks).items() if k > 1]:
            raise ValueError(f"drones {extra} have more than one task; only one task per drone is supported yet")
        self.n = len(world.drones)
        self.arena_size = world.arena_size
        self.min_up_z = np.cos(np.deg2rad(crash_tilt_deg))
        self.task_ids = [t.id for t in world.tasks]
        self.drone = np.array([drone_index[t.drone_id] for t in world.tasks], dtype=int)
        self.timeout = np.array([float(t.params.get("timeout", DEFAULT_TIMEOUT)) for t in world.tasks])
        self.targets = np.zeros((len(world.tasks), 3))
        self._groups = []
        for type_name in dict.fromkeys(t.type for t in world.tasks):
            idx = np.array([k for k, t in enumerate(world.tasks) if t.type == type_name], dtype=int)
            evaluator = TASK_TYPES.get(type_name)([world.tasks[k] for k in idx], world)
            self.targets[idx] = evaluator.targets
            self._groups.append((idx, evaluator))
        self.reset()

    def reset(self) -> None:
        m = len(self.task_ids)
        self.status = np.full(m, TaskStatus.ACTIVE, dtype=int)
        self.reason = np.full(m, FailureReason.NONE, dtype=int)
        for _, evaluator in self._groups:
            evaluator.reset()

    @property
    def all_done(self) -> bool:
        return bool(np.isin(self.status, (TaskStatus.SUCCEEDED, TaskStatus.FAILED)).all())

    def goals(self) -> tuple[np.ndarray, np.ndarray]:
        """Per drone: target (n, 3) and whether it has a goal to fly to, (n,) bool (not after a failure)."""
        target, active = np.zeros((self.n, 3)), np.zeros(self.n, dtype=bool)
        alive = self.status != TaskStatus.FAILED
        target[self.drone[alive]] = self.targets[alive]
        active[self.drone[alive]] = True
        return target, active

    def update(self, state: FleetState, collisions: np.ndarray, t: float) -> np.ndarray:
        """Advance every task to time ``t``; returns the indices of tasks whose status changed."""
        before = self.status.copy()
        active = self.status == TaskStatus.ACTIVE
        d = self.drone
        failures = [
            (FailureReason.COLLISION, collisions[d]),
            (FailureReason.CRASH, state.R[d, 2, 2] < self.min_up_z),
            (FailureReason.OUT_OF_BOUNDS, outside_arena(state.pos[d], self.arena_size)),
            (FailureReason.TIMEOUT, t > self.timeout),
        ]
        for reason, hit in failures:
            fail = active & hit
            self.status[fail], self.reason[fail] = TaskStatus.FAILED, reason
            active &= ~fail
        for idx, evaluator in self._groups:
            sub = FleetState(*(a[d[idx]] for a in (state.pos, state.quat, state.R, state.vel, state.omega)))
            done = evaluator.update(sub, t, active[idx])
            self.status[idx[done]] = TaskStatus.SUCCEEDED
        return np.flatnonzero(self.status != before)
