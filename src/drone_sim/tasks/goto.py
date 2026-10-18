"""goto_point: reach a target and stay within ``tolerance`` of it for ``dwell`` seconds.

Params: ``target`` [x, y, z] (required), ``tolerance`` m (0.3), ``dwell`` s (1.0); ``timeout`` is shared, see board.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from drone_sim.core.state import FleetState
from drone_sim.tasks.base import outside_arena
from drone_sim.world import TaskSpec, WorldSpec


class GotoPoint:
    def __init__(self, specs: Sequence[TaskSpec], world: WorldSpec):
        for s in specs:
            target = s.params.get("target")
            if target is None or len(target) != 3:
                raise ValueError(f"task {s.id!r}: goto_point needs a 'target' [x, y, z]")
            if outside_arena(np.array([target], dtype=float), world.arena_size)[0]:
                raise ValueError(f"task {s.id!r}: target {list(target)} is outside the arena")
        self.targets = np.array([s.params["target"] for s in specs], dtype=float).reshape(-1, 3)
        self.tolerance = np.array([s.params.get("tolerance", 0.3) for s in specs], dtype=float)
        self.dwell = np.array([s.params.get("dwell", 1.0) for s in specs], dtype=float)
        self.reset()

    def reset(self) -> None:
        self._inside_since = np.full(len(self.targets), np.nan)

    def update(self, state: FleetState, t: float, active: np.ndarray) -> np.ndarray:
        inside = np.linalg.norm(state.pos - self.targets, axis=1) < self.tolerance
        self._inside_since[inside & np.isnan(self._inside_since)] = t
        self._inside_since[~inside] = np.nan
        return active & inside & (t - self._inside_since >= self.dwell)
