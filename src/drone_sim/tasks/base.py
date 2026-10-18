"""Task statuses, failure reasons, and the TaskType plug-in interface.

A task type only defines what success means and where its drones should fly; the failure checks shared by every
task (collision, crash, leaving the arena, timeout) live in ``TaskBoard``.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from enum import IntEnum
from typing import Protocol

import numpy as np

from drone_sim.core.registry import Registry
from drone_sim.core.state import FleetState
from drone_sim.world import TaskSpec, WorldSpec


class TaskStatus(IntEnum):
    PENDING = 0
    ACTIVE = 1
    SUCCEEDED = 2
    FAILED = 3


class FailureReason(IntEnum):
    NONE = 0
    COLLISION = 1
    CRASH = 2
    OUT_OF_BOUNDS = 3
    TIMEOUT = 4


class TaskEvaluator(Protocol):
    """Batched over all tasks of one type. Arrays are indexed by task within this evaluator."""

    targets: np.ndarray  # (k, 3) where each task's drone should fly

    def reset(self) -> None: ...

    def update(self, state: FleetState, t: float, active: np.ndarray) -> np.ndarray:
        """``state`` holds the k tasks' drones; returns (k,) bool: succeeded this tick (only where ``active``)."""
        ...


TaskType = Callable[[Sequence[TaskSpec], WorldSpec], TaskEvaluator]
TASK_TYPES = Registry[TaskType]("task type")


def outside_arena(pos: np.ndarray, arena_size) -> np.ndarray:
    """(n,) bool for positions outside the arena box (x, y centred on the origin; floor at z = 0)."""
    sx, sy, height = arena_size
    return (np.abs(pos[:, 0]) > sx / 2) | (np.abs(pos[:, 1]) > sy / 2) | (pos[:, 2] > height) | (pos[:, 2] < 0.0)
