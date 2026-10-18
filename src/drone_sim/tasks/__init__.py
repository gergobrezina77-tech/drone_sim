"""Tasks: task types and their batched evaluators, deciding success or failure (with a reason)
independently of the drone software.

May import: world, vehicle, core. Must not import physics, environment or autonomy.
"""

from drone_sim.tasks.base import TASK_TYPES, FailureReason, TaskEvaluator, TaskStatus, TaskType, outside_arena
from drone_sim.tasks.board import TaskBoard
from drone_sim.tasks.goto import GotoPoint

TASK_TYPES.register("goto_point", GotoPoint)

__all__ = [
    "TASK_TYPES",
    "FailureReason",
    "GotoPoint",
    "TaskBoard",
    "TaskEvaluator",
    "TaskStatus",
    "TaskType",
    "outside_arena",
]
