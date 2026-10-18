"""Events emitted by a run, and the end-of-run report."""

from __future__ import annotations

from dataclasses import dataclass

from drone_sim.tasks import FailureReason, TaskStatus


@dataclass(frozen=True)
class TaskSucceeded:
    time: float
    task_id: str
    drone_id: str


@dataclass(frozen=True)
class TaskFailed:
    time: float
    task_id: str
    drone_id: str
    reason: FailureReason


@dataclass(frozen=True)
class DroneCollided:
    time: float
    drone_id: str


@dataclass(frozen=True)
class RunFinished:
    time: float


Event = TaskSucceeded | TaskFailed | DroneCollided | RunFinished


@dataclass(frozen=True)
class TaskOutcome:
    task_id: str
    drone_id: str
    status: TaskStatus
    reason: FailureReason
    time: float | None  # when it ended; None while still active


@dataclass(frozen=True)
class RunReport:
    time: float
    outcomes: tuple[TaskOutcome, ...]
