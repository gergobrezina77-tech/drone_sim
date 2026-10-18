"""The simulation: editor mode vs play mode, and the fixed-step tick pipeline that wires the layers together.

EDIT --start--> RUNNING <--pause/resume--> PAUSED;  RUNNING --all tasks done--> FINISHED;  any --reset--> EDIT.

The world is only edited in EDIT. ``start`` builds a run from it (physics through its compile cache, flight
software, effects, task board); ``reset`` throws the run away, so the next start begins from the world exactly as
authored. The physics backend instance is kept across runs, so unchanged geometry never recompiles.
"""

from __future__ import annotations

from enum import Enum

import numpy as np

from drone_sim.autonomy import FLIGHT_SOFTWARE, Goals, Observation
from drone_sim.core.state import FleetState
from drone_sim.engine.events import (
    DroneCollided,
    Event,
    RunFinished,
    RunReport,
    TaskFailed,
    TaskOutcome,
    TaskSucceeded,
)
from drone_sim.environment import EffectStack
from drone_sim.physics import PHYSICS_BACKENDS
from drone_sim.tasks import FailureReason, TaskBoard, TaskStatus
from drone_sim.world import Command, WorldEditor, WorldSpec


class SimState(Enum):
    EDIT = "edit"
    RUNNING = "running"
    PAUSED = "paused"
    FINISHED = "finished"


class IllegalTransition(RuntimeError):
    pass


class Simulation:
    def __init__(
        self,
        editor: WorldEditor | None = None,
        physics: str = "mujoco",
        software: str = "goto",
        control_dt: float = 0.004,
        seed: int = 0,
    ):
        self._editor = editor or WorldEditor()
        self.physics = PHYSICS_BACKENDS.get(physics)()
        self._software_name = software
        self.substeps = max(1, round(control_dt / self.physics.timestep))
        self.control_dt = self.substeps * self.physics.timestep
        self.seed = seed
        self.state = SimState.EDIT
        self.events: list[Event] = []

    # --- editing (EDIT only) -------------------------------------------------------------------------------------

    @property
    def world(self) -> WorldSpec:
        return self._editor.world

    def edit(self, command: Command) -> None:
        self._require(SimState.EDIT, "edit")
        self._editor.apply(command)

    def undo(self) -> bool:
        self._require(SimState.EDIT, "undo")
        return self._editor.undo()

    def redo(self) -> bool:
        self._require(SimState.EDIT, "redo")
        return self._editor.redo()

    # --- transitions ---------------------------------------------------------------------------------------------

    def start(self) -> None:
        self._require(SimState.EDIT, "start")
        world = self.world
        board = TaskBoard(world)  # validates task setups first: a bad one leaves us in EDIT
        self.physics.build(world)
        self.software = FLIGHT_SOFTWARE.get(self._software_name)()
        self.software.reset(world)
        self.effects = EffectStack.from_world(world)
        self.effects.reset(self.seed)
        self.board = board
        self._drone_ids = [d.id for d in world.drones]
        self._collided = np.zeros(len(world.drones), dtype=bool)
        self._ended_at = np.full(len(world.tasks), np.nan)
        self.events = []
        self.state = SimState.RUNNING

    def pause(self) -> None:
        self._require(SimState.RUNNING, "pause")
        self.state = SimState.PAUSED

    def resume(self) -> None:
        self._require(SimState.PAUSED, "resume")
        self.state = SimState.RUNNING

    def reset(self) -> None:
        if self.state is SimState.EDIT:
            raise IllegalTransition("reset: already in edit mode")
        self.events = []
        self.state = SimState.EDIT

    def _require(self, state: SimState, action: str) -> None:
        if self.state is not state:
            raise IllegalTransition(f"{action}: needs {state.value} mode, simulation is {self.state.value}")

    # --- running -------------------------------------------------------------------------------------------------

    @property
    def time(self) -> float:
        return 0.0 if self.state is SimState.EDIT else self.physics.time

    def tick(self) -> None:
        """Advance one control period."""
        self._require(SimState.RUNNING, "tick")
        physics, s = self.physics, self.physics.state()
        self.software.set_goals(Goals(*self.board.goals()))
        physics.set_thrusts(self.software.step(Observation(s, physics.time), self.control_dt))
        physics.set_external_forces(self.effects.step(self.control_dt, s))
        physics.step(self.substeps)

        s, hits, t = physics.state(), physics.collisions(), physics.time
        changed = self.board.update(s, hits, t)
        new_hits = hits & ~self._collided
        self._collided |= hits
        self.events += [DroneCollided(t, self._drone_ids[i]) for i in np.flatnonzero(new_hits)]
        failed = np.zeros(physics.n, dtype=bool)
        for k in changed:
            self._ended_at[k] = t
            task_id, drone = self.board.task_ids[k], int(self.board.drone[k])
            if self.board.status[k] == TaskStatus.SUCCEEDED:
                self.events.append(TaskSucceeded(t, task_id, self._drone_ids[drone]))
            else:
                self.events.append(TaskFailed(t, task_id, self._drone_ids[drone], FailureReason(self.board.reason[k])))
                failed[drone] = True
        if (new_hits | failed).any():
            physics.freeze(new_hits | failed)
        if len(self.board.task_ids) and self.board.all_done:
            self.events.append(RunFinished(t))
            self.state = SimState.FINISHED

    def run_until_finished(self, max_seconds: float) -> None:
        for _ in range(round(max_seconds / self.control_dt)):
            if self.state is not SimState.RUNNING:
                return
            self.tick()

    # --- outputs -------------------------------------------------------------------------------------------------

    def fleet_state(self) -> FleetState:
        """Live state while a run exists; the authored start poses (at rest) in EDIT."""
        if self.state is not SimState.EDIT:
            return self.physics.state()
        drones = self.world.drones
        n = len(drones)
        yaw = np.array([d.yaw for d in drones], dtype=float)
        quat = np.stack([np.cos(yaw / 2), 0 * yaw, 0 * yaw, np.sin(yaw / 2)], axis=1)
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.zeros((n, 3, 3))
        R[:, 0, 0], R[:, 0, 1], R[:, 1, 0], R[:, 1, 1], R[:, 2, 2] = c, -s, s, c, 1.0
        pos = np.array([d.position for d in drones], dtype=float).reshape(n, 3)
        return FleetState(pos, quat, R, np.zeros((n, 3)), np.zeros((n, 3)))

    def report(self) -> RunReport:
        if self.state is SimState.EDIT:
            raise IllegalTransition("report: no run in edit mode")
        b = self.board
        outcomes = tuple(
            TaskOutcome(
                b.task_ids[k],
                self._drone_ids[b.drone[k]],
                TaskStatus(b.status[k]),
                FailureReason(b.reason[k]),
                None if np.isnan(self._ended_at[k]) else float(self._ended_at[k]),
            )
            for k in range(len(b.task_ids))
        )
        return RunReport(self.physics.time, outcomes)
