# DS-012: engine: simulation state machine and tick pipeline

- **Status:** Done
- **Layer:** engine
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-008, DS-009, DS-010, DS-011

## Goal

Wire the layers together behind EDIT/RUNNING/PAUSED/FINISHED.

## Scope

- `Simulation(world_editor, backend=..., software=...)` with `start()`, `pause()`, `resume()`, `reset()`, `tick()`, `state`.
- Start compiles through the physics cache; reset returns to EDIT with the authored start setup.
- Tick pipeline as in ARCHITECTURE.md; a collision fails the drone's task and freezes it.
- Event list (TaskSucceeded, TaskFailed(reason), RunFinished) and a run report.
- Editing is rejected unless in EDIT.

## Acceptance criteria

- [x] End-to-end headless test: one drone, one goto_point task, no obstacles → SUCCEEDED.
  (`tests/test_engine.py::test_a_drone_flies_to_its_point_and_the_task_succeeds`, MuJoCo, from the ground)
- [x] Same with a wall in the way → FAILED(COLLISION); reset → EDIT with original poses.
  (`::test_a_wall_in_the_way_fails_the_task_by_collision_and_freezes_the_drone`,
  `::test_reset_returns_to_edit_with_the_authored_start_setup`,
  `::test_edit_after_reset_then_rerun_without_the_wall_succeeds`)
- [x] Determinism: two runs with the same seed give identical event streams.
  (`::test_same_seed_gives_identical_event_streams_with_turbulence`: two drones, turbulent wind)
- [x] Illegal transitions raise. (`::test_illegal_transitions_from_edit_raise` [4],
  `::test_state_machine_cycle_and_illegal_transitions_while_running`)
- [x] `scripts/check.sh` passes (198 passed). Also: invalid task setup stays in EDIT, task-less worlds run until
  stopped, restart reuses the compiled model.

## Notes

- `engine/simulation.py`: `Simulation(editor, physics="mujoco", software="goto", control_dt=0.004, seed=0)`,
  `SimState`, `IllegalTransition`; `engine/events.py`: `TaskSucceeded`, `TaskFailed(reason)`, `DroneCollided`,
  `RunFinished`, `RunReport` / `TaskOutcome`.
- The editor is private; `edit` / `undo` / `redo` only in EDIT. `start` builds the `TaskBoard` first, so an invalid
  task setup raises and stays in EDIT. Backends and software are chosen by registry name.
- Tick: goals from the board -> software -> thrusts; effects -> external forces (held over the control period's
  substeps) -> physics substeps -> collisions -> board update -> freeze drones that collided or whose task failed
  (any reason) -> events. Order on a crash tick: `DroneCollided`, `TaskFailed`, then `RunFinished`.
- A drone that collides is frozen even if it has no task. Runs finish when every task is terminal; a world without
  tasks runs until stopped.
- `fleet_state()` returns the authored start poses in EDIT (for the UI's plan view), live state otherwise.
