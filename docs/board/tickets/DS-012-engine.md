# DS-012: engine: simulation state machine and tick pipeline

- **Status:** Ready
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

- [ ] End-to-end headless test: one drone, one goto_point task, no obstacles → SUCCEEDED.
- [ ] Same with a wall in the way → FAILED(COLLISION); reset → EDIT with original poses.
- [ ] Determinism: two runs with the same seed give identical event streams.
- [ ] Illegal transitions raise.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
