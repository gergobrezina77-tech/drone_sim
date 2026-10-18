# DS-005: world: editing commands with undo/redo and validation

- **Status:** Done
- **Layer:** world
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-004

## Goal

All world edits go through commands so the UI gets undo/redo and validation for free.

## Scope

- `WorldEditor` holding a `WorldSpec` with `apply(cmd)`, `undo()`, `redo()`.
- Commands: PlaceDrone, AddObstacle, MoveEntity, RemoveEntity, AssignTask, RemoveTask.
- Validation: inside arena, drone start not inside an obstacle, task references an existing drone; invalid commands raise and leave the spec unchanged.

## Acceptance criteria

- [x] Tests: apply/undo/redo sequences restore the exact previous spec.
  (`tests/test_world_editor.py::test_undo_and_redo_restore_exact_snapshots`, `::test_a_new_edit_clears_the_redo_history`)
- [x] Removing a drone also removes its tasks, and undo restores both.
  (`::test_removing_a_drone_removes_its_tasks_and_undo_restores_both`)
- [x] Each validation rule has a failing-case test.
  (`::test_invalid_edits_are_rejected_without_changing_the_world`, 16 cases: duplicate id, outside arena
  (drone xy / z, box, cylinder), inside obstacle (centre, rotor clearance, new obstacle over a drone, move into a
  pillar), unknown vehicle, unknown drone, unknown entity / task. Each also checks the rejected edit is not
  recorded in the undo history.)
- [x] `scripts/check.sh` passes (108 passed)

## Notes

- `world/editor.py`: commands are frozen dataclasses with a pure `apply(world) -> world`; `WorldEditor` validates
  the result before recording it. Undo/redo are snapshot stacks (cheap: specs are immutable and share structure).
- `validate(world)` checks the whole world each time and returns all messages; `InvalidEdit` joins them.
  The editor also validates its initial world.
- Ids are unique across drones, obstacles and tasks, so `MoveEntity` / `RemoveEntity` address any entity by id.
- "Inside obstacle" uses the vehicle's footprint `radius` (rotor tips) as clearance, via `core.geometry.distance`.
- Obstacles must fit entirely inside the arena (bounding box), not just their reference point.
- Not here: task-type-specific checks (e.g. a goto target inside the arena). The world layer cannot import `tasks`;
  that belongs to the task types (DS-011) or the engine when compiling a run.
