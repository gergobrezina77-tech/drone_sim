# DS-005: world: editing commands with undo/redo and validation

- **Status:** Backlog
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

- [ ] Tests: apply/undo/redo sequences restore the exact previous spec.
- [ ] Removing a drone also removes its tasks, and undo restores both.
- [ ] Each validation rule has a failing-case test.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
