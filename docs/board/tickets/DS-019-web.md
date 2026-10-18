# DS-019: web: plan view editor

- **Status:** Backlog
- **Layer:** web
- **Milestone:** M2 Server and web UI
- **Depends on:** DS-018

## Goal

Build worlds in a top-down view while stopped.

## Scope

- Edit/Run mode switch; orthographic top-down camera in Edit.
- Tools: place drone, draw box (drag rectangle + height), draw pillar (drag radius + height), select, move, delete, undo/redo — all sent as DS-015 commands, so validation lives server-side.
- Grid snapping.

## Acceptance criteria

- [ ] Manual: build a scene, run, reset, edit, rerun; mode switch feels instant.
- [ ] Tool state machine unit-tested.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
