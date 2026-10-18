# DS-004: world: WorldSpec data model and JSON I/O

- **Status:** Ready
- **Layer:** world
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-003

## Goal

The single editable, serialisable description of a scenario.

## Scope

- Dataclasses: `WorldSpec` (schema_version, arena size, obstacles, drones, tasks, environment settings), `ObstacleSpec` (id, shape), `DroneSpec` (id, vehicle id, start pose), `TaskSpec` (id, drone id, `type: str`, `params: dict`) — task params stay generic so new task types need no schema change.
- `to_json` / `from_json` with schema version check.
- `geometry_hash()` covering only what the compiled physics model depends on (obstacles, drone count and vehicle types) — used later for model caching.

## Acceptance criteria

- [ ] Round-trip test: spec → JSON → spec is equal.
- [ ] Unknown task types and params survive a round trip untouched.
- [ ] Moving a drone's start pose or editing a task does not change `geometry_hash`; adding an obstacle does.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
