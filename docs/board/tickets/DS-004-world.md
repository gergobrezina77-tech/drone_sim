# DS-004: world: WorldSpec data model and JSON I/O

- **Status:** Done
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

- [x] Round-trip test: spec → JSON → spec is equal.
  (`tests/test_world_spec.py::test_json_round_trip_is_lossless`, `::test_empty_world_round_trips`)
- [x] Unknown task types and params survive a round trip untouched.
  (`::test_unknown_task_types_and_params_survive_a_round_trip`)
- [x] Moving a drone's start pose or editing a task does not change `geometry_hash`; adding an obstacle does.
  (`::test_geometry_hash_ignores_start_poses_and_tasks`, `::test_geometry_hash_changes_with_what_the_physics_model_depends_on`)
- [x] `scripts/check.sh` passes (86 passed). Also: `::test_wrong_schema_version_is_rejected`.

## Notes

- `world/spec.py`. Specs are frozen dataclasses with tuple collections: values with exact equality, edited by
  producing new specs (DS-005 undo can keep snapshots).
- Obstacles reuse `core.geometry` `Box` / `Cylinder` as their shape; JSON tags them `"type": "box" | "cylinder"`.
- `DroneSpec` start pose = position + yaw (drones start level); `vehicle` defaults to `quad_x_500g`.
- `environment` is `{effect_name: params}`, free-form like task params, so new effects need no schema change.
- `arena_size` = (x extent, y extent, height); centred on the origin, floor at z = 0.
- `geometry_hash` = sha256 of arena + obstacles + per-drone vehicle ids (order matters: drone i's model).
  Excludes start poses, tasks and environment.
- Not validated here (ids unique, references valid): that is DS-005's validation.
