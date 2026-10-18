# DS-006: world: SceneConstructor interface + plan2d constructor

- **Status:** Done
- **Layer:** world
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-005, DS-022

## Goal

Make the way a scene is authored swappable; ship the simple plan-view method first.

## Scope

- `SceneConstructor` protocol: turns high-level authoring input into `WorldEditor` commands.
- Registry by name; `plan2d` implementation: rectangle footprint + height → box obstacle, circle footprint + height → pillar; place drone at (x, y) on the ground.
- A trivial second constructor in tests (e.g. `grid_generator`) proves another method plugs in.

## Acceptance criteria

- [x] `plan2d` inputs produce the expected `ObstacleSpec`s (tests).
  (`tests/test_world_constructors.py::test_plan2d_box_from_a_dragged_rectangle_in_any_direction`,
  `::test_plan2d_pillar_from_a_circle`, `::test_plan2d_drone_is_placed_on_the_ground`,
  `::test_generated_ids_skip_ids_already_in_use`, `::test_plan2d_rejects_bad_actions` [4 cases])
- [x] Swappability test: both constructors used through the registry with no changes to callers.
  (`::test_a_different_construction_method_plugs_in_by_name`: a test-only grid generator, same `run()` helper)
- [x] `scripts/check.sh` passes (122 passed)

## Notes

- `world/constructors.py`: `SceneConstructor.build(action: dict, world) -> Command`; `SCENE_CONSTRUCTORS` registry
  (`core.Registry`); `plan2d` registered. Actions are plain dicts (the shape the UI will send in M2).
- `plan2d` ops: `box` (two dragged corners + height, any drag direction), `pillar` (centre, radius, height),
  `drone` (x, y, optional yaw / vehicle). Optional `id`, otherwise `new_id(world, prefix)` = first unused `box1`,
  `box2`, ...
- Added `Batch` to the editor: several commands as one edit, validated once, undone in one step
  (`::test_a_batch_is_validated_as_a_whole`). Needed so a procedural constructor is one undoable action.
- Drones are placed at `DRONE_REST_Z = 0.07`, a constant matching legacy `scene.REST_Z`; moving it onto the
  vehicle is DS-023 (kept out of this ticket to stay in the world layer).
