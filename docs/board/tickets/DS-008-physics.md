# DS-008: physics: MuJoCo backend with model cache and collision flags

- **Status:** Done
- **Layer:** physics
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-007, DS-006

## Goal

Fast, scalable MuJoCo implementation of the backend.

## Scope

- MJCF generation from `WorldSpec` (reuse `scene.py` logic).
- Contact filtering: drones collide only with the floor in the solver (cheap, no bounce); drone–obstacle and drone–drone collisions are detected geometrically with `core.geometry` and reported via `collisions()`.
- Compiled-model cache keyed by `geometry_hash`: restart with unchanged geometry = `mj_resetData` + write start poses, no recompile.
- `freeze(mask)`: failed drones stop (zero thrust, held in place) and drop out of the dynamics.

## Acceptance criteria

- [x] Hover test: a drone given hover thrust stays within 1 cm over 2 s.
  (`tests/test_physics_mujoco.py::test_open_loop_hover_holds_position_within_a_centimetre_for_two_seconds`)
- [x] Collision test: a drone flown into a box is flagged on the right tick.
  (`::test_collision_is_flagged_on_the_step_the_footprint_reaches_the_obstacle`: within 1.5 physics steps of the
  analytic contact time; plus `::test_no_bounce_drones_pass_through_obstacles_in_the_solver`)
- [x] Restart with unchanged geometry < 20 ms for 100 drones (measured in test, generous bound).
  (`::test_restart_with_unchanged_geometry_is_fast_and_does_not_recompile`; measured 0.83 ms)
- [x] Moving a drone's start pose and restarting does not recompile.
  (same test; `::test_changed_geometry_recompiles` for the converse)
- [x] `scripts/check.sh` passes (151 passed). The 10 contract tests now also run against `mujoco`.

## Notes

- `physics/mjcf.py` builds MJCF from `WorldSpec` (ported from legacy `scene.py`, not imported: layers must not
  depend on legacy code; DS-014 can point `scene.py` at this). `physics/mujoco_backend.py`: `MujocoBackend`,
  registered as `mujoco`.
- Solver contacts are drone <-> floor only (contype/conaffinity bits); obstacles are visual. Drone-obstacle and
  drone-drone contact is geometric: AABB prefilter, then `core.geometry.distance` only for nearby pairs; drones
  pairwise.
- Start poses are written into qpos on reset (bodies sit at the origin in the MJCF), so a new start pose never
  recompiles. Cache key = `world.geometry_hash()`; `compile_count` exposed for tests.
- **Performance finding:** rotor-site `motor` actuators made a MuJoCo step quadratic in fleet size (transmission
  moments are O(actuators x dofs)): 500 drones = 21.8 ms/step, 18.6 ms of it in kinematics. Replaced by applying
  each drone's thrust as a body wrench every substep (`[T, tau] = allocation @ f`, rotated by the current attitude),
  physically equivalent and linear: 500 drones = 1.18 ms/step.
- Measurements (this machine): compile 51 ms (100 drones) / 668 ms (500); restart 0.83 ms / 17 ms; physics step
  0.28 ms (7.0x real time) / 1.18 ms (1.7x real time).
- For DS-013: `collisions()` costs 0.7 ms at 100 drones but 23 ms at 500, dominated by the O(n^2) drone-drone
  check. Needs a spatial prefilter (sort-and-sweep or a cell grid).
