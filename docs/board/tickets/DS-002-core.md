# DS-002: core: math and batched geometry primitives

- **Status:** Done
- **Layer:** core
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-001

## Goal

Shared, dependency-free math and geometry that every layer can use, vectorised for large fleets.

## Scope

- Move `quat_to_mat`, `rot_z` into `core/math.py`; keep old names importable from legacy modules.
- `core/geometry.py`: `Box` and `Cylinder` shapes, and a batched `distance(points (n,3), shapes) -> (dist (n,m), normal (n,m,3))` built from today's `Obstacle.closest_point/distance` logic.
- `core/rng.py`: seeded `numpy.random.Generator` factory for deterministic runs.

## Acceptance criteria

- [x] Unit tests: distance outside, on surface and inside each shape; normals are unit length.
  (`tests/test_core.py::test_distance_and_normal` [9 cases], `::test_inside_at_exact_centre_still_gives_a_unit_normal`)
- [x] Batched result equals the per-shape loop result (property test on random points).
  (`tests/test_core.py::test_batched_distance_matches_reference_on_random_points`: 300 seeded random points x 25
  shapes against a naive one-point-one-shape reference written in the test)
- [x] Micro-benchmark in the test: 1000 points × 200 shapes evaluates well under 50 ms.
  (`tests/test_core.py::test_batched_distance_is_fast_for_large_scenes`; measured 23 ms best-of-5, 100 points 1.7 ms)
- [x] `scripts/check.sh` passes (72 passed)

## Notes

- Module is `core/rotations.py`, not `core/math.py`, so it does not share a name with the stdlib `math` module.
- `core/geometry.py`: `Box`, `Cylinder`, `closest_points`, `distance`. One vectorised pass per shape type; shape
  order is preserved in the output columns. Points on or inside a shape get distance 0 and a horizontal normal away
  from the shape's axis; exactly on the axis falls back to +x so normals are always unit length (the old code
  returned a zero vector there).
- `core/rng.py`: `rng_stream(seed, name)` gives each subsystem its own stream (seed + crc32 of the name), so one
  subsystem's draws never shift another's. Tested.
- Legacy: `drone.quat_to_mat` and `planner.rot_z` now import from core; `scene.Obstacle` delegates
  `closest_point`/`distance` to `core.geometry` via a new `shape` property, removing the duplicate algorithm.
  Existing obstacle-avoidance tests still pass.
- Performance note for DS-008: 23 ms for 1000x200 is fine for a check but too slow per tick at that scale.
  Collision checks there should only query nearby shapes (e.g. a coarse grid or bounding-box prefilter).
