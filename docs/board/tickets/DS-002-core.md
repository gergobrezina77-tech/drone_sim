# DS-002: core: math and batched geometry primitives

- **Status:** Ready
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

- [ ] Unit tests: distance outside, on surface and inside each shape; normals are unit length.
- [ ] Batched result equals the per-shape loop result (property test on random points).
- [ ] Micro-benchmark in the test: 1000 points × 200 shapes evaluates well under 50 ms.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
