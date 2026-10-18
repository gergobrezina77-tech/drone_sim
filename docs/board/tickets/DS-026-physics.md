# DS-026: physics: spatial prefilter for drone-drone collisions

- **Status:** Done
- **Layer:** physics
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-013

## Goal

`MujocoBackend.collisions()` is 77 % of the tick at 500 drones (DS-013 profile): the drone-drone check builds an
n x n distance matrix. Make it near-linear so large fleets run in real time.

## Scope

- Replace the n x n drone-drone check with a spatial prefilter (sort-and-sweep along one axis or a uniform cell
  grid), computing exact distances only for candidate pairs. Same results.
- No change to the backend interface or the contract tests.

## Acceptance criteria

- [x] The prefiltered check gives the same flags as the brute-force one on random fleets (property test).
  (`tests/test_physics_collision.py::test_matches_brute_force_on_random_fleets`, 20 seeded fleets incl. shared x
  coordinates and exact duplicates; plus the 10 backend contract tests, unchanged)
- [x] `collisions()` for 500 drones is at least 10x faster than before (23 ms measured in DS-008).
  (1.18 ms in the benchmark world = 19.5x; `::test_500_drones_on_a_grid_is_fast` guards the pair check)
- [x] `scripts/bench.py` numbers re-recorded in the notes.
- [x] `scripts/check.sh` passes (225 passed)

## Notes

- Split out of DS-013 (engine) because the fix is in the physics layer.
- New `physics/collision.py`: `aabb`, `touching_obstacles` (bounding-box prefilter, moved out of the backend) and
  `touching_pairs` (sort-and-sweep along the widest axis; the sweep stops at the first neighbour offset with no
  candidate, which is safe because sorted gaps only grow with the offset). Backend `collisions()` combines them and
  drops pairs involving frozen drones.
- Found while reviewing: an empty sweep would have crashed `np.concatenate([])`; fixed and covered by
  `::test_far_apart_drones_give_no_pairs`.
- Benchmark after (before in DS-013):

  | drones | compile ms | restart ms | tick ms | x real time |
  | --- | --- | --- | --- | --- |
  | 10 | 20.2 | 1.62 | 1.307 (1.290) | 3.06 (3.10) |
  | 100 | 31.0 | 1.58 | 2.182 (2.019) | 1.83 (1.98) |
  | 500 | 365.1 | 6.78 | **5.466** (17.398) | **0.73** (0.23) |

  100-drone tick is within run-to-run noise. At 500 drones the top cost is now `mj_step` itself (0.50 s of 2.0 s
  profiled), then the collision checks (0.32 + 0.16 s) and `quat_to_mat` (0.13 s).
