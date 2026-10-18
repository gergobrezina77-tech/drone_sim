# DS-026: physics: spatial prefilter for drone-drone collisions

- **Status:** Ready
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

- [ ] The prefiltered check gives the same flags as the brute-force one on random fleets (property test).
- [ ] `collisions()` for 500 drones is at least 10x faster than before (23 ms measured in DS-008).
- [ ] `scripts/bench.py` numbers re-recorded in the notes.
- [ ] `scripts/check.sh` passes

## Notes

Split out of DS-013 (engine) because the fix is in the physics layer.
