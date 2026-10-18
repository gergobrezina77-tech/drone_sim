# DS-008: physics: MuJoCo backend with model cache and collision flags

- **Status:** Ready
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

- [ ] Hover test: a drone given hover thrust stays within 1 cm over 2 s.
- [ ] Collision test: a drone flown into a box is flagged on the right tick.
- [ ] Restart with unchanged geometry < 20 ms for 100 drones (measured in test, generous bound).
- [ ] Moving a drone's start pose and restarting does not recompile.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
