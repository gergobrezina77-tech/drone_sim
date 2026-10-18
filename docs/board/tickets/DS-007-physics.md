# DS-007: physics: PhysicsBackend interface and fleet state

- **Status:** Backlog
- **Layer:** physics
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-004

## Goal

Physics behind an interface so MuJoCo can be replaced or complemented later.

## Scope

- `FleetState` struct-of-arrays: pos, quat, R, vel, omega (all `(n, …)`).
- `PhysicsBackend` protocol: `build(world, vehicles)`, `reset()`, `state() -> FleetState`, `set_thrusts((n,4))`, `set_external_forces((n,3))`, `step(substeps)`, `collisions() -> bool (n,)`, `freeze(mask)`, `timestep`.
- Backend registry by name.

## Acceptance criteria

- [ ] Interface documented; a fake in-memory backend in tests satisfies the protocol.
- [ ] `physics` imports nothing from `autonomy`, `tasks`, `environment` (contract).
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
