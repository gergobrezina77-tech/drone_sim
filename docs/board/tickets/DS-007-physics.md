# DS-007: physics: PhysicsBackend interface and fleet state

- **Status:** Done
- **Layer:** physics
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-004, DS-024

## Goal

Physics behind an interface so MuJoCo can be replaced or complemented later.

## Scope

- Uses `core.FleetState` (DS-024): pos, quat, R, vel, omega (all `(n, …)`).
- `PhysicsBackend` protocol: `build(world, vehicles)`, `reset()`, `state() -> FleetState`, `set_thrusts((n,4))`, `set_external_forces((n,3))`, `step(substeps)`, `collisions() -> bool (n,)`, `freeze(mask)`, `timestep`.
- Backend registry by name.

## Acceptance criteria

- [x] Interface documented; a fake in-memory backend in tests satisfies the protocol.
  (`physics/backend.py` docstring states the contract; `tests/fake_physics.py` = `test_kinematic` backend;
  `tests/test_physics_contract.py::test_satisfies_the_protocol` + 9 behavioural contract tests)
- [x] `physics` imports nothing from `autonomy`, `tasks`, `environment` (contract).
  (layer contract: siblings are independent; `lint-imports` KEPT)
- [x] `scripts/check.sh` passes (133 passed)

## Notes

- `physics/backend.py`: `PhysicsBackend` (runtime-checkable Protocol) and `PHYSICS_BACKENDS`, a registry of
  zero-argument factories. Signature is `build(world)`: vehicle params are looked up from the world's vehicle ids,
  so no separate `vehicles` argument is needed.
- Semantics pinned down in the docstring: thrusts/forces are held until changed; collisions = touching an obstacle
  or another drone within the footprint radius, floor contact excluded; frozen drones hold pose, zero velocity, are
  never flagged and ignored by others; reset clears everything; deterministic.
- `tests/test_physics_contract.py` is parametrised over `PHYSICS_BACKENDS.names()`, so the MuJoCo backend (DS-008)
  is held to the same 10 tests automatically once registered.
- The fake is a point mass (thrust along body z, no rotation dynamics, flat floor at the start height). It lives in
  `tests/` and stays usable for fast engine tests (DS-012).
