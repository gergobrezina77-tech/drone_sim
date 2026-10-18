# DS-024: core: FleetState shared runtime state

- **Status:** Done
- **Layer:** core
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-002

## Goal

The batched drone state (pos, quat, R, vel, omega) is produced by physics and read by environment, autonomy and
tasks. Those four are sibling layers that may not import each other, so the type must live below them, in core.

## Scope

- `core/state.py`: `FleetState` (struct-of-arrays, `n` property), moved from legacy `drone.DroneState`.
- Legacy `drone.DroneState` becomes an alias, so existing code and tests are unchanged.

## Acceptance criteria

- [x] `FleetState` is importable from `drone_sim.core` and `drone.DroneState is FleetState`.
  (`tests/test_core.py::test_fleet_state_is_shared_with_the_legacy_name`)
- [x] `scripts/check.sh` passes (123 passed; all legacy tests unchanged)

## Notes

- Split out of DS-007, whose scope placed `FleetState` in physics, which siblings cannot import.
- Class moved verbatim; `drone.DroneState = FleetState`. `drone_sim.core` now exports `FleetState`.
