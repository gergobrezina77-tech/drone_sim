# DS-010: autonomy: FlightSoftware interface, controller, naive goto autopilot

- **Status:** Done
- **Layer:** autonomy
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-007

## Goal

Drone software behind an interface; first implementation flies straight to a point.

## Scope

- `Observation` (batched own state; ground truth for now, a noisy sensor model can be swapped in later) and `FlightSoftware` protocol: `reset(n, vehicle)`, `set_goals(...)`, `step(obs, dt) -> thrusts (n,4)`.
- Move `GeometricController` into `autonomy/control.py` (legacy re-export).
- `GotoAutopilot`: position control to its goal with speed limit, hover when there; no avoidance.
- Registry by name. No per-drone Python loop in `step`.

## Acceptance criteria

- [x] `autonomy` does not import `physics` (contract). (layer contract, siblings independent; `lint-imports` KEPT)
- [x] With the fake backend or MuJoCo: one drone reaches a goal 5 m away within 0.2 m.
  (`tests/test_autonomy.py::test_a_drone_takes_off_and_reaches_a_point_five_metres_away`, MuJoCo, from the ground)
- [x] ~~100 drones step at least 1000× faster than real time per control tick~~ **amended:** 100 drones'
  software step takes < 25 % of the 4 ms control period.
  (`::test_software_step_for_100_drones_is_a_small_fraction_of_the_control_period`; measured 0.30 ms = 13x real
  time.) Why amended: 1000x real time means 4 us per tick, below the fixed overhead of a handful of numpy calls;
  no batched Python controller can meet it. The meaningful requirement is that flight software is a small share of
  the tick budget.
- [x] `scripts/check.sh` passes (166 passed). Also: `::test_speed_is_limited`,
  `::test_drones_without_a_goal_stay_idle_on_the_ground`, `::test_mixed_vehicle_types_and_start_yaw_are_kept`.

## Notes

- `autonomy/control.py`: legacy `controller.py` moved here (git mv), typed on `core.FleetState`; `controller.py`
  re-exports. `autonomy/software.py`: `Observation(state, time)`, `Goals(target (n,3), active (n,))`,
  `FlightSoftware` protocol (`reset(world)`, `set_goals`, `step(obs, dt) -> (n,4)`), `FLIGHT_SOFTWARE` registry,
  `GotoAutopilot` registered as `goto`.
- `reset(world)` instead of the ticket's `reset(n, vehicle)`: a world can mix vehicle types, so the software needs
  each drone's vehicle id. Drones are grouped by vehicle, one vectorised `GeometricController` per group; the only
  Python loop is over vehicle types.
- Goto: velocity command = position error x gain, capped at `max_speed` (default 3 m/s), tracked by the velocity
  loop (with integral) so the drone settles on the target. Heading held at the start yaw. No active goal = motors off.
- Measured: 0.30 ms per control tick for 100 drones (13x real time), 0.59 ms for 500 (7x).
