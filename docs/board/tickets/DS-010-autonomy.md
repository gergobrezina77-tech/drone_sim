# DS-010: autonomy: FlightSoftware interface, controller, naive goto autopilot

- **Status:** Backlog
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

- [ ] `autonomy` does not import `physics` (contract).
- [ ] With the fake backend or MuJoCo: one drone reaches a goal 5 m away within 0.2 m.
- [ ] 100 drones step at least 1000× faster than real time per control tick (benchmark test, generous).
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
