# DS-003: vehicle: drone parameter model and registry

- **Status:** Ready
- **Layer:** vehicle
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-002

## Goal

Drone physical parameters as pure data, selectable by id so new airframes plug in later.

## Scope

- Move `DroneParams` into `vehicle/` (no MuJoCo import); legacy `drone.py` re-exports it.
- Registry: `register_vehicle(id, params)`, `get_vehicle(id)`; default `quad_x_500g`.

## Acceptance criteria

- [ ] `vehicle` imports nothing from MuJoCo (checked by a test or contract).
- [ ] Registry tests: lookup, duplicate id rejected, unknown id raises a clear error.
- [ ] Legacy tests still pass.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
