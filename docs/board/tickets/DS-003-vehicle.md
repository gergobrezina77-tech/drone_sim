# DS-003: vehicle: drone parameter model and registry

- **Status:** Done
- **Layer:** vehicle
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-002

## Goal

Drone physical parameters as pure data, selectable by id so new airframes plug in later.

## Scope

- Move `DroneParams` into `vehicle/` (no MuJoCo import); legacy `drone.py` re-exports it.
- Registry: `register_vehicle(id, params)`, `get_vehicle(id)`; default `quad_x_500g`.

## Acceptance criteria

- [x] `vehicle` imports nothing from MuJoCo (checked by a test or contract).
  (`tests/test_vehicle.py::test_vehicle_layer_has_no_physics_engine_imports`, AST scan of the package)
- [x] Registry tests: lookup, duplicate id rejected, unknown id raises a clear error.
  (`::test_default_vehicle_is_registered`, `::test_register_and_lookup_new_vehicle`, `::test_duplicate_id_is_rejected`,
  `::test_unknown_id_error_names_the_known_ids`)
- [x] Legacy tests still pass. (`::test_legacy_import_path_is_the_same_class` + all existing tests)
- [x] `scripts/check.sh` passes (79 passed)

## Notes

- `vehicle/params.py`: `DroneParams` and `GRAVITY` moved verbatim from `drone.py`; `vehicle/registry.py`:
  `register_vehicle`, `get_vehicle`, `vehicle_ids`, default `quad_x_500g` = `DroneParams()`.
- `drone.py` re-exports both, so every legacy import keeps working unchanged.
- The "no MuJoCo" check is an AST scan, not a `sys.modules` check: importing any `drone_sim.*` module runs the
  top-level `drone_sim/__init__.py`, which imports the legacy MuJoCo modules. Making that init lighter belongs to
  DS-014. A stronger option is an import-linter `forbidden` contract (only `physics` may import `mujoco`); that is a
  contract change, so it is proposed to the user rather than made here.
