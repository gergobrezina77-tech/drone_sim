# DS-022: core: generic name registry

- **Status:** Done
- **Layer:** core
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-003

## Goal

One small, shared registry type for every extension point (vehicles, scene constructors, physics backends, effects,
flight software, task types) instead of a hand-written copy per layer.

## Scope

- `core/registry.py`: `Registry[T]` with `register(name, item)` (duplicate rejected), `get(name)` (unknown name
  raises `KeyError` listing the known names), `names()`, `in`.
- `vehicle/registry.py` uses it; its public functions (`register_vehicle`, `get_vehicle`, `vehicle_ids`) keep their
  behaviour.

## Acceptance criteria

- [x] Registry tests: register/get, duplicate rejected, unknown name error lists known names, `names()` sorted.
  (`tests/test_core_registry.py`, 4 tests)
- [x] Existing vehicle registry tests pass unchanged. (`tests/test_vehicle.py`, untouched)
- [x] `scripts/check.sh` passes (112 passed)

## Notes

- Added while starting DS-006 (second hand-written registry; four more planned in M1).
- `Registry[T](kind)`; `register` returns the item. Error messages name the kind, e.g. "unknown vehicle 'x'; known: ...".
- `vehicle/registry.py` now wraps a module-level `VEHICLES = Registry[DroneParams]("vehicle")`.
