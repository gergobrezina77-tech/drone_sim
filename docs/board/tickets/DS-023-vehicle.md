# DS-023: vehicle: per-airframe rest height

- **Status:** Done
- **Layer:** vehicle
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-006

## Goal

The body-origin height of a drone resting on its legs is an airframe property. Today it is `REST_Z = 0.07` in
legacy `scene.py` and a matching constant in the plan2d scene constructor.

## Scope

- Add `rest_height` to `DroneParams` (default 0.07).
- plan2d places drones at `get_vehicle(vehicle).rest_height`; remove its constant.
- Legacy `scene.REST_Z` reads the default vehicle value.

## Acceptance criteria

- [x] A vehicle with a different `rest_height` is placed at that height by plan2d.
  (`tests/test_world_constructors.py::test_plan2d_places_each_vehicle_at_its_own_rest_height`)
- [x] `scripts/check.sh` passes (201 passed)

## Notes

- Split out of DS-006 to keep that ticket inside the world layer.
- `DroneParams.rest_height = 0.07`. plan2d reads the vehicle's value (an unknown vehicle is now an `InvalidEdit`
  from plan2d itself); `world.constructors.DRONE_REST_Z` removed; legacy `scene.REST_Z` reads the default vehicle.
- Tests and `engine/benchmark.py` that used the constant now read `get_vehicle(DEFAULT_VEHICLE).rest_height`
  (import-only changes).
