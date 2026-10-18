# DS-023: vehicle: per-airframe rest height

- **Status:** Backlog
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

- [ ] A vehicle with a different `rest_height` is placed at that height by plan2d.
- [ ] `scripts/check.sh` passes

## Notes

Split out of DS-006 to keep that ticket inside the world layer.
