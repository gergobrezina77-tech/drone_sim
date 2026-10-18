# DS-028: world: per-entity JSON (de)serialisation

- **Status:** Done
- **Layer:** world
- **Milestone:** M2 Server and web UI
- **Depends on:** DS-004

## Goal

The API protocol sends single drones, obstacles and tasks (e.g. "place this drone"). Today only the whole
`WorldSpec` can be (de)serialised; the per-entity helpers are private.

## Scope

- `to_dict()` / `from_dict()` on `DroneSpec`, `ObstacleSpec`, `TaskSpec`; `WorldSpec.to_dict/from_dict` use them.
- JSON format unchanged.

## Acceptance criteria

- [x] Each entity round-trips through `to_dict` / `from_dict`.
  (`tests/test_world_spec.py::test_each_entity_round_trips_on_its_own`)
- [x] Existing world JSON tests pass unchanged.
- [x] `scripts/check.sh` passes (226 passed)

## Notes

- Split out of DS-015 (api) because the change is in the world layer.
- `from_dict` fills optional fields with defaults (drone `yaw` 0, `vehicle` default airframe; task `params` {}),
  so a UI can send minimal entities. Output format unchanged.
