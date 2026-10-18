# DS-006: world: SceneConstructor interface + plan2d constructor

- **Status:** Ready
- **Layer:** world
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-005, DS-022

## Goal

Make the way a scene is authored swappable; ship the simple plan-view method first.

## Scope

- `SceneConstructor` protocol: turns high-level authoring input into `WorldEditor` commands.
- Registry by name; `plan2d` implementation: rectangle footprint + height → box obstacle, circle footprint + height → pillar; place drone at (x, y) on the ground.
- A trivial second constructor in tests (e.g. `grid_generator`) proves another method plugs in.

## Acceptance criteria

- [ ] `plan2d` inputs produce the expected `ObstacleSpec`s (tests).
- [ ] Swappability test: both constructors used through the registry with no changes to callers.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
