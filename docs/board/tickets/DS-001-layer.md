# DS-001: Layer package skeleton + strict layer contract

- **Status:** Ready
- **Layer:** all
- **Milestone:** M1 Layered headless core
- **Depends on:** none

## Goal

Create the empty layer packages so every later ticket has a home, and make the layer contract strict.

## Scope

- Create `core, vehicle, world, physics, environment, autonomy, tasks, engine, api` packages under `src/drone_sim/`, each `__init__.py` with a docstring stating the layer's responsibility and what it must not import.
- Remove the optional-layer parentheses from `[tool.importlinter]` so all layers are required.
- Add a test that imports each layer package.

## Acceptance criteria

- [ ] `lint-imports` reports the contract KEPT with no optional layers.
- [ ] A deliberate upward import (tried locally, not committed) makes `lint-imports` fail.
- [ ] `scripts/check.sh` passes; existing 43 tests still pass.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
