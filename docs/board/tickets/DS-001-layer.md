# DS-001: Layer package skeleton + strict layer contract

- **Status:** Done
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

- [x] `lint-imports` reports the contract KEPT with no optional layers.
  (`tests/test_layers.py::test_contract_covers_every_layer_and_none_is_optional` + gate)
- [x] A deliberate upward import (tried locally, not committed) makes `lint-imports` fail.
  (`tests/test_layers.py::test_contract_rejects_an_upward_import`; also tried once in the real tree:
  `core -> engine` reported BROKEN, then reverted)
- [x] `scripts/check.sh` passes; existing 43 tests still pass. (54 passed: 43 existing + 11 new)

## Notes

- The upward-import test runs the real `[tool.importlinter]` config against a throwaway `drone_sim` package in
  `tmp_path` (put first on `PYTHONPATH`), so it checks the contract itself without touching `src/`. It already
  passed before the parentheses were removed (optional layers are enforced once they exist); it stays as the
  regression test that the contract catches upward imports.
- Each layer `__init__.py` docstring states its responsibility and which layers it may import.
- Contract edit in `pyproject.toml` approved by the user before starting (it is otherwise off limits).
