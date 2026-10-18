# DS-009: environment: Effect interface + wind effect

- **Status:** Done
- **Layer:** environment
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-007

## Goal

Effects on the drone as composable, batched plug-ins.

## Scope

- `Effect` protocol: `reset(n, rng)`, `step(dt, state) -> forces (n,3)` (modifiers such as thrust scaling can be added to the protocol later).
- Move wind and drag into `WindEffect`; legacy `disturbances.py` re-exports.
- Effect registry; `EffectStack` sums forces from several effects.

## Acceptance criteria

- [x] No wind gives exactly zero force; constant wind gives the expected drag.
  (`tests/test_environment.py::test_calm_air_and_drones_at_rest_give_exactly_zero_force`,
  `::test_constant_wind_pushes_stationary_drones_with_the_expected_drag`,
  `::test_wind_is_always_present_so_drag_always_acts`)
- [x] Seeded turbulence is reproducible. (`::test_seeded_turbulence_is_reproducible_and_seed_dependent`)
- [x] Legacy wind tests still pass. (`tests/test_swarm.py` unchanged; `::test_legacy_wind_names_are_the_environment_ones`)
- [x] `scripts/check.sh` passes (159 passed). Also: `::test_the_stack_sums_every_configured_effect`,
  `::test_unknown_effect_names_are_rejected`, `::test_gusts_can_be_triggered_through_the_wind_effect`.

## Notes

- `environment/effects.py`: `Effect` protocol (`reset(rng)`, `step(dt, state) -> (n,3)` world forces), `EFFECTS`
  registry of factories `(settings, world) -> Effect`, `EffectStack.from_world(world)` builds one effect per
  `world.environment` entry; `reset(seed)` gives each effect its own `rng_stream(seed, "effect:<name>")`.
- `environment/wind.py`: legacy `disturbances.py` moved here (git mv) + `WindEffect` (wind field + drag with each
  drone's vehicle `drag_coef`). `disturbances.py` re-exports.
- Decision: `wind` is always on (calm unless configured) because it carries aerodynamic drag; without it drones
  would fly with no drag at all, unlike the legacy sim.
- The protocol's `reset(rng)` replaces the ticket's `reset(n, rng)`: `n` is known from the world at construction.
  Thrust-scaling style modifiers (battery, motor faults) can extend the protocol when the first one is built.
