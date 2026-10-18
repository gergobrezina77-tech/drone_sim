# DS-009: environment: Effect interface + wind effect

- **Status:** Ready
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

- [ ] No wind gives exactly zero force; constant wind gives the expected drag.
- [ ] Seeded turbulence is reproducible.
- [ ] Legacy wind tests still pass.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
