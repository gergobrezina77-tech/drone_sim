# DS-014: legacy: rebase SwarmSim, envs and CLI on the layers (soft)

- **Status:** Done
- **Layer:** legacy
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-012

## Goal

Keep the old entry points working on top of the new layers where cheap; nice to have, not required.

## Scope

- Point `SwarmSim`, the Gym envs and the CLI at the layered modules; drop duplicated code.
- If something is too costly to port, document it as legacy instead.

## Acceptance criteria

- [x] Existing tests pass, or are explicitly retired with a note in this ticket. (all legacy tests pass unchanged;
  none retired)
- [x] `scripts/check.sh` passes

## Notes

- **Outcome: documented as legacy, not ported** (allowed by the scope). Reasons:
  - Gym envs randomise per-episode mass and inertia by editing the compiled MuJoCo model; the `PhysicsBackend`
    interface has no such operation, so porting them is an interface change -> DS-027 (Later).
  - `SwarmSim` relies on the legacy scene's physical drone-obstacle / drone-drone contact; `MujocoBackend` detects
    contact geometrically with no bounce. Porting would change legacy behaviour and its tests.
- Already de-duplicated by earlier tickets (legacy modules re-export): rotations (DS-002), vehicle params (DS-003),
  fleet state (DS-024), obstacle geometry (DS-002), wind (DS-009), controller (DS-010).
- Docs: `docs/ARCHITECTURE.md` legacy section now maps old names to layers and says what stays separate and why;
  README gained a "Layered engine" section with a runnable example (checked) and points at the benchmark.
