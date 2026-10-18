# DS-014: legacy: rebase SwarmSim, envs and CLI on the layers (soft)

- **Status:** Ready
- **Layer:** legacy
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-012

## Goal

Keep the old entry points working on top of the new layers where cheap; nice to have, not required.

## Scope

- Point `SwarmSim`, the Gym envs and the CLI at the layered modules; drop duplicated code.
- If something is too costly to port, document it as legacy instead.

## Acceptance criteria

- [ ] Existing tests pass, or are explicitly retired with a note in this ticket.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
