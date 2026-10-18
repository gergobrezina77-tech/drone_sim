# DS-025: tasks: several tasks per drone, run in sequence

- **Status:** Ready
- **Layer:** tasks
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-011

## Goal

Let a drone carry a list of tasks (e.g. go to A, then B) instead of at most one.

## Scope

- `TaskBoard`: per drone, tasks run in world order; the first non-terminal one is ACTIVE, later ones PENDING.
- A drone failure (collision, crash, out of bounds) fails its remaining tasks with the same reason.
- Vectorised cursor per drone; no per-drone Python loop in `update`.

## Acceptance criteria

- [ ] Two goto tasks for one drone succeed in order; the second becomes ACTIVE when the first succeeds.
- [ ] A collision during the first task fails both.
- [ ] `scripts/check.sh` passes

## Notes

Split out of DS-011, which supports one task per drone and rejects worlds with more.
