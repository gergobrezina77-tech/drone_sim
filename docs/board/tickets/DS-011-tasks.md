# DS-011: tasks: TaskType interface + goto_point evaluator

- **Status:** Backlog
- **Layer:** tasks
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-004

## Goal

Tasks decide success or failure independently from the drone software.

## Scope

- `TaskStatus` enum (PENDING, ACTIVE, SUCCEEDED, FAILED) and `FailureReason` (COLLISION, CRASH, OUT_OF_BOUNDS, TIMEOUT).
- `TaskType` protocol: build a batched evaluator from `TaskSpec`s; `evaluate(state, collisions, t)` updates status arrays; `goal_for_autonomy()` exposes targets.
- `goto_point`: success when within tolerance for a dwell time; failure on collision, timeout, leaving the arena or tilt beyond a crash limit.
- Registry by type name.

## Acceptance criteria

- [ ] Tests for each outcome with synthetic states (no physics needed).
- [ ] Terminal states are sticky.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
