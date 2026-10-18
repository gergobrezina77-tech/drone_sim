# DS-011: tasks: TaskType interface + goto_point evaluator

- **Status:** Done
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

- [x] Tests for each outcome with synthetic states (no physics needed).
  (`tests/test_tasks.py`: `::test_success_needs_the_drone_inside_the_tolerance_for_the_dwell_time`,
  `::test_shared_failure_checks` [collision, crash, out of bounds x2], `::test_timeout`,
  `::test_failure_wins_over_success_on_the_same_tick`)
- [x] Terminal states are sticky. (`::test_terminal_states_are_sticky`)
- [x] `scripts/check.sh` passes (184 passed). Also: goals exposure, change reporting, reset, invalid setups.

## Notes

- `tasks/base.py`: `TaskStatus`, `FailureReason`, `TaskEvaluator` protocol (`targets`, `reset`,
  `update(state, t, active) -> succeeded`), `TASK_TYPES` registry, `outside_arena`. `tasks/goto.py`: `GotoPoint`
  (`target`, `tolerance` 0.3, `dwell` 1.0), registered as `goto_point`. `tasks/board.py`: `TaskBoard`.
- Split of responsibilities: a task type only defines success and targets; the failure checks every task shares
  (collision > crash (tilt > 80 deg) > out of bounds > timeout, default 60 s, `timeout` param) are in `TaskBoard`.
  Failure is checked before success, so a collision on the arrival tick is a failure.
- The ticket's `goal_for_autonomy()` is `TaskBoard.goals() -> (target (n,3), active (n,))`, plain arrays because
  `tasks` may not import `autonomy` (siblings); the engine wraps them in `autonomy.Goals`. After success the goal
  stays active (hover at the target); after failure it is dropped.
- One task per drone: worlds with more are rejected with a clear message; sequencing is DS-025.
- Task-type validation happens when the board is built (goto target present and inside the arena); this is where
  the world layer could not check (see DS-005 notes).
