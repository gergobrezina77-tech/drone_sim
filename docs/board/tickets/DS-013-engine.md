# DS-013: engine: performance benchmark and budgets

- **Status:** Backlog
- **Layer:** engine
- **Milestone:** M1 Layered headless core
- **Depends on:** DS-012

## Goal

Make scale measurable so regressions are visible.

## Scope

- `scripts/bench.py`: N = 10 / 100 / 500 drones with obstacles; reports sim-time / wall-time ratio, tick ms, restart ms, compile ms.
- Profile the 500-drone case and record the top hotspots in the ticket's notes.
- A coarse regression test (marked `slow`) with generous bounds.

## Acceptance criteria

- [ ] Benchmark runs and prints a table; numbers recorded in the ticket notes.
- [ ] At least the 100-drone case runs faster than real time on this machine.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
