# DS-013: engine: performance benchmark and budgets

- **Status:** Done
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

- [x] Benchmark runs and prints a table; numbers recorded in the ticket notes. (`poetry run python scripts/bench.py`)
- [x] At least the 100-drone case runs faster than real time on this machine. (1.98x;
  `tests/test_performance.py::test_100_drones_run_faster_than_real_time_and_restart_quickly`, marked `slow`)
- [x] `scripts/check.sh` passes

## Notes

- `engine/benchmark.py`: `benchmark_world(n)` (grid of drones each flying 6 m forward and up, a row of pillars)
  and `measure(n, seconds)` -> compile ms, restart ms (edited start pose, same geometry), tick ms, real-time factor.
  Shared by `scripts/bench.py` (table, or `--profile N` for a cProfile top 15) and the regression test.
- `slow` pytest marker registered in `pyproject.toml`; slow tests still run in the gate (they take < 1 s).
- Measurements (this machine, 3 s simulated, control 4 ms = 2 physics substeps):

  | drones | compile ms | restart ms | tick ms | x real time |
  | --- | --- | --- | --- | --- |
  | 10 | 15.3 | 1.59 | 1.290 | 3.10 |
  | 100 | 31.8 | 3.67 | 2.019 | 1.98 |
  | 500 | 355.3 | 6.70 | 17.398 | 0.23 |

- Hotspots, 500 drones (cProfile): `MujocoBackend.collisions` 5.3 s of 6.9 s (77 %), almost all in the n x n
  drone-drone distance matrix (`norm` + ufunc reduce). Next: `mj_step` 0.56 s, `quat_to_mat` 0.16 s.
  -> DS-026 (physics: spatial prefilter).
- Hotspots, 10 drones: no single one; ~300 small numpy calls per tick (~1.2 ms fixed overhead). Fine at this size.
