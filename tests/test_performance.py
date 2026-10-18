"""Coarse performance regression guard (generous bounds; scripts/bench.py gives the full picture)."""

import pytest

from drone_sim.engine.benchmark import benchmark_world, measure


@pytest.mark.slow
def test_100_drones_run_faster_than_real_time_and_restart_quickly():
    r = measure(100, sim_seconds=1.0)
    assert r.realtime_factor > 1.0, f"{r.realtime_factor:.2f}x real time"
    assert r.restart_ms < 50.0, f"restart {r.restart_ms:.1f} ms"


def test_benchmark_world_is_valid_and_sized():
    world = benchmark_world(37)
    assert len(world.drones) == len(world.tasks) == 37 and world.obstacles
