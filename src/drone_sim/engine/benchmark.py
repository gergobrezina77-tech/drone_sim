"""Benchmark scenario and measurements, shared by ``scripts/bench.py`` and the performance regression test."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from drone_sim.core.geometry import Cylinder
from drone_sim.engine.simulation import Simulation
from drone_sim.world import DroneSpec, MoveEntity, ObstacleSpec, TaskSpec, WorldEditor, WorldSpec
from drone_sim.world.constructors import DRONE_REST_Z


def benchmark_world(n: int, spacing: float = 1.5) -> WorldSpec:
    """n drones on a grid, each flying 6 m forward and up; a row of pillars beside their paths."""
    cols = int(np.ceil(np.sqrt(n)))
    size = max(40.0, cols * spacing + 20.0)
    drones, tasks = [], []
    for k in range(n):
        x, y = -cols * spacing / 2 + (k // cols) * spacing, -cols * spacing / 2 + (k % cols) * spacing
        drones.append(DroneSpec(f"d{k}", position=(x, y, DRONE_REST_Z)))
        tasks.append(TaskSpec(f"t{k}", f"d{k}", "goto_point", {"target": [x + 6.0, y, 2.0], "dwell": 0.5}))
    pillars = tuple(
        ObstacleSpec(f"p{j}", Cylinder((-size / 2 + 3.0 + j * 2.0, size / 2 - 3.0, 0.0), 0.3, 4.0))
        for j in range(int(size // 2) - 3)
    )
    return WorldSpec(arena_size=(size, size, 15.0), drones=tuple(drones), obstacles=pillars, tasks=tuple(tasks))


@dataclass
class BenchResult:
    n: int
    compile_ms: float
    restart_ms: float
    tick_ms: float
    realtime_factor: float  # simulated seconds per wall-clock second


def measure(n: int, sim_seconds: float = 3.0) -> BenchResult:
    world = benchmark_world(n)
    sim = Simulation(WorldEditor(world))
    t0 = time.perf_counter()
    sim.start()
    compile_ms = (time.perf_counter() - t0) * 1e3

    sim.reset()  # restart with an edited start pose: same geometry, so no recompile
    first = world.drones[0]
    sim.edit(MoveEntity(first.id, (first.position[0], first.position[1] + 0.1, first.position[2])))
    t0 = time.perf_counter()
    sim.start()
    restart_ms = (time.perf_counter() - t0) * 1e3

    ticks = round(sim_seconds / sim.control_dt)
    t0 = time.perf_counter()
    for _ in range(ticks):
        sim.tick()
    wall = time.perf_counter() - t0
    return BenchResult(n, compile_ms, restart_ms, wall / ticks * 1e3, ticks * sim.control_dt / wall)
