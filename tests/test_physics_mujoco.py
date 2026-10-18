"""MuJoCo backend specifics. The generic behaviour is covered by tests/test_physics_contract.py."""

import time

import numpy as np
import pytest

from drone_sim.core.geometry import Box, Cylinder
from drone_sim.physics import PHYSICS_BACKENDS
from drone_sim.vehicle import DEFAULT_VEHICLE, get_vehicle
from drone_sim.world import DroneSpec, ObstacleSpec, WorldSpec

DRONE_REST_Z = get_vehicle(DEFAULT_VEHICLE).rest_height

PARAMS = get_vehicle("quad_x_500g")


def mujoco_backend(world):
    backend = PHYSICS_BACKENDS.get("mujoco")()
    backend.build(world)
    return backend


def grid_world(n: int, offset: float = 0.0) -> WorldSpec:
    cols = int(np.ceil(np.sqrt(n)))
    drones = tuple(
        DroneSpec(f"d{k}", position=(offset + (k // cols) * 1.0, (k % cols) * 1.0, DRONE_REST_Z)) for k in range(n)
    )
    obstacles = (
        ObstacleSpec("wall", Box((-5.0, 0.0, 1.5), (0.2, 6.0, 1.5))),
        ObstacleSpec("pillar", Cylinder((-8.0, 3.0, 0.0), 0.4, 5.0)),
    )
    return WorldSpec(drones=drones, obstacles=obstacles)


def test_open_loop_hover_holds_position_within_a_centimetre_for_two_seconds():
    b = mujoco_backend(WorldSpec(drones=(DroneSpec("d", position=(0.0, 0.0, 2.0)),)))
    b.set_thrusts(np.full((1, 4), PARAMS.hover_thrust))
    b.step(round(2.0 / b.timestep))
    assert np.linalg.norm(b.state().pos[0] - (0.0, 0.0, 2.0)) < 0.01


def test_collision_is_flagged_on_the_step_the_footprint_reaches_the_obstacle():
    """Mid-air at hover thrust with a constant sideways force: x(t) = x0 + F t^2 / (2 m) exactly (no drag here)."""
    x0, force, wall_face = 0.0, 1.0, 0.5
    wall = ObstacleSpec("wall", Box((wall_face + 0.1, 0.0, 2.0), (0.1, 1.0, 2.0)))
    b = mujoco_backend(WorldSpec(drones=(DroneSpec("d", position=(x0, 0.0, 2.0)),), obstacles=(wall,)))
    b.set_thrusts(np.full((1, 4), PARAMS.hover_thrust))
    b.set_external_forces(np.array([[force, 0.0, 0.0]]))
    first_flag = None
    for k in range(1, 1000):
        b.step(1)
        if b.collisions()[0]:
            first_flag = k
            break
    t_contact = np.sqrt(2 * PARAMS.mass * (wall_face - PARAMS.radius - x0) / force)
    assert first_flag is not None
    assert abs(first_flag - t_contact / b.timestep) <= 1.5


def test_no_bounce_drones_pass_through_obstacles_in_the_solver():
    """Obstacle contact is detected, never simulated: the drone keeps its momentum (the engine decides what fails)."""
    wall = ObstacleSpec("wall", Box((0.5, 0.0, 2.0), (0.05, 1.0, 2.0)))
    b = mujoco_backend(WorldSpec(drones=(DroneSpec("d", position=(0.0, 0.0, 2.0)),), obstacles=(wall,)))
    b.set_thrusts(np.full((1, 4), PARAMS.hover_thrust))
    b.set_external_forces(np.array([[2.0, 0.0, 0.0]]))
    b.step(round(1.0 / b.timestep))
    assert b.state().pos[0, 0] > 0.9


def test_restart_with_unchanged_geometry_is_fast_and_does_not_recompile():
    b = mujoco_backend(grid_world(100))
    compiles = b.compile_count
    b.set_thrusts(np.full((100, 4), 1.3 * PARAMS.hover_thrust))
    b.step(200)
    moved = grid_world(100, offset=2.0)  # every start pose changed, geometry identical
    t0 = time.perf_counter()
    b.build(moved)
    elapsed = time.perf_counter() - t0
    assert b.compile_count == compiles
    assert elapsed < 0.020, f"restart took {elapsed * 1000:.1f} ms"
    np.testing.assert_allclose(b.state().pos, [d.position for d in moved.drones], atol=1e-9)
    assert b.time == 0.0


def test_changed_geometry_recompiles():
    b = mujoco_backend(grid_world(4))
    compiles = b.compile_count
    world = grid_world(4)
    b.build(
        WorldSpec(drones=world.drones, obstacles=(*world.obstacles, ObstacleSpec("x", Box((9.0, 9.0, 1.0), (1, 1, 1)))))
    )
    assert b.compile_count == compiles + 1


def test_world_without_drones_builds_and_steps():
    b = mujoco_backend(WorldSpec())
    b.step(5)
    assert b.n == 0 and b.collisions().shape == (0,)


@pytest.mark.parametrize("n", [1, 7])
def test_state_shapes(n):
    s = mujoco_backend(grid_world(n)).state()
    assert s.pos.shape == (n, 3) and s.quat.shape == (n, 4) and s.R.shape == (n, 3, 3)
    assert s.vel.shape == (n, 3) and s.omega.shape == (n, 3)
