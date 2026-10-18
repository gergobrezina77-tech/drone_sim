"""Behavioural contract every PhysicsBackend must meet. Runs against every registered backend.

Drones start resting on the floor; "airborne" tests first give them a little more than hover thrust.
"""

import numpy as np
import pytest

import tests.fake_physics  # noqa: F401  (registers the test_kinematic backend)
from drone_sim.core.geometry import Box
from drone_sim.core.rotations import rot_z
from drone_sim.physics import PHYSICS_BACKENDS, PhysicsBackend
from drone_sim.vehicle import get_vehicle
from drone_sim.world import DroneSpec, ObstacleSpec, WorldSpec
from drone_sim.world.constructors import DRONE_REST_Z

HOVER = get_vehicle("quad_x_500g").hover_thrust

TWO_DRONES = WorldSpec(
    drones=(
        DroneSpec("a", position=(0.0, 0.0, DRONE_REST_Z)),
        DroneSpec("b", position=(3.0, 0.0, DRONE_REST_Z), yaw=0.7),
    ),
)


@pytest.fixture(params=PHYSICS_BACKENDS.names())
def make(request):
    def build(world: WorldSpec = TWO_DRONES) -> PhysicsBackend:
        backend = PHYSICS_BACKENDS.get(request.param)()
        backend.build(world)
        return backend

    return build


def run(backend, seconds, thrust_scale=(0.0, 0.0), force=None):
    backend.set_thrusts(np.repeat(np.array(thrust_scale)[:, None] * HOVER, 4, axis=1))
    backend.set_external_forces(np.zeros((backend.n, 3)) if force is None else np.asarray(force, float))
    backend.step(round(seconds / backend.timestep))


def test_satisfies_the_protocol(make):
    assert isinstance(make(), PhysicsBackend)


def test_build_places_drones_at_rest_at_their_start_poses(make):
    b = make()
    s = b.state()
    assert b.n == 2 and b.time == 0.0
    np.testing.assert_allclose(s.pos, [d.position for d in TWO_DRONES.drones], atol=1e-6)
    np.testing.assert_allclose(s.vel, 0.0, atol=1e-9)
    np.testing.assert_allclose(s.R[1], rot_z(0.7), atol=1e-6)
    assert not b.collisions().any()


def test_step_advances_time(make):
    b = make()
    b.step(10)
    assert b.time == pytest.approx(10 * b.timestep)


def test_thrust_above_hover_lifts_and_zero_thrust_rests_on_the_floor(make):
    b = make()
    run(b, 1.0, thrust_scale=(1.3, 0.0))
    z = b.state().pos[:, 2]
    assert z[0] > DRONE_REST_Z + 0.2
    assert z[1] == pytest.approx(DRONE_REST_Z, abs=0.01)


def test_external_force_pushes_an_airborne_drone(make):
    b = make()
    run(b, 0.5, thrust_scale=(1.3, 1.3))
    x0 = b.state().pos[:, 0].copy()
    run(b, 1.0, thrust_scale=(1.0, 1.0), force=[[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    dx = b.state().pos[:, 0] - x0
    assert dx[0] > 0.3 and abs(dx[1]) < 0.1


def test_frozen_drones_ignore_thrust_and_force(make):
    b = make()
    b.freeze(np.array([True, False]))
    start = b.state().pos.copy()
    run(b, 1.0, thrust_scale=(2.0, 1.3), force=[[2.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    s = b.state()
    np.testing.assert_allclose(s.pos[0], start[0], atol=1e-6)
    np.testing.assert_allclose(s.vel[0], 0.0, atol=1e-9)
    assert s.pos[1, 2] > start[1, 2] + 0.1


def test_flying_into_an_obstacle_is_flagged_for_that_drone_only(make):
    wall = ObstacleSpec("wall", Box(center=(0.8, 0.0, 1.0), half_extents=(0.1, 1.0, 1.0)))
    b = make(WorldSpec(drones=TWO_DRONES.drones, obstacles=(wall,)))
    flagged = np.zeros(2, dtype=bool)
    for _ in range(100):  # 1 s in 10 ms chunks: climb a little and push towards the wall
        run(b, 0.01, thrust_scale=(1.1, 1.1), force=[[2.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        flagged |= b.collisions()
    assert flagged.tolist() == [True, False]


def test_drones_touching_each_other_are_both_flagged(make):
    world = WorldSpec(
        drones=(
            DroneSpec("a", position=(0.0, 0.0, DRONE_REST_Z)),
            DroneSpec("b", position=(0.6, 0.0, DRONE_REST_Z)),
            DroneSpec("c", position=(5.0, 0.0, DRONE_REST_Z)),
        )
    )
    b = make(world)
    flagged = np.zeros(3, dtype=bool)
    for _ in range(100):
        b.set_thrusts(np.full((3, 4), 1.1 * HOVER))
        b.set_external_forces(np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]))
        b.step(round(0.01 / b.timestep))
        flagged |= b.collisions()
    assert flagged.tolist() == [True, True, False]


def test_reset_restores_the_start_state_and_unfreezes(make):
    b = make()
    b.freeze(np.array([False, True]))
    run(b, 0.5, thrust_scale=(1.5, 1.5))
    b.reset()
    s = b.state()
    assert b.time == 0.0
    np.testing.assert_allclose(s.pos, [d.position for d in TWO_DRONES.drones], atol=1e-6)
    run(b, 0.5, thrust_scale=(1.5, 1.5))
    assert (b.state().pos[:, 2] > DRONE_REST_Z + 0.05).all()  # neither drone is frozen any more


def test_identical_inputs_give_identical_results(make):
    b1, b2 = make(), make()
    for b in (b1, b2):
        run(b, 0.7, thrust_scale=(1.4, 1.2), force=[[0.3, -0.2, 0.0], [0.0, 0.5, 0.0]])
    np.testing.assert_array_equal(b1.state().pos, b2.state().pos)
