"""autonomy layer: FlightSoftware interface and the naive goto autopilot, flown in the MuJoCo backend."""

import time

import numpy as np
import pytest

from drone_sim.autonomy import FLIGHT_SOFTWARE, FlightSoftware, Goals, GotoAutopilot, Observation
from drone_sim.physics import PHYSICS_BACKENDS
from drone_sim.vehicle import DEFAULT_VEHICLE, DroneParams, get_vehicle, register_vehicle, vehicle_ids
from drone_sim.world import DroneSpec, WorldSpec

DRONE_REST_Z = get_vehicle(DEFAULT_VEHICLE).rest_height

CONTROL_DT = 0.004

if "test_heavy" not in vehicle_ids():
    register_vehicle("test_heavy", DroneParams(mass=0.9, inertia=(4e-3, 4e-3, 7e-3), max_thrust=5.0))


def fly(world: WorldSpec, targets, seconds: float, active=None):
    """Closed loop: physics (MuJoCo) <-> flight software, no environment effects."""
    physics = PHYSICS_BACKENDS.get("mujoco")()
    physics.build(world)
    software = FLIGHT_SOFTWARE.get("goto")()
    software.reset(world)
    n = len(world.drones)
    software.set_goals(Goals(np.asarray(targets, float), np.ones(n, bool) if active is None else np.asarray(active)))
    substeps = round(CONTROL_DT / physics.timestep)
    for _ in range(round(seconds / CONTROL_DT)):
        physics.set_thrusts(software.step(Observation(physics.state(), physics.time), CONTROL_DT))
        physics.step(substeps)
    return physics.state()


def test_registered_software_satisfies_the_protocol():
    assert isinstance(FLIGHT_SOFTWARE.get("goto")(), FlightSoftware)


def test_a_drone_takes_off_and_reaches_a_point_five_metres_away():
    world = WorldSpec(drones=(DroneSpec("a", position=(0.0, 0.0, DRONE_REST_Z)),))
    target = (3.0, 3.0, 2.5)  # |target - start| = 5.0 m
    s = fly(world, [target], seconds=10.0)
    assert np.linalg.norm(s.pos[0] - target) < 0.2
    assert np.linalg.norm(s.vel[0]) < 0.2


def test_speed_is_limited():
    world = WorldSpec(drones=(DroneSpec("a", position=(0.0, 0.0, 2.0)),))
    physics = PHYSICS_BACKENDS.get("mujoco")()
    physics.build(world)
    software = GotoAutopilot(max_speed=2.0)
    software.reset(world)
    software.set_goals(Goals(np.array([[25.0, 0.0, 2.0]]), np.ones(1, bool)))
    top = 0.0
    for _ in range(round(6.0 / CONTROL_DT)):
        physics.set_thrusts(software.step(Observation(physics.state(), physics.time), CONTROL_DT))
        physics.step(2)
        top = max(top, float(np.linalg.norm(physics.state().vel[0])))
    assert 1.8 < top < 2.3


def test_drones_without_a_goal_stay_idle_on_the_ground():
    world = WorldSpec(
        drones=(DroneSpec("a", position=(0.0, 0.0, DRONE_REST_Z)), DroneSpec("b", position=(2.0, 0.0, DRONE_REST_Z)))
    )
    s = fly(world, [(0.0, 0.0, 2.0), (2.0, 0.0, 2.0)], seconds=3.0, active=[True, False])
    assert s.pos[0, 2] > 1.8
    assert s.pos[1, 2] == pytest.approx(DRONE_REST_Z, abs=0.01)


def test_mixed_vehicle_types_and_start_yaw_are_kept():
    world = WorldSpec(
        drones=(
            DroneSpec("a", position=(0.0, 0.0, DRONE_REST_Z), yaw=0.7),
            DroneSpec("b", position=(2.0, 0.0, DRONE_REST_Z), vehicle="test_heavy"),
        )
    )
    targets = [(0.0, 2.0, 1.5), (2.0, -2.0, 2.0)]
    s = fly(world, targets, seconds=8.0)
    np.testing.assert_allclose(s.pos, targets, atol=0.2)
    yaw_a = np.arctan2(s.R[0, 1, 0], s.R[0, 0, 0])
    assert yaw_a == pytest.approx(0.7, abs=0.05)


def test_software_step_for_100_drones_is_a_small_fraction_of_the_control_period():
    world = WorldSpec(drones=tuple(DroneSpec(f"d{k}", position=(k % 10, k // 10, 2.0)) for k in range(100)))
    physics = PHYSICS_BACKENDS.get("mujoco")()
    physics.build(world)
    software = FLIGHT_SOFTWARE.get("goto")()
    software.reset(world)
    software.set_goals(Goals(np.zeros((100, 3)) + (0.0, 0.0, 3.0), np.ones(100, bool)))
    obs = Observation(physics.state(), 0.0)
    software.step(obs, CONTROL_DT)
    best = min(_timed(software.step, obs, CONTROL_DT) for _ in range(20))
    assert best < 0.25 * CONTROL_DT, f"{best * 1e3:.3f} ms per control tick"


def _timed(f, *args):
    t0 = time.perf_counter()
    f(*args)
    return time.perf_counter() - t0


def test_legacy_controller_import_path_still_works():
    from drone_sim.autonomy.control import GeometricController as new
    from drone_sim.controller import GeometricController as legacy

    assert legacy is new
