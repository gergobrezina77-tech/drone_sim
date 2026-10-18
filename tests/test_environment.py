"""environment layer: Effect plug-ins, the effect stack, and the wind effect."""

import numpy as np
import pytest

from drone_sim.core.state import FleetState
from drone_sim.environment import EFFECTS, EffectStack, WindEffect
from drone_sim.vehicle import get_vehicle
from drone_sim.world import DroneSpec, WorldSpec

DRAG = get_vehicle("quad_x_500g").drag_coef
WORLD = WorldSpec(drones=(DroneSpec("a", position=(0.0, 0.0, 1.0)), DroneSpec("b", position=(2.0, 0.0, 1.0))))


def fleet(vel) -> FleetState:
    vel = np.asarray(vel, dtype=float)
    n = len(vel)
    return FleetState(
        np.zeros((n, 3)), np.tile([1.0, 0, 0, 0], (n, 1)), np.tile(np.eye(3), (n, 1, 1)), vel, np.zeros((n, 3))
    )


def stack(environment, seed=0) -> EffectStack:
    s = EffectStack.from_world(WorldSpec(drones=WORLD.drones, environment=environment))
    s.reset(seed)
    return s


def test_calm_air_and_drones_at_rest_give_exactly_zero_force():
    s = stack({})
    for _ in range(50):
        np.testing.assert_array_equal(s.step(0.004, fleet(np.zeros((2, 3)))), 0.0)


def test_wind_is_always_present_so_drag_always_acts():
    s = stack({})
    assert any(isinstance(e, WindEffect) for e in s.effects)
    np.testing.assert_allclose(s.step(0.004, fleet([[1.0, 0, 0], [0, -2.0, 0]])), [[-DRAG, 0, 0], [0, 2 * DRAG, 0]])


def test_constant_wind_pushes_stationary_drones_with_the_expected_drag():
    s = stack({"wind": {"mean": [2.0, 0.0, -0.5]}})
    np.testing.assert_allclose(s.step(0.004, fleet(np.zeros((2, 3)))), np.tile([2.0 * DRAG, 0, -0.5 * DRAG], (2, 1)))


def test_seeded_turbulence_is_reproducible_and_seed_dependent():
    env = {"wind": {"turbulence": 0.8, "local_turbulence": 0.3}}
    runs = []
    for seed in (5, 5, 6):
        s = stack(env, seed)
        runs.append(np.array([s.step(0.004, fleet(np.zeros((2, 3)))) for _ in range(100)]))
    np.testing.assert_array_equal(runs[0], runs[1])
    assert not np.allclose(runs[0], runs[2])
    assert np.abs(runs[0]).max() > 0.0


class ConstantPush:
    def __init__(self, settings, world):
        self.force = np.tile(np.asarray(settings["force"], dtype=float), (len(world.drones), 1))

    def reset(self, rng):
        pass

    def step(self, dt, state):
        return self.force


if "test_push" not in EFFECTS:
    EFFECTS.register("test_push", ConstantPush)


def test_the_stack_sums_every_configured_effect():
    s = stack({"wind": {"mean": [1.0, 0.0, 0.0]}, "test_push": {"force": [0.0, 0.0, 3.0]}})
    np.testing.assert_allclose(s.step(0.004, fleet(np.zeros((2, 3)))), np.tile([DRAG, 0, 3.0], (2, 1)))


def test_unknown_effect_names_are_rejected():
    with pytest.raises(KeyError, match="unknown effect 'lava'"):
        EffectStack.from_world(WorldSpec(drones=WORLD.drones, environment={"lava": {}}))


def test_gusts_can_be_triggered_through_the_wind_effect():
    s = stack({"wind": {"gust_strength": 4.0, "gust_duration": 1.0}})
    wind = next(e for e in s.effects if isinstance(e, WindEffect))
    wind.gust(direction=(0.0, 1.0, 0.0))
    forces = [s.step(0.01, fleet(np.zeros((2, 3))))[0, 1] for _ in range(100)]
    assert max(forces) == pytest.approx(4.0 * DRAG, rel=0.01)


def test_legacy_wind_names_are_the_environment_ones():
    from drone_sim import disturbances
    from drone_sim.environment import wind

    assert disturbances.WindField is wind.WindField
    assert disturbances.WindConfig is wind.WindConfig
    assert disturbances.aero_drag is wind.aero_drag
