import numpy as np
import pytest

from drone_sim.disturbances import WindConfig
from drone_sim.planner import FORMATIONS, Mission, Waypoint, assign_slots, formation_offsets
from drone_sim.scene import Obstacle, spawn_grid
from drone_sim.sim import SwarmSim
from drone_sim.swarm import pairwise


@pytest.mark.parametrize("name", FORMATIONS)
@pytest.mark.parametrize("n", [1, 5, 12, 30])
def test_formations_are_centred_and_spaced(name, n):
    off = formation_offsets(name, n, spacing=0.9)
    assert off.shape == (n, 3)
    np.testing.assert_allclose(off.mean(axis=0), 0, atol=1e-9)
    if n > 1:
        assert pairwise(off)[1].min() > 0.5


def test_assign_slots_is_a_permutation():
    rng = np.random.default_rng(0)
    slots = formation_offsets("circle", 10)
    out = assign_slots(rng.normal(size=(10, 3)), slots)
    assert sorted(map(tuple, out.round(9))) == sorted(map(tuple, slots.round(9)))


def contacts_between_drones(sim):
    body_drone = np.full(sim.model.nbody, -1)
    body_drone[sim.drones.body_ids] = np.arange(sim.n)
    return sum(
        body_drone[sim.model.geom_bodyid[c.geom1]] >= 0 and body_drone[sim.model.geom_bodyid[c.geom2]] >= 0
        for c in sim.data.contact[: sim.data.ncon]
    )


def test_formation_mission_converges_without_collisions():
    n = 10
    wps = [
        Waypoint((0, 0, 1.5), "grid", hold=3.0),
        Waypoint((5, 0, 2.0), "v", hold=4.0),
        Waypoint((5, 4, 2.5), "circle", hold=6.0),
    ]
    sim = SwarmSim(n, mission=Mission(wps, spawn_grid(n)), wind=WindConfig(mean=(1.0, 0, 0), turbulence=0.5))
    min_sep, contacts = np.inf, 0

    def check(s):
        nonlocal min_sep, contacts
        if s.t > 1.0:
            min_sep = min(min_sep, s.min_separation())
        contacts += contacts_between_drones(s)

    sim.run(30.0, check)
    assert sim.mission.idx == 2 and sim.mission.formation == "circle"
    assert contacts == 0
    assert min_sep > 0.36  # rotor tips touch at ~0.35 m
    assert sim.formation_error().max() < 0.25


def test_drone_flows_around_pillar():
    n = 1
    wps = [Waypoint((0, 0, 1.5), "grid", hold=0.5), Waypoint((8, 0, 1.5), "grid", hold=1.0, speed=1.5)]
    pillar = Obstacle("pillar", (4.0, 0.05, 0.0), (0.3, 4.0))
    sim = SwarmSim(n, mission=Mission(wps, spawn_grid(n)), obstacles=[pillar])
    closest = np.inf

    def check(s):
        nonlocal closest
        closest = min(closest, float(pillar.distance(s.drones.state().pos)[0][0]))

    sim.run(14.0, check)
    assert closest > sim.params.radius  # never touched the pillar
    assert np.linalg.norm(sim.drones.state().pos[0] - [8, 0, 1.5]) < 0.2  # and still got there
