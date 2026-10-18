"""engine layer: state machine, tick pipeline, events and reports, end to end on the MuJoCo backend."""

import numpy as np
import pytest

from drone_sim.core.geometry import Box
from drone_sim.engine import (
    DroneCollided,
    IllegalTransition,
    RunFinished,
    SimState,
    Simulation,
    TaskFailed,
    TaskSucceeded,
)
from drone_sim.tasks import FailureReason, TaskStatus
from drone_sim.world import (
    AddObstacle,
    AssignTask,
    DroneSpec,
    MoveEntity,
    ObstacleSpec,
    PlaceDrone,
    TaskSpec,
    WorldEditor,
    WorldSpec,
)
from drone_sim.world.constructors import DRONE_REST_Z

START = (0.0, 0.0, DRONE_REST_Z)
TARGET = [4.0, 0.0, 2.0]
WALL = ObstacleSpec("wall", Box((2.0, 0.0, 1.5), (0.1, 3.0, 1.5)))


def sim_with(*commands, seed=0) -> Simulation:
    sim = Simulation(WorldEditor(WorldSpec()), seed=seed)
    sim.edit(PlaceDrone(DroneSpec("d1", position=START)))
    sim.edit(AssignTask(TaskSpec("t1", "d1", "goto_point", {"target": TARGET, "timeout": 20.0})))
    for c in commands:
        sim.edit(c)
    return sim


def run_to_end(sim: Simulation, max_seconds: float = 30.0) -> None:
    sim.start()
    sim.run_until_finished(max_seconds)
    assert sim.state is SimState.FINISHED


def test_a_drone_flies_to_its_point_and_the_task_succeeds():
    sim = sim_with()
    run_to_end(sim)
    assert [type(e) for e in sim.events] == [TaskSucceeded, RunFinished]
    report = sim.report()
    assert report.outcomes[0].status is TaskStatus.SUCCEEDED and report.outcomes[0].task_id == "t1"
    assert np.linalg.norm(sim.fleet_state().pos[0] - TARGET) < 0.3


def test_a_wall_in_the_way_fails_the_task_by_collision_and_freezes_the_drone():
    sim = sim_with(AddObstacle(WALL))
    run_to_end(sim)
    assert [type(e) for e in sim.events] == [DroneCollided, TaskFailed, RunFinished]
    failed = sim.events[1]
    assert failed.reason is FailureReason.COLLISION and failed.task_id == "t1"
    frozen_at = sim.fleet_state().pos.copy()
    assert frozen_at[0, 0] < 2.0  # stopped at the wall, did not pass through


def test_reset_returns_to_edit_with_the_authored_start_setup():
    sim = sim_with(AddObstacle(WALL))
    run_to_end(sim)
    world_before = sim.world
    sim.reset()
    assert sim.state is SimState.EDIT and sim.world == world_before and sim.events == []
    np.testing.assert_allclose(sim.fleet_state().pos[0], START)


def test_edit_after_reset_then_rerun_without_the_wall_succeeds():
    sim = sim_with(AddObstacle(WALL))
    run_to_end(sim)
    sim.reset()
    sim.edit(MoveEntity("wall", (2.0, 8.0, 1.5)))
    run_to_end(sim)
    assert isinstance(sim.events[0], TaskSucceeded)


def test_same_seed_gives_identical_event_streams_with_turbulence():
    def events(seed):
        sim = sim_with(seed=seed)
        sim.edit(PlaceDrone(DroneSpec("d2", position=(0.0, 3.0, DRONE_REST_Z))))
        sim.edit(AssignTask(TaskSpec("t2", "d2", "goto_point", {"target": [3.0, 3.0, 1.5]})))
        world = sim.world
        sim = Simulation(
            WorldEditor(WorldSpec(drones=world.drones, tasks=world.tasks, environment={"wind": {"turbulence": 0.6}})),
            seed=seed,
        )
        run_to_end(sim)
        return sim.events, sim.fleet_state().pos

    (e1, p1), (e2, p2) = events(3), events(3)
    assert e1 == e2
    np.testing.assert_array_equal(p1, p2)


@pytest.mark.parametrize(
    "action",
    [
        lambda s: s.pause(),
        lambda s: s.resume(),
        lambda s: s.tick(),
        lambda s: s.reset(),
    ],
)
def test_illegal_transitions_from_edit_raise(action):
    sim = sim_with()
    with pytest.raises(IllegalTransition):
        action(sim)
    assert sim.state is SimState.EDIT


def test_state_machine_cycle_and_illegal_transitions_while_running():
    sim = sim_with()
    sim.start()
    assert sim.state is SimState.RUNNING
    for illegal in (sim.start, sim.resume, lambda: sim.edit(MoveEntity("d1", (1.0, 1.0, DRONE_REST_Z)))):
        with pytest.raises(IllegalTransition):
            illegal()
    sim.tick()
    sim.pause()
    assert sim.state is SimState.PAUSED
    t = sim.time
    with pytest.raises(IllegalTransition):
        sim.tick()
    with pytest.raises(IllegalTransition):
        sim.undo()
    assert sim.time == t
    sim.resume()
    sim.tick()
    assert sim.time > t
    sim.reset()
    assert sim.state is SimState.EDIT


def test_editing_and_undo_work_in_edit():
    sim = sim_with()
    sim.edit(MoveEntity("d1", (1.0, 1.0, DRONE_REST_Z)))
    assert sim.world.drones[0].position == (1.0, 1.0, DRONE_REST_Z)
    assert sim.undo() and sim.world.drones[0].position == START
    assert sim.redo() and sim.world.drones[0].position == (1.0, 1.0, DRONE_REST_Z)


def test_an_invalid_task_setup_keeps_the_simulation_in_edit():
    sim = Simulation(WorldEditor(WorldSpec()))
    sim.edit(PlaceDrone(DroneSpec("d1", position=START)))
    sim.edit(AssignTask(TaskSpec("t1", "d1", "goto_point", {"target": [999.0, 0.0, 1.0]})))
    with pytest.raises(ValueError, match="outside the arena"):
        sim.start()
    assert sim.state is SimState.EDIT


def test_a_world_without_tasks_runs_until_stopped():
    sim = Simulation(WorldEditor(WorldSpec()))
    sim.edit(PlaceDrone(DroneSpec("d1", position=START)))
    sim.start()
    for _ in range(50):
        sim.tick()
    assert sim.state is SimState.RUNNING and sim.events == []


def test_restarting_with_unchanged_geometry_reuses_the_compiled_physics_model():
    sim = sim_with(AddObstacle(WALL))
    sim.start()
    sim.reset()
    sim.edit(MoveEntity("d1", (0.0, 1.0, DRONE_REST_Z)))
    sim.start()
    assert sim.physics.compile_count == 1
