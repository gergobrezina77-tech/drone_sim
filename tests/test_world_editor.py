"""world layer: editing commands, undo/redo, validation."""

import pytest

from drone_sim.core.geometry import Box, Cylinder
from drone_sim.world import (
    AddObstacle,
    AssignTask,
    DroneSpec,
    InvalidEdit,
    MoveEntity,
    ObstacleSpec,
    PlaceDrone,
    RemoveEntity,
    RemoveTask,
    TaskSpec,
    WorldEditor,
    WorldSpec,
)

WALL = ObstacleSpec("wall", Box(center=(5.0, 0.0, 1.5), half_extents=(0.2, 4.0, 1.5)))
PILLAR = ObstacleSpec("pillar", Cylinder(base=(-3.0, 2.0, 0.0), radius=0.4, height=5.0))
D1 = DroneSpec("d1", position=(0.0, 0.0, 0.07))
D2 = DroneSpec("d2", position=(1.0, 0.0, 0.07))
T1 = TaskSpec("t1", "d1", "goto_point", {"target": [8.0, 0.0, 2.0]})


def populated() -> WorldEditor:
    ed = WorldEditor(WorldSpec(arena_size=(40.0, 30.0, 15.0)))
    for cmd in (AddObstacle(WALL), AddObstacle(PILLAR), PlaceDrone(D1), PlaceDrone(D2), AssignTask(T1)):
        ed.apply(cmd)
    return ed


def test_commands_build_the_expected_world():
    w = populated().world
    assert w.obstacles == (WALL, PILLAR) and w.drones == (D1, D2) and w.tasks == (T1,)


def test_undo_and_redo_restore_exact_snapshots():
    ed = WorldEditor(WorldSpec())
    snapshots = [ed.world]
    for cmd in (AddObstacle(WALL), PlaceDrone(D1), AssignTask(T1), MoveEntity("d1", (2.0, 3.0, 0.07), yaw=1.0)):
        ed.apply(cmd)
        snapshots.append(ed.world)
    for expected in reversed(snapshots[:-1]):
        assert ed.undo()
        assert ed.world == expected
    assert not ed.undo()
    for expected in snapshots[1:]:
        assert ed.redo()
        assert ed.world == expected
    assert not ed.redo()


def test_a_new_edit_clears_the_redo_history():
    ed = populated()
    ed.undo()
    ed.apply(RemoveEntity("pillar"))
    assert not ed.redo()


def test_move_drone_and_obstacles():
    ed = populated()
    ed.apply(MoveEntity("d2", (3.0, -2.0, 0.07), yaw=0.5))
    ed.apply(MoveEntity("wall", (8.0, 1.0, 1.5)))
    ed.apply(MoveEntity("pillar", (-6.0, 2.0, 0.0)))
    w = ed.world
    assert w.drones[1] == DroneSpec("d2", position=(3.0, -2.0, 0.07), yaw=0.5)
    assert w.obstacles[0].shape.center == (8.0, 1.0, 1.5)
    assert w.obstacles[1].shape.base == (-6.0, 2.0, 0.0)


def test_removing_a_drone_removes_its_tasks_and_undo_restores_both():
    ed = populated()
    before = ed.world
    ed.apply(RemoveEntity("d1"))
    assert [d.id for d in ed.world.drones] == ["d2"] and ed.world.tasks == ()
    ed.undo()
    assert ed.world == before


def test_remove_obstacle_and_task():
    ed = populated()
    ed.apply(RemoveEntity("wall"))
    ed.apply(RemoveTask("t1"))
    assert ed.world.obstacles == (PILLAR,) and ed.world.tasks == ()


# --- validation: every rule rejects the edit and leaves the world unchanged ---------------------------------------


@pytest.mark.parametrize(
    "cmd, reason",
    [
        (PlaceDrone(DroneSpec("d1", position=(3.0, 3.0, 0.07))), "duplicate id"),
        (AddObstacle(ObstacleSpec("t1", Box((0.0, 9.0, 1.0), (1.0, 1.0, 1.0)))), "duplicate id"),
        (PlaceDrone(DroneSpec("d3", position=(25.0, 0.0, 0.07))), "outside the arena"),
        (PlaceDrone(DroneSpec("d3", position=(0.0, 0.0, 16.0))), "outside the arena"),
        (AddObstacle(ObstacleSpec("big", Box((0.0, 0.0, 1.0), (30.0, 1.0, 1.0)))), "outside the arena"),
        (AddObstacle(ObstacleSpec("tall", Cylinder((0.0, 9.0, 0.0), 0.5, 20.0))), "outside the arena"),
        (PlaceDrone(DroneSpec("d3", position=(5.0, 0.0, 0.07))), "inside obstacle"),
        (PlaceDrone(DroneSpec("d3", position=(5.3, 0.0, 0.07))), "inside obstacle"),  # rotor tips would touch
        (AddObstacle(ObstacleSpec("box", Box((0.0, 0.0, 1.0), (0.5, 0.5, 1.0)))), "inside obstacle"),
        (MoveEntity("d2", (-3.0, 2.0, 0.07)), "inside obstacle"),
        (PlaceDrone(DroneSpec("d3", position=(3.0, 3.0, 0.07), vehicle="no_such_drone")), "unknown vehicle"),
        (AssignTask(TaskSpec("t2", "ghost", "goto_point", {})), "unknown drone"),
        (MoveEntity("ghost", (0.0, 0.0, 0.0)), "no entity"),
        (MoveEntity("t1", (0.0, 0.0, 0.0)), "no entity"),
        (RemoveEntity("ghost"), "no entity"),
        (RemoveTask("ghost"), "no task"),
    ],
)
def test_invalid_edits_are_rejected_without_changing_the_world(cmd, reason):
    ed = populated()
    before = ed.world
    with pytest.raises(InvalidEdit, match=reason):
        ed.apply(cmd)
    assert ed.world == before
    ed.undo()  # the rejected edit was not recorded: one undo removes the last *valid* edit (the task)
    assert ed.world.tasks == ()
