"""world layer: swappable scene constructors; plan2d is the first one."""

from dataclasses import dataclass

import pytest

from drone_sim.core.geometry import Box, Cylinder
from drone_sim.vehicle import DEFAULT_VEHICLE, DroneParams, get_vehicle, register_vehicle, vehicle_ids
from drone_sim.world import (
    SCENE_CONSTRUCTORS,
    AddObstacle,
    Batch,
    DroneSpec,
    InvalidEdit,
    ObstacleSpec,
    WorldEditor,
    WorldSpec,
)

REST_Z = get_vehicle(DEFAULT_VEHICLE).rest_height


def run(editor: WorldEditor, constructor: str, action: dict) -> None:
    """Caller code is identical for every constructor: look it up by name, apply the command it builds."""
    editor.apply(SCENE_CONSTRUCTORS.get(constructor).build(action, editor.world))


def test_plan2d_box_from_a_dragged_rectangle_in_any_direction():
    ed = WorldEditor(WorldSpec())
    run(ed, "plan2d", {"op": "box", "corner_a": [4.0, 3.0], "corner_b": [2.0, -1.0], "height": 2.5})
    assert ed.world.obstacles == (ObstacleSpec("box1", Box((3.0, 1.0, 1.25), (1.0, 2.0, 1.25))),)


def test_plan2d_pillar_from_a_circle():
    ed = WorldEditor(WorldSpec())
    run(ed, "plan2d", {"op": "pillar", "center": [-2.0, 5.0], "radius": 0.4, "height": 6.0})
    assert ed.world.obstacles == (ObstacleSpec("pillar1", Cylinder((-2.0, 5.0, 0.0), 0.4, 6.0)),)


def test_plan2d_drone_is_placed_on_the_ground():
    ed = WorldEditor(WorldSpec())
    run(ed, "plan2d", {"op": "drone", "at": [1.0, 2.0], "yaw": 0.5})
    assert ed.world.drones == (DroneSpec("drone1", position=(1.0, 2.0, REST_Z), yaw=0.5),)


def test_generated_ids_skip_ids_already_in_use():
    ed = WorldEditor(WorldSpec())
    ed.apply(AddObstacle(ObstacleSpec("box1", Box((10.0, 10.0, 1.0), (1.0, 1.0, 1.0)))))
    run(ed, "plan2d", {"op": "box", "corner_a": [0.0, 0.0], "corner_b": [1.0, 1.0], "height": 1.0})
    run(ed, "plan2d", {"op": "drone", "at": [5.0, 5.0], "id": "scout"})
    assert [o.id for o in ed.world.obstacles] == ["box1", "box2"]
    assert ed.world.drones[0].id == "scout"


@pytest.mark.parametrize(
    "action",
    [
        {"op": "teleport"},
        {"op": "box", "corner_a": [0.0, 0.0], "corner_b": [0.0, 2.0], "height": 1.0},  # zero width
        {"op": "box", "corner_a": [0.0, 0.0], "corner_b": [1.0, 1.0], "height": 0.0},
        {"op": "pillar", "center": [0.0, 0.0], "radius": 0.0, "height": 1.0},
    ],
)
def test_plan2d_rejects_bad_actions(action):
    ed = WorldEditor(WorldSpec())
    with pytest.raises(InvalidEdit):
        run(ed, "plan2d", action)
    assert ed.world == WorldSpec()


# --- swappability: a second, test-only constructor goes through exactly the same caller code ----------------------


@dataclass(frozen=True)
class GridGenerator:
    """Procedural method: an n x n grid of pillars in one action."""

    def build(self, action, world):
        n, spacing = action["n"], action["spacing"]
        pillars = [
            ObstacleSpec(f"g{i}_{j}", Cylinder((i * spacing, j * spacing, 0.0), 0.2, 3.0))
            for i in range(n)
            for j in range(n)
        ]
        return Batch(tuple(AddObstacle(p) for p in pillars))


if "test_grid" not in SCENE_CONSTRUCTORS:
    SCENE_CONSTRUCTORS.register("test_grid", GridGenerator())


def test_a_different_construction_method_plugs_in_by_name():
    ed = WorldEditor(WorldSpec())
    run(ed, "plan2d", {"op": "drone", "at": [-5.0, -5.0]})
    run(ed, "test_grid", {"n": 3, "spacing": 2.0})
    assert len(ed.world.obstacles) == 9 and len(ed.world.drones) == 1
    ed.undo()  # the whole generated grid is one edit
    assert ed.world.obstacles == () and len(ed.world.drones) == 1


def test_a_batch_is_validated_as_a_whole():
    ed = WorldEditor(WorldSpec())
    run(ed, "plan2d", {"op": "drone", "at": [2.0, 2.0]})
    before = ed.world
    with pytest.raises(InvalidEdit, match="inside obstacle"):
        run(ed, "test_grid", {"n": 2, "spacing": 2.0})  # pillar g1_1 lands on the drone
    assert ed.world == before


def test_plan2d_places_each_vehicle_at_its_own_rest_height():
    if "test_tall_legs" not in vehicle_ids():
        register_vehicle("test_tall_legs", DroneParams(rest_height=0.15))
    ed = WorldEditor(WorldSpec())
    run(ed, "plan2d", {"op": "drone", "at": [0.0, 0.0], "vehicle": "test_tall_legs"})
    run(ed, "plan2d", {"op": "drone", "at": [2.0, 0.0]})
    assert [d.position[2] for d in ed.world.drones] == [0.15, 0.07]
