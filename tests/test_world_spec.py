"""world layer: WorldSpec data model, JSON round trip, geometry hash."""

import dataclasses
import json

import pytest

from drone_sim.core.geometry import Box, Cylinder
from drone_sim.world import SCHEMA_VERSION, DroneSpec, ObstacleSpec, TaskSpec, WorldSpec


def sample_world() -> WorldSpec:
    return WorldSpec(
        arena_size=(40.0, 30.0, 15.0),
        obstacles=(
            ObstacleSpec("wall", Box(center=(5.0, 0.0, 1.5), half_extents=(0.2, 4.0, 1.5))),
            ObstacleSpec("pillar", Cylinder(base=(-3.0, 2.0, 0.0), radius=0.4, height=5.0)),
        ),
        drones=(
            DroneSpec("d1", position=(0.0, 0.0, 0.07)),
            DroneSpec("d2", position=(1.0, 0.0, 0.07), yaw=1.57),
        ),
        tasks=(TaskSpec("t1", "d1", "goto_point", {"target": [8.0, 0.0, 2.0], "tolerance": 0.3}),),
        environment={"wind": {"mean": [1.0, 0.0, 0.0], "turbulence": 0.2}},
    )


def test_json_round_trip_is_lossless():
    world = sample_world()
    assert WorldSpec.from_json(world.to_json()) == world


def test_json_is_plain_and_versioned():
    data = json.loads(sample_world().to_json())
    assert data["schema_version"] == SCHEMA_VERSION
    assert data["obstacles"][0]["shape"]["type"] == "box"
    assert data["obstacles"][1]["shape"]["type"] == "cylinder"


def test_wrong_schema_version_is_rejected():
    data = json.loads(sample_world().to_json())
    data["schema_version"] = SCHEMA_VERSION + 1
    with pytest.raises(ValueError, match="schema version"):
        WorldSpec.from_json(json.dumps(data))


def test_unknown_task_types_and_params_survive_a_round_trip():
    exotic = TaskSpec("t9", "d2", "survey_area", {"polygon": [[0, 0], [5, 0], [5, 5]], "nested": {"passes": 3}})
    world = dataclasses.replace(sample_world(), tasks=(exotic,))
    assert WorldSpec.from_json(world.to_json()).tasks == (exotic,)


def test_empty_world_round_trips():
    assert WorldSpec.from_json(WorldSpec().to_json()) == WorldSpec()


def test_geometry_hash_ignores_start_poses_and_tasks():
    world = sample_world()
    moved = dataclasses.replace(
        world,
        drones=(dataclasses.replace(world.drones[0], position=(9.0, 9.0, 0.07), yaw=3.0), world.drones[1]),
        tasks=(TaskSpec("t1", "d1", "goto_point", {"target": [0.0, 0.0, 5.0]}),),
        environment={},
    )
    assert moved.geometry_hash() == world.geometry_hash()


def test_geometry_hash_changes_with_what_the_physics_model_depends_on():
    world = sample_world()
    extra_obstacle = dataclasses.replace(
        world, obstacles=(*world.obstacles, ObstacleSpec("box2", Box((0.0, 5.0, 1.0), (1.0, 1.0, 1.0))))
    )
    extra_drone = dataclasses.replace(world, drones=(*world.drones, DroneSpec("d3", position=(2.0, 0.0, 0.07))))
    other_vehicle = dataclasses.replace(world, drones=(world.drones[0], DroneSpec("d2", vehicle="other_quad")))
    bigger_arena = dataclasses.replace(world, arena_size=(80.0, 30.0, 15.0))
    hashes = {w.geometry_hash() for w in (world, extra_obstacle, extra_drone, other_vehicle, bigger_arena)}
    assert len(hashes) == 5


def test_each_entity_round_trips_on_its_own():
    world = sample_world()
    for entity in (*world.obstacles, *world.drones, *world.tasks):
        assert type(entity).from_dict(entity.to_dict()) == entity
    assert world.to_dict()["drones"][0] == world.drones[0].to_dict()
