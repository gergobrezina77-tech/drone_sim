"""World: the editable, serialisable WorldSpec, editing commands with undo/redo, scene constructors and JSON I/O.

May import: vehicle, core.
"""

from drone_sim.world.constructors import SCENE_CONSTRUCTORS, Plan2D, SceneConstructor, new_id
from drone_sim.world.editor import (
    AddObstacle,
    AssignTask,
    Batch,
    Command,
    InvalidEdit,
    MoveEntity,
    PlaceDrone,
    RemoveEntity,
    RemoveTask,
    WorldEditor,
    validate,
)
from drone_sim.world.spec import SCHEMA_VERSION, DroneSpec, ObstacleSpec, TaskSpec, WorldSpec

__all__ = [
    "SCENE_CONSTRUCTORS",
    "SCHEMA_VERSION",
    "AddObstacle",
    "AssignTask",
    "Batch",
    "Command",
    "DroneSpec",
    "InvalidEdit",
    "MoveEntity",
    "ObstacleSpec",
    "PlaceDrone",
    "Plan2D",
    "RemoveEntity",
    "RemoveTask",
    "SceneConstructor",
    "TaskSpec",
    "WorldEditor",
    "WorldSpec",
    "new_id",
    "validate",
]
