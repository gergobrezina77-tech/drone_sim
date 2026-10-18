"""World: the editable, serialisable WorldSpec, editing commands with undo/redo, scene constructors and JSON I/O.

May import: vehicle, core.
"""

from drone_sim.world.editor import (
    AddObstacle,
    AssignTask,
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
    "SCHEMA_VERSION",
    "AddObstacle",
    "AssignTask",
    "Command",
    "DroneSpec",
    "InvalidEdit",
    "MoveEntity",
    "ObstacleSpec",
    "PlaceDrone",
    "RemoveEntity",
    "RemoveTask",
    "TaskSpec",
    "WorldEditor",
    "WorldSpec",
    "validate",
]
