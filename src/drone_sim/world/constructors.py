"""Scene constructors: interchangeable ways of authoring a world.

A constructor turns one authoring action (a plain dict, e.g. from the UI) into one editor ``Command``; callers look
constructors up by name in ``SCENE_CONSTRUCTORS``. ``plan2d`` is the first: top-down footprints extruded to a
height, and drones placed on the ground.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Protocol

from drone_sim.core.geometry import Box, Cylinder
from drone_sim.core.registry import Registry
from drone_sim.vehicle import DEFAULT_VEHICLE, get_vehicle
from drone_sim.world.editor import AddObstacle, Command, InvalidEdit, PlaceDrone
from drone_sim.world.spec import DroneSpec, ObstacleSpec, WorldSpec


class SceneConstructor(Protocol):
    def build(self, action: Mapping[str, Any], world: WorldSpec) -> Command: ...


SCENE_CONSTRUCTORS = Registry[SceneConstructor]("scene constructor")


def new_id(world: WorldSpec, prefix: str) -> str:
    """First ``{prefix}{k}`` (k = 1, 2, ...) not used by any entity in ``world``."""
    used = {d.id for d in world.drones} | {o.id for o in world.obstacles} | {t.id for t in world.tasks}
    k = 1
    while f"{prefix}{k}" in used:
        k += 1
    return f"{prefix}{k}"


class Plan2D:
    """Top-down authoring. Actions:

    - ``{"op": "box", "corner_a": [x, y], "corner_b": [x, y], "height": h}``: dragged rectangle, extruded from the floor
    - ``{"op": "pillar", "center": [x, y], "radius": r, "height": h}``
    - ``{"op": "drone", "at": [x, y], "yaw": 0.0, "vehicle": "..."}``: placed resting on the ground
    Every action takes an optional ``"id"``; otherwise one is generated.
    """

    def build(self, action: Mapping[str, Any], world: WorldSpec) -> Command:
        op = action.get("op")
        if op == "box":
            (ax, ay), (bx, by), h = action["corner_a"], action["corner_b"], float(action["height"])
            hx, hy = abs(bx - ax) / 2, abs(by - ay) / 2
            if min(hx, hy, h) <= 0:
                raise InvalidEdit("a box needs a non-zero width, depth and height")
            shape = Box(((ax + bx) / 2, (ay + by) / 2, h / 2), (hx, hy, h / 2))
            return AddObstacle(ObstacleSpec(action.get("id") or new_id(world, "box"), shape))
        if op == "pillar":
            (x, y), r, h = action["center"], float(action["radius"]), float(action["height"])
            if min(r, h) <= 0:
                raise InvalidEdit("a pillar needs a positive radius and height")
            shape = Cylinder((float(x), float(y), 0.0), r, h)
            return AddObstacle(ObstacleSpec(action.get("id") or new_id(world, "pillar"), shape))
        if op == "drone":
            x, y = action["at"]
            vehicle = action.get("vehicle", DEFAULT_VEHICLE)
            try:
                rest_z = get_vehicle(vehicle).rest_height
            except KeyError as e:
                raise InvalidEdit(str(e)) from None
            drone = DroneSpec(
                action.get("id") or new_id(world, "drone"),
                position=(float(x), float(y), rest_z),
                yaw=float(action.get("yaw", 0.0)),
                vehicle=vehicle,
            )
            return PlaceDrone(drone)
        raise InvalidEdit(f"unknown plan2d op {op!r}")


SCENE_CONSTRUCTORS.register("plan2d", Plan2D())
