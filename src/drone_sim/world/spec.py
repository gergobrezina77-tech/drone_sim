"""WorldSpec: the single editable, serialisable description of a scenario.

Specs are immutable values; edits produce new specs. Task parameters and environment settings are free-form dicts
keyed by type/effect name, so new task types and effects need no schema change.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from typing import Any

from drone_sim.core.geometry import Box, Cylinder, Shape
from drone_sim.vehicle import DEFAULT_VEHICLE

SCHEMA_VERSION = 1

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class ObstacleSpec:
    id: str
    shape: Shape


@dataclass(frozen=True)
class DroneSpec:
    id: str
    position: Vec3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    vehicle: str = DEFAULT_VEHICLE


@dataclass(frozen=True)
class TaskSpec:
    id: str
    drone_id: str
    type: str
    params: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class WorldSpec:
    arena_size: Vec3 = (60.0, 60.0, 20.0)  # x, y extent centred on the origin; height above the floor at z = 0
    obstacles: tuple[ObstacleSpec, ...] = ()
    drones: tuple[DroneSpec, ...] = ()
    tasks: tuple[TaskSpec, ...] = ()
    environment: dict[str, dict[str, Any]] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "arena_size": list(self.arena_size),
            "obstacles": [{"id": o.id, "shape": _shape_to_dict(o.shape)} for o in self.obstacles],
            "drones": [
                {"id": d.id, "position": list(d.position), "yaw": d.yaw, "vehicle": d.vehicle} for d in self.drones
            ],
            "tasks": [{"id": t.id, "drone_id": t.drone_id, "type": t.type, "params": t.params} for t in self.tasks],
            "environment": self.environment,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> WorldSpec:
        if data.get("schema_version") != SCHEMA_VERSION:
            raise ValueError(
                f"unsupported world schema version {data.get('schema_version')!r}; expected {SCHEMA_VERSION}"
            )
        return cls(
            arena_size=_vec3(data["arena_size"]),
            obstacles=tuple(ObstacleSpec(o["id"], _shape_from_dict(o["shape"])) for o in data["obstacles"]),
            drones=tuple(
                DroneSpec(d["id"], _vec3(d["position"]), float(d["yaw"]), d["vehicle"]) for d in data["drones"]
            ),
            tasks=tuple(TaskSpec(t["id"], t["drone_id"], t["type"], t["params"]) for t in data["tasks"]),
            environment=data["environment"],
        )

    @classmethod
    def from_json(cls, text: str) -> WorldSpec:
        return cls.from_dict(json.loads(text))

    def geometry_hash(self) -> str:
        """Hash of what a compiled physics model depends on: arena, obstacles, and each drone's vehicle type.

        Start poses, tasks and environment settings are excluded, so changing them allows a fast restart.
        """
        d = self.to_dict()
        key = {"arena": d["arena_size"], "obstacles": d["obstacles"], "vehicles": [x.vehicle for x in self.drones]}
        return hashlib.sha256(json.dumps(key, sort_keys=True).encode()).hexdigest()


def _vec3(v) -> Vec3:
    x, y, z = v
    return float(x), float(y), float(z)


def _shape_to_dict(s: Shape) -> dict[str, Any]:
    if isinstance(s, Box):
        return {"type": "box", "center": list(s.center), "half_extents": list(s.half_extents)}
    return {"type": "cylinder", "base": list(s.base), "radius": s.radius, "height": s.height}


def _shape_from_dict(d: dict[str, Any]) -> Shape:
    if d["type"] == "box":
        return Box(_vec3(d["center"]), _vec3(d["half_extents"]))
    if d["type"] == "cylinder":
        return Cylinder(_vec3(d["base"]), float(d["radius"]), float(d["height"]))
    raise ValueError(f"unknown shape type {d['type']!r}")
