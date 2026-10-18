"""Editing a WorldSpec through commands, with validation and undo/redo.

Every command is a pure function ``WorldSpec -> WorldSpec``. The editor validates the result and only then records
it; because specs are immutable, undo/redo are plain snapshot stacks.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
from typing import Protocol

import numpy as np

from drone_sim.core.geometry import Box, Shape, distance
from drone_sim.vehicle import get_vehicle
from drone_sim.world.spec import DroneSpec, ObstacleSpec, TaskSpec, Vec3, WorldSpec


class InvalidEdit(ValueError):
    pass


class Command(Protocol):
    def apply(self, world: WorldSpec) -> WorldSpec: ...


@dataclass(frozen=True)
class PlaceDrone:
    drone: DroneSpec

    def apply(self, world: WorldSpec) -> WorldSpec:
        return replace(world, drones=(*world.drones, self.drone))


@dataclass(frozen=True)
class AddObstacle:
    obstacle: ObstacleSpec

    def apply(self, world: WorldSpec) -> WorldSpec:
        return replace(world, obstacles=(*world.obstacles, self.obstacle))


@dataclass(frozen=True)
class AssignTask:
    task: TaskSpec

    def apply(self, world: WorldSpec) -> WorldSpec:
        return replace(world, tasks=(*world.tasks, self.task))


@dataclass(frozen=True)
class MoveEntity:
    """Move a drone (its start position, optionally yaw) or an obstacle (its box centre / cylinder base)."""

    id: str
    position: Vec3
    yaw: float | None = None

    def apply(self, world: WorldSpec) -> WorldSpec:
        if any(d.id == self.id for d in world.drones):
            yaw = {} if self.yaw is None else {"yaw": self.yaw}
            drones = tuple(replace(d, position=self.position, **yaw) if d.id == self.id else d for d in world.drones)
            return replace(world, drones=drones)
        if any(o.id == self.id for o in world.obstacles):
            obstacles = tuple(
                replace(o, shape=_moved(o.shape, self.position)) if o.id == self.id else o for o in world.obstacles
            )
            return replace(world, obstacles=obstacles)
        raise InvalidEdit(f"no entity {self.id!r} to move")


@dataclass(frozen=True)
class RemoveEntity:
    """Remove a drone (and its tasks) or an obstacle."""

    id: str

    def apply(self, world: WorldSpec) -> WorldSpec:
        if any(d.id == self.id for d in world.drones):
            return replace(
                world,
                drones=tuple(d for d in world.drones if d.id != self.id),
                tasks=tuple(t for t in world.tasks if t.drone_id != self.id),
            )
        if any(o.id == self.id for o in world.obstacles):
            return replace(world, obstacles=tuple(o for o in world.obstacles if o.id != self.id))
        raise InvalidEdit(f"no entity {self.id!r} to remove")


@dataclass(frozen=True)
class RemoveTask:
    id: str

    def apply(self, world: WorldSpec) -> WorldSpec:
        if not any(t.id == self.id for t in world.tasks):
            raise InvalidEdit(f"no task {self.id!r} to remove")
        return replace(world, tasks=tuple(t for t in world.tasks if t.id != self.id))


@dataclass(frozen=True)
class Batch:
    """Several commands as one edit: validated once at the end, undone in one step."""

    commands: tuple[Command, ...]

    def apply(self, world: WorldSpec) -> WorldSpec:
        for command in self.commands:
            world = command.apply(world)
        return world


def _moved(shape: Shape, position: Vec3) -> Shape:
    return replace(shape, center=position) if isinstance(shape, Box) else replace(shape, base=position)


def _bounds(shape: Shape) -> tuple[np.ndarray, np.ndarray]:
    if isinstance(shape, Box):
        c, h = np.array(shape.center), np.array(shape.half_extents)
        return c - h, c + h
    b, r = np.array(shape.base), shape.radius
    return b - (r, r, 0.0), b + (r, r, shape.height)


def validate(world: WorldSpec) -> list[str]:
    """Every rule violated by ``world``, as human-readable messages (empty when valid)."""
    errors = []
    ids = Counter([d.id for d in world.drones] + [o.id for o in world.obstacles] + [t.id for t in world.tasks])
    errors += [f"duplicate id {i!r}" for i, k in ids.items() if k > 1]

    sx, sy, height = world.arena_size
    lo, hi = np.array([-sx / 2, -sy / 2, 0.0]), np.array([sx / 2, sy / 2, height])
    for d in world.drones:
        if np.any(np.array(d.position) < lo) or np.any(np.array(d.position) > hi):
            errors.append(f"drone {d.id!r} is outside the arena")
    for o in world.obstacles:
        omin, omax = _bounds(o.shape)
        if np.any(omin < lo) or np.any(omax > hi):
            errors.append(f"obstacle {o.id!r} is outside the arena")

    radii = []
    for d in world.drones:
        try:
            radii.append(get_vehicle(d.vehicle).radius)
        except KeyError:
            errors.append(f"drone {d.id!r} uses unknown vehicle {d.vehicle!r}")
            radii.append(0.0)
    if world.drones and world.obstacles:
        dist, _ = distance(np.array([d.position for d in world.drones]), [o.shape for o in world.obstacles])
        for i, j in zip(*np.nonzero(dist < np.array(radii)[:, None]), strict=True):
            errors.append(f"drone {world.drones[i].id!r} starts inside obstacle {world.obstacles[j].id!r}")

    drone_ids = {d.id for d in world.drones}
    errors += [
        f"task {t.id!r} refers to unknown drone {t.drone_id!r}" for t in world.tasks if t.drone_id not in drone_ids
    ]
    return errors


class WorldEditor:
    def __init__(self, world: WorldSpec | None = None):
        world = world or WorldSpec()
        if errors := validate(world):
            raise InvalidEdit("; ".join(errors))
        self.world = world
        self._undo: list[WorldSpec] = []
        self._redo: list[WorldSpec] = []

    def apply(self, command: Command) -> None:
        new = command.apply(self.world)
        if errors := validate(new):
            raise InvalidEdit("; ".join(errors))
        self._undo.append(self.world)
        self._redo.clear()
        self.world = new

    def undo(self) -> bool:
        if not self._undo:
            return False
        self._redo.append(self.world)
        self.world = self._undo.pop()
        return True

    def redo(self) -> bool:
        if not self._redo:
            return False
        self._undo.append(self.world)
        self.world = self._redo.pop()
        return True
