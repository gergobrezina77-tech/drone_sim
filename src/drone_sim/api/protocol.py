"""Wire protocol between the simulator and any UI. See docs/PROTOCOL.md.

Client -> server: JSON text messages, ``{"type": ...}``.
Server -> client: JSON text messages, plus binary state frames while a run exists.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

import numpy as np

from drone_sim.engine import DroneCollided, Event, RunFinished, RunReport, TaskFailed, TaskOutcome, TaskSucceeded
from drone_sim.tasks import FailureReason, TaskStatus
from drone_sim.world import (
    AddObstacle,
    AssignTask,
    Command,
    DroneSpec,
    MoveEntity,
    ObstacleSpec,
    PlaceDrone,
    RemoveEntity,
    RemoveTask,
    TaskSpec,
    WorldSpec,
)

PROTOCOL_VERSION = 1


class ProtocolError(ValueError):
    pass


# --- client -> server ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Hello:
    version: int


@dataclass(frozen=True)
class Construct:
    """An authoring action for a registered scene constructor (e.g. ``plan2d``)."""

    constructor: str
    action: dict[str, Any]


@dataclass(frozen=True)
class Edit:
    command: Command


@dataclass(frozen=True)
class Simple:
    """Messages without a payload: undo, redo, start, pause, resume, reset."""

    kind: str


@dataclass(frozen=True)
class SetWorld:
    world: WorldSpec


@dataclass(frozen=True)
class GetWorld:
    pass


SIMPLE_KINDS = ("undo", "redo", "start", "pause", "resume", "reset")
ClientMessage = Hello | Construct | Edit | Simple | SetWorld | GetWorld


def command_to_dict(c: Command) -> dict[str, Any]:
    if isinstance(c, PlaceDrone):
        return {"op": "place_drone", "drone": c.drone.to_dict()}
    if isinstance(c, AddObstacle):
        return {"op": "add_obstacle", "obstacle": c.obstacle.to_dict()}
    if isinstance(c, MoveEntity):
        return {"op": "move", "id": c.id, "position": list(c.position)} | ({} if c.yaw is None else {"yaw": c.yaw})
    if isinstance(c, RemoveEntity):
        return {"op": "remove", "id": c.id}
    if isinstance(c, AssignTask):
        return {"op": "assign_task", "task": c.task.to_dict()}
    if isinstance(c, RemoveTask):
        return {"op": "remove_task", "id": c.id}
    raise ProtocolError(f"command {type(c).__name__} has no wire format")


def command_from_dict(d: dict[str, Any]) -> Command:
    op = d.get("op")
    if op == "place_drone":
        return PlaceDrone(DroneSpec.from_dict(d["drone"]))
    if op == "add_obstacle":
        return AddObstacle(ObstacleSpec.from_dict(d["obstacle"]))
    if op == "move":
        x, y, z = d["position"]
        return MoveEntity(d["id"], (float(x), float(y), float(z)), None if d.get("yaw") is None else float(d["yaw"]))
    if op == "remove":
        return RemoveEntity(d["id"])
    if op == "assign_task":
        return AssignTask(TaskSpec.from_dict(d["task"]))
    if op == "remove_task":
        return RemoveTask(d["id"])
    raise ProtocolError(f"unknown edit op {op!r}")


def encode_client(m: ClientMessage) -> str:
    if isinstance(m, Hello):
        data = {"type": "hello", "version": m.version}
    elif isinstance(m, Construct):
        data = {"type": "construct", "constructor": m.constructor, "action": m.action}
    elif isinstance(m, Edit):
        data = {"type": "edit", "command": command_to_dict(m.command)}
    elif isinstance(m, Simple):
        data = {"type": m.kind}
    elif isinstance(m, SetWorld):
        data = {"type": "set_world", "world": m.world.to_dict()}
    else:
        data = {"type": "get_world"}
    return json.dumps(data)


def parse_client(text: str) -> ClientMessage:
    data = _load_object(text)
    kind = data.get("type")
    try:
        if kind == "hello":
            if data["version"] != PROTOCOL_VERSION:
                raise ProtocolError(
                    f"protocol version {data['version']!r} not supported; server speaks {PROTOCOL_VERSION}"
                )
            return Hello(data["version"])
        if kind == "construct":
            return Construct(data["constructor"], data["action"])
        if kind == "edit":
            return Edit(command_from_dict(data["command"]))
        if kind in SIMPLE_KINDS:
            return Simple(kind)
        if kind == "set_world":
            return SetWorld(WorldSpec.from_dict(data["world"]))
        if kind == "get_world":
            return GetWorld()
    except (KeyError, TypeError, ValueError) as e:
        if isinstance(e, ProtocolError):
            raise
        raise ProtocolError(f"malformed {kind!r} message: missing or invalid {e}") from None
    if kind is None:
        raise ProtocolError("message has no 'type'")
    raise ProtocolError(f"unknown message type {kind!r}")


def _load_object(text: str) -> dict[str, Any]:
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        raise ProtocolError(f"invalid JSON: {e}") from None
    if not isinstance(data, dict):
        raise ProtocolError("a message must be a JSON object")
    return data


# --- server -> client: JSON ------------------------------------------------------------------------------------


def server_welcome(world: WorldSpec, state: str) -> str:
    return json.dumps({"type": "welcome", "version": PROTOCOL_VERSION, "world": world.to_dict(), "state": state})


def server_world(world: WorldSpec) -> str:
    return json.dumps({"type": "world", "world": world.to_dict()})


def server_state(state: str, time: float) -> str:
    return json.dumps({"type": "state", "state": state, "time": time})


def server_error(message: str) -> str:
    return json.dumps({"type": "error", "message": message})


def event_to_dict(e: Event) -> dict[str, Any]:
    if isinstance(e, TaskSucceeded):
        return {"kind": "task_succeeded", "time": e.time, "task_id": e.task_id, "drone_id": e.drone_id}
    if isinstance(e, TaskFailed):
        return {
            "kind": "task_failed",
            "time": e.time,
            "task_id": e.task_id,
            "drone_id": e.drone_id,
            "reason": e.reason.name.lower(),
        }
    if isinstance(e, DroneCollided):
        return {"kind": "drone_collided", "time": e.time, "drone_id": e.drone_id}
    return {"kind": "run_finished", "time": e.time}


def event_from_dict(d: dict[str, Any]) -> Event:
    kind = d["kind"]
    if kind == "task_succeeded":
        return TaskSucceeded(d["time"], d["task_id"], d["drone_id"])
    if kind == "task_failed":
        return TaskFailed(d["time"], d["task_id"], d["drone_id"], FailureReason[d["reason"].upper()])
    if kind == "drone_collided":
        return DroneCollided(d["time"], d["drone_id"])
    return RunFinished(d["time"])


def server_event(e: Event) -> str:
    return json.dumps({"type": "event", "event": event_to_dict(e)})


def server_report(r: RunReport) -> str:
    outcomes = [
        {
            "task_id": o.task_id,
            "drone_id": o.drone_id,
            "status": o.status.name.lower(),
            "reason": o.reason.name.lower(),
            "time": o.time,
        }
        for o in r.outcomes
    ]
    return json.dumps({"type": "report", "report": {"time": r.time, "outcomes": outcomes}})


def parse_server(text: str) -> dict[str, Any]:
    """Decode a server JSON message; world / event / report payloads become domain objects (for Python clients)."""
    data = _load_object(text)
    if "world" in data:
        data["world"] = WorldSpec.from_dict(data["world"])
    if "event" in data:
        data["event"] = event_from_dict(data["event"])
    if "report" in data:
        r = data["report"]
        data["report"] = RunReport(
            r["time"],
            tuple(
                TaskOutcome(
                    o["task_id"],
                    o["drone_id"],
                    TaskStatus[o["status"].upper()],
                    FailureReason[o["reason"].upper()],
                    o["time"],
                )
                for o in r["outcomes"]
            ),
        )
    return data


# --- server -> client: binary state frames ---------------------------------------------------------------------


class DroneStatus(IntEnum):
    IDLE = 0  # no task
    ACTIVE = 1
    SUCCEEDED = 2
    FAILED = 3  # task failed or the drone crashed


FRAME_MAGIC = b"DSF1"
_HEADER = struct.Struct("<4sIdI")  # magic, seq, sim time, n drones -> 20 bytes


@dataclass
class Frame:
    seq: int
    time: float
    pos: np.ndarray  # (n, 3)
    quat: np.ndarray  # (n, 4) w x y z
    status: np.ndarray  # (n,) DroneStatus codes


def encode_frame(f: Frame) -> bytes:
    n = len(f.pos)
    return b"".join(
        (
            _HEADER.pack(FRAME_MAGIC, f.seq, f.time, n),
            np.ascontiguousarray(f.pos, dtype="<f4").tobytes(),
            np.ascontiguousarray(f.quat, dtype="<f4").tobytes(),
            np.ascontiguousarray(f.status, dtype=np.uint8).tobytes(),
        )
    )


def decode_frame(data: bytes) -> Frame:
    if data[:4] != FRAME_MAGIC:
        raise ProtocolError("not a state frame (bad magic)")
    _, seq, time, n = _HEADER.unpack_from(data)
    if len(data) != _HEADER.size + n * (12 + 16 + 1):
        raise ProtocolError(f"state frame length {len(data)} does not match {n} drones")
    o = _HEADER.size
    pos = np.frombuffer(data, "<f4", n * 3, o).reshape(n, 3)
    quat = np.frombuffer(data, "<f4", n * 4, o + n * 12).reshape(n, 4)
    status = np.frombuffer(data, np.uint8, n, o + n * 28)
    return Frame(seq, time, pos, quat, status)
