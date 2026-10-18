"""api layer: protocol messages and binary state frames round-trip; malformed input is rejected."""

import json

import numpy as np
import pytest

from drone_sim.api.protocol import (
    PROTOCOL_VERSION,
    Construct,
    DroneStatus,
    Edit,
    Frame,
    GetWorld,
    Hello,
    ProtocolError,
    SetWorld,
    Simple,
    decode_frame,
    encode_client,
    encode_frame,
    parse_client,
    parse_server,
    server_error,
    server_event,
    server_report,
    server_state,
    server_welcome,
    server_world,
)
from drone_sim.core.geometry import Box, Cylinder
from drone_sim.engine import DroneCollided, RunFinished, RunReport, TaskFailed, TaskOutcome, TaskSucceeded
from drone_sim.tasks import FailureReason, TaskStatus
from drone_sim.world import (
    AddObstacle,
    AssignTask,
    DroneSpec,
    MoveEntity,
    ObstacleSpec,
    PlaceDrone,
    RemoveEntity,
    RemoveTask,
    TaskSpec,
    WorldSpec,
)

WORLD = WorldSpec(
    drones=(DroneSpec("d1", position=(1.0, 2.0, 0.07), yaw=0.5),),
    obstacles=(ObstacleSpec("p1", Cylinder((3.0, 0.0, 0.0), 0.4, 5.0)),),
    tasks=(TaskSpec("t1", "d1", "goto_point", {"target": [5.0, 0.0, 2.0]}),),
)

CLIENT_MESSAGES = [
    Hello(PROTOCOL_VERSION),
    Construct("plan2d", {"op": "box", "corner_a": [0, 0], "corner_b": [1, 2], "height": 2.0}),
    Edit(PlaceDrone(DroneSpec("d2", position=(0.0, 1.0, 0.07)))),
    Edit(AddObstacle(ObstacleSpec("b1", Box((0.0, 5.0, 1.0), (1.0, 1.0, 1.0))))),
    Edit(MoveEntity("d1", (2.0, 2.0, 0.07), yaw=1.0)),
    Edit(MoveEntity("p1", (4.0, 0.0, 0.0))),
    Edit(RemoveEntity("p1")),
    Edit(AssignTask(TaskSpec("t2", "d1", "goto_point", {"target": [1.0, 1.0, 1.0]}))),
    Edit(RemoveTask("t1")),
    *(Simple(kind) for kind in ("undo", "redo", "start", "pause", "resume", "reset")),
    SetWorld(WORLD),
    GetWorld(),
]


@pytest.mark.parametrize("message", CLIENT_MESSAGES, ids=lambda m: type(m).__name__)
def test_every_client_message_round_trips_through_json(message):
    text = encode_client(message)
    assert isinstance(json.loads(text), dict)
    assert parse_client(text) == message


@pytest.mark.parametrize(
    "text",
    [
        server_welcome(WORLD, "edit"),
        server_world(WORLD),
        server_state("running", 3.25),
        server_error("drone 'd9' is outside the arena"),
        server_event(TaskSucceeded(1.5, "t1", "d1")),
        server_event(TaskFailed(2.0, "t1", "d1", FailureReason.COLLISION)),
        server_event(DroneCollided(2.0, "d1")),
        server_event(RunFinished(2.0)),
        server_report(RunReport(4.0, (TaskOutcome("t1", "d1", TaskStatus.FAILED, FailureReason.TIMEOUT, 4.0),))),
        server_report(RunReport(4.0, (TaskOutcome("t1", "d1", TaskStatus.ACTIVE, FailureReason.NONE, None),))),
    ],
)
def test_every_server_message_parses_back(text):
    data = parse_server(text)
    assert json.loads(text)["type"] == data["type"]


def test_server_payloads_decode_to_domain_objects():
    assert parse_server(server_world(WORLD))["world"] == WORLD
    assert parse_server(server_welcome(WORLD, "edit")) == {
        "type": "welcome",
        "version": PROTOCOL_VERSION,
        "world": WORLD,
        "state": "edit",
    }
    event = TaskFailed(2.0, "t1", "d1", FailureReason.COLLISION)
    assert parse_server(server_event(event))["event"] == event
    report = RunReport(4.0, (TaskOutcome("t1", "d1", TaskStatus.SUCCEEDED, FailureReason.NONE, 3.0),))
    assert parse_server(server_report(report))["report"] == report


@pytest.mark.parametrize("n", [0, 1, 500])
def test_frame_round_trip(n):
    rng = np.random.default_rng(n)
    q = rng.normal(size=(n, 4))
    frame = Frame(
        seq=42,
        time=12.5,
        pos=rng.uniform(-30, 30, (n, 3)).astype(np.float32),
        quat=(q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-9)).astype(np.float32),
        status=rng.integers(0, len(DroneStatus), n).astype(np.uint8),
    )
    back = decode_frame(encode_frame(frame))
    assert back.seq == 42 and back.time == 12.5
    np.testing.assert_array_equal(back.pos, frame.pos)
    np.testing.assert_array_equal(back.quat, frame.quat)
    np.testing.assert_array_equal(back.status, frame.status)


def test_a_500_drone_frame_is_under_25_kb():
    n = 500
    frame = Frame(0, 0.0, np.zeros((n, 3), np.float32), np.zeros((n, 4), np.float32), np.zeros(n, np.uint8))
    assert len(encode_frame(frame)) < 25_000


@pytest.mark.parametrize(
    "text, message",
    [
        ("not json", "invalid JSON"),
        ("[1, 2]", "object"),
        ('{"no": "type"}', "type"),
        ('{"type": "teleport"}', "unknown message type 'teleport'"),
        ('{"type": "hello", "version": 99}', "version"),
        ('{"type": "edit", "command": {"op": "explode"}}', "unknown edit op 'explode'"),
        ('{"type": "edit", "command": {"op": "move"}}', "missing"),
        ('{"type": "construct", "action": {}}', "missing"),
    ],
)
def test_malformed_client_messages_raise_protocol_errors(text, message):
    with pytest.raises(ProtocolError, match=message):
        parse_client(text)


def test_bad_frames_are_rejected():
    with pytest.raises(ProtocolError, match="magic"):
        decode_frame(b"XXXX" + bytes(16))
    with pytest.raises(ProtocolError, match="length"):
        decode_frame(encode_frame(Frame(0, 0.0, np.zeros((2, 3)), np.zeros((2, 4)), np.zeros(2)))[:-1])
