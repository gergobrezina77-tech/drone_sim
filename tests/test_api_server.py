"""api layer: the session logic (no network) and the WebSocket app end to end."""

import json
import time

import numpy as np
import pytest
from fastapi.testclient import TestClient

from drone_sim.api.protocol import (
    PROTOCOL_VERSION,
    Construct,
    DroneStatus,
    Edit,
    GetWorld,
    Hello,
    SetWorld,
    Simple,
    decode_frame,
    encode_client,
    parse_server,
)
from drone_sim.api.server import Session, create_app
from drone_sim.engine import Simulation, TaskFailed, TaskSucceeded
from drone_sim.engine.benchmark import benchmark_world
from drone_sim.tasks import FailureReason
from drone_sim.world import AssignTask, TaskSpec, WorldEditor, WorldSpec

GOTO = AssignTask(TaskSpec("t1", "drone1", "goto_point", {"target": [3.0, 0.0, 2.0], "dwell": 0.3}))
WALL = Construct("plan2d", {"op": "box", "corner_a": [1.4, -3.0], "corner_b": [1.6, 3.0], "height": 4.0})


def send(session, message):
    out = session.handle(encode_client(message))
    return [parse_server(m) for m in out.reply], out.broadcast


def build(session, *messages):
    for m in (Construct("plan2d", {"op": "drone", "at": [0.0, 0.0]}), *messages):
        replies, _ = send(session, m)
        assert not any(r["type"] == "error" for r in replies), replies


def run_session(session, wall_seconds):
    out = []
    for _ in range(round(wall_seconds * 30)):
        out += session.advance(1 / 30)
    return [m for m in out if isinstance(m, bytes)], [parse_server(m) for m in out if isinstance(m, str)]


# --- session -----------------------------------------------------------------------------------------------------


def test_hello_gets_a_welcome_with_the_world():
    s = Session(Simulation())
    replies, broadcast = send(s, Hello(PROTOCOL_VERSION))
    assert replies == [{"type": "welcome", "version": PROTOCOL_VERSION, "world": WorldSpec(), "state": "edit"}]
    assert broadcast == []


def test_authoring_and_edits_broadcast_the_new_world():
    s = Session(Simulation())
    _, broadcast = send(s, Construct("plan2d", {"op": "drone", "at": [1.0, 2.0]}))
    world = parse_server(broadcast[0])["world"]
    assert [d.id for d in world.drones] == ["drone1"]
    _, broadcast = send(s, Edit(AssignTask(TaskSpec("t1", "drone1", "goto_point", {"target": [1, 1, 1]}))))
    assert len(parse_server(broadcast[0])["world"].tasks) == 1
    _, broadcast = send(s, Simple("undo"))
    assert parse_server(broadcast[0])["world"].tasks == ()


def test_rejected_edits_reply_with_an_error_and_change_nothing():
    s = Session(Simulation())
    for bad in (
        '{"type": "teleport"}',
        encode_client(Construct("plan2d", {"op": "drone", "at": [999.0, 0.0]})),
        encode_client(Construct("no_such_constructor", {})),
        encode_client(Simple("undo")),
        encode_client(Simple("pause")),
    ):
        out = s.handle(bad)
        assert [parse_server(m)["type"] for m in out.reply] == ["error"] and out.broadcast == []
    assert s.sim.world == WorldSpec()


def test_get_and_set_world():
    s = Session(Simulation())
    build(s, Edit(GOTO))
    replies, _ = send(s, GetWorld())
    saved = replies[0]["world"]
    send(s, Simple("start"))
    send(s, Simple("reset"))
    _, broadcast = send(s, SetWorld(WorldSpec()))
    assert parse_server(broadcast[0])["world"] == WorldSpec()
    send(s, Simple("undo"))  # loading is undoable
    assert s.sim.world == saved


def test_a_run_streams_frames_and_events_and_ends_with_a_report():
    s = Session(Simulation(), speed=4.0)
    build(s, Edit(GOTO))
    _, broadcast = send(s, Simple("start"))
    assert parse_server(broadcast[0]) == {"type": "state", "state": "running", "time": 0.0}
    frames, messages = run_session(s, wall_seconds=3.0)
    first, last = decode_frame(frames[0]), decode_frame(frames[-1])
    assert first.status.tolist() == [DroneStatus.ACTIVE] and last.status.tolist() == [DroneStatus.SUCCEEDED]
    assert np.linalg.norm(last.pos[0] - (3.0, 0.0, 2.0)) < 0.3
    seqs = [d.seq for d in map(decode_frame, frames)]
    assert seqs == list(range(seqs[0], seqs[0] + len(seqs)))  # consecutive (start's own frame came first)
    kinds = [m["type"] for m in messages]
    assert kinds == ["event", "event", "state", "report"]
    assert isinstance(messages[0]["event"], TaskSucceeded) and messages[2]["state"] == "finished"


def test_a_crash_marks_the_drone_failed_in_frames():
    s = Session(Simulation(), speed=4.0)
    build(s, WALL, Edit(GOTO))
    send(s, Simple("start"))
    frames, messages = run_session(s, wall_seconds=2.0)
    assert decode_frame(frames[-1]).status.tolist() == [DroneStatus.FAILED]
    failed = [m["event"] for m in messages if m["type"] == "event" and isinstance(m["event"], TaskFailed)]
    assert failed[0].reason is FailureReason.COLLISION


def test_pause_stops_time_and_reset_returns_to_edit_with_the_world():
    s = Session(Simulation())
    build(s, Edit(GOTO))
    send(s, Simple("start"))
    run_session(s, 0.2)
    send(s, Simple("pause"))
    t = s.sim.time
    frames, _ = run_session(s, 0.2)
    assert s.sim.time == t and frames == []
    _, broadcast = send(s, Simple("reset"))
    assert [parse_server(m)["type"] for m in broadcast] == ["state", "world"]


def test_session_keeps_up_with_100_drones_in_real_time():
    s = Session(Simulation(WorldEditor(benchmark_world(100))))
    send(s, Simple("start"))
    s.advance(1 / 30)
    t0 = time.perf_counter()
    for _ in range(30):
        s.advance(1 / 30)
    wall = time.perf_counter() - t0
    assert s.sim.time == pytest.approx(31 / 30, abs=0.01)
    assert wall < 1.0, f"1 s of simulated time took {wall:.2f} s"


# --- app over a real WebSocket ------------------------------------------------------------------------------------


def test_websocket_end_to_end():
    app = create_app(speed=8.0)
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        ws.send_text(encode_client(Hello(PROTOCOL_VERSION)))
        assert json.loads(ws.receive_text())["type"] == "welcome"
        ws.send_text(encode_client(Construct("plan2d", {"op": "drone", "at": [0.0, 0.0]})))
        assert json.loads(ws.receive_text())["type"] == "world"
        ws.send_text(encode_client(Edit(GOTO)))
        assert json.loads(ws.receive_text())["type"] == "world"
        ws.send_text(encode_client(Simple("start")))
        frames, events = 0, []
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            message = ws.receive()
            if message.get("bytes") is not None:
                frames += 1
            elif message.get("text") is not None:
                data = parse_server(message["text"])
                if data["type"] == "event":
                    events.append(data["event"])
                if data["type"] == "report":
                    break
        assert frames > 0
        assert any(isinstance(e, TaskSucceeded) for e in events)
