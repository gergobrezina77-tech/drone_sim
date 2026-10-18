"""The simulator server: one shared simulation, any number of WebSocket clients.

``Session`` is the whole protocol logic and is synchronous (tested without a network). ``create_app`` wraps it
in FastAPI: a WebSocket at ``/ws`` and a background loop that advances the simulation at the frame rate, running the
ticks in a worker thread so the event loop stays responsive. Optionally serves the built web UI at ``/``.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from drone_sim.api.protocol import (
    Construct,
    DroneStatus,
    Edit,
    Frame,
    GetWorld,
    Hello,
    ProtocolError,
    SetWorld,
    Simple,
    encode_frame,
    parse_client,
    server_error,
    server_event,
    server_report,
    server_state,
    server_welcome,
    server_world,
)
from drone_sim.engine import DroneCollided, IllegalTransition, SimState, Simulation
from drone_sim.tasks import TaskStatus
from drone_sim.world import SCENE_CONSTRUCTORS, InvalidEdit, WorldSpec

Message = str | bytes


@dataclass
class Outgoing:
    reply: list[Message] = field(default_factory=list)  # to the sender only
    broadcast: list[Message] = field(default_factory=list)  # to every client


@dataclass(frozen=True)
class ReplaceWorld:
    """Editor command for loading a whole world (undoable like any edit)."""

    world: WorldSpec

    def apply(self, world: WorldSpec) -> WorldSpec:
        return self.world


class Session:
    def __init__(self, sim: Simulation, speed: float = 1.0):
        self.sim, self.speed = sim, speed
        self._seq = 0
        self._sent_events = 0
        self._target_time = 0.0

    # --- client messages -----------------------------------------------------------------------------------------

    def handle(self, text: str) -> Outgoing:
        try:
            return self._dispatch(parse_client(text))
        except (ProtocolError, InvalidEdit, IllegalTransition, KeyError, ValueError) as e:
            return Outgoing(reply=[server_error(e.args[0] if e.args else str(e))])

    def _dispatch(self, m) -> Outgoing:
        sim = self.sim
        if isinstance(m, Hello):
            return Outgoing(reply=[server_welcome(sim.world, sim.state.value)])
        if isinstance(m, Construct):
            sim.edit(SCENE_CONSTRUCTORS.get(m.constructor).build(m.action, sim.world))
        elif isinstance(m, Edit):
            sim.edit(m.command)
        elif isinstance(m, SetWorld):
            sim.edit(ReplaceWorld(m.world))
        elif isinstance(m, GetWorld):
            return Outgoing(reply=[server_world(sim.world)])
        elif isinstance(m, Simple) and m.kind in ("undo", "redo"):
            if not (sim.undo() if m.kind == "undo" else sim.redo()):
                raise IllegalTransition(f"nothing to {m.kind}")
        elif isinstance(m, Simple) and m.kind == "start":
            sim.start()
            self._sent_events, self._target_time = 0, 0.0
            return Outgoing(broadcast=[self._state(), self._frame()])
        elif isinstance(m, Simple):
            {"pause": sim.pause, "resume": sim.resume, "reset": sim.reset}[m.kind]()
            return Outgoing(broadcast=[self._state()] + ([server_world(sim.world)] if m.kind == "reset" else []))
        return Outgoing(broadcast=[server_world(sim.world)])

    # --- running -------------------------------------------------------------------------------------------------

    def advance(self, wall_seconds: float) -> list[Message]:
        """Run the simulation for ``wall_seconds * speed`` of simulated time (if running); return what to send."""
        sim = self.sim
        if sim.state is not SimState.RUNNING:
            return []
        self._target_time += wall_seconds * self.speed
        nominal = wall_seconds * self.speed / sim.control_dt
        ticks = int((self._target_time - sim.time) / sim.control_dt + 1e-9)
        if ticks > 2 * nominal + 1:  # falling behind: run slower than real time instead of spiralling
            ticks = int(2 * nominal + 1)
            self._target_time = sim.time + ticks * sim.control_dt
        for _ in range(ticks):
            sim.tick()
            if sim.state is not SimState.RUNNING:
                break
        out: list[Message] = [self._frame()]
        out += [server_event(e) for e in sim.events[self._sent_events :]]
        self._sent_events = len(sim.events)
        if sim.state is SimState.FINISHED:
            out += [self._state(), server_report(sim.report())]
        return out

    def _state(self) -> str:
        return server_state(self.sim.state.value, self.sim.time)

    def _frame(self) -> bytes:
        s = self.sim.fleet_state()
        frame = Frame(self._seq, self.sim.time, s.pos, s.quat, self._drone_status())
        self._seq += 1
        return encode_frame(frame)

    def _drone_status(self) -> np.ndarray:
        sim = self.sim
        status = np.full(len(sim.world.drones), DroneStatus.IDLE, dtype=np.uint8)
        if sim.state is SimState.EDIT:
            return status
        b = sim.board
        status[b.drone] = np.select(
            [b.status == TaskStatus.SUCCEEDED, b.status == TaskStatus.FAILED],
            [DroneStatus.SUCCEEDED, DroneStatus.FAILED],
            DroneStatus.ACTIVE,
        )
        index = {d.id: i for i, d in enumerate(sim.world.drones)}
        crashed = [index[e.drone_id] for e in sim.events if isinstance(e, DroneCollided)]
        status[crashed] = DroneStatus.FAILED
        return status


def create_app(
    sim_factory: Callable[[], Simulation] = Simulation,
    speed: float = 1.0,
    frame_rate: float = 30.0,
    static_dir: Path | None = None,
) -> FastAPI:
    session = Session(sim_factory(), speed)
    clients: set[WebSocket] = set()
    lock = asyncio.Lock()

    async def broadcast(messages: list[Message]) -> None:
        for ws in list(clients):
            try:
                for m in messages:
                    await (ws.send_bytes(m) if isinstance(m, bytes) else ws.send_text(m))
            except (WebSocketDisconnect, RuntimeError):  # a client that went away mid-send
                clients.discard(ws)

    async def run_loop() -> None:
        period, last = 1.0 / frame_rate, time.monotonic()
        while True:
            await asyncio.sleep(max(0.0, last + period - time.monotonic()))
            now = time.monotonic()
            async with lock:
                out = await asyncio.to_thread(session.advance, now - last)
            last = now
            if out:
                await broadcast(out)

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        task = asyncio.create_task(run_loop())
        yield
        task.cancel()

    app = FastAPI(title="drone-sim", lifespan=lifespan)
    app.state.session = session

    @app.websocket("/ws")
    async def websocket(ws: WebSocket) -> None:
        await ws.accept()
        clients.add(ws)
        try:
            while True:
                text = await ws.receive_text()
                async with lock:
                    out = session.handle(text)
                for m in out.reply:
                    await ws.send_text(m)
                await broadcast(out.broadcast)
        except WebSocketDisconnect:
            clients.discard(ws)

    if static_dir is not None and static_dir.is_dir():
        app.mount("/", StaticFiles(directory=static_dir, html=True), name="web")
    return app


def main() -> None:
    import uvicorn

    p = argparse.ArgumentParser(description="Run the drone-sim server (WebSocket at /ws, web UI at /).")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--speed", type=float, default=1.0, help="simulated seconds per wall-clock second")
    p.add_argument("--static", type=Path, default=Path("web/dist"), help="built web UI to serve, if present")
    a = p.parse_args()
    uvicorn.run(create_app(speed=a.speed, static_dir=a.static), host=a.host, port=a.port)
