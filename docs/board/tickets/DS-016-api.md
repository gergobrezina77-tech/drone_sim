# DS-016: api: WebSocket server running the engine

- **Status:** Done
- **Layer:** api
- **Milestone:** M2 Server and web UI
- **Depends on:** DS-015

## Goal

Serve the simulation to the browser.

## Scope

- `websockets`-based server; the simulation runs in its own thread at real-time pace with a fixed step; state streamed at ~30 Hz, decoupled from the physics rate.
- Command queue applied between ticks; `drone-sim-server` entry point.

## Acceptance criteria

- [x] Integration test with a Python WebSocket client: place drone, assign task, start, receive frames, receive
  TaskSucceeded. (`tests/test_api_server.py::test_websocket_end_to_end`, FastAPI test client over the real app;
  also smoke-tested manually against a running `drone-sim-server` with the `websockets` client: 16 frames,
  `task_succeeded`, report)
- [x] The server keeps up with 100 drones in real time.
  (`::test_session_keeps_up_with_100_drones_in_real_time`: 1 s simulated in < 1 s wall at 30 frames/s)
- [x] `scripts/check.sh` passes (276 passed). Also session tests for welcome, edits, errors, load/save, frames,
  crash status, pause and reset.

## Notes

- **Stack:** FastAPI + uvicorn (user chose "more flexible and quicker" over plain `websockets`): one app serves the
  WebSocket, future REST endpoints and the built web UI; its test client drives WebSockets in-process. Deps added:
  `fastapi`, `uvicorn`, `websockets` (explicit; uvicorn's WebSocket implementation); dev `httpx`.
- `api/server.py`: `Session` holds all protocol logic, synchronous (`handle(text) -> Outgoing(reply, broadcast)`,
  `advance(wall_seconds) -> messages`). `create_app(speed, frame_rate, static_dir)`: `/ws`, plus a background
  loop at 30 frames/s that runs `advance` in a worker thread under an asyncio lock (the event loop stays free).
  Serves `web/dist` at `/` when it exists. Entry point `drone-sim-server` (`--port 8765`, `--speed`).
- Pacing: sim time tracks wall time x `speed`; if the sim cannot keep up, ticks per frame are capped at 2x nominal
  and the target is pulled back (runs slower than real time instead of spiralling).
- `set_world` is an undoable `ReplaceWorld` command defined in the api layer (the editor's `Command` protocol is
  structural, so no world change was needed).
- Per-drone frame status is derived from public engine data: task board status, and `DroneCollided` events for
  drones that crashed (even without a task).
- One shared simulation for all connected clients; every client sees every broadcast.
