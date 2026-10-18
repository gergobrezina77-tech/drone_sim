# DS-016: api: WebSocket server running the engine

- **Status:** Backlog
- **Layer:** api
- **Milestone:** M2 Server and web UI
- **Depends on:** DS-015

## Goal

Serve the simulation to the browser.

## Scope

- `websockets`-based server; the simulation runs in its own thread at real-time pace with a fixed step; state streamed at ~30 Hz, decoupled from the physics rate.
- Command queue applied between ticks; `drone-sim-server` entry point.

## Acceptance criteria

- [ ] Integration test with a Python WebSocket client: place drone, assign task, start, receive frames, receive TaskSucceeded.
- [ ] The server keeps up with 100 drones in real time.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
