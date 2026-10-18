# DS-015: api: protocol definition (commands + state frames)

- **Status:** Backlog
- **Layer:** api
- **Milestone:** M2 Server and web UI
- **Depends on:** DS-012

## Goal

A versioned, documented contract between the simulator and any UI.

## Scope

- JSON commands: world edits (mirroring DS-005 commands), start/pause/resume/reset, load/save.
- Binary state frames: header + float32 struct-of-arrays (pos, quat, status) — compact for many drones.
- Hello/version handshake; `docs/PROTOCOL.md`.

## Acceptance criteria

- [ ] Encode/decode round-trip tests for every message.
- [ ] A 500-drone state frame is under 25 KB.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
