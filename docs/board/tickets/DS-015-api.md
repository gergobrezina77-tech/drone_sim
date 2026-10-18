# DS-015: api: protocol definition (commands + state frames)

- **Status:** Done
- **Layer:** api
- **Milestone:** M2 Server and web UI
- **Depends on:** DS-012, DS-028

## Goal

A versioned, documented contract between the simulator and any UI.

## Scope

- JSON commands: world edits (mirroring DS-005 commands), start/pause/resume/reset, load/save.
- Binary state frames: header + float32 struct-of-arrays (pos, quat, status) — compact for many drones.
- Hello/version handshake; `docs/PROTOCOL.md`.

## Acceptance criteria

- [x] Encode/decode round-trip tests for every message.
  (`tests/test_api_protocol.py::test_every_client_message_round_trips_through_json` [17 messages incl. all six
  edit ops], `::test_every_server_message_parses_back` [10], `::test_server_payloads_decode_to_domain_objects`,
  `::test_frame_round_trip` [0, 1, 500 drones])
- [x] A 500-drone state frame is under 25 KB. (`::test_a_500_drone_frame_is_under_25_kb`: 14 520 bytes)
- [x] `scripts/check.sh` passes (267 passed). Also malformed messages / frames raise `ProtocolError`.

## Notes

- `api/protocol.py` (pure encode/decode, no networking); spec in `docs/PROTOCOL.md`.
- Authoring goes through `construct` (any registered scene constructor), so the UI never hard-codes how scenes
  are built; generic edits (move, remove, tasks) go through `edit` with the six command ops.
- Load / save are `set_world` / `get_world` (whole world as JSON); writing files is DS-021.
- Binary frame: 20-byte header (magic, seq, time, n) + float32 pos + float32 quat + uint8 status per drone.
  Per-drone status (idle / active / succeeded / failed) is derived by the server from public engine data (DS-016).
- `parse_server` turns world / event / report payloads back into domain objects, for Python clients and tests.
