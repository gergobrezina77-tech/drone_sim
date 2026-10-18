# DS-021: world save/load from the UI

- **Status:** Backlog
- **Layer:** web
- **Milestone:** M2 Server and web UI
- **Depends on:** DS-020

## Goal

Keep and reopen worlds.

## Scope

- Save/load commands writing `WorldSpec` JSON to a `worlds/` directory; file picker in the UI.

## Acceptance criteria

- [ ] A saved world reloads identically (server-side round-trip test).
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
