# DS-018: web: run controls, status and outcome markers

- **Status:** Backlog
- **Layer:** web
- **Milestone:** M2 Server and web UI
- **Depends on:** DS-017

## Goal

Start, pause, resume and reset from the UI and see what happened.

## Scope

- Toolbar: start/pause/resume/reset with state-aware enabling; status bar (state, sim time, speed).
- Drone colouring by task status; failure markers with reason; end-of-run summary.

## Acceptance criteria

- [ ] Manual check of the whole cycle; UI state logic unit-tested.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
