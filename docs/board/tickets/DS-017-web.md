# DS-017: web: scaffold, connection and 3D view

- **Status:** Ready
- **Layer:** web
- **Milestone:** M2 Server and web UI
- **Depends on:** DS-016

## Goal

Browser front end showing the live world.

## Scope

- `web/`: Vite + TypeScript + Three.js; connect, handshake, decode frames.
- 3D view: floor/arena, obstacles, drones as one `InstancedMesh` (scales to many drones), orbit camera.
- Add `tsc --noEmit` and unit tests (vitest) to `scripts/check.sh`.

## Acceptance criteria

- [ ] Frame decoder unit tests pass in vitest.
- [ ] Manual check: drones visible and moving with the server running.
- [ ] `scripts/check.sh` passes

## Notes

_Implementation notes, decisions and measurements go here._
