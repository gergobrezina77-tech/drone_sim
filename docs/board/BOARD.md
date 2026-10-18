# Board

Single source of truth for work status. Workflow rules are in the project `CLAUDE.md`.

Columns: **Ready** (all dependencies done) → **In progress** (at most one) → **Done**. **Backlog** tickets move to Ready when their dependencies are Done.

## In progress

_none_

## Ready

| Ticket | Title | Layer | Depends on |
| --- | --- | --- | --- |
| [DS-025](tickets/DS-025-tasks.md) | tasks: several tasks per drone, run in sequence | tasks | DS-011 |
| [DS-012](tickets/DS-012-engine.md) | engine: simulation state machine and tick pipeline | engine | DS-008, DS-009, DS-010, DS-011 |
| [DS-023](tickets/DS-023-vehicle.md) | vehicle: per-airframe rest height | vehicle | DS-006 |

## Backlog

### M1 Layered headless core

| Ticket | Title | Layer | Depends on |
| --- | --- | --- | --- |
| [DS-013](tickets/DS-013-engine.md) | engine: performance benchmark and budgets | engine | DS-012 |
| [DS-014](tickets/DS-014-legacy.md) | legacy: rebase SwarmSim, envs and CLI on the layers (soft) | legacy | DS-012 |

### M2 Server and web UI

| Ticket | Title | Layer | Depends on |
| --- | --- | --- | --- |
| [DS-015](tickets/DS-015-api.md) | api: protocol definition (commands + state frames) | api | DS-012 |
| [DS-016](tickets/DS-016-api.md) | api: WebSocket server running the engine | api | DS-015 |
| [DS-017](tickets/DS-017-web.md) | web: scaffold, connection and 3D view | web | DS-016 |
| [DS-018](tickets/DS-018-web.md) | web: run controls, status and outcome markers | web | DS-017 |
| [DS-019](tickets/DS-019-web.md) | web: plan view editor | web | DS-018 |
| [DS-020](tickets/DS-020-web.md) | web: goto task tool and task panel | web | DS-019 |
| [DS-021](tickets/DS-021-world.md) | world save/load from the UI | web | DS-020 |

## Done

| Ticket | Title | Layer | Depends on |
| --- | --- | --- | --- |
| [DS-011](tickets/DS-011-tasks.md) | tasks: TaskType interface + goto_point evaluator | tasks | DS-004 |
| [DS-010](tickets/DS-010-autonomy.md) | autonomy: FlightSoftware interface, controller, naive goto autopilot | autonomy | DS-007 |
| [DS-009](tickets/DS-009-environment.md) | environment: Effect interface + wind effect | environment | DS-007 |
| [DS-008](tickets/DS-008-physics.md) | physics: MuJoCo backend with model cache and collision flags | physics | DS-007, DS-006 |
| [DS-007](tickets/DS-007-physics.md) | physics: PhysicsBackend interface and fleet state | physics | DS-004, DS-024 |
| [DS-024](tickets/DS-024-core.md) | core: FleetState shared runtime state | core | DS-002 |
| [DS-006](tickets/DS-006-world.md) | world: SceneConstructor interface + plan2d constructor | world | DS-005, DS-022 |
| [DS-022](tickets/DS-022-core.md) | core: generic name registry | core | DS-003 |
| [DS-005](tickets/DS-005-world.md) | world: editing commands with undo/redo and validation | world | DS-004 |
| [DS-004](tickets/DS-004-world.md) | world: WorldSpec data model and JSON I/O | world | DS-003 |
| [DS-003](tickets/DS-003-vehicle.md) | vehicle: drone parameter model and registry | vehicle | DS-002 |
| [DS-002](tickets/DS-002-core.md) | core: math and batched geometry primitives | core | DS-001 |
| [DS-001](tickets/DS-001-layer.md) | Layer package skeleton + strict layer contract | all | – |
