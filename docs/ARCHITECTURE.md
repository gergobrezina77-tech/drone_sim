# Architecture

Interactive drone simulator: build a world in a plan view, place drones, give them tasks, run, watch them
succeed or fail, reset, edit, run again. The design goals, in priority order:

1. **Modularity / forward compatibility.** Every swappable part sits behind a small interface with a registry,
   so a more complex implementation plugs in without touching callers.
2. **Scale on limited compute.** Fleet state is struct-of-arrays (`(n, 3)` numpy arrays); hot paths are
   vectorised over drones. No per-drone Python loops in the tick.
3. **Fast iteration.** Stop → edit → start must feel instant (compiled physics model is cached by world geometry).

## Editor mode vs play mode

```
            ┌──────────── EDIT ◄──────────── reset ─────────────┐
            │  edit the WorldSpec (plan view)                    │
            ▼                                                    │
   start: compile WorldSpec → Runtime ──► RUNNING ⇄ PAUSED ──► FINISHED (report)
```

- **WorldSpec** (`world/`): plain, serialisable, versioned data. The only thing the editor mutates. Editing is
  allowed in EDIT only.
- **Runtime** (`engine/`): disposable, built from a WorldSpec on start. Reset discards it and returns to EDIT with
  the world exactly as authored. A collision fails the drone (no bounce); the run ends when every task is terminal.

## Layers

Enforced by `import-linter` (`[tool.importlinter]` in `pyproject.toml`): a layer may import only layers below it;
siblings on one row may not import each other.

| Layer | Package | Responsibility | Must not know about |
| --- | --- | --- | --- |
| API | `api/` | WebSocket protocol: commands in, state frames out | MuJoCo, numerics |
| Engine | `engine/` | State machine, fixed-step tick pipeline, compile/caching, events, telemetry | UI |
| Physics | `physics/` | `PhysicsBackend` interface + MuJoCo impl: rigid bodies, thrust, forces, collision flags | tasks, autonomy |
| Environment | `environment/` | Effects on drones (wind, later battery, sensor noise, motor faults) as batched forces/modifiers | physics engine |
| Autonomy | `autonomy/` | Drone software: observations + task goals → motor thrusts (controller, autopilots) | physics engine, ground truth beyond its observation |
| Tasks | `tasks/` | Task types + batched evaluators → success / failure with reason | autonomy internals |
| World | `world/` | WorldSpec, editing commands + undo, scene constructors (plan-2D first), JSON I/O | everything above |
| Vehicle | `vehicle/` | Drone parameters (airframe, motors, limits) — pure data | everything above |
| Core | `core/` | Math, geometry primitives, ids, RNG | everything |

The web UI (`web/`, TypeScript + Three.js) talks to `api/` only.

## Tick pipeline (engine)

```
observe(fleet state) → autonomy.step → effects.step (forces) → physics.step × substeps
  → collision flags → tasks.evaluate → events → telemetry/state frame
```

## Extension points (registries)

| Extension | Interface | First implementation |
| --- | --- | --- |
| Scene construction | `world.SceneConstructor` | `plan2d` (footprints extruded to a height) |
| Physics | `physics.PhysicsBackend` | `mujoco` |
| Effects | `environment.Effect` | `wind` |
| Drone software | `autonomy.FlightSoftware` | `goto` (naive straight-line position control) |
| Tasks | `tasks.TaskType` | `goto_point` |
| Vehicle models | `vehicle` registry | `quad_x_500g` (today's `DroneParams`) |

## Legacy code

The original swarm simulator (`sim.py`, `swarm.py`, `planner.py`, `scene.py`, `drone.py`, `render.py`, `envs/`,
`cli.py`) keeps working beside the layers; it is not part of the layer contract. Shared building blocks were moved
into the layers and the legacy modules re-export them:

| Legacy name | Now lives in |
| --- | --- |
| `drone.quat_to_mat`, `planner.rot_z` | `core.rotations` |
| `drone.DroneState` | `core.FleetState` |
| `drone.DroneParams`, `drone.GRAVITY` | `vehicle` |
| `scene.Obstacle.distance` / `closest_point` | `core.geometry` (via `Obstacle.shape`) |
| `disturbances.*` (wind) | `environment.wind` |
| `controller.*` | `autonomy.control` |

Still separate, on purpose (DS-014): the legacy MuJoCo scene (`scene.build_scene` + `drone.DroneGroup`) simulates
physical drone-obstacle and drone-drone contact with rotor-site motor actuators, while `physics.MujocoBackend`
detects contact geometrically (no bounce) and applies thrust as a body wrench. Porting `SwarmSim` would change its
physics; porting the Gym envs needs per-episode mass/inertia randomisation in the backend (DS-027).

Work is tracked in [board/BOARD.md](board/BOARD.md).
