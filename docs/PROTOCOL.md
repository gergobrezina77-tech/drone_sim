# Simulator protocol (version 1)

One WebSocket per client. Implementation: `src/drone_sim/api/protocol.py`.

- **Client → server:** JSON text messages `{"type": ..., ...}`.
- **Server → client:** JSON text messages, plus **binary** state frames while a run exists.

## Handshake

The client sends `{"type": "hello", "version": 1}`. The server answers with
`{"type": "welcome", "version": 1, "world": <World>, "state": "edit"}`. Any other version gets an `error`.

## Client messages

| type | payload | allowed in | effect |
| --- | --- | --- | --- |
| `hello` | `version` | any | handshake |
| `construct` | `constructor`, `action` | edit | run an authoring action through a registered scene constructor (`plan2d`: see `world/constructors.py`) |
| `edit` | `command` (below) | edit | apply one world command |
| `undo`, `redo` | — | edit | undo / redo the last edit |
| `start`, `pause`, `resume`, `reset` | — | see the state machine | simulation control |
| `set_world` | `world` | edit | replace the whole world (load); undoable |
| `get_world` | — | any | server replies with `world` (save) |

Edit commands (`command.op`):

| op | fields |
| --- | --- |
| `place_drone` | `drone`: Drone |
| `add_obstacle` | `obstacle`: Obstacle |
| `move` | `id`, `position` [x, y, z], optional `yaw` |
| `remove` | `id` (a drone also loses its tasks) |
| `assign_task` | `task`: Task |
| `remove_task` | `id` |

State machine: `edit` --start--> `running` <--pause / resume--> `paused`; `running` → `finished` when every task
has ended; `reset` from any non-edit state returns to `edit` with the world as authored.

## Server messages

| type | payload | when |
| --- | --- | --- |
| `welcome` | `version`, `world`, `state` | after `hello` |
| `world` | `world` | after every successful edit, and on `get_world` |
| `state` | `state`, `time` | after every state change |
| `event` | `event` (below) | as they happen during a run |
| `report` | `report`: `{time, outcomes: [{task_id, drone_id, status, reason, time}]}` | when a run finishes |
| `error` | `message` | a rejected message or edit; nothing changed |

Events: `{"kind": "task_succeeded", time, task_id, drone_id}`, `{"kind": "task_failed", time, task_id, drone_id,
reason}` with reason `collision | crash | out_of_bounds | timeout`, `{"kind": "drone_collided", time, drone_id}`,
`{"kind": "run_finished", time}`. Task statuses in reports: `pending | active | succeeded | failed`.

## Data shapes

Same JSON as saved worlds (`WorldSpec.to_dict`):

- **World:** `{schema_version: 1, arena_size: [x, y, height], obstacles: [Obstacle], drones: [Drone], tasks: [Task],
  environment: {effect_name: settings}}`. The arena is centred on the origin; the floor is z = 0.
- **Drone:** `{id, position: [x, y, z], yaw, vehicle}` (`yaw` and `vehicle` optional on input).
- **Obstacle:** `{id, shape}` with shape `{type: "box", center, half_extents}` or
  `{type: "cylinder", base, radius, height}` (base = centre of the bottom face).
- **Task:** `{id, drone_id, type, params}`; `goto_point` params: `target` [x, y, z], optional `tolerance` (0.3 m),
  `dwell` (1.0 s), `timeout` (60 s).

## Binary state frames

Little-endian. Drone `i` is `world.drones[i]`.

| offset | size | content |
| --- | --- | --- |
| 0 | 4 | magic `DSF1` |
| 4 | 4 | uint32 frame sequence number |
| 8 | 8 | float64 simulation time, s |
| 16 | 4 | uint32 n = number of drones |
| 20 | 12 n | float32 positions, `[x, y, z]` per drone |
| 20 + 12 n | 16 n | float32 orientations, quaternion `[w, x, y, z]` per drone |
| 20 + 28 n | n | uint8 status per drone: 0 idle (no task), 1 active, 2 succeeded, 3 failed / crashed |

500 drones = 14 520 bytes per frame.
