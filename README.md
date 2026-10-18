# drone-sim

A quadrotor **swarm** simulator built on [MuJoCo](https://mujoco.org). It combines realistic rigid-body physics, a geometric flight controller, formation flying and boids, obstacle and inter-drone avoidance, and wind. It also has **Gymnasium** environments for training autonomy policies.

## Quick start

```bash
poetry install                               # creates ./.venv with everything (see docs/POETRY.md)
poetry run drone-swarm --n 12                # live MuJoCo viewer, demo mission
poetry run drone-record --n 16 --out media/swarm.mp4 --wind 1.5 --turbulence 0.5 --gust-every 20
poetry run drone-random --env swarm --n 6 --policy expert
poetry run pytest
```

Viewer keys: `6` next formation · `7` toggle formation/boids · `8` wind gust · `9` clear trails.
Common options: `--mode boids`, `--wind 2 --turbulence 0.8`, `--no-obstacles`, `--n 30`.

## Layered engine

A layered simulation engine (`core`, `vehicle`, `world`, `physics`, `environment`, `autonomy`, `tasks`, `engine`,
`api`) is being built for an interactive editor: author a world, place drones, give them tasks, run, watch them
succeed or fail, reset, edit, run again. See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) and the work board in
[docs/board/BOARD.md](docs/board/BOARD.md).

```python
from drone_sim.engine import Simulation
from drone_sim.world import AssignTask, DroneSpec, PlaceDrone, TaskSpec

sim = Simulation()
sim.edit(PlaceDrone(DroneSpec("d1", position=(0.0, 0.0, 0.07))))
sim.edit(AssignTask(TaskSpec("t1", "d1", "goto_point", {"target": [4.0, 0.0, 2.0]})))
sim.start()
sim.run_until_finished(30.0)
print(sim.report())
```

`poetry run python scripts/bench.py` prints compile / restart / per-tick cost for 10, 100 and 500 drones.

## What's inside (swarm simulator)

The modules below are the original swarm simulator, which keeps working beside the layered engine; see the legacy
section of [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for what moved into the layers.

| module | what it does |
| --- | --- |
| `drone.py` | `DroneParams`: mass, inertia, arm, thrust limits, rotor drag, and the allocation matrix. `DroneGroup` gives vectorised state and actuator access for N drones. |
| `scene.py` | Procedural MJCF: N quadrotors (free joint, 4 `motor` actuators whose gear includes the rotor reaction torque), arena, pillars and boxes (`Obstacle`). |
| `controller.py` | Vectorised geometric SE(3) controller: position or velocity → desired acceleration (with tilt limit and anti-windup) → attitude + collective thrust → torques → motor mixer with yaw de-prioritisation. It also has a body-rate (CTBR) loop. |
| `planner.py` | Formation library (`line`, `v`, `circle`, `grid`, `sphere`, `helix`), greedy slot assignment, and `Mission`: the virtual centre moves through waypoints, with smooth morphs that spread drones over altitude lanes to avoid crossings. |
| `swarm.py` | Velocity-level behaviours: formation tracking, Reynolds boids, separation with closing-speed damping, and obstacle repulsion plus slide-around. Priority order: task < obstacles < separation. |
| `disturbances.py` | Wind: mean flow, Ornstein–Uhlenbeck turbulence (shared and per drone), and gusts, applied as drag on the air-relative velocity. |
| `sim.py` | `SwarmSim`: mission → behaviours → controller → physics. Physics runs at 500 Hz and control at 250 Hz. |
| `envs/` | `DroneSim/Hover-v0` and `DroneSim/Swarm-v0` Gymnasium environments. |
| `render.py` | Trails, markers and the follow camera, shared by the viewer, the recorder and the envs. |

## Gymnasium environments

```python
import gymnasium as gym
import drone_sim.envs  # registers the ids

env = gym.make("DroneSim/Hover-v0", action_mode="motors", wind_max=2.0, render_mode="rgb_array")
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
expert = env.unwrapped.expert_action()   # privileged scripted controller, as a baseline or for imitation
```

- **Action modes** (4 values per drone, in [-1, 1]):
  - `motors`: individual thrusts
  - `ctbr`: collective thrust + body rates
  - `velocity`: world velocity + yaw rate
- **Hover-v0**:
  - Observation `(18,)`: goal − position, velocity, rotation matrix, body rates.
  - Reward: alive bonus minus distance, speed, spin and action jitter. A crash costs −10.
  - Each reset randomises the start pose, the goal, the mass (±10%) and optionally the wind.
- **Swarm-v0**:
  - Observation `(n, 18 + 6k)`: each drone's own features plus its k nearest neighbours' relative position and velocity.
  - Action `(n, 4)`. Each drone flies to its slot in a random formation.
  - The step returns a scalar team reward, with per-agent rewards in `info["agent_rewards"]` for parameter-sharing or multi-agent RL.
- **Expert baseline:** `expert_action()` flies straight to the goals. It is a baseline and is not collision-aware.

RL training libraries are deliberately not dependencies yet. `docs/POETRY.md` shows how to add them as an optional group.

## Known limitations

- Motor dynamics are instantaneous, and aerodynamics are linear drag only (no ground effect or blade flapping).
- Formation mode is collision-free in the test missions, with 30 drones and wind. Boids is emergent: in large flocks a pair can occasionally brush rotors.
- On WSL, rendering uses WSLg's GLFW, because EGL and OSMesa are unavailable there. Offscreen recording at 720p is slower than real time.
