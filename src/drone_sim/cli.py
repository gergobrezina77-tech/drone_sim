"""Command-line entry points (declared under [project.scripts] in pyproject.toml)."""

from __future__ import annotations

import argparse
import time

import numpy as np

from drone_sim.disturbances import WindConfig
from drone_sim.planner import FORMATIONS
from drone_sim.render import FollowCamera, Trails
from drone_sim.scene import drone_colors
from drone_sim.sim import SwarmSim


def _swarm_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--n", type=int, default=12, help="number of drones")
    p.add_argument("--mode", choices=["formation", "boids"], default="formation")
    p.add_argument("--wind", type=float, default=0.0, help="mean wind speed (m/s, along +x)")
    p.add_argument("--turbulence", type=float, default=0.0, help="turbulence std (m/s)")
    p.add_argument("--no-obstacles", action="store_true")
    p.add_argument("--seed", type=int, default=0)


def _make_sim(a: argparse.Namespace) -> SwarmSim:
    wind = WindConfig(mean=(a.wind, 0.0, 0.0), turbulence=a.turbulence, local_turbulence=0.3 * a.turbulence)
    return SwarmSim(a.n, obstacles=[] if a.no_obstacles else None, wind=wind, mode=a.mode, seed=a.seed)


def swarm() -> None:
    """Interactive viewer with the demo mission."""
    p = argparse.ArgumentParser(description="Fly a drone swarm in the MuJoCo viewer.")
    _swarm_args(p)
    a = p.parse_args()
    from mujoco import viewer as mj_viewer  # lazy: needs a display

    sim = _make_sim(a)
    trails = Trails(drone_colors(a.n), length=50)
    keys = {"6": "next formation", "7": "toggle formation/boids", "8": "wind gust", "9": "clear trails"}
    pending: list[str] = []

    def on_key(keycode: int) -> None:
        ch = chr(keycode) if 32 <= keycode < 127 else ""
        if ch in keys:
            pending.append(ch)

    print("Keys: " + ", ".join(f"[{k}] {v}" for k, v in keys.items()))
    frame_dt = 1.0 / 60.0
    steps_per_frame = max(1, round(frame_dt / sim.control_dt))
    with mj_viewer.launch_passive(sim.model, sim.data, key_callback=on_key) as viewer:
        viewer.cam.lookat[:] = (2.0, 3.0, 1.5)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 16.0, 225.0, -30.0
        while viewer.is_running():
            start = time.perf_counter()
            while pending:
                ch = pending.pop(0)
                if ch == "6":
                    nxt = (
                        FORMATIONS[(FORMATIONS.index(sim.mission.formation) + 1) % len(FORMATIONS)]
                        if sim.mission.formation in FORMATIONS
                        else FORMATIONS[0]
                    )
                    sim.mission.set_formation(nxt)
                    print(f"formation -> {nxt}")
                elif ch == "7":
                    sim.brain.mode = "boids" if sim.brain.mode == "formation" else "formation"
                    print(f"mode -> {sim.brain.mode}")
                elif ch == "8":
                    sim.wind.gust()
                    print("gust!")
                elif ch == "9":
                    trails.clear()
            with viewer.lock():
                for _ in range(steps_per_frame):
                    sim.step()
                trails.push(sim.t, sim.drones.state().pos)
                viewer.user_scn.ngeom = 0
                trails.draw(viewer.user_scn)
            viewer.sync()
            time.sleep(max(0.0, frame_dt - (time.perf_counter() - start)))


def record() -> None:
    """Headless offscreen recording to mp4."""
    p = argparse.ArgumentParser(description="Record the demo mission to a video file.")
    _swarm_args(p)
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--out", default="swarm.mp4")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--gust-every", type=float, default=0.0, help="trigger a wind gust every N seconds")
    p.add_argument("--no-shadows", action="store_true", help="faster rendering (software GL on WSL)")
    a = p.parse_args()
    import imageio.v2 as imageio
    import mujoco

    sim = _make_sim(a)
    trails = Trails(drone_colors(a.n), length=60)
    camera = FollowCamera()
    renderer = mujoco.Renderer(sim.model, a.height, a.width)
    steps_per_frame = max(1, round(1.0 / (a.fps * sim.control_dt)))
    n_frames = int(a.seconds * a.fps)
    next_gust = a.gust_every or np.inf
    t0 = time.perf_counter()
    with imageio.get_writer(a.out, fps=a.fps, codec="libx264", quality=8, macro_block_size=8) as writer:
        for f in range(n_frames):
            for _ in range(steps_per_frame):
                sim.step()
                trails.push(sim.t, sim.drones.state().pos)
            if sim.t >= next_gust:
                sim.wind.gust()
                next_gust += a.gust_every
            cam = camera.update(sim.drones.state().pos, 1.0 / a.fps)
            renderer.update_scene(sim.data, camera=cam)
            if a.no_shadows:
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            trails.draw(renderer.scene)
            writer.append_data(renderer.render())
            if f % (5 * a.fps) == 0:
                print(f"  t={sim.t:5.1f}s  formation={sim.mission.formation:7s}  min sep={sim.min_separation():.2f} m")
    renderer.close()
    print(f"wrote {a.out} ({a.seconds:.0f}s of sim in {time.perf_counter() - t0:.0f}s)")


def random_agent() -> None:
    """Exercise the Gymnasium envs with random or scripted-expert actions."""
    p = argparse.ArgumentParser(description="Roll out DroneSim Gymnasium envs.")
    p.add_argument("--env", choices=["hover", "swarm"], default="hover")
    p.add_argument("--n", type=int, default=4, help="drones (swarm env)")
    p.add_argument("--action-mode", choices=["motors", "ctbr", "velocity"], default=None)
    p.add_argument("--policy", choices=["random", "expert"], default="expert")
    p.add_argument("--episodes", type=int, default=3)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--wind", type=float, default=0.0, help="max random mean wind (m/s)")
    p.add_argument("--render", choices=["human", "rgb_array"], default=None)
    p.add_argument("--video", default=None, help="save rgb_array frames to this mp4")
    a = p.parse_args()
    import gymnasium as gym

    import drone_sim.envs  # noqa: F401  (registers the env ids)

    kwargs = {"wind_max": a.wind, "render_mode": "rgb_array" if a.video else a.render}
    if a.action_mode:
        kwargs["action_mode"] = a.action_mode
    if a.env == "swarm":
        kwargs["n_drones"] = a.n
    env = gym.make("DroneSim/Hover-v0" if a.env == "hover" else "DroneSim/Swarm-v0", **kwargs)
    print(
        f"obs space {env.observation_space.shape}  action space {env.action_space.shape}  "
        f"mode={env.unwrapped.action_mode}  control dt={env.unwrapped.control_dt * 1000:.0f} ms"
    )
    frames = []
    for ep in range(a.episodes):
        env.reset(seed=a.seed + ep)
        ret, steps, done = 0.0, 0, False
        while not done:
            action = env.unwrapped.expert_action() if a.policy == "expert" else env.action_space.sample()
            _, reward, terminated, truncated, info = env.step(action)
            ret, steps, done = ret + reward, steps + 1, terminated or truncated
            if kwargs["render_mode"] == "rgb_array" and steps % 2 == 0:
                frame = env.render()
                if a.video:
                    frames.append(frame)
        dist = info["distance"] if "distance" in info else float(np.max(info["distances"]))
        extra = f"  formation={info['formation']}" if "formation" in info else ""
        print(f"episode {ep}: steps={steps} return={ret:8.1f} terminated={terminated} final dist={dist:.3f} m{extra}")
    env.close()
    if a.video:
        import imageio.v2 as imageio

        imageio.mimwrite(a.video, frames, fps=env.metadata["render_fps"] // 2, macro_block_size=8)
        print(f"wrote {a.video}")
