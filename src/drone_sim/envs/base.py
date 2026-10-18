"""Shared machinery for the Gymnasium drone environments."""

from __future__ import annotations

from typing import ClassVar

import gymnasium as gym
import mujoco
import numpy as np

from drone_sim.controller import GeometricController
from drone_sim.disturbances import WindConfig, WindField, aero_drag
from drone_sim.drone import DroneGroup, DroneParams, DroneState
from drone_sim.render import FollowCamera, add_marker
from drone_sim.scene import Obstacle, build_scene, drone_colors, spawn_grid

ACTION_MODES = ("motors", "ctbr", "velocity")
MAX_RATES = np.array([6.0, 6.0, 3.0])  # rad/s, ctbr mode
MAX_VEL = 2.0  # m/s, velocity mode
MAX_YAW_RATE = 1.5  # rad/s, velocity mode
STATE_DIM = 18  # goal error (3) + velocity (3) + rotation matrix (9) + body rates (3)


def drone_features(s: DroneState, goals: np.ndarray) -> np.ndarray:
    return np.concatenate([goals - s.pos, s.vel, s.R.reshape(s.n, 9), s.omega], axis=1)


class DroneEnvBase(gym.Env):
    """N drones in one MuJoCo world, driven by normalised actions in [-1, 1].

    Action modes (per drone, 4 values):
      * ``motors``   - individual motor thrusts, -1 = off, +1 = max thrust
      * ``ctbr``     - collective thrust + body rates, tracked by an inner rate loop
      * ``velocity`` - world-frame velocity + yaw rate, tracked by the geometric controller

    Subclasses implement ``_sample_episode``, ``_observation`` and ``_evaluate`` and set the spaces.
    """

    metadata: ClassVar[dict] = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(
        self,
        n_drones: int,
        action_mode: str = "motors",
        control_freq: float = 100.0,
        episode_seconds: float = 10.0,
        obstacles: list[Obstacle] = (),
        mass_randomization: float = 0.1,
        wind_max: float = 0.0,
        turbulence: float = 0.0,
        params: DroneParams | None = None,
        render_mode: str | None = None,
        width: int = 640,
        height: int = 480,
    ):
        if action_mode not in ACTION_MODES:
            raise ValueError(f"action_mode must be one of {ACTION_MODES}")
        if render_mode is not None and render_mode not in self.metadata["render_modes"]:
            raise ValueError(f"render_mode must be one of {self.metadata['render_modes']}")
        self.n, self.action_mode, self.render_mode = n_drones, action_mode, render_mode
        self.width, self.height = width, height
        self.params = params or DroneParams()
        self.obstacles = list(obstacles)
        self.mass_randomization, self.wind_max, self.turbulence = mass_randomization, wind_max, turbulence

        self.model = mujoco.MjModel.from_xml_string(
            build_scene(n_drones, self.params, spawn_grid(n_drones, z=1.0), self.obstacles)
        )
        self.data = mujoco.MjData(self.model)
        self.drones = DroneGroup(self.model, self.data, n_drones)
        self.controller = GeometricController(self.params, n_drones)  # executes ctbr / velocity actions
        self._expert = GeometricController(self.params, n_drones)  # privileged scripted baseline
        self.substeps = max(1, round(1.0 / (control_freq * self.model.opt.timestep)))
        self.control_dt = self.substeps * self.model.opt.timestep
        self.max_steps = round(episode_seconds / self.control_dt)
        self.metadata = {**self.metadata, "render_fps": round(1.0 / self.control_dt)}

        self._nominal_mass = self.model.body_mass[self.drones.body_ids].copy()
        self._nominal_inertia = self.model.body_inertia[self.drones.body_ids].copy()
        self.wind = WindField(WindConfig(), n_drones)
        self.goals = np.zeros((n_drones, 3))
        self._yaw_ref = np.zeros(n_drones)
        self._prev_action = np.zeros((n_drones, 4))
        self._steps = 0
        self._renderer = None
        self._viewer = None
        self._camera = FollowCamera(orbit_deg_per_s=0.0)
        self._colors = drone_colors(n_drones)

    # ---- hooks for subclasses ----

    def _sample_episode(self) -> None:
        raise NotImplementedError

    def _observation(self, s: DroneState) -> np.ndarray:
        raise NotImplementedError

    def _evaluate(self, s: DroneState, action: np.ndarray) -> tuple[float, bool, dict]:
        raise NotImplementedError

    def _format_action(self, a: np.ndarray) -> np.ndarray:
        return a.astype(np.float32)

    # ---- helpers ----

    def crashed(self, s: DroneState) -> np.ndarray:
        """Per drone: hit the ground, flipped past ~80 degrees, or drifted far from its goal."""
        far = np.linalg.norm(s.pos - self.goals, axis=1) > 6.0
        return (s.pos[:, 2] < 0.1) | (s.R[:, 2, 2] < 0.17) | far

    def _actions_to_thrusts(self, a: np.ndarray, s: DroneState) -> np.ndarray:
        f_max = self.params.max_thrust
        if self.action_mode == "motors":
            return (a + 1.0) * 0.5 * f_max
        if self.action_mode == "ctbr":
            thrust = (a[:, 0] + 1.0) * 0.5 * 4.0 * f_max
            return self.controller.rate_control(s, thrust, a[:, 1:] * MAX_RATES)
        self._yaw_ref = self._yaw_ref + a[:, 3] * MAX_YAW_RATE * self.control_dt
        return self.controller.velocity_control(s, a[:, :3] * MAX_VEL, self._yaw_ref, self.control_dt)

    def expert_action(self) -> np.ndarray:
        """Action from a privileged geometric controller flying each drone to its goal.

        Useful as a scripted baseline, for imitation learning, or to sanity check an env."""
        s = self.drones.state()
        exp = self._expert
        if self.action_mode == "velocity":
            v = 1.5 * (self.goals - s.pos)
            v *= np.minimum(1.0, MAX_VEL / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-9))
            a = np.hstack([v / MAX_VEL, np.clip(-self._yaw_ref / (MAX_YAW_RATE * self.control_dt), -1, 1)[:, None]])
        else:
            accel = exp.position_accel(s, self.goals, dt=self.control_dt)
            thrust, R_des = exp.accel_to_attitude(s, accel, 0.0)
            if self.action_mode == "motors":
                a = exp.attitude_control(s, thrust, R_des) / self.params.max_thrust * 2.0 - 1.0
            else:
                rates = exp.attitude_to_rates(s, R_des)
                a = np.hstack([(thrust / (4.0 * self.params.max_thrust) * 2.0 - 1.0)[:, None], rates / MAX_RATES])
        return self._format_action(np.clip(a, -1.0, 1.0))

    # ---- gym API ----

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        rng = self.np_random
        mujoco.mj_resetData(self.model, self.data)
        scale = 1.0 + rng.uniform(-self.mass_randomization, self.mass_randomization, self.n)
        self.model.body_mass[self.drones.body_ids] = self._nominal_mass * scale
        self.model.body_inertia[self.drones.body_ids] = self._nominal_inertia * scale[:, None]
        heading = rng.uniform(0, 2 * np.pi)
        mean = rng.uniform(0, self.wind_max) * np.array([np.cos(heading), np.sin(heading), 0.0])
        self.wind = WindField(WindConfig(mean=tuple(mean), turbulence=self.turbulence), self.n, rng)
        self.controller.reset()
        self._expert.reset()
        self._yaw_ref[:] = 0.0
        self._prev_action[:] = 0.0
        self._steps = 0
        self._sample_episode()
        mujoco.mj_forward(self.model, self.data)
        s = self.drones.state()
        return self._observation(s), {"goals": self.goals.copy()}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64).reshape(self.n, 4), -1.0, 1.0)
        self.drones.set_thrusts(self._actions_to_thrusts(a, self.drones.state()))
        air = self.wind.step(self.control_dt)
        for _ in range(self.substeps):
            self.drones.set_external_forces(aero_drag(self.drones.linear_velocity(), air, self.params.drag_coef))
            mujoco.mj_step(self.model, self.data)
        self._steps += 1
        s = self.drones.state()
        reward, terminated, info = self._evaluate(s, a)
        self._prev_action = a
        truncated = self._steps >= self.max_steps and not terminated
        if self.render_mode == "human":
            self.render()
        return self._observation(s), float(reward), terminated, truncated, info

    def _decorate(self, scene: mujoco.MjvScene) -> None:
        for goal, rgb in zip(self.goals, self._colors):
            add_marker(scene, goal, (*rgb, 0.45), size=0.05)

    def render(self):
        if self.render_mode is None:
            gym.logger.warn("render() called without a render_mode; pass render_mode='rgb_array' or 'human'.")
            return None
        points = np.vstack([self.drones.state().pos, self.goals])
        if self.render_mode == "rgb_array":
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, self.height, self.width)
            cam = self._camera.update(points, self.control_dt)
            self._renderer.update_scene(self.data, camera=cam)
            self._renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0  # close-up shadow acne; faster too
            self._decorate(self._renderer.scene)
            return self._renderer.render()
        from mujoco import viewer  # lazy: needs a display

        if self._viewer is None:
            self._viewer = viewer.launch_passive(self.model, self.data)
        with self._viewer.lock():
            self._viewer.user_scn.ngeom = 0
            self._decorate(self._viewer.user_scn)
        self._viewer.sync()
        return None

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
