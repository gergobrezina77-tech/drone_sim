"""Single drone: fly to a random goal and hold position there."""

from __future__ import annotations

import numpy as np
from gymnasium import spaces

from drone_sim.drone import DroneState
from drone_sim.envs.base import STATE_DIM, DroneEnvBase, drone_features

CRASH_PENALTY = 10.0


def random_tilt_quat(rng: np.random.Generator, max_deg: float) -> np.ndarray:
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = np.deg2rad(rng.uniform(0, max_deg))
    return np.concatenate([[np.cos(angle / 2)], np.sin(angle / 2) * axis])


def hover_reward(s: DroneState, goals: np.ndarray, action: np.ndarray, prev_action: np.ndarray) -> np.ndarray:
    """Per-drone shaped reward: alive bonus minus distance, speed, spin and action jitter."""
    dist = np.linalg.norm(goals - s.pos, axis=1)
    return (
        1.0
        - dist
        - 0.05 * np.linalg.norm(s.vel, axis=1)
        - 0.02 * np.linalg.norm(s.omega, axis=1)
        - 0.05 * np.sum((action - prev_action) ** 2, axis=1)
    )


class HoverEnv(DroneEnvBase):
    """Observation (18,): goal - position, velocity, body-to-world rotation (row-major), body rates.

    Action (4,) in [-1, 1], interpreted according to ``action_mode`` (see DroneEnvBase).
    Episodes start mid-air with a random tilt and velocity, and end on a crash or after
    ``episode_seconds``. Mass and (optionally) wind are randomised every reset.
    """

    def __init__(self, action_mode: str = "motors", **kwargs):
        super().__init__(n_drones=1, action_mode=action_mode, **kwargs)
        self.observation_space = spaces.Box(-np.inf, np.inf, (STATE_DIM,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (4,), np.float32)

    def _sample_episode(self) -> None:
        rng = self.np_random
        pos = rng.uniform([-1.0, -1.0, 0.8], [1.0, 1.0, 2.0])
        self.goals = rng.uniform([-1.5, -1.5, 0.8], [1.5, 1.5, 2.5])[None]
        self.drones.set_pose(pos[None], random_tilt_quat(rng, 15.0)[None], rng.normal(0, 0.2, (1, 3)))

    def _observation(self, s: DroneState) -> np.ndarray:
        return drone_features(s, self.goals)[0].astype(np.float32)

    def _format_action(self, a: np.ndarray) -> np.ndarray:
        return a[0].astype(np.float32)

    def _evaluate(self, s: DroneState, action: np.ndarray) -> tuple[float, bool, dict]:
        reward = hover_reward(s, self.goals, action, self._prev_action)[0]
        crashed = bool(self.crashed(s)[0])
        if crashed:
            reward -= CRASH_PENALTY
        return reward, crashed, {"distance": float(np.linalg.norm(self.goals[0] - s.pos[0])), "crashed": crashed}
