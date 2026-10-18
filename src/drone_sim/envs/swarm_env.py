"""Multi-drone: every drone flies to its slot in a random formation without collisions.

Designed for a shared (parameter-sharing) policy: observations and actions have a leading
agent axis, and per-agent rewards are reported in ``info["agent_rewards"]`` next to the
scalar team reward that the Gymnasium API requires.
"""

from __future__ import annotations

import numpy as np
from gymnasium import spaces

from drone_sim.drone import DroneState
from drone_sim.envs.base import STATE_DIM, DroneEnvBase, drone_features
from drone_sim.envs.hover_env import CRASH_PENALTY, hover_reward
from drone_sim.planner import FORMATIONS, formation_offsets, rot_z
from drone_sim.swarm import pairwise

COLLISION_PENALTY = 2.0


class SwarmEnv(DroneEnvBase):
    """Observation (n, 18 + 6k): each drone's own 18 features (see HoverEnv) followed by
    the relative position and velocity of its k nearest neighbours (zero padded).

    Action (n, 4) in [-1, 1]; defaults to ``velocity`` mode.
    """

    def __init__(self, n_drones: int = 4, k_neighbors: int = 3, action_mode: str = "velocity", **kwargs):
        super().__init__(n_drones=n_drones, action_mode=action_mode, **kwargs)
        self.k = k_neighbors
        self.observation_space = spaces.Box(-np.inf, np.inf, (n_drones, STATE_DIM + 6 * k_neighbors), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (n_drones, 4), np.float32)
        self.formation = None

    def _sample_episode(self) -> None:
        rng = self.np_random
        center = rng.uniform([-1.0, -1.0, 1.5], [1.0, 1.0, 2.0])
        start = center + formation_offsets("grid", self.n, 1.0) + rng.normal(0, 0.1, (self.n, 3))
        self.formation = FORMATIONS[rng.integers(len(FORMATIONS))]
        goal_center = center + np.concatenate([rng.uniform(-2.0, 2.0, 2), [rng.uniform(-0.3, 0.7)]])
        slots = formation_offsets(self.formation, self.n, 1.0) @ rot_z(rng.uniform(0, 2 * np.pi)).T
        self.goals = goal_center + slots[rng.permutation(self.n)]
        self.goals[:, 2] = np.maximum(self.goals[:, 2], 0.6)
        self.drones.set_pose(start)

    def _observation(self, s: DroneState) -> np.ndarray:
        own = drone_features(s, self.goals)
        nbr = np.zeros((self.n, 6 * self.k))
        if self.n > 1 and self.k > 0:
            _, dist = pairwise(s.pos)
            k = min(self.k, self.n - 1)
            idx = np.argsort(dist, axis=1)[:, :k]
            rel = np.concatenate([s.pos[idx] - s.pos[:, None], s.vel[idx] - s.vel[:, None]], axis=2)
            nbr[:, : 6 * k] = rel.reshape(self.n, 6 * k)
        return np.hstack([own, nbr]).astype(np.float32)

    def _evaluate(self, s: DroneState, action: np.ndarray) -> tuple[float, bool, dict]:
        rewards = hover_reward(s, self.goals, action, self._prev_action)
        too_close = pairwise(s.pos)[1] < 2 * self.params.radius
        rewards -= COLLISION_PENALTY * too_close.sum(axis=1)
        crashed = self.crashed(s)
        rewards[crashed] -= CRASH_PENALTY
        info = {
            "agent_rewards": rewards,
            "distances": np.linalg.norm(self.goals - s.pos, axis=1),
            "near_collisions": int(too_close.sum() // 2),
            "crashed": crashed,
            "formation": self.formation,
        }
        return float(rewards.mean()), bool(crashed.any()), info
