import gymnasium as gym
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

import drone_sim.envs  # noqa: F401  (registers env ids)
from drone_sim.envs import HoverEnv, SwarmEnv

pytestmark = pytest.mark.filterwarnings("ignore:.*Box observation space.*infinity")


@pytest.mark.parametrize("mode", ["motors", "ctbr", "velocity"])
def test_hover_env_api(mode):
    check_env(HoverEnv(action_mode=mode), skip_render_check=True)


@pytest.mark.parametrize("mode", ["motors", "velocity"])
def test_swarm_env_api(mode):
    check_env(SwarmEnv(n_drones=5, action_mode=mode), skip_render_check=True)


@pytest.mark.parametrize("mode", ["motors", "ctbr", "velocity"])
def test_expert_reaches_goal_in_hover_env(mode):
    env = gym.make("DroneSim/Hover-v0", action_mode=mode, episode_seconds=6.0)
    env.reset(seed=1)
    done = False
    while not done:
        _, _, terminated, truncated, info = env.step(env.unwrapped.expert_action())
        done = terminated or truncated
    assert not terminated and info["distance"] < 0.1


def test_random_motor_actions_crash():
    env = HoverEnv(action_mode="motors")
    env.reset(seed=0)
    env.action_space.seed(0)
    for _ in range(env.max_steps):
        _, reward, terminated, _, info = env.step(env.action_space.sample())
        if terminated:
            break
    assert terminated and info["crashed"] and reward < -5


def test_swarm_observation_layout():
    env = SwarmEnv(n_drones=3, k_neighbors=4)
    obs, info = env.reset(seed=0)
    assert obs.shape == (3, 18 + 6 * 4)
    s = env.drones.state()
    np.testing.assert_allclose(obs[:, :3], info["goals"] - s.pos, atol=1e-5)
    assert np.all(obs[:, 18 + 6 * 2 :] == 0)  # only 2 neighbours exist; the rest is padding
    _, _, _, _, info = env.step(env.action_space.sample())
    assert info["agent_rewards"].shape == (3,)


def test_rgb_render():
    env = HoverEnv(render_mode="rgb_array", width=160, height=120)
    env.reset(seed=0)
    frame = env.render()
    assert frame.shape == (120, 160, 3) and frame.std() > 0
    env.close()
