from gymnasium.envs.registration import register

from drone_sim.envs.hover_env import HoverEnv
from drone_sim.envs.swarm_env import SwarmEnv

register(id="DroneSim/Hover-v0", entry_point="drone_sim.envs.hover_env:HoverEnv")
register(id="DroneSim/Swarm-v0", entry_point="drone_sim.envs.swarm_env:SwarmEnv")

__all__ = ["HoverEnv", "SwarmEnv"]
