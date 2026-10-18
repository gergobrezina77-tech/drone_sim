"""Physics: the PhysicsBackend interface and its implementations (MuJoCo first); batched fleet state,
thrust and force input, collision flags.

May import: world, vehicle, core. Must not import environment, autonomy or tasks.
"""

from drone_sim.physics.backend import PHYSICS_BACKENDS, PhysicsBackend
from drone_sim.physics.mujoco_backend import MujocoBackend

PHYSICS_BACKENDS.register("mujoco", MujocoBackend)

__all__ = ["PHYSICS_BACKENDS", "MujocoBackend", "PhysicsBackend"]
