"""Physics: the PhysicsBackend interface and its implementations (MuJoCo first); batched fleet state,
thrust and force input, collision flags.

May import: world, vehicle, core. Must not import environment, autonomy or tasks.
"""

from drone_sim.physics.backend import PHYSICS_BACKENDS, PhysicsBackend

__all__ = ["PHYSICS_BACKENDS", "PhysicsBackend"]
