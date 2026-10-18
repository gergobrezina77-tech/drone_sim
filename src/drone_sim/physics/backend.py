"""The PhysicsBackend interface: everything the engine needs from a physics engine, nothing more.

All per-drone data is batched: drone ``i`` is ``world.drones[i]``.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol, runtime_checkable

import numpy as np

from drone_sim.core.registry import Registry
from drone_sim.core.state import FleetState
from drone_sim.world import WorldSpec


@runtime_checkable
class PhysicsBackend(Protocol):
    """Rigid-body simulation of a fleet in a static world.

    Contract (checked for every registered backend by ``tests/test_physics_contract.py``):

    - ``build(world)``: compile the world (implementations may cache by ``world.geometry_hash()``) and reset.
    - ``reset()``: drones at rest at their start poses, time 0, thrusts and forces zero, nothing frozen.
    - ``set_thrusts((n, 4))``: per-motor thrust in N, clipped to the vehicle's limits; held until changed.
    - ``set_external_forces((n, 3))``: world-frame force at the centre of mass in N; held until changed.
    - ``step(substeps)``: advance ``substeps * timestep`` seconds.
    - ``collisions() -> (n,) bool``: drones currently touching an obstacle or another drone (within the vehicle's
      footprint radius). Floor contact is not a collision. Frozen drones are never flagged and are ignored.
    - ``freeze(mask)``: those drones hold their pose with zero velocity, ignoring thrust and forces, until reset.
    - Identical inputs give identical results (deterministic).
    """

    timestep: float
    time: float

    @property
    def n(self) -> int: ...

    def build(self, world: WorldSpec) -> None: ...

    def reset(self) -> None: ...

    def state(self) -> FleetState: ...

    def set_thrusts(self, thrusts: np.ndarray) -> None: ...

    def set_external_forces(self, forces: np.ndarray) -> None: ...

    def step(self, substeps: int = 1) -> None: ...

    def collisions(self) -> np.ndarray: ...

    def freeze(self, mask: np.ndarray) -> None: ...


PHYSICS_BACKENDS = Registry[Callable[[], PhysicsBackend]]("physics backend")
