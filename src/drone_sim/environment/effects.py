"""Effects on drones as batched, composable plug-ins.

An effect is built from its settings in ``WorldSpec.environment`` (keyed by its registered name) and produces a
world-frame force per drone each step. Each effect gets its own random stream, so adding one never changes another.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Any, Protocol

import numpy as np

from drone_sim.core.registry import Registry
from drone_sim.core.rng import rng_stream
from drone_sim.core.state import FleetState
from drone_sim.world import WorldSpec


class Effect(Protocol):
    def reset(self, rng: np.random.Generator) -> None: ...

    def step(self, dt: float, state: FleetState) -> np.ndarray:
        """World-frame force on each drone, (n, 3), in N."""
        ...


EffectFactory = Callable[[Mapping[str, Any], WorldSpec], Effect]
EFFECTS = Registry[EffectFactory]("effect")

ALWAYS_ON = ("wind",)  # carries aerodynamic drag, so it is active (calm) even when not configured


class EffectStack:
    def __init__(self, effects: dict[str, Effect], n: int):
        self._effects, self.n = effects, n

    @classmethod
    def from_world(cls, world: WorldSpec) -> EffectStack:
        settings = {name: {} for name in ALWAYS_ON} | dict(world.environment)
        return cls({name: EFFECTS.get(name)(cfg, world) for name, cfg in settings.items()}, len(world.drones))

    @property
    def effects(self) -> list[Effect]:
        return list(self._effects.values())

    def reset(self, seed: int) -> None:
        for name, effect in self._effects.items():
            effect.reset(rng_stream(seed, f"effect:{name}"))

    def step(self, dt: float, state: FleetState) -> np.ndarray:
        total = np.zeros((self.n, 3))
        for effect in self._effects.values():
            total += effect.step(dt, state)
        return total
