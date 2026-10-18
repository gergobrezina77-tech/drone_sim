"""Wind: mean flow + Ornstein-Uhlenbeck turbulence + triggered gusts, applied through
aerodynamic drag on the air-relative velocity."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from drone_sim.core.state import FleetState
from drone_sim.vehicle import get_vehicle
from drone_sim.world import WorldSpec


@dataclass
class WindConfig:
    mean: tuple[float, float, float] = (0.0, 0.0, 0.0)  # m/s
    turbulence: float = 0.0  # std of the swarm-wide turbulent component, m/s
    local_turbulence: float = 0.0  # std of the per-drone component, m/s
    tau: float = 2.0  # turbulence correlation time, s
    gust_strength: float = 5.0  # m/s
    gust_duration: float = 2.0  # s


class WindField:
    def __init__(self, cfg: WindConfig, n: int, rng: np.random.Generator | None = None):
        self.cfg, self.n = cfg, n
        self.rng = rng or np.random.default_rng()
        self.reset()

    def reset(self) -> None:
        self._shared = np.zeros(3)
        self._local = np.zeros((self.n, 3))
        self._gust = None  # (time elapsed, direction)

    def gust(self, direction=None) -> None:
        if direction is None:
            a = self.rng.uniform(0, 2 * np.pi)
            direction = (np.cos(a), np.sin(a), 0.0)
        d = np.asarray(direction, dtype=float)
        self._gust = [0.0, d / max(np.linalg.norm(d), 1e-9)]

    def step(self, dt: float) -> np.ndarray:
        """Advance the field and return the air velocity at each drone (n, 3)."""
        c = self.cfg
        decay, noise = dt / c.tau, np.sqrt(2 * dt / c.tau)
        self._shared += -self._shared * decay + c.turbulence * noise * self.rng.standard_normal(3)
        self._local += -self._local * decay + c.local_turbulence * noise * self.rng.standard_normal((self.n, 3))
        air = np.asarray(c.mean) + self._shared + self._local
        if self._gust is not None:
            self._gust[0] += dt
            t, d = self._gust
            if t >= c.gust_duration:
                self._gust = None
            else:
                air = air + c.gust_strength * np.sin(np.pi * t / c.gust_duration) * d
        return air


def aero_drag(vel: np.ndarray, air_vel: np.ndarray, coef: float) -> np.ndarray:
    return -coef * (vel - air_vel)


class WindEffect:
    """Effect plug-in: the wind field plus aerodynamic drag on each drone's air-relative velocity.

    Settings are ``WindConfig`` fields (``mean`` as a list). Always active, calm by default, because drag needs it.
    """

    def __init__(self, settings: Mapping[str, Any], world: WorldSpec):
        cfg = dict(settings)
        if "mean" in cfg:
            cfg["mean"] = tuple(float(v) for v in cfg["mean"])
        self.cfg = WindConfig(**cfg)
        self.n = len(world.drones)
        self.drag = np.array([get_vehicle(d.vehicle).drag_coef for d in world.drones])
        self.field = WindField(self.cfg, self.n)

    def reset(self, rng: np.random.Generator) -> None:
        self.field = WindField(self.cfg, self.n, rng)

    def step(self, dt: float, state: FleetState) -> np.ndarray:
        return -self.drag[:, None] * (state.vel - self.field.step(dt))

    def gust(self, direction=None) -> None:
        self.field.gust(direction)
