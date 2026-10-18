"""Batched runtime state of a fleet of drones, shared by physics, environment, autonomy and tasks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class FleetState:
    pos: np.ndarray  # (n, 3) world position
    quat: np.ndarray  # (n, 4) orientation, w x y z
    R: np.ndarray  # (n, 3, 3) body-to-world rotation
    vel: np.ndarray  # (n, 3) world linear velocity
    omega: np.ndarray  # (n, 3) body-frame angular velocity

    @property
    def n(self) -> int:
        return len(self.pos)
