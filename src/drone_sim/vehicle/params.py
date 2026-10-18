"""Physical parameters of a quadrotor, as pure data."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

GRAVITY = 9.81


@dataclass(frozen=True)
class DroneParams:
    """Physical parameters of an X-configuration quadrotor (single source of truth for
    both the MJCF model and the controller's allocation matrix)."""

    mass: float = 0.5  # kg
    inertia: tuple[float, float, float] = (2.3e-3, 2.3e-3, 4.0e-3)  # kg m^2
    arm: float = 0.12  # centre-to-motor distance, m
    rotor_radius: float = 0.055  # m; rotor tips reach arm + rotor_radius = 0.175 m
    max_thrust: float = 3.5  # per motor, N  (thrust/weight ~ 2.85)
    yaw_coef: float = 0.016  # rotor drag torque / thrust, m
    drag_coef: float = 0.2  # linear aerodynamic drag, N / (m/s)
    radius: float = 0.18  # footprint radius (rotor tips) used by swarm logic and rewards, m
    rest_height: float = 0.07  # body-origin height when resting on its legs, m

    @property
    def motor_angles(self) -> np.ndarray:
        # front-left, rear-left, rear-right, front-right (x forward, y left)
        return np.deg2rad([45.0, 135.0, 225.0, 315.0])

    @property
    def motor_xy(self) -> np.ndarray:
        a = self.motor_angles
        return self.arm * np.stack([np.cos(a), np.sin(a)], axis=1)

    @property
    def spins(self) -> np.ndarray:
        # diagonal motors spin the same way; sign of the reaction torque about body z
        return np.array([1.0, -1.0, 1.0, -1.0])

    @property
    def allocation(self) -> np.ndarray:
        """A such that [T, tau_x, tau_y, tau_z] = A @ f for motor thrusts f."""
        x, y = self.motor_xy.T
        return np.stack([np.ones(4), y, -x, self.spins * self.yaw_coef])

    @property
    def hover_thrust(self) -> float:
        return self.mass * GRAVITY / 4.0
