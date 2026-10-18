"""Vectorised geometric quadrotor controller (Lee, Leok & McClamroch 2010).

All methods operate on N drones at once and return per-motor thrusts of shape (N, 4).
Cascade: position/velocity -> desired acceleration -> desired attitude + collective
thrust -> body torques -> motor mixer.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from drone_sim.core.state import FleetState
from drone_sim.vehicle import GRAVITY, DroneParams

E3 = np.array([0.0, 0.0, 1.0])


@dataclass
class Gains:
    kp_pos: tuple = (5.0, 5.0, 8.0)
    kd_pos: tuple = (4.0, 4.0, 5.0)
    ki_pos: tuple = (0.5, 0.5, 1.0)
    kv: tuple = (3.0, 3.0, 4.0)  # velocity-command mode
    ki_vel: tuple = (0.6, 0.6, 1.0)
    k_att: tuple = (0.92, 0.92, 0.26)  # J * w^2 with w = 20, 20, 8 rad/s
    k_rate: tuple = (0.074, 0.074, 0.051)  # 2 * zeta * w * J
    k_rate_ctbr: tuple = (25.0, 25.0, 12.0)  # 1/s, body-rate tracking
    k_att_to_rate: tuple = (8.0, 8.0, 4.0)  # 1/s, attitude error -> rate setpoint
    max_tilt_deg: float = 35.0
    integral_limit: float = 2.0
    integral_zone: float = 0.5  # m; only integrate position error near the setpoint (anti-windup)


def _vee(m: np.ndarray) -> np.ndarray:
    return np.stack([m[:, 2, 1], m[:, 0, 2], m[:, 1, 0]], axis=1)


def _normalize(v: np.ndarray) -> np.ndarray:
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)


class GeometricController:
    def __init__(self, params: DroneParams, n: int, gains: Gains | None = None):
        self.params, self.n = params, n
        self.gains = gains or Gains()
        self.J = np.asarray(params.inertia)
        self.mixer = np.linalg.inv(params.allocation)
        self.reset()

    def reset(self) -> None:
        self._int_pos = np.zeros((self.n, 3))
        self._int_vel = np.zeros((self.n, 3))

    # ---- outer loops: produce a desired acceleration (gravity included) ----

    def position_accel(self, s: FleetState, pos_ref, vel_ref=None, acc_ref=None, dt: float = 0.0) -> np.ndarray:
        g = self.gains
        e_p = np.asarray(pos_ref) - s.pos
        e_v = (0.0 if vel_ref is None else np.asarray(vel_ref)) - s.vel
        e_int = np.where(np.abs(e_p) < g.integral_zone, e_p, 0.0)
        self._int_pos = np.clip(self._int_pos + e_int * dt, -g.integral_limit, g.integral_limit)
        a = np.asarray(g.kp_pos) * e_p + np.asarray(g.kd_pos) * e_v + np.asarray(g.ki_pos) * self._int_pos
        if acc_ref is not None:
            a = a + acc_ref
        return self._limit_tilt(a + GRAVITY * E3)

    def velocity_accel(self, s: FleetState, vel_cmd, dt: float) -> np.ndarray:
        g = self.gains
        e_v = np.asarray(vel_cmd) - s.vel
        self._int_vel = np.clip(self._int_vel + e_v * dt, -g.integral_limit, g.integral_limit)
        a = np.asarray(g.kv) * e_v + np.asarray(g.ki_vel) * self._int_vel
        return self._limit_tilt(a + GRAVITY * E3)

    def _limit_tilt(self, a: np.ndarray) -> np.ndarray:
        a = a.copy()
        a[:, 2] = np.maximum(a[:, 2], 0.25 * GRAVITY)  # never command free fall
        h = np.linalg.norm(a[:, :2], axis=1)
        h_max = a[:, 2] * np.tan(np.deg2rad(self.gains.max_tilt_deg))
        a[:, :2] *= np.minimum(1.0, h_max / np.maximum(h, 1e-9))[:, None]
        return a

    # ---- acceleration -> attitude ----

    def accel_to_attitude(self, s: FleetState, a_des: np.ndarray, yaw) -> tuple[np.ndarray, np.ndarray]:
        """Collective thrust (N,) and desired rotation (N, 3, 3) for a desired acceleration."""
        force = self.params.mass * a_des
        thrust = np.clip(np.einsum("ij,ij->i", force, s.R[:, :, 2]), 0.0, 4 * self.params.max_thrust)
        b3 = _normalize(force)
        yaw = np.broadcast_to(yaw, (self.n,))
        b1c = np.stack([np.cos(yaw), np.sin(yaw), np.zeros(self.n)], axis=1)
        b2 = _normalize(np.cross(b3, b1c))
        b1 = np.cross(b2, b3)
        return thrust, np.stack([b1, b2, b3], axis=-1)

    def attitude_error(self, s: FleetState, R_des: np.ndarray) -> np.ndarray:
        Rt = np.swapaxes(s.R, 1, 2)
        Rdt = np.swapaxes(R_des, 1, 2)
        return 0.5 * _vee(Rdt @ s.R - Rt @ R_des)

    def attitude_to_rates(self, s: FleetState, R_des: np.ndarray) -> np.ndarray:
        return -np.asarray(self.gains.k_att_to_rate) * self.attitude_error(s, R_des)

    # ---- inner loops: produce motor thrusts ----

    def attitude_control(self, s: FleetState, thrust: np.ndarray, R_des: np.ndarray) -> np.ndarray:
        g = self.gains
        e_R = self.attitude_error(s, R_des)
        tau = -np.asarray(g.k_att) * e_R - np.asarray(g.k_rate) * s.omega + np.cross(s.omega, self.J * s.omega)
        return self.mix(thrust, tau)

    def rate_control(self, s: FleetState, thrust: np.ndarray, rates_des: np.ndarray) -> np.ndarray:
        k = np.asarray(self.gains.k_rate_ctbr)
        tau = self.J * k * (rates_des - s.omega) + np.cross(s.omega, self.J * s.omega)
        return self.mix(thrust, tau)

    def mix(self, thrust: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """Allocate [T, tau] to motors. Yaw has the lowest priority: it is scaled down
        until all motors are within limits, then any remaining excess is clipped."""
        f_max = self.params.max_thrust
        f_base = thrust[:, None] * self.mixer[:, 0] + tau[:, :2] @ self.mixer[:, 1:3].T
        f_yaw = tau[:, 2:3] * self.mixer[:, 3]
        with np.errstate(divide="ignore", invalid="ignore"):
            room = np.where(f_yaw > 0, (f_max - f_base) / f_yaw, np.where(f_yaw < 0, -f_base / f_yaw, np.inf))
        scale = np.clip(room.min(axis=1), 0.0, 1.0)
        return np.clip(f_base + scale[:, None] * f_yaw, 0.0, f_max)

    # ---- convenience end-to-end modes ----

    def position_control(self, s: FleetState, pos_ref, vel_ref=None, acc_ref=None, yaw=0.0, dt: float = 0.0):
        thrust, R_des = self.accel_to_attitude(s, self.position_accel(s, pos_ref, vel_ref, acc_ref, dt), yaw)
        return self.attitude_control(s, thrust, R_des)

    def velocity_control(self, s: FleetState, vel_cmd, yaw=0.0, dt: float = 0.0):
        thrust, R_des = self.accel_to_attitude(s, self.velocity_accel(s, vel_cmd, dt), yaw)
        return self.attitude_control(s, thrust, R_des)
