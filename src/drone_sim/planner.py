"""Formation shapes and waypoint missions for the swarm's virtual centre."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

FORMATIONS = ("line", "v", "circle", "grid", "sphere", "helix")


def formation_offsets(name: str, n: int, spacing: float = 0.9) -> np.ndarray:
    """Slot offsets (n, 3) around the formation centre, in the formation frame
    (x = direction of travel). Neighbouring slots are roughly ``spacing`` apart."""
    i = np.arange(n, dtype=float)
    if name == "line":
        off = np.stack([np.zeros(n), (i - (n - 1) / 2) * spacing, np.zeros(n)], axis=1)
    elif name == "v":
        k = np.ceil(i / 2)
        side = np.where(i % 2 == 1, 1.0, -1.0)
        half_angle = np.deg2rad(50.0)
        off = np.stack(
            [-k * spacing * np.cos(half_angle), side * k * spacing * np.sin(half_angle), np.zeros(n)], axis=1
        )
    elif name == "circle":
        r = max(spacing * n / (2 * np.pi), spacing)
        a = 2 * np.pi * i / n
        off = np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)
    elif name == "grid":
        cols = int(np.ceil(np.sqrt(n)))
        off = np.stack([-(i // cols) * spacing, (i % cols) * spacing, np.zeros(n)], axis=1)
    elif name == "sphere":
        r = max(spacing * np.sqrt(n / (2 * np.pi)), spacing)
        golden = np.pi * (3 - np.sqrt(5))
        z = 1 - 2 * (i + 0.5) / n
        rho = np.sqrt(1 - z * z)
        off = r * np.stack([rho * np.cos(golden * i), rho * np.sin(golden * i), z], axis=1)
    elif name == "helix":
        r = 1.2 * spacing
        a = i * spacing / r
        dz = min(0.3 * spacing, 2.5 / max(n - 1, 1))  # keep the helix at most 2.5 m tall
        off = np.stack([r * np.cos(a), r * np.sin(a), i * dz], axis=1)
    else:
        raise ValueError(f"unknown formation {name!r}; choose from {FORMATIONS}")
    return off - off.mean(axis=0)


def assign_slots(current: np.ndarray, slots: np.ndarray) -> np.ndarray:
    """Greedy nearest-pair assignment of drones to slots (keeps morphs short and
    mostly crossing-free). Returns ``slots`` reordered so row i belongs to drone i."""
    cost = np.linalg.norm(current[:, None, :] - slots[None, :, :], axis=-1)
    out = np.empty_like(slots)
    for _ in range(len(slots)):
        i, j = np.unravel_index(np.argmin(cost), cost.shape)
        out[i] = slots[j]
        cost[i, :] = np.inf
        cost[:, j] = np.inf
    return out


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


@dataclass
class Waypoint:
    pos: tuple[float, float, float]
    formation: str = "grid"
    hold: float = 2.0  # seconds to hover after arriving
    speed: float = 1.2  # cruise speed of the formation centre, m/s
    spacing: float = 0.9


@dataclass
class MissionRef:
    center: np.ndarray  # (3,)
    center_vel: np.ndarray  # (3,)
    heading: float
    targets: np.ndarray  # (n, 3) slot positions in world frame
    target_vel: np.ndarray  # (n, 3)
    formation: str


class Mission:
    """Moves a virtual centre through waypoints (trapezoidal speed profile, rate-limited
    heading) and morphs the formation between shapes. Drones track their slots."""

    def __init__(
        self,
        waypoints: list[Waypoint],
        start_positions: np.ndarray,
        morph_time: float = 3.0,
        morph_lanes: float = 0.5,
        loop: bool = False,
        accel: float = 0.8,
        yaw_rate: float = 0.6,
    ):
        self.waypoints, self.loop = waypoints, loop
        self.morph_time, self.accel, self.yaw_rate = morph_time, accel, yaw_rate
        # during a morph drones spread over three altitude lanes so crossing paths pass
        # above/below each other; the lane offset rises and falls with sin(pi * progress)
        self.lanes = morph_lanes * ((np.arange(len(start_positions)) % 3) - 1.0)
        self.n = len(start_positions)
        self.center = start_positions.mean(axis=0)
        self.vel = np.zeros(3)
        self.heading = 0.0
        self.offsets = start_positions - self.center
        self.formation = "spawn"
        self.spacing = None
        self.t = 0.0
        self.idx = 0
        self.done = False
        self._hold_t = None
        self._prev_targets = None
        self._morph = None
        self._enter(0)

    def _enter(self, idx: int) -> None:
        self.idx = idx
        self._hold_t = None
        wp = self.waypoints[idx]
        if wp.formation != self.formation or wp.spacing != self.spacing:
            self.set_formation(wp.formation, wp.spacing)

    def set_formation(self, name: str, spacing: float | None = None) -> None:
        spacing = spacing or self.spacing or 0.9
        new = assign_slots(self.offsets, formation_offsets(name, self.n, spacing))
        if self._morph is not None:  # interrupted morph: start from the current lane-free blend
            self.offsets[:, 2] -= np.sin(np.pi * min((self.t - self._morph[0]) / self.morph_time, 1.0)) * self.lanes
        self._morph = (self.t, self.offsets.copy(), new)
        self.formation, self.spacing = name, spacing

    def step(self, dt: float) -> MissionRef:
        self.t += dt
        wp = self.waypoints[self.idx]
        if self._hold_t is None:
            to = np.asarray(wp.pos, dtype=float) - self.center
            dist = np.linalg.norm(to)
            v_des = to / max(dist, 1e-9) * min(wp.speed, np.sqrt(2 * self.accel * dist))
            dv = v_des - self.vel
            dv *= min(1.0, self.accel * dt / max(np.linalg.norm(dv), 1e-9))
            self.vel = self.vel + dv
            self.center = self.center + self.vel * dt
            if dist < 0.03:
                self._hold_t = 0.0
                self.vel = np.zeros(3)
        elif not self.done:
            self._hold_t += dt
            if self._hold_t >= wp.hold:
                if self.idx + 1 < len(self.waypoints):
                    self._enter(self.idx + 1)
                elif self.loop:
                    self._enter(0)
                else:
                    self.done = True

        if np.linalg.norm(self.vel[:2]) > 0.3:
            target = np.arctan2(self.vel[1], self.vel[0])
            dh = (target - self.heading + np.pi) % (2 * np.pi) - np.pi
            self.heading += np.clip(dh, -self.yaw_rate * dt, self.yaw_rate * dt)

        if self._morph is not None:
            t0, a, b = self._morph
            p = min((self.t - t0) / self.morph_time, 1.0)
            s = _smoothstep(p)
            self.offsets = (1 - s) * a + s * b
            self.offsets[:, 2] += np.sin(np.pi * p) * self.lanes
            if p >= 1.0:
                self._morph = None

        targets = self.center + self.offsets @ rot_z(self.heading).T
        target_vel = np.zeros_like(targets) if self._prev_targets is None else (targets - self._prev_targets) / dt
        self._prev_targets = targets
        return MissionRef(self.center.copy(), self.vel.copy(), self.heading, targets, target_vel, self.formation)


def demo_mission(start_positions: np.ndarray, loop: bool = True) -> Mission:
    """Take off, then tour the pillar field cycling through every formation."""
    wps = [
        Waypoint((0.0, 0.0, 1.5), "grid", hold=2.0),
        Waypoint((6.0, 0.0, 2.0), "line", hold=1.0),
        Waypoint((10.0, 6.0, 2.5), "v", hold=1.0),
        Waypoint((4.0, 10.0, 3.0), "circle", hold=2.0),
        Waypoint((-4.0, 6.0, 3.0), "sphere", hold=3.0),
        Waypoint((-6.0, -4.0, 2.5), "helix", hold=3.0),
        Waypoint((0.0, 0.0, 1.5), "grid", hold=2.0),
    ]
    return Mission(wps, start_positions, loop=loop)
