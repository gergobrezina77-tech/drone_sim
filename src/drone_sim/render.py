"""Drawing helpers shared by the live viewer, the offscreen recorder and the envs.

Extra geoms (trails, markers) are appended to an ``MjvScene``: ``viewer.user_scn`` for the
passive viewer or ``renderer.scene`` after ``update_scene`` for offscreen rendering.
"""

from __future__ import annotations

import mujoco
import numpy as np

_IDENTITY = np.eye(3).flatten()


def add_marker(scene: mujoco.MjvScene, pos, rgba, size: float = 0.06) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    mujoco.mjv_initGeom(
        scene.geoms[scene.ngeom],
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.full(3, size),
        np.asarray(pos, float),
        _IDENTITY,
        np.asarray(rgba, np.float32),
    )
    scene.ngeom += 1


def add_segment(scene: mujoco.MjvScene, a, b, rgba, width: float = 3.0) -> None:
    """Line segment; ``width`` is in pixels. Lines are far cheaper than capsules to render."""
    if scene.ngeom >= scene.maxgeom:
        return
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        g, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), _IDENTITY, np.asarray(rgba, np.float32)
    )
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, width, np.asarray(a, float), np.asarray(b, float))
    scene.ngeom += 1


class Trails:
    """Fading position history per drone."""

    def __init__(self, colors: np.ndarray, length: int = 60, interval: float = 0.05):
        self.colors, self.length, self.interval = colors, length, interval
        self.history: list[np.ndarray] = []
        self._last_t = -np.inf

    def push(self, t: float, positions: np.ndarray) -> None:
        if t - self._last_t >= self.interval:
            self._last_t = t
            self.history.append(positions.copy())
            del self.history[: -self.length]

    def clear(self) -> None:
        self.history.clear()
        self._last_t = -np.inf

    def draw(self, scene: mujoco.MjvScene, width: float = 3.0) -> None:
        h = self.history
        for k in range(1, len(h)):
            alpha = 0.7 * k / len(h)
            for i, rgb in enumerate(self.colors):
                add_segment(scene, h[k - 1][i], h[k][i], (*rgb, alpha), width)


class FollowCamera:
    """Free camera that smoothly tracks a group of points and slowly orbits it."""

    def __init__(self, azimuth: float = 135.0, elevation: float = -25.0, orbit_deg_per_s: float = 4.0):
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.azimuth, self.cam.elevation = azimuth, elevation
        self.orbit = orbit_deg_per_s
        self._lookat = None
        self._dist = None

    def update(self, points: np.ndarray, dt: float, smoothing: float = 2.0) -> mujoco.MjvCamera:
        center = points.mean(axis=0)
        spread = np.linalg.norm(points - center, axis=1).max() if len(points) > 1 else 0.0
        dist = 2.0 + 1.8 * spread
        if self._lookat is None:
            self._lookat, self._dist = center, dist
        a = 1.0 - np.exp(-smoothing * dt)
        self._lookat = self._lookat + a * (center - self._lookat)
        self._dist = self._dist + a * (dist - self._dist)
        self.cam.lookat[:] = self._lookat
        self.cam.distance = self._dist
        self.cam.azimuth += self.orbit * dt
        return self.cam
