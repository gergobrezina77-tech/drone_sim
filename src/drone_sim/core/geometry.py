"""Axis-aligned shape primitives and batched point-to-shape distance queries.

``distance(points, shapes)`` evaluates every point against every shape in one vectorised pass per shape type,
so collision checks and avoidance scale to large fleets and scenes.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

_EPS = 1e-9


@dataclass(frozen=True)
class Box:
    center: tuple[float, float, float]
    half_extents: tuple[float, float, float]


@dataclass(frozen=True)
class Cylinder:
    """Vertical cylinder standing on ``base`` (centre of its bottom face)."""

    base: tuple[float, float, float]
    radius: float
    height: float


Shape = Box | Cylinder


def closest_points(points: np.ndarray, shapes: Sequence[Shape]) -> np.ndarray:
    """Closest point on (or in) each shape to each point: (n, 3) x m shapes -> (n, m, 3)."""
    p = np.asarray(points, dtype=float)
    out = np.empty((len(p), len(shapes), 3))
    boxes = [j for j, s in enumerate(shapes) if isinstance(s, Box)]
    cyls = [j for j, s in enumerate(shapes) if isinstance(s, Cylinder)]
    if len(boxes) + len(cyls) != len(shapes):
        raise TypeError("shapes must be Box or Cylinder")
    if boxes:
        c = np.array([shapes[j].center for j in boxes])
        h = np.array([shapes[j].half_extents for j in boxes])
        out[:, boxes] = np.clip(p[:, None, :], c - h, c + h)
    if cyls:
        base = np.array([shapes[j].base for j in cyls])
        r = np.array([shapes[j].radius for j in cyls])
        top = base[:, 2] + np.array([shapes[j].height for j in cyls])
        d = p[:, None, :2] - base[:, :2]
        rho = np.linalg.norm(d, axis=-1, keepdims=True)
        xy = base[:, :2] + d / np.maximum(rho, _EPS) * np.minimum(rho, r[:, None])
        z = np.clip(p[:, None, 2], base[:, 2], top)
        out[:, cyls] = np.concatenate([xy, z[..., None]], axis=-1)
    return out


def distance(points: np.ndarray, shapes: Sequence[Shape]) -> tuple[np.ndarray, np.ndarray]:
    """Distance from each point to each shape's surface, (n, m), and the outward unit normal, (n, m, 3).

    Points on or inside a shape get distance 0 and a horizontal normal pointing away from the shape's axis.
    """
    p = np.asarray(points, dtype=float)
    diff = p[:, None, :] - closest_points(p, shapes)
    d = np.linalg.norm(diff, axis=-1)
    normal = diff / np.maximum(d, _EPS)[..., None]
    ii, jj = np.nonzero(d < _EPS)
    if len(ii):
        axis = np.array([s.center if isinstance(s, Box) else s.base for s in shapes])
        radial = p[ii] - axis[jj]
        radial[:, 2] = 0.0
        norm = np.linalg.norm(radial, axis=1, keepdims=True)
        normal[ii, jj] = np.where(norm > _EPS, radial / np.maximum(norm, _EPS), (1.0, 0.0, 0.0))
    return d, normal
