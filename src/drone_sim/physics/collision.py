"""Geometric collision detection for drone footprints (spheres of the vehicle's radius).

Both checks avoid all-pairs work: obstacles are prefiltered by axis-aligned bounding boxes, and drone pairs by
sort-and-sweep along the fleet's widest axis.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from drone_sim.core.geometry import Box, Shape, distance


def aabb(shape: Shape) -> tuple[np.ndarray, np.ndarray]:
    if isinstance(shape, Box):
        c, h = np.array(shape.center), np.array(shape.half_extents)
        return c - h, c + h
    b, r = np.array(shape.base), shape.radius
    return b - (r, r, 0.0), b + (r, r, shape.height)


def touching_obstacles(
    pos: np.ndarray, radius: np.ndarray, shapes: Sequence[Shape], lo: np.ndarray, hi: np.ndarray
) -> np.ndarray:
    """(n,) bool: footprint overlaps any shape. ``lo`` / ``hi`` are the shapes' bounding boxes, (m, 3)."""
    hit = np.zeros(len(pos), dtype=bool)
    if not len(shapes) or not len(pos):
        return hit
    r = radius[:, None, None]
    near = ((pos[:, None, :] + r >= lo[None]) & (pos[:, None, :] - r <= hi[None])).all(axis=-1)
    for j in np.flatnonzero(near.any(axis=0)):
        idx = np.flatnonzero(near[:, j])
        d, _ = distance(pos[idx], [shapes[j]])
        hit[idx[d[:, 0] < radius[idx]]] = True
    return hit


def touching_pairs(pos: np.ndarray, radius: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Index pairs (i, j) of drones whose footprints overlap, each pair once.

    Sort along the widest axis; compare each drone with its k-th sorted neighbour for k = 1, 2, ... while any such
    neighbour is closer along that axis than the largest possible contact distance. Sorted gaps only grow with k,
    so the first k with no candidate ends the sweep.
    """
    n = len(pos)
    if n < 2:
        return np.zeros(0, int), np.zeros(0, int)
    axis = int(np.argmax(pos.max(axis=0) - pos.min(axis=0)))
    order = np.argsort(pos[:, axis], kind="stable")
    coord = pos[order, axis]
    reach = 2.0 * radius.max()
    found_i, found_j = [np.zeros(0, int)], [np.zeros(0, int)]
    for k in range(1, n):
        cand = np.flatnonzero(coord[k:] - coord[:-k] < reach)
        if not len(cand):
            break
        a, b = order[cand], order[cand + k]
        hit = np.linalg.norm(pos[a] - pos[b], axis=1) < radius[a] + radius[b]
        found_i.append(a[hit])
        found_j.append(b[hit])
    return np.concatenate(found_i), np.concatenate(found_j)
