"""Rotation helpers, batched where it matters."""

from __future__ import annotations

import numpy as np


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """Batch (n, 4) wxyz quaternions -> (n, 3, 3) rotation matrices."""
    w, x, y, z = np.moveaxis(q, -1, 0)
    # fmt: off
    return np.stack(
        [
            1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
        ],
        axis=-1,
    ).reshape(*q.shape[:-1], 3, 3)
    # fmt: on


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
