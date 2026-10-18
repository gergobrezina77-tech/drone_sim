"""physics: collision detection helpers, checked against brute force."""

import time

import numpy as np
import pytest

from drone_sim.physics.collision import touching_pairs


def brute_force(pos, radius):
    gap = np.linalg.norm(pos[:, None] - pos[None], axis=-1) - (radius[:, None] + radius[None])
    i, j = np.nonzero(np.triu(gap < 0, k=1))
    return set(zip(i.tolist(), j.tolist(), strict=True))


def as_set(pairs):
    i, j = pairs
    return {(min(a, b), max(a, b)) for a, b in zip(i.tolist(), j.tolist(), strict=True)}


@pytest.mark.parametrize("seed", range(20))
def test_matches_brute_force_on_random_fleets(seed):
    rng = np.random.default_rng(seed)
    n = int(rng.integers(0, 300))
    spread = rng.uniform(0.5, 20.0)
    pos = rng.uniform(-spread, spread, (n, 3))
    if n and seed % 3 == 0:
        pos[:, 0] = np.round(pos[:, 0])  # many drones sharing an x coordinate (grid columns)
    if n > 4 and seed % 4 == 0:
        pos[1] = pos[0]  # exact duplicates
    radius = rng.uniform(0.1, 0.4, n)
    assert as_set(touching_pairs(pos, radius)) == brute_force(pos, radius)


def test_each_pair_is_reported_once():
    pos = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0]])
    i, j = touching_pairs(pos, np.full(3, 0.18))
    assert len(i) == len(as_set((i, j))) == 3


def test_empty_and_single():
    for n in (0, 1):
        i, j = touching_pairs(np.zeros((n, 3)), np.full(n, 0.2))
        assert len(i) == len(j) == 0


def test_500_drones_on_a_grid_is_fast():
    k = np.arange(500)
    pos = np.stack([k // 23 * 1.5, k % 23 * 1.5, np.full(500, 2.0)], axis=1)
    radius = np.full(500, 0.18)
    touching_pairs(pos, radius)
    best = min(_timed(touching_pairs, pos, radius) for _ in range(5))
    assert best < 0.0023, f"{best * 1e3:.2f} ms"  # 10x faster than the 23 ms n x n check (DS-008)


def _timed(f, *args):
    t0 = time.perf_counter()
    f(*args)
    return time.perf_counter() - t0


def test_far_apart_drones_give_no_pairs():
    pos = np.array([[0.0, 0.0, 1.0], [10.0, 0.0, 1.0], [20.0, 0.0, 1.0]])
    i, j = touching_pairs(pos, np.full(3, 0.18))
    assert len(i) == len(j) == 0
