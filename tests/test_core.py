"""core layer: rotations, batched geometry and deterministic RNG streams."""

import time

import numpy as np
import pytest

from drone_sim.core.geometry import Box, Cylinder, distance
from drone_sim.core.rng import rng_stream
from drone_sim.core.rotations import quat_to_mat, rot_z

BOX = Box(center=(0.0, 0.0, 1.0), half_extents=(1.0, 2.0, 1.0))
PILLAR = Cylinder(base=(5.0, 0.0, 0.0), radius=0.5, height=3.0)


# --- rotations ---------------------------------------------------------------------------------------------------


def test_quat_to_mat_matches_rot_z_for_yaw_quaternions():
    yaw = np.array([0.0, 0.3, -2.0, np.pi])
    q = np.stack([np.cos(yaw / 2), 0 * yaw, 0 * yaw, np.sin(yaw / 2)], axis=1)
    R = quat_to_mat(q)
    for k, a in enumerate(yaw):
        np.testing.assert_allclose(R[k], rot_z(a), atol=1e-12)


def test_quat_to_mat_returns_rotations():
    q = np.random.default_rng(0).normal(size=(50, 4))
    R = quat_to_mat(q / np.linalg.norm(q, axis=1, keepdims=True))
    np.testing.assert_allclose(R @ R.transpose(0, 2, 1), np.broadcast_to(np.eye(3), R.shape), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-12)


def test_legacy_names_still_importable():
    from drone_sim.drone import quat_to_mat as legacy_q
    from drone_sim.planner import rot_z as legacy_r

    assert legacy_q is quat_to_mat and legacy_r is rot_z


# --- geometry: hand-checked cases --------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "point, shape, expected_d, expected_normal",
    [
        ((3.0, 0.0, 1.0), BOX, 2.0, (1, 0, 0)),  # outside, facing +x
        ((0.0, 0.0, 3.5), BOX, 1.5, (0, 0, 1)),  # outside, above
        ((2.0, 3.0, 1.0), BOX, np.sqrt(2.0), (1 / np.sqrt(2), 1 / np.sqrt(2), 0)),  # outside, edge region
        ((1.0, 0.0, 1.0), BOX, 0.0, (1, 0, 0)),  # on the surface
        ((0.5, 0.0, 1.0), BOX, 0.0, (1, 0, 0)),  # inside: horizontal radial normal
        ((7.0, 0.0, 1.0), PILLAR, 1.5, (1, 0, 0)),  # outside, side
        ((5.0, 0.0, 4.0), PILLAR, 1.0, (0, 0, 1)),  # outside, above the top cap
        ((5.0, 0.5, 2.0), PILLAR, 0.0, (0, 1, 0)),  # on the surface
        ((5.0, -0.2, 1.0), PILLAR, 0.0, (0, -1, 0)),  # inside: horizontal radial normal
    ],
)
def test_distance_and_normal(point, shape, expected_d, expected_normal):
    d, n = distance(np.array([point]), [shape])
    assert d.shape == (1, 1) and n.shape == (1, 1, 3)
    assert d[0, 0] == pytest.approx(expected_d)
    np.testing.assert_allclose(n[0, 0], expected_normal, atol=1e-9)


def test_inside_at_exact_centre_still_gives_a_unit_normal():
    _, n = distance(np.array([BOX.center]), [BOX])
    assert np.linalg.norm(n[0, 0]) == pytest.approx(1.0)


def test_no_shapes_gives_empty_columns():
    d, n = distance(np.zeros((4, 3)), [])
    assert d.shape == (4, 0) and n.shape == (4, 0, 3)


# --- geometry: batched result equals a per-point reference -------------------------------------------------------


def _reference_closest(p, s):
    """Deliberately naive, one point and one shape at a time."""
    if isinstance(s, Box):
        c, h = np.array(s.center), np.array(s.half_extents)
        return np.minimum(np.maximum(p, c - h), c + h)
    b = np.array(s.base)
    v = p[:2] - b[:2]
    rho = np.hypot(*v)
    xy = p[:2] if rho <= s.radius else b[:2] + v / rho * s.radius
    return np.array([*xy, min(max(p[2], b[2]), b[2] + s.height)])


def _random_shapes(rng, k):
    shapes = []
    for _ in range(k):
        c = rng.uniform(-10, 10, 3)
        if rng.random() < 0.5:
            shapes.append(Box(tuple(c), tuple(rng.uniform(0.1, 2.0, 3))))
        else:
            shapes.append(Cylinder((c[0], c[1], 0.0), float(rng.uniform(0.1, 1.5)), float(rng.uniform(0.5, 6.0))))
    return shapes


def test_batched_distance_matches_reference_on_random_points():
    rng = np.random.default_rng(42)
    shapes = _random_shapes(rng, 25)
    points = rng.uniform(-12, 12, (300, 3))
    d, n = distance(points, shapes)
    for i, p in enumerate(points):
        for j, s in enumerate(shapes):
            diff = p - _reference_closest(p, s)
            assert d[i, j] == pytest.approx(np.linalg.norm(diff), abs=1e-9)
            if d[i, j] > 1e-6:
                np.testing.assert_allclose(n[i, j], diff / np.linalg.norm(diff), atol=1e-9)
    np.testing.assert_allclose(np.linalg.norm(n, axis=-1), 1.0, atol=1e-9)


def test_batched_distance_is_fast_for_large_scenes():
    rng = np.random.default_rng(1)
    shapes, points = _random_shapes(rng, 200), rng.uniform(-12, 12, (1000, 3))
    distance(points, shapes)  # warm-up
    best = min(_timed(distance, points, shapes) for _ in range(3))
    assert best < 0.050, f"1000 points x 200 shapes took {best * 1000:.1f} ms"


def _timed(f, *args):
    t0 = time.perf_counter()
    f(*args)
    return time.perf_counter() - t0


# --- rng ---------------------------------------------------------------------------------------------------------


def test_rng_stream_is_reproducible_and_independent_per_name():
    a1, a2 = rng_stream(7, "wind").random(5), rng_stream(7, "wind").random(5)
    np.testing.assert_array_equal(a1, a2)
    assert not np.allclose(rng_stream(7, "wind").random(5), rng_stream(7, "sensors").random(5))
    assert not np.allclose(rng_stream(7, "wind").random(5), rng_stream(8, "wind").random(5))


def test_rng_stream_does_not_depend_on_creation_order():
    first = rng_stream(3, "sensors").random(3)
    rng_stream(3, "wind").random(100)  # another subsystem drawing numbers must not shift this one
    np.testing.assert_array_equal(rng_stream(3, "sensors").random(3), first)


# --- fleet state -------------------------------------------------------------------------------------------------


def test_fleet_state_is_shared_with_the_legacy_name():
    from drone_sim.core import FleetState
    from drone_sim.drone import DroneState

    assert DroneState is FleetState
    s = FleetState(
        np.zeros((3, 3)),
        np.tile([1.0, 0, 0, 0], (3, 1)),
        np.tile(np.eye(3), (3, 1, 1)),
        np.zeros((3, 3)),
        np.zeros((3, 3)),
    )
    assert s.n == 3
