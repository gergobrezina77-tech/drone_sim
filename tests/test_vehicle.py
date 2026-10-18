"""vehicle layer: parameters as pure data, registry by id."""

import ast
from pathlib import Path

import pytest

import drone_sim.vehicle as vehicle_pkg
from drone_sim.vehicle import DEFAULT_VEHICLE, GRAVITY, DroneParams, get_vehicle, register_vehicle, vehicle_ids


def test_vehicle_layer_has_no_physics_engine_imports():
    for path in Path(vehicle_pkg.__file__).parent.rglob("*.py"):
        tree = ast.parse(path.read_text())
        imported = [a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names]
        imported += [n.module or "" for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
        assert not [m for m in imported if m.split(".")[0] == "mujoco"], path


def test_default_vehicle_is_registered():
    assert DEFAULT_VEHICLE in vehicle_ids()
    assert get_vehicle(DEFAULT_VEHICLE) == DroneParams()


def test_hover_thrust_balances_weight():
    p = get_vehicle(DEFAULT_VEHICLE)
    assert 4 * p.hover_thrust == pytest.approx(p.mass * GRAVITY)


def test_register_and_lookup_new_vehicle():
    heavy = DroneParams(mass=1.2, max_thrust=7.0)
    register_vehicle("test_heavy_quad", heavy)
    assert get_vehicle("test_heavy_quad") is heavy


def test_duplicate_id_is_rejected():
    with pytest.raises(ValueError, match="already registered"):
        register_vehicle(DEFAULT_VEHICLE, DroneParams(mass=9.0))
    assert get_vehicle(DEFAULT_VEHICLE) == DroneParams()


def test_unknown_id_error_names_the_known_ids():
    with pytest.raises(KeyError, match=DEFAULT_VEHICLE):
        get_vehicle("no_such_drone")


def test_legacy_import_path_is_the_same_class():
    from drone_sim.drone import GRAVITY as legacy_g
    from drone_sim.drone import DroneParams as legacy

    assert legacy is DroneParams and legacy_g == GRAVITY
