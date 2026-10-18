"""core layer: the generic name registry used by every extension point."""

import pytest

from drone_sim.core.registry import Registry


def test_register_and_get():
    reg = Registry[int]("widget")
    assert reg.register("a", 1) == 1
    assert reg.get("a") == 1 and "a" in reg and "b" not in reg


def test_duplicate_name_is_rejected_and_keeps_the_original():
    reg = Registry[int]("widget")
    reg.register("a", 1)
    with pytest.raises(ValueError, match="widget 'a' is already registered"):
        reg.register("a", 2)
    assert reg.get("a") == 1


def test_unknown_name_error_lists_the_known_names():
    reg = Registry[int]("widget")
    reg.register("zeta", 1)
    reg.register("alpha", 2)
    with pytest.raises(KeyError, match="unknown widget 'nope'; known: alpha, zeta"):
        reg.get("nope")


def test_names_are_sorted():
    reg = Registry[int]("widget")
    for n in ("c", "a", "b"):
        reg.register(n, 0)
    assert reg.names() == ["a", "b", "c"]
