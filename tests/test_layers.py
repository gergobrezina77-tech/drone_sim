"""The layer packages exist and the import-linter layer contract is strict and actually enforced."""

import importlib
import os
import subprocess
import sys
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
LAYERS = ["api", "engine", "physics", "environment", "autonomy", "tasks", "world", "vehicle", "core"]


def _contract_layers() -> list[str]:
    config = tomllib.loads((ROOT / "pyproject.toml").read_text())["tool"]["importlinter"]
    (contract,) = [c for c in config["contracts"] if c["type"] == "layers"]
    return contract["layers"]


@pytest.mark.parametrize("layer", LAYERS)
def test_layer_package_imports_and_documents_its_role(layer):
    module = importlib.import_module(f"drone_sim.{layer}")
    assert module.__doc__ and module.__doc__.strip()


def test_contract_covers_every_layer_and_none_is_optional():
    layers = _contract_layers()
    assert not any("(" in row for row in layers)
    named = {name.strip().removeprefix("drone_sim.") for row in layers for name in row.split("|")}
    assert named == set(LAYERS)


def test_contract_rejects_an_upward_import(tmp_path):
    """Run the real contract against a throwaway package where core imports engine."""
    pkg = tmp_path / "drone_sim"
    for layer in LAYERS:
        (pkg / layer).mkdir(parents=True)
        (pkg / layer / "__init__.py").write_text("")
    (pkg / "__init__.py").write_text("")
    (pkg / "core" / "__init__.py").write_text("import drone_sim.engine\n")
    (tmp_path / "pyproject.toml").write_text((ROOT / "pyproject.toml").read_text())

    env = {**os.environ, "PYTHONPATH": str(tmp_path)}
    lint = Path(sys.executable).with_name("lint-imports")
    result = subprocess.run([str(lint)], check=False, cwd=tmp_path, env=env, capture_output=True, text=True)

    assert result.returncode != 0
    assert "drone_sim.core -> drone_sim.engine" in result.stdout
