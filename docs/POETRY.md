# Poetry cheat-sheet (as used in this project)

Poetry manages three things for a Python project:

1. **Metadata and dependencies**: declared in `pyproject.toml`.
2. **A lock file**: `poetry.lock` pins the exact version of every package, including transitive ones, so everyone gets the same environment.
3. **A virtual environment**: an isolated Python interpreter with only this project's packages.

## One-time machine setup

```bash
# Poetry itself must NOT live inside a project env, so it gets its own isolated install.
# pipx installs CLI tools into private venvs. Here pipx was bootstrapped without sudo:
python3 -m venv ~/.local/share/pipx-bootstrap
~/.local/share/pipx-bootstrap/bin/pip install pipx
ln -s ~/.local/share/pipx-bootstrap/bin/pipx ~/.local/bin/pipx   # ~/.profile puts ~/.local/bin on PATH
pipx install poetry

# Create envs as ./.venv inside each project (VS Code picks it up automatically)
poetry config virtualenvs.in-project true
poetry config --list                       # see all settings
```

`pipx upgrade poetry` updates Poetry later.

## Creating a project

```bash
poetry new drone-sim          # src/ layout: src/drone_sim/, tests/, pyproject.toml, README.md
# or, in an existing folder:
poetry init                   # interactive pyproject.toml creation
```

## Dependencies

```bash
poetry add mujoco numpy                 # main dependencies -> [project].dependencies
poetry add "imageio[ffmpeg]"            # with an extra
poetry add --group dev pytest ruff      # dev-only -> [dependency-groups].dev
poetry add "numpy@^2.5"                 # explicit constraint (^2.5 means >=2.5,<3.0)
poetry remove ruff --group dev
poetry show --tree                      # who depends on what
poetry show --outdated                  # what could be upgraded
poetry update numpy                     # upgrade within constraints, rewrites poetry.lock
```

**Lesson from this project:** Poetry resolves dependencies for the whole `requires-python` range, not only for the interpreter you have. With `>=3.10` the solve failed because the latest imageio needs Python ≥ 3.11 and the latest numpy needs ≥ 3.12. The fix was to state the truth: `requires-python = ">=3.12,<3.13"`. The other option is per-dependency Python markers.

## The lock file

- `poetry.lock` is the exact, reproducible solution of the constraints. **Commit it** for applications.
- `poetry lock` re-solves after you edit `pyproject.toml` by hand.
- `poetry check` validates `pyproject.toml` and that the lock file is in sync.
- `poetry install` installs exactly what is locked, plus this project itself in editable mode.
- `poetry install --without dev`, `poetry install --with rl`, and `poetry install --sync` (remove anything not in the lock).

## Using the environment

```bash
poetry run pytest                        # run one command inside the env
poetry run drone-swarm --n 12            # console scripts from [project.scripts]
eval $(poetry env activate)              # activate in the current shell (Poetry 2.x)
deactivate
poetry env info                          # interpreter path, Python version, validity
poetry env info --path                   # -> /home/gergo/projects/drone-sim/.venv
poetry env list / poetry env remove python3.12   # manage or recreate envs
```

Poetry 2 removed `poetry shell`, which is now a plugin. `poetry env activate` prints the activate command, and wrapping it in `eval $(...)` runs it.

## Console scripts (entry points)

In `pyproject.toml`:

```toml
[project.scripts]
drone-swarm = "drone_sim.cli:swarm"
```

After `poetry install`, `drone-swarm` exists in `.venv/bin/`. `poetry run drone-swarm` works without activating the env.

## Later: an optional RL group

```bash
poetry source add --priority=explicit pytorch-cpu https://download.pytorch.org/whl/cpu
poetry add --group rl --source pytorch-cpu torch
poetry add --group rl stable-baselines3
```

Then add this to `pyproject.toml` so the group is not installed by default:

```toml
[tool.poetry.group.rl]
optional = true
```

and install it with `poetry install --with rl`.

## Building and publishing (for reference)

```bash
poetry build            # dist/*.whl and dist/*.tar.gz
poetry publish          # to PyPI (needs a token: poetry config pypi-token.pypi <token>)
poetry version minor    # bump 0.1.0 -> 0.2.0
```
