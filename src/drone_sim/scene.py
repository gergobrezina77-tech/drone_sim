"""Procedural MJCF generation: N quadrotors, an arena and obstacles."""

from __future__ import annotations

import colorsys
from dataclasses import dataclass

import mujoco
import numpy as np

from drone_sim.drone import DroneParams

REST_Z = 0.07  # body-origin height when a drone sits on its legs


@dataclass(frozen=True)
class Obstacle:
    """Axis-aligned obstacle shared by the scene builder and the avoidance logic.

    pillar: ``pos`` = base centre, ``size`` = (radius, height)
    box:    ``pos`` = centre,      ``size`` = half extents
    """

    kind: str
    pos: tuple[float, float, float]
    size: tuple[float, ...]

    def closest_point(self, p: np.ndarray) -> np.ndarray:
        c = np.asarray(self.pos, dtype=float)
        if self.kind == "pillar":
            r, h = self.size
            d = p[:, :2] - c[:2]
            rho = np.linalg.norm(d, axis=1, keepdims=True)
            xy = c[:2] + d / np.maximum(rho, 1e-9) * np.minimum(rho, r)
            z = np.clip(p[:, 2:3], c[2], c[2] + h)
            return np.hstack([xy, z])
        if self.kind == "box":
            h = np.asarray(self.size)
            return np.clip(p, c - h, c + h)
        raise ValueError(f"unknown obstacle kind {self.kind!r}")

    def distance(self, p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Distance from points (n, 3) to the surface and the outward unit normal."""
        diff = p - self.closest_point(p)
        d = np.linalg.norm(diff, axis=1)
        normal = diff / np.maximum(d, 1e-9)[:, None]
        inside = d < 1e-9
        if inside.any():  # fall back to a horizontal radial direction
            radial = p[inside] - np.asarray(self.pos)
            radial[:, 2] = 0.0
            normal[inside] = radial / np.maximum(np.linalg.norm(radial, axis=1, keepdims=True), 1e-9)
        return d, normal

    def mjcf(self) -> str:
        if self.kind == "pillar":
            r, h = self.size
            x, y, z = self.pos
            return f'<geom type="cylinder" pos="{x} {y} {z + h / 2}" size="{r} {h / 2}" material="pillar"/>'
        sx, sy, sz = self.size
        return f'<geom type="box" pos="{" ".join(map(str, self.pos))}" size="{sx} {sy} {sz}" material="wall"/>'


def default_obstacles() -> list[Obstacle]:
    """Pillar field laid out along the demo mission route."""
    spots = [(3.0, 0.4), (8.5, 2.5), (7.0, 9.0), (0.0, 8.5), (-5.5, 1.5), (-2.0, -3.5)]
    return [Obstacle("pillar", (x, y, 0.0), (0.3, 5.0)) for x, y in spots]


def drone_colors(n: int) -> np.ndarray:
    return np.array([colorsys.hsv_to_rgb((0.55 + 0.85 * i / max(n, 1)) % 1.0, 0.75, 0.95) for i in range(n)])


def spawn_grid(n: int, spacing: float = 0.7, z: float = REST_Z) -> np.ndarray:
    cols = int(np.ceil(np.sqrt(n)))
    idx = np.arange(n)
    pos = np.stack([idx // cols, idx % cols, np.zeros(n)], axis=1).astype(float) * spacing
    pos[:, :2] -= pos[:, :2].mean(axis=0)
    pos[:, 2] = z
    return pos


def _drone_mjcf(i: int, params: DroneParams, pos: np.ndarray, rgb: np.ndarray) -> tuple[str, str]:
    name = f"drone{i}"
    jx, jy, jz = params.inertia
    color = " ".join(f"{c:.3f}" for c in rgb)
    visual = 'contype="0" conaffinity="0"'
    parts = [
        f'<body name="{name}" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}">',
        f'<freejoint name="{name}_root"/>',
        f'<inertial pos="0 0 0" mass="{params.mass}" diaginertia="{jx} {jy} {jz}"/>',
        f'<geom name="{name}_hull" type="box" size=".055 .04 .022" rgba="{color} 1"/>',
        f'<geom type="box" size=".035 .028 .012" pos="-.005 0 .032" rgba=".08 .08 .1 1" {visual}/>',
        f'<geom type="sphere" size=".009" pos=".058 0 .004" rgba=".2 1 .3 1" {visual}/>',
    ]
    for j, (x, y) in enumerate(params.motor_xy):
        parts += [
            f'<geom type="capsule" fromto="0 0 0 {x:.4f} {y:.4f} 0" size=".009" rgba=".15 .15 .17 1" {visual}/>',
            f'<geom type="cylinder" pos="{x:.4f} {y:.4f} .012" size=".016 .014" rgba=".6 .62 .66 1" {visual}/>',
            f'<geom type="cylinder" pos="{x:.4f} {y:.4f} .03" size="{params.rotor_radius} .002" rgba="{color} .4"/>',
            (
                f'<geom type="capsule" fromto="{0.35 * x:.4f} {0.35 * y:.4f} -.015 {0.5 * x:.4f} {0.5 * y:.4f} -.058" '
                f'size=".006" rgba=".1 .1 .1 1"/>'
            ),
            f'<site name="{name}_m{j}" pos="{x:.4f} {y:.4f} .03" size=".004"/>',
        ]
    parts.append("</body>")
    actuators = "\n".join(
        f'<motor name="{name}_m{j}" site="{name}_m{j}" ctrllimited="true" ctrlrange="0 {params.max_thrust}" '
        f'gear="0 0 1 0 0 {s * params.yaw_coef}"/>'
        for j, s in enumerate(params.spins)
    )
    return "\n".join(parts), actuators


def build_scene(
    n_drones: int,
    params: DroneParams | None = None,
    spawn: np.ndarray | None = None,
    obstacles: list[Obstacle] = (),
    arena: float = 30.0,
    timestep: float = 0.002,
) -> str:
    params = params or DroneParams()
    spawn = spawn_grid(n_drones) if spawn is None else np.asarray(spawn, dtype=float)
    colors = drone_colors(n_drones)
    bodies, actuators = (
        zip(*(_drone_mjcf(i, params, spawn[i], colors[i]) for i in range(n_drones))) if n_drones else ((), ())
    )
    return f"""
<mujoco model="drone_swarm">
  <option timestep="{timestep}" integrator="implicitfast"/>
  <visual>
    <global offwidth="1920" offheight="1080" azimuth="135" elevation="-25"/>
    <quality shadowsize="4096"/>
    <headlight ambient=".35 .35 .38" diffuse=".5 .5 .5" specular=".1 .1 .1"/>
    <map znear=".01" zfar="300" haze=".25"/>
    <rgba haze=".25 .35 .5 1"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1=".4 .6 .85" rgb2=".03 .05 .12" width="512" height="3072"/>
    <texture name="grid" type="2d" builtin="checker" rgb1=".13 .15 .18" rgb2=".17 .19 .23"
             width="512" height="512" mark="edge" markrgb=".3 .35 .42"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".12"/>
    <material name="pillar" rgba=".95 .5 .15 1" specular=".3" shininess=".5"/>
    <material name="wall" rgba=".55 .6 .7 1"/>
  </asset>
  <worldbody>
    <light directional="true" pos="0 0 20" dir=".3 .25 -1" castshadow="true" diffuse=".75 .75 .72"/>
    <geom name="floor" type="plane" size="{arena} {arena} .1" material="grid"/>
    {"".join(o.mjcf() for o in obstacles)}
    {"".join(bodies)}
  </worldbody>
  <actuator>
    {"".join(actuators)}
  </actuator>
</mujoco>
"""


def make_model(*args, **kwargs) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(build_scene(*args, **kwargs))
