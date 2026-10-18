"""WorldSpec -> MJCF for the MuJoCo backend.

Collision filtering: only drone <-> floor contacts are simulated (drone geoms contype=2 / conaffinity=1, floor
contype=1 / conaffinity=1). Obstacles are visual only. Drone-obstacle and drone-drone contact is detected
geometrically by the backend, so nothing ever bounces.

No actuators: the backend applies each drone's motor thrusts as a body wrench. Site-transmission motors cost
O(actuators x dofs) per step in MuJoCo, i.e. quadratic in the number of drones.

Start poses are not baked in: every drone body sits at the origin and the backend writes start poses into qpos on
reset, so changing them never needs a recompile.
"""

from __future__ import annotations

from drone_sim.core.geometry import Box, Shape
from drone_sim.vehicle import DroneParams, get_vehicle
from drone_sim.world import WorldSpec

_COLLIDE_FLOOR = 'contype="2" conaffinity="1"'
_VISUAL = 'contype="0" conaffinity="0"'


def drone_body_name(i: int) -> str:
    return f"drone{i}"


def _obstacle(shape: Shape) -> str:
    if isinstance(shape, Box):
        c, h = shape.center, shape.half_extents
        return f'<geom type="box" pos="{c[0]} {c[1]} {c[2]}" size="{h[0]} {h[1]} {h[2]}" rgba=".55 .6 .7 1" {_VISUAL}/>'
    x, y, z = shape.base
    return (
        f'<geom type="cylinder" pos="{x} {y} {z + shape.height / 2}" size="{shape.radius} {shape.height / 2}" '
        f'rgba=".95 .5 .15 1" {_VISUAL}/>'
    )


def _drone(i: int, p: DroneParams) -> str:
    jx, jy, jz = p.inertia
    parts = [
        f'<body name="{drone_body_name(i)}">',
        "<freejoint/>",
        f'<inertial pos="0 0 0" mass="{p.mass}" diaginertia="{jx} {jy} {jz}"/>',
        f'<geom type="box" size=".055 .04 .022" rgba=".2 .6 .95 1" {_COLLIDE_FLOOR}/>',
    ]
    for x, y in p.motor_xy:
        parts += [
            f'<geom type="capsule" fromto="0 0 0 {x:.4f} {y:.4f} 0" size=".009" rgba=".15 .15 .17 1" {_VISUAL}/>',
            (
                f'<geom type="cylinder" pos="{x:.4f} {y:.4f} .03" size="{p.rotor_radius} .002" '
                f'rgba=".2 .6 .95 .4" {_VISUAL}/>'
            ),
            (
                f'<geom type="capsule" fromto="{0.35 * x:.4f} {0.35 * y:.4f} -.015 {0.5 * x:.4f} {0.5 * y:.4f} -.058" '
                f'size=".006" rgba=".1 .1 .1 1" {_COLLIDE_FLOOR}/>'
            ),
        ]
    parts.append("</body>")
    return "\n".join(parts)


def build_mjcf(world: WorldSpec, timestep: float) -> str:
    sx, sy, _ = world.arena_size
    return f"""
<mujoco model="drone_sim">
  <option timestep="{timestep}" integrator="implicitfast"/>
  <worldbody>
    <light directional="true" pos="0 0 20" dir=".3 .25 -1"/>
    <geom name="floor" type="plane" size="{sx / 2} {sy / 2} .1" rgba=".15 .17 .2 1" contype="1" conaffinity="1"/>
    {"".join(_obstacle(o.shape) for o in world.obstacles)}
    {"".join(_drone(i, get_vehicle(d.vehicle)) for i, d in enumerate(world.drones))}
  </worldbody>
</mujoco>
"""
