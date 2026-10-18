"""Swarm behaviours. Every function maps the swarm's state to velocity commands (n, 3),
which the geometric controller then tracks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from drone_sim.drone import DroneState
from drone_sim.planner import MissionRef
from drone_sim.scene import Obstacle


@dataclass
class SwarmConfig:
    v_max: float = 2.5
    k_formation: float = 1.5
    separation_dist: float = 0.9
    contact_dist: float = 0.36  # rotor tips touch at 2 * (arm + rotor radius) = 0.35 m
    k_separation: float = 1.5
    k_closing: float = 0.8
    v_max_total: float = 3.5  # task velocity is capped at v_max, avoidance may add on top
    drone_radius: float = 0.18
    obstacle_influence: float = 1.0
    k_obstacle: float = 1.0
    min_altitude: float = 0.4
    # boids
    neighbor_radius: float = 2.0
    w_cohesion: float = 0.4
    w_alignment: float = 0.8
    migration_speed: float = 1.2


def pairwise(pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """diff[i, j] = pos[i] - pos[j] and distances, with the diagonal set to inf."""
    diff = pos[:, None, :] - pos[None, :, :]
    dist = np.linalg.norm(diff, axis=-1)
    np.fill_diagonal(dist, np.inf)
    return diff, dist


def saturate(v: np.ndarray, v_max: float) -> np.ndarray:
    speed = np.linalg.norm(v, axis=1, keepdims=True)
    return v * np.minimum(1.0, v_max / np.maximum(speed, 1e-9))


def separation_velocity(
    pos: np.ndarray, vel: np.ndarray, d_safe: float, d_contact: float, k: float, k_closing: float
) -> np.ndarray:
    """Short-range repulsion: zero at ``d_safe``, ``k`` m/s at ``d_contact``, stronger inside,
    plus damping of the closing speed of neighbours that are inside ``d_safe``."""
    diff, dist = pairwise(pos)
    near = dist < d_safe  # inf diagonal -> False
    gap = np.clip((d_safe - dist) / (d_safe - d_contact), 0.0, 3.0)
    unit = diff / np.where(near, np.maximum(dist, 1e-3), np.inf)[..., None]
    closing = np.maximum(-np.einsum("ijk,ijk->ij", vel[:, None, :] - vel[None, :, :], unit), 0.0)
    return ((k * gap + k_closing * closing)[..., None] * unit).sum(axis=1)


def formation_velocity(pos: np.ndarray, ref: MissionRef, k: float) -> np.ndarray:
    return ref.target_vel + k * (ref.targets - pos)


def boids_velocity(s: DroneState, cfg: SwarmConfig, goal: np.ndarray | None = None) -> np.ndarray:
    """Reynolds flocking: cohesion + alignment (+ migration towards ``goal``)."""
    _, dist = pairwise(s.pos)
    nbr = dist < cfg.neighbor_radius
    count = nbr.sum(axis=1, keepdims=True)
    has = count > 0
    mean_pos = np.where(has, (nbr[..., None] * s.pos[None]).sum(1) / np.maximum(count, 1), s.pos)
    mean_vel = np.where(has, (nbr[..., None] * s.vel[None]).sum(1) / np.maximum(count, 1), s.vel)
    v = cfg.w_cohesion * (mean_pos - s.pos) + cfg.w_alignment * mean_vel
    if goal is not None:
        to = goal - s.pos
        v += cfg.migration_speed * to / np.maximum(np.linalg.norm(to, axis=1, keepdims=True), 1.0)
    return v


def avoid_obstacles(pos: np.ndarray, v: np.ndarray, obstacles: list[Obstacle], cfg: SwarmConfig) -> np.ndarray:
    """Repel from nearby obstacles, cancel the velocity component into them and turn it
    into a sideways slide so drones flow around pillars instead of stalling in front."""
    v = v.copy()
    up = np.array([0.0, 0.0, 1.0])
    for obs in obstacles:
        d, n = obs.distance(pos)
        d = d - cfg.drone_radius
        near = d < cfg.obstacle_influence
        if not near.any():
            continue
        dn, nn, vn = d[near], n[near], v[near]
        w = np.clip(1.0 - dn / cfg.obstacle_influence, 0.0, 1.0)[:, None]
        push = cfg.k_obstacle * (1.0 / np.maximum(dn, 0.05) - 1.0 / cfg.obstacle_influence)
        into = np.minimum(np.einsum("ij,ij->i", vn, nn), 0.0)[:, None]
        tangent = np.cross(up, nn)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        tangent *= np.where(np.einsum("ij,ij->i", vn, tangent) >= 0, 1.0, -1.0)[:, None]
        v[near] = vn - w * into * nn - w * into * tangent + push[:, None] * nn
    return v


class SwarmBrain:
    """Combines the behaviours into one velocity command per drone."""

    MODES = ("formation", "boids")

    def __init__(self, cfg: SwarmConfig | None = None, obstacles: list[Obstacle] = (), mode: str = "formation"):
        self.cfg = cfg or SwarmConfig()
        self.obstacles = list(obstacles)
        self.mode = mode

    def command(self, s: DroneState, ref: MissionRef) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.cfg
        if self.mode == "formation":
            v = formation_velocity(s.pos, ref, cfg.k_formation)
        else:
            v = boids_velocity(s, cfg, goal=ref.center)
        # priority (lowest -> highest): task, obstacles, inter-drone separation
        v = saturate(avoid_obstacles(s.pos, saturate(v, cfg.v_max), self.obstacles, cfg), cfg.v_max)
        v = v + separation_velocity(
            s.pos, s.vel, cfg.separation_dist, cfg.contact_dist, cfg.k_separation, cfg.k_closing
        )
        low = s.pos[:, 2] < cfg.min_altitude
        v[low, 2] = np.maximum(v[low, 2], 2.0 * (cfg.min_altitude - s.pos[low, 2]))
        return saturate(v, cfg.v_max_total), np.full(s.n, ref.heading)
