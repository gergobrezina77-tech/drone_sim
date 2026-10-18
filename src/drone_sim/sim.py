"""The swarm simulation loop: mission -> swarm behaviours -> controller -> physics."""

from __future__ import annotations

import mujoco
import numpy as np

from drone_sim.controller import GeometricController
from drone_sim.disturbances import WindConfig, WindField, aero_drag
from drone_sim.drone import DroneGroup, DroneParams
from drone_sim.planner import Mission, MissionRef, demo_mission
from drone_sim.scene import Obstacle, build_scene, default_obstacles, spawn_grid
from drone_sim.swarm import SwarmBrain, SwarmConfig, pairwise


class SwarmSim:
    def __init__(
        self,
        n_drones: int = 12,
        mission: Mission | None = None,
        obstacles: list[Obstacle] | None = None,
        wind: WindConfig | None = None,
        mode: str = "formation",
        params: DroneParams | None = None,
        swarm_cfg: SwarmConfig | None = None,
        control_dt: float = 0.004,
        seed: int = 0,
    ):
        self.n = n_drones
        self.params = params or DroneParams()
        self.obstacles = default_obstacles() if obstacles is None else list(obstacles)
        spawn = spawn_grid(n_drones)
        self.model = mujoco.MjModel.from_xml_string(build_scene(n_drones, self.params, spawn, self.obstacles))
        self.data = mujoco.MjData(self.model)
        self.drones = DroneGroup(self.model, self.data, n_drones)
        self.controller = GeometricController(self.params, n_drones)
        self.brain = SwarmBrain(swarm_cfg, self.obstacles, mode)
        self.wind = WindField(wind or WindConfig(), n_drones, np.random.default_rng(seed))
        self.mission = mission or demo_mission(spawn)
        self.substeps = max(1, round(control_dt / self.model.opt.timestep))
        self.control_dt = self.substeps * self.model.opt.timestep
        self.ref: MissionRef | None = None
        self.air = np.zeros((n_drones, 3))
        mujoco.mj_forward(self.model, self.data)

    @property
    def t(self) -> float:
        return self.data.time

    def step(self) -> None:
        """Advance one control period."""
        s = self.drones.state()
        self.ref = self.mission.step(self.control_dt)
        v_cmd, yaw = self.brain.command(s, self.ref)
        self.drones.set_thrusts(self.controller.velocity_control(s, v_cmd, yaw, self.control_dt))
        self.air = self.wind.step(self.control_dt)
        for _ in range(self.substeps):
            self.drones.set_external_forces(aero_drag(self.drones.linear_velocity(), self.air, self.params.drag_coef))
            mujoco.mj_step(self.model, self.data)

    def run(self, seconds: float, callback=None) -> None:
        for _ in range(round(seconds / self.control_dt)):
            self.step()
            if callback is not None:
                callback(self)

    def min_separation(self) -> float:
        if self.n < 2:
            return np.inf
        return float(pairwise(self.drones.state().pos)[1].min())

    def formation_error(self) -> np.ndarray:
        return np.linalg.norm(self.drones.state().pos - self.ref.targets, axis=1)
