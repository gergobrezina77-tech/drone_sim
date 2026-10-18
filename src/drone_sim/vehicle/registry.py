"""Vehicle models by id, so worlds can name a drone type and new airframes plug in without code changes elsewhere."""

from __future__ import annotations

from drone_sim.core.registry import Registry
from drone_sim.vehicle.params import DroneParams

DEFAULT_VEHICLE = "quad_x_500g"

VEHICLES = Registry[DroneParams]("vehicle")


def register_vehicle(vehicle_id: str, params: DroneParams) -> None:
    VEHICLES.register(vehicle_id, params)


def get_vehicle(vehicle_id: str) -> DroneParams:
    return VEHICLES.get(vehicle_id)


def vehicle_ids() -> list[str]:
    return VEHICLES.names()


register_vehicle(DEFAULT_VEHICLE, DroneParams())
