"""Vehicle models by id, so worlds can name a drone type and new airframes plug in without code changes elsewhere."""

from __future__ import annotations

from drone_sim.vehicle.params import DroneParams

DEFAULT_VEHICLE = "quad_x_500g"

_VEHICLES: dict[str, DroneParams] = {}


def register_vehicle(vehicle_id: str, params: DroneParams) -> None:
    if vehicle_id in _VEHICLES:
        raise ValueError(f"vehicle {vehicle_id!r} is already registered")
    _VEHICLES[vehicle_id] = params


def get_vehicle(vehicle_id: str) -> DroneParams:
    try:
        return _VEHICLES[vehicle_id]
    except KeyError:
        raise KeyError(f"unknown vehicle {vehicle_id!r}; known: {', '.join(sorted(_VEHICLES))}") from None


def vehicle_ids() -> list[str]:
    return sorted(_VEHICLES)


register_vehicle(DEFAULT_VEHICLE, DroneParams())
