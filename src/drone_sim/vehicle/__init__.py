"""Vehicle: drone parameter models (airframe, motors, limits) as pure data, plus a registry by id.

May import: core. No physics engine imports.
"""

from drone_sim.vehicle.params import GRAVITY, DroneParams
from drone_sim.vehicle.registry import DEFAULT_VEHICLE, get_vehicle, register_vehicle, vehicle_ids

__all__ = ["DEFAULT_VEHICLE", "GRAVITY", "DroneParams", "get_vehicle", "register_vehicle", "vehicle_ids"]
