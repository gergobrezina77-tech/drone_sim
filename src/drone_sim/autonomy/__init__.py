"""Autonomy: drone software. Turns observations and goals into motor thrusts (controllers, autopilots)
behind the FlightSoftware interface.

May import: world, vehicle, core. Must not import physics, environment or tasks.
"""

from drone_sim.autonomy.control import Gains, GeometricController
from drone_sim.autonomy.software import FLIGHT_SOFTWARE, FlightSoftware, Goals, GotoAutopilot, Observation

FLIGHT_SOFTWARE.register("goto", GotoAutopilot)

__all__ = [
    "FLIGHT_SOFTWARE",
    "FlightSoftware",
    "Gains",
    "GeometricController",
    "Goals",
    "GotoAutopilot",
    "Observation",
]
