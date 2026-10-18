"""Engine: the simulation state machine (EDIT, RUNNING, PAUSED, FINISHED) and fixed-step tick pipeline
that wires physics, environment, autonomy and tasks together; compile caching, events, run reports.

May import every layer below it. Must not import api.
"""

from drone_sim.engine.events import (
    DroneCollided,
    Event,
    RunFinished,
    RunReport,
    TaskFailed,
    TaskOutcome,
    TaskSucceeded,
)
from drone_sim.engine.simulation import IllegalTransition, SimState, Simulation

__all__ = [
    "DroneCollided",
    "Event",
    "IllegalTransition",
    "RunFinished",
    "RunReport",
    "SimState",
    "Simulation",
    "TaskFailed",
    "TaskOutcome",
    "TaskSucceeded",
]
