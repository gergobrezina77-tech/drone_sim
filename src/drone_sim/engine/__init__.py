"""Engine: the simulation state machine (EDIT, RUNNING, PAUSED, FINISHED) and fixed-step tick pipeline
that wires physics, environment, autonomy and tasks together; compile caching, events, run reports.

May import every layer below it. Must not import api.
"""
