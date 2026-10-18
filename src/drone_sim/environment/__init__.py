"""Environment: effects on drones (wind first; later battery, sensor noise, motor faults) as batched,
composable plug-ins that produce forces or modifiers.

May import: world, vehicle, core. Must not import physics, autonomy or tasks.
"""

from drone_sim.environment.effects import EFFECTS, Effect, EffectStack
from drone_sim.environment.wind import WindConfig, WindEffect, WindField, aero_drag

EFFECTS.register("wind", WindEffect)

__all__ = ["EFFECTS", "Effect", "EffectStack", "WindConfig", "WindEffect", "WindField", "aero_drag"]
