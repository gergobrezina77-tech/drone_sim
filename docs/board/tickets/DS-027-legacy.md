# DS-027: legacy: Gym envs on the physics backend

- **Status:** Ready
- **Layer:** legacy
- **Milestone:** Later
- **Depends on:** DS-014

## Goal

Run the `DroneSim/Hover-v0` and `DroneSim/Swarm-v0` Gymnasium envs on `physics.MujocoBackend` instead of the legacy
scene, so RL training and the interactive engine share one physics implementation.

## Scope

- Backend support for per-episode mass / inertia randomisation (an interface change: needs approval).
- Envs use the backend for state, thrust, wind forces and rendering; `scene.build_scene` / `DroneGroup` retired if
  nothing else uses them.

## Acceptance criteria

- [ ] Existing env tests pass (or are amended with a note where physics legitimately differs: no physical
  drone-obstacle contact).
- [ ] `scripts/check.sh` passes

## Notes

Deferred from DS-014: porting needs a physics interface change and changes legacy physics behaviour.
