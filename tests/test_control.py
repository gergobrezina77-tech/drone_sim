import mujoco
import numpy as np

from drone_sim.controller import GeometricController
from drone_sim.drone import DroneGroup, DroneParams
from drone_sim.scene import build_scene

PARAMS = DroneParams()
DT = 0.004


def make_single(pos=(0.0, 0.0, 1.0)):
    model = mujoco.MjModel.from_xml_string(build_scene(1, PARAMS, np.array([pos])))
    data = mujoco.MjData(model)
    return model, data, DroneGroup(model, data, 1)


def fly(model, data, drones, ctrl, ref_fn, seconds):
    """Run the position controller; returns the max tilt (deg) seen."""
    max_tilt = 0.0
    for k in range(int(seconds / DT)):
        s = drones.state()
        pos_ref, yaw = ref_fn(k * DT)
        drones.set_thrusts(ctrl.position_control(s, pos_ref, yaw=yaw, dt=DT))
        drones.set_external_forces(-PARAMS.drag_coef * s.vel)
        for _ in range(2):
            mujoco.mj_step(model, data)
        max_tilt = max(max_tilt, np.degrees(np.arccos(np.clip(s.R[0, 2, 2], -1, 1))))
    return max_tilt


def test_mixer_inverts_allocation():
    ctrl = GeometricController(PARAMS, 1)
    np.testing.assert_allclose(PARAMS.allocation @ ctrl.mixer, np.eye(4), atol=1e-12)


def test_mixer_respects_motor_limits_and_keeps_thrust():
    ctrl = GeometricController(PARAMS, 3)
    thrust = np.array([4.9, 4.9, 13.0])
    tau = np.array([[0.0, 0.0, 5.0], [0.05, -0.05, 0.0], [0.0, 0.0, 0.0]])
    f = ctrl.mix(thrust, tau)
    assert np.all(f >= 0) and np.all(f <= PARAMS.max_thrust)
    # a saturating yaw demand is scaled down rather than sacrificing collective thrust
    np.testing.assert_allclose(f[0].sum(), 4.9, rtol=1e-6)


def test_open_loop_hover_is_balanced():
    model, data, drones = make_single()
    drones.set_thrusts(np.full((1, 4), PARAMS.hover_thrust))
    for _ in range(1000):
        mujoco.mj_step(model, data)
    s = drones.state()
    np.testing.assert_allclose(s.pos[0], [0, 0, 1], atol=1e-6)
    np.testing.assert_allclose(s.omega[0], 0, atol=1e-6)


def test_closed_loop_hover_from_disturbed_start():
    model, data, drones = make_single()
    data.qvel[:6] = [0.5, -0.3, 0.2, 2.0, -1.0, 0.5]  # kick it
    ctrl = GeometricController(PARAMS, 1)
    fly(model, data, drones, ctrl, lambda t: (np.array([[0.0, 0.0, 1.0]]), 0.0), 3.0)
    assert np.linalg.norm(drones.state().pos[0] - [0, 0, 1]) < 0.05


def test_waypoint_step_response_converges_without_flipping():
    model, data, drones = make_single()
    ctrl = GeometricController(PARAMS, 1)
    target = np.array([[2.0, 1.0, 2.0]])
    max_tilt = fly(model, data, drones, ctrl, lambda t: (target, 0.5), 7.0)
    s = drones.state()
    assert np.linalg.norm(s.pos[0] - target[0]) < 0.05
    assert max_tilt < 40.0
    assert abs(np.arctan2(s.R[0, 1, 0], s.R[0, 0, 0]) - 0.5) < 0.02
