"""tasks layer: shared failure checks, the goto_point task type, sticky outcomes. Synthetic states, no physics."""

import numpy as np
import pytest

from drone_sim.core.rotations import quat_to_mat
from drone_sim.core.state import FleetState
from drone_sim.tasks import TASK_TYPES, FailureReason, TaskBoard, TaskStatus
from drone_sim.world import DroneSpec, TaskSpec, WorldSpec

ARENA = (20.0, 20.0, 10.0)
DT = 0.01


def world(*tasks, n=2) -> WorldSpec:
    drones = tuple(DroneSpec(f"d{i}", position=(float(i), 0.0, 0.07)) for i in range(n))
    return WorldSpec(arena_size=ARENA, drones=drones, tasks=tasks)


def goto(task_id, drone_id, target, **params) -> TaskSpec:
    return TaskSpec(task_id, drone_id, "goto_point", {"target": list(target), **params})


def fleet(pos, tilt_deg=None) -> FleetState:
    pos = np.asarray(pos, dtype=float)
    n = len(pos)
    tilt = np.deg2rad(np.zeros(n) if tilt_deg is None else np.asarray(tilt_deg, float))
    quat = np.stack([np.cos(tilt / 2), np.sin(tilt / 2), 0 * tilt, 0 * tilt], axis=1)  # roll about x
    return FleetState(pos, quat, quat_to_mat(quat), np.zeros((n, 3)), np.zeros((n, 3)))


def board_for(*tasks, n=2) -> TaskBoard:
    b = TaskBoard(world(*tasks, n=n))
    b.reset()
    return b


def hold(board, pos, seconds, t0=0.0, collisions=None, tilt=None):
    """Feed a constant state for `seconds`; return the time after."""
    t = t0
    for _ in range(round(seconds / DT)):
        t += DT
        board.update(fleet(pos, tilt), np.zeros(len(pos), bool) if collisions is None else np.asarray(collisions), t)
    return t


def test_goto_point_is_registered():
    assert "goto_point" in TASK_TYPES


def test_tasks_start_active_and_expose_their_targets_as_goals():
    b = board_for(goto("t1", "d1", (5.0, 5.0, 2.0)))
    assert b.status.tolist() == [TaskStatus.ACTIVE]
    target, active = b.goals()
    np.testing.assert_allclose(target[1], (5.0, 5.0, 2.0))
    assert active.tolist() == [False, True]  # d0 has no task


def test_success_needs_the_drone_inside_the_tolerance_for_the_dwell_time():
    b = board_for(goto("t1", "d0", (3.0, 0.0, 2.0), tolerance=0.3, dwell=1.0))
    t = hold(b, [(3.2, 0.0, 2.0), (1.0, 0.0, 0.07)], 0.5)
    assert b.status[0] == TaskStatus.ACTIVE
    t = hold(b, [(3.5, 0.0, 2.0), (1.0, 0.0, 0.07)], 0.2, t)  # leaves the tolerance: dwell timer restarts
    t = hold(b, [(3.1, 0.0, 2.0), (1.0, 0.0, 0.07)], 0.9, t)
    assert b.status[0] == TaskStatus.ACTIVE
    hold(b, [(3.1, 0.0, 2.0), (1.0, 0.0, 0.07)], 0.2, t)
    assert b.status[0] == TaskStatus.SUCCEEDED and b.reason[0] == FailureReason.NONE


def test_after_success_the_drone_keeps_its_goal_and_hovers_there():
    b = board_for(goto("t1", "d0", (3.0, 0.0, 2.0), dwell=0.1))
    hold(b, [(3.0, 0.0, 2.0), (1.0, 0.0, 0.07)], 0.5)
    target, active = b.goals()
    assert b.status[0] == TaskStatus.SUCCEEDED and active[0]
    np.testing.assert_allclose(target[0], (3.0, 0.0, 2.0))


@pytest.mark.parametrize(
    "pos, collisions, tilt, reason",
    [
        ([(1.0, 0.0, 1.0), (1.0, 0.0, 0.07)], [True, False], None, FailureReason.COLLISION),
        ([(1.0, 0.0, 1.0), (1.0, 0.0, 0.07)], None, [85.0, 0.0], FailureReason.CRASH),
        ([(11.0, 0.0, 1.0), (1.0, 0.0, 0.07)], None, None, FailureReason.OUT_OF_BOUNDS),
        ([(1.0, 0.0, 10.5), (1.0, 0.0, 0.07)], None, None, FailureReason.OUT_OF_BOUNDS),
    ],
)
def test_shared_failure_checks(pos, collisions, tilt, reason):
    b = board_for(goto("t1", "d0", (3.0, 0.0, 2.0)))
    hold(b, pos, DT, collisions=collisions, tilt=tilt)
    assert b.status[0] == TaskStatus.FAILED and b.reason[0] == reason
    _, active = b.goals()
    assert not active[0]


def test_timeout():
    b = board_for(goto("t1", "d0", (3.0, 0.0, 2.0), timeout=2.0))
    t = hold(b, [(0.0, 0.0, 1.0), (1.0, 0.0, 0.07)], 1.9)
    assert b.status[0] == TaskStatus.ACTIVE
    hold(b, [(0.0, 0.0, 1.0), (1.0, 0.0, 0.07)], 0.2, t)
    assert b.status[0] == TaskStatus.FAILED and b.reason[0] == FailureReason.TIMEOUT


def test_failure_wins_over_success_on_the_same_tick():
    b = board_for(goto("t1", "d0", (3.0, 0.0, 2.0), dwell=0.0))
    hold(b, [(3.0, 0.0, 2.0), (1.0, 0.0, 0.07)], DT, collisions=[True, False])
    assert b.status[0] == TaskStatus.FAILED and b.reason[0] == FailureReason.COLLISION


def test_terminal_states_are_sticky():
    b = board_for(goto("t1", "d0", (3.0, 0.0, 2.0), dwell=0.1), goto("t2", "d1", (0.0, 5.0, 2.0)))
    t = hold(b, [(3.0, 0.0, 2.0), (1.0, 0.0, 1.0)], 0.3, collisions=[False, True])
    assert b.status.tolist() == [TaskStatus.SUCCEEDED, TaskStatus.FAILED]
    hold(b, [(15.0, 0.0, 2.0), (0.0, 5.0, 2.0)], 3.0, t, collisions=[True, False], tilt=[90.0, 0.0])
    assert b.status.tolist() == [TaskStatus.SUCCEEDED, TaskStatus.FAILED]
    assert b.reason.tolist() == [FailureReason.NONE, FailureReason.COLLISION]
    assert b.all_done


def test_update_reports_which_tasks_changed_this_tick():
    b = board_for(goto("t1", "d0", (3.0, 0.0, 2.0)), goto("t2", "d1", (0.0, 5.0, 2.0)))
    changed = b.update(fleet([(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)]), np.array([False, True]), DT)
    assert changed.tolist() == [1]
    assert b.update(fleet([(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)]), np.array([False, True]), 2 * DT).tolist() == []


def test_reset_restarts_every_task():
    b = board_for(goto("t1", "d0", (3.0, 0.0, 2.0)))
    hold(b, [(1.0, 0.0, 1.0), (1.0, 0.0, 0.07)], DT, collisions=[True, False])
    b.reset()
    assert b.status.tolist() == [TaskStatus.ACTIVE] and b.reason.tolist() == [FailureReason.NONE]


@pytest.mark.parametrize(
    "tasks, message",
    [
        ((goto("t1", "d0", (3.0, 0.0, 2.0)), goto("t2", "d0", (4.0, 0.0, 2.0))), "more than one task"),
        ((goto("t1", "d0", (30.0, 0.0, 2.0)),), "outside the arena"),
        ((TaskSpec("t1", "d0", "goto_point", {}),), "target"),
        ((TaskSpec("t1", "d0", "dance", {}),), "unknown task type 'dance'"),
    ],
)
def test_invalid_task_setups_are_rejected_with_a_clear_message(tasks, message):
    with pytest.raises((ValueError, KeyError), match=message):
        TaskBoard(world(*tasks))


def test_no_tasks():
    b = board_for()
    target, active = b.goals()
    assert b.all_done and not active.any() and target.shape == (2, 3)
