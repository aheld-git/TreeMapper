import math

import numpy as np
import pytest

from isaac.controller import KinematicController, State

DT = 1.0 / 60.0
G = 9.81


def _run(ctl, setpoint_fn, t_end, dt=DT):
    """Step the controller; setpoint_fn(now) -> setpoint tuple or None. Returns list of States."""
    states = [ctl.state]
    n = int(round(t_end / dt))
    for i in range(1, n + 1):
        now = i * dt
        states.append(ctl.step(dt, now, setpoint_fn(now)))
    return states


def _vel(states):
    return np.array([[s.vx, s.vy, s.vz] for s in states])


def _pos(states):
    return np.array([[s.x, s.y, s.z] for s in states])


def test_initial_state_is_zero():
    s = KinematicController().state
    assert s == State(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert State._fields == ("x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw")


def test_reaches_setpoint_with_limits():
    ctl = KinematicController()
    states = _run(ctl, lambda now: (10.0, 0.0, 0.0, 0.0, now), 12.0)
    v = _vel(states)
    p = _pos(states)
    assert np.max(np.hypot(v[:, 0], v[:, 1])) <= 6.0 + 1e-9
    a = np.linalg.norm(np.diff(v, axis=0), axis=1) / DT
    assert np.max(a) <= 2.0 + 1e-9
    assert np.linalg.norm(p[-1] - np.array([10.0, 0.0, 0.0])) < 0.05
    assert np.linalg.norm(v[-1]) < 0.05
    # no overshoot of a static setpoint
    assert np.max(p[:, 0]) <= 10.0 + 1e-6
    # it did get fast: peak speed is the accel-limited triangular profile (sqrt(10*2)=4.47)
    assert np.max(v[:, 0]) > 4.0


def test_reaches_far_setpoint_at_speed_limit():
    ctl = KinematicController()
    states = _run(ctl, lambda now: (100.0, 0.0, 0.0, 0.0, now), 10.0)
    v = _vel(states)
    assert np.max(np.hypot(v[:, 0], v[:, 1])) == pytest.approx(6.0, abs=1e-9)


def test_diagonal_move_respects_norm_limits_and_no_overshoot():
    ctl = KinematicController()
    target = np.array([12.0, -9.0, 3.0])
    states = _run(ctl, lambda now: (target[0], target[1], target[2], 0.0, now), 20.0)
    v = _vel(states)
    p = _pos(states)
    assert np.max(np.hypot(v[:, 0], v[:, 1])) <= 6.0 + 1e-9
    assert np.max(np.abs(v[:, 2])) <= 2.0 + 1e-9
    a = np.linalg.norm(np.diff(v, axis=0), axis=1) / DT
    assert np.max(a) <= 2.0 + 1e-9
    assert np.linalg.norm(p[-1] - target) < 0.05
    assert np.max(p[:, 0]) <= target[0] + 1e-3
    assert np.min(p[:, 1]) >= target[1] - 1e-3
    assert np.max(p[:, 2]) <= target[2] + 1e-3


def test_vertical_limit():
    ctl = KinematicController()
    states = _run(ctl, lambda now: (0.0, 0.0, 30.0, 0.0, now), 25.0)
    v = _vel(states)
    assert np.max(np.abs(v[:, 2])) <= 2.0 + 1e-9
    assert np.max(np.abs(v[:, 2])) == pytest.approx(2.0, abs=1e-9)
    assert np.max(np.hypot(v[:, 0], v[:, 1])) < 1e-9
    a = np.linalg.norm(np.diff(v, axis=0), axis=1) / DT
    assert np.max(a) <= 2.0 + 1e-9
    assert states[-1].z == pytest.approx(30.0, abs=0.05)
    # and downwards
    states = _run(ctl, lambda now: (0.0, 0.0, 10.0, 0.0, now), 5.0)
    assert states[-1].z < 30.0
    assert np.min(_vel(states)[:, 2]) >= -2.0 - 1e-9


def test_tilt_follows_acceleration():
    ctl = KinematicController()
    states = _run(ctl, lambda now: (100.0, 0.0, 0.0, 0.0, now), 2.0)
    assert states[-1].pitch == pytest.approx(math.atan(2.0 / G), rel=0.02)
    assert abs(states[-1].roll) < 1e-9
    assert states[-1].vx == pytest.approx(4.0, abs=1e-6)


def test_tilt_is_expressed_in_body_frame():
    # yaw 90 deg: world +x is body-right, so accelerating along +x gives roll, no pitch
    ctl = KinematicController()
    ctl.reset(yaw=math.pi / 2)
    states = _run(ctl, lambda now: (100.0, 0.0, 0.0, math.pi / 2, now), 2.0)
    assert abs(states[-1].pitch) < 1e-6
    assert states[-1].roll == pytest.approx(math.atan(2.0 / G), rel=0.02)
    # accelerating to body-left (world -x at yaw 90 deg... world +y is body-left at yaw 0)
    ctl = KinematicController()
    states = _run(ctl, lambda now: (0.0, 100.0, 0.0, 0.0, now), 2.0)
    assert abs(states[-1].pitch) < 1e-6
    assert abs(states[-1].roll) == pytest.approx(math.atan(2.0 / G), rel=0.02)


def test_tilt_lag_is_first_order():
    ctl = KinematicController(tilt_tau=0.15)
    states = _run(ctl, lambda now: (100.0, 0.0, 0.0, 0.0, now), 0.15)
    target = math.atan(2.0 / G)
    # after one time constant a first-order lag is at 1 - 1/e of the target
    assert states[-1].pitch == pytest.approx(target * (1.0 - math.exp(-1.0)), rel=0.02)


def test_hover_on_setpoint_timeout():
    ctl = KinematicController()
    # stream setpoints for 2 s, then none at all
    states = _run(ctl, lambda now: (100.0, 0.0, 0.0, 0.0, now) if now <= 2.0 else None, 10.0)
    v = _vel(states)
    p = _pos(states)
    a = np.linalg.norm(np.diff(v, axis=0), axis=1) / DT
    assert np.max(a) <= 2.0 + 1e-9
    # still accelerating until the setpoint is stale (last stamp 2.0, stale after 3.0)
    i3 = int(round(3.0 / DT))
    assert states[i3].vx == pytest.approx(6.0, abs=1e-6)
    # decelerates, then hovers
    assert np.linalg.norm(v[-1]) < 1e-9
    assert np.linalg.norm(p[-1] - p[-30]) < 1e-9
    assert abs(states[-1].pitch) < 1e-3
    # stops after v/a_max = 3 s of braking: never ran away to the setpoint
    assert p[-1, 0] < 40.0


def test_stale_setpoint_stamp_triggers_hover():
    ctl = KinematicController()
    sp = (100.0, 0.0, 0.0, 0.0, 0.0)  # same stamp every step: goes stale after 1 s
    states = _run(ctl, lambda now: sp, 8.0)
    i1 = int(round(1.0 / DT))
    assert states[i1].vx == pytest.approx(2.0, abs=1e-6)
    assert np.linalg.norm(_vel(states)[-1]) < 1e-9
    # accelerates 1 s to 2 m/s (1 m), then brakes at a_max (1 m): hovers near x = 2
    assert states[-1].x == pytest.approx(2.0, abs=0.05)


def test_no_setpoint_ever_holds_position():
    ctl = KinematicController()
    states = _run(ctl, lambda now: None, 3.0)
    assert states[-1] == states[0]


def test_setpoint_remembered_between_messages():
    ctl = KinematicController()
    ctl.step(DT, DT, (5.0, 0.0, 0.0, 0.0, DT))
    s = ctl.step(DT, 2 * DT, None)  # no new message, still fresh
    assert s.vx == pytest.approx(2.0 * 2 * DT, abs=1e-9)


def test_yaw_rate_limit_and_shortest_path():
    ctl = KinematicController()
    # from yaw 170 deg to -170 deg: the short way is +20 deg through 180
    ctl.reset(yaw=math.radians(170.0))
    target = math.radians(-170.0)
    states = _run(ctl, lambda now: (0.0, 0.0, 0.0, target, now), 1.0)
    yaws = np.array([s.yaw for s in states])
    d = np.diff(yaws)
    d = (d + math.pi) % (2 * math.pi) - math.pi
    assert np.max(np.abs(d)) / DT <= math.radians(60.0) + 1e-9
    assert np.max(np.abs(d)) / DT == pytest.approx(math.radians(60.0), rel=1e-6)
    err = (states[-1].yaw - target + math.pi) % (2 * math.pi) - math.pi
    assert abs(err) < 1e-9
    # 20 deg at 60 deg/s takes a third of a second: it never went the long way round
    assert all(abs(((s.yaw - math.radians(170.0)) + math.pi) % (2 * math.pi) - math.pi) <= math.radians(20.0) + 1e-9
               for s in states)


def test_yaw_wraps_into_pi_range():
    ctl = KinematicController()
    ctl.reset(yaw=math.radians(179.0))
    states = _run(ctl, lambda now: (0.0, 0.0, 0.0, math.radians(-179.0), now), 0.5)
    assert all(-math.pi <= s.yaw <= math.pi for s in states)


def test_yaw_held_on_timeout():
    ctl = KinematicController()
    ctl.reset(yaw=0.0)
    sp = (0.0, 0.0, 0.0, 1.0, 0.0)
    states = _run(ctl, lambda now: sp, 3.0)
    yaw_at_timeout = states[int(round(1.0 / DT))].yaw
    assert states[-1].yaw == pytest.approx(yaw_at_timeout, abs=1e-9)
