import math
import os

import numpy as np
import pytest
from isaac import frames

CAMERA_JSON = os.path.join(os.path.dirname(__file__), "..", "specs", "camera.json")


def test_rot_quat_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        R = frames.body_rotation(*rng.uniform(-1.2, 1.2, size=3))
        q = frames.rot_to_quat(R)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-12
        assert np.allclose(frames.quat_to_rot(q), R, atol=1e-12)


def test_body_rotation_conventions():
    # yaw 90 deg: body x (forward) points along world +y
    assert np.allclose(frames.body_rotation(0, 0, math.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    # positive pitch is nose down: body x tilts towards world -z
    assert (frames.body_rotation(0, 0.2, 0) @ [1, 0, 0])[2] < 0
    # positive roll is left side up: body y tilts towards world +z
    assert (frames.body_rotation(0.2, 0, 0) @ [0, 1, 0])[2] > 0


def test_split_stamp():
    assert frames.split_stamp(12.5) == (12, 500000000)
    assert frames.split_stamp(0.0) == (0, 0)
    sec, nsec = frames.split_stamp(3.9999999999)
    assert 0 <= nsec < 1000000000 and abs(sec + nsec * 1e-9 - 4.0) < 1e-6


def test_rig_from_camera_json():
    rig = frames.load_rig(CAMERA_JSON)
    assert (rig.width, rig.height) == (640, 400)
    assert rig.fx == pytest.approx(454.53265, abs=1e-4) and rig.fy == rig.fx
    assert (rig.cx, rig.cy) == (pytest.approx(346.7086, abs=1e-3), pytest.approx(209.6815, abs=1e-3))
    assert rig.baseline_m == pytest.approx(0.160, abs=1e-4)


def test_mount_is_nadir_with_image_up_forward():
    rig = frames.load_rig(CAMERA_JSON)
    t, R = rig.optical_pose_in_body("left")
    assert np.allclose(t, [0.15, 0.08, 0.0], atol=1e-5)
    assert np.allclose(R @ [0, 0, 1], [0, 0, -1], atol=1e-5)    # optical axis points down
    assert np.allclose(R @ [1, 0, 0], [0, -1, 0], atol=1e-5)    # image right = body right (-y)
    assert np.allclose(R @ [0, 1, 0], [-1, 0, 0], atol=1e-5)    # image down = body backward
    tr, Rr = rig.optical_pose_in_body("right")
    assert np.allclose(tr - t, [0.0, -0.16, 0.0], atol=1e-4)    # +B along the left optical x
    assert np.allclose(Rr, R)


def test_usd_params_and_projection():
    rig = frames.load_rig(CAMERA_JSON)
    p = rig.usd_camera_params()
    assert p["focal_mm"] / p["h_aperture_mm"] * rig.width == pytest.approx(rig.fx)
    assert p["focal_mm"] / p["v_aperture_mm"] * rig.height == pytest.approx(rig.fy)
    # principal point right and below the centre -> negative horizontal, positive vertical offset
    assert p["h_off_mm"] < 0 < p["v_off_mm"]
    # a point on the optical axis projects to (cx, cy)
    assert rig.project([0, 0, 7.0]) == pytest.approx((rig.cx, rig.cy))
    # a USD camera looks along -z: its pose is the optical pose times diag(1, -1, -1)
    _, R = rig.usd_pose_in_body("left")
    assert np.allclose(R @ [0, 0, -1], [0, 0, -1], atol=1e-5)


def test_project_world_point_hover():
    rig = frames.load_rig(CAMERA_JSON)
    R = frames.body_rotation(0, 0, 0)
    # a point straight below the left camera at 10 m lands on the principal point
    u, v = frames.project_world_point(rig, [1.0, 2.0, 12.0], R, [1.15, 2.08, 2.0])
    assert (u, v) == pytest.approx((rig.cx, rig.cy), abs=1e-2)   # the mount quaternion is nadir to ~1e-7
    # a point 1 m ahead (body +x) is up in the image (image up = forward)
    u2, v2 = frames.project_world_point(rig, [1.0, 2.0, 12.0], R, [2.15, 2.08, 2.0])
    assert v2 < v and u2 == pytest.approx(u)
    # a point to the body's right (-y) is right in the image
    u3, _ = frames.project_world_point(rig, [1.0, 2.0, 12.0], R, [1.15, 1.08, 2.0])
    assert u3 > u


def test_yaw_rotates_the_image():
    rig = frames.load_rig(CAMERA_JSON)
    # drone at the origin, 10 m up, a point 2 m along world +x. Yaw 0: forward is +x -> image up.
    pt = [2.0, 0.0, 0.0]
    _, v0 = frames.project_world_point(rig, [0, 0, 10.0], frames.body_rotation(0, 0, 0), pt)
    # yaw 90 deg: forward is +y, the point is on the body's right -> right of the principal point
    u90, v90 = frames.project_world_point(rig, [0, 0, 10.0], frames.body_rotation(0, 0, math.pi / 2), pt)
    assert v0 < rig.cy
    assert u90 > rig.cx + 20


def test_clock_skew_guard_is_nan_safe():
    dt = 1.0 / 60.0
    assert frames.clock_skew_ok(10.0, 10.0, dt)
    assert frames.clock_skew_ok(10.0 + 0.49 * dt, 10.0, dt)
    assert not frames.clock_skew_ok(10.0 + 0.51 * dt, 10.0, dt)
    assert not frames.clock_skew_ok(10.0 - 0.51 * dt, 10.0, dt)
    # a NaN graph time, our time or step must stop the sim, not pass (abs(nan) > x is False)
    assert not frames.clock_skew_ok(float("nan"), 10.0, dt)
    assert not frames.clock_skew_ok(10.0, float("nan"), dt)
    assert not frames.clock_skew_ok(10.0, 10.0, float("nan"))
    assert not frames.clock_skew_ok(float("inf"), 10.0, dt)
