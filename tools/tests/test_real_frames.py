"""Real-flight input path: calibration units, disparity offset, camera chain, pairing, heartbeats, and the
existing-map converter (all without the bags)."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from treemapper_tools.frames import Calibration, backproject, quat_to_rot
from treemapper_tools.real_existing import bboxes_to_heights, markers_to_trees
from treemapper_tools.real_frames import (baseline_from_P, correct_disparity, disparity_offset, heartbeat_times,
                                          mount_to_body_cam, nearest_within, off_nadir_deg, optical_axis_in_body)

# the real flight's camera_info (first message) and tf_static body -> sensor_down
P_LEFT = [477.151617, 0.0, 322.721085, 0.0, 0.0, 477.151617, 200.055622, 0.0, 0.0, 0.0, 1.0, 0.0]
P_RIGHT = [477.151617, 0.0, 317.803955, -76633.363, 0.0, 477.151617, 200.055622, 0.0, 0.0, 0.0, 1.0, 0.0]
T_MOUNT = [0.3, 0.08, -0.2], [0.0, 0.6087614290087207, 0.0, 0.7933533402912352]


def test_baseline_units():
    b, unit = baseline_from_P(P_RIGHT)
    assert unit == "mm" and b == pytest.approx(0.16061, abs=1e-5)
    b, unit = baseline_from_P([454.53, 0, 0, -454.53 * 0.16])  # REP-104 metres
    assert unit == "m" and b == pytest.approx(0.16)
    with pytest.raises(ValueError):
        baseline_from_P([454.53, 0, 0, -454.53 * 5.0])  # 5 m or 5 mm: neither plausible
    with pytest.raises(ValueError):
        baseline_from_P([0.0, 0, 0, -1.0])


def test_disparity_offset_correction():
    off = disparity_offset(P_LEFT, P_RIGHT)
    assert off == pytest.approx(4.91713, abs=1e-4)
    raw = np.array([[12.25, 4.9, np.nan, 0.0, 30.0]], np.float32)
    c = correct_disparity(raw, off)
    assert c[0, 0] == pytest.approx(7.333, abs=1e-3) and c[0, 4] == pytest.approx(25.083, abs=1e-3)
    assert np.isnan(c[0, 1]) and np.isnan(c[0, 2]) and np.isnan(c[0, 3])  # <= offset, NaN, 0 -> invalid
    # 12.25 px raw at the image centre -> 10.45 m with the correction (2.5x too short without it)
    b, _ = baseline_from_P(P_RIGHT)
    assert P_LEFT[0] * b / c[0, 0] == pytest.approx(10.45, abs=0.01)


def test_mount_chain_geometry():
    T = mount_to_body_cam(*T_MOUNT)
    # optical axis: down and forward (body FLU), 15 deg off nadir; image right = body right (-y)
    a = optical_axis_in_body(T)
    assert off_nadir_deg(T) == pytest.approx(15.0, abs=1e-6)
    assert a[0] > 0 and a[2] < 0 and abs(a[1]) < 1e-9
    R = quat_to_rot(T[3:])
    np.testing.assert_allclose(R @ [1, 0, 0], [0, -1, 0], atol=1e-9)   # image x -> body right
    assert (R @ [0, 1, 0])[0] < 0                                       # image down -> body backward
    # a level drone at 10 m sees the principal point on the ground ahead by (10 - lever z) tan 15 deg
    cal = Calibration(640, 400, P_LEFT[0], P_LEFT[5], P_LEFT[2], P_LEFT[6], 0.16061, 4, T, "world_nwu")
    z_cam = 10.0 - 0.2
    d = cal.fx * cal.baseline / (z_cam / math.cos(math.radians(15)))
    p = backproject(cal, np.array([cal.cx]), np.array([cal.cy]), np.array([d]), (0, 0, 10, 0, 0, 0, 1))[0]
    assert p[2] == pytest.approx(0.0, abs=1e-6)
    assert p[0] == pytest.approx(0.3 + z_cam * math.tan(math.radians(15)), abs=1e-6)
    assert p[1] == pytest.approx(0.08, abs=1e-6)
    # treating sensor_down as optical (no axis swap) would look sideways, not down
    assert off_nadir_deg(mount_to_body_cam(*T_MOUNT, mount_is_optical=True)) > 60


def test_nearest_pairing_and_heartbeats():
    keys = [1.0, 1.05, 1.2]
    assert nearest_within(keys, 1.04, 0.02) == 1.05
    assert nearest_within(keys, 1.1, 0.02) is None
    assert nearest_within(keys, 1.025, 0.03) == 1.0  # tie -> earlier
    assert nearest_within([], 1.0, 1.0) is None
    hb = heartbeat_times([0.0, 0.1, 1.0], 0.0, 1.0, 0.2)
    assert hb == pytest.approx([0.4, 0.6, 0.8])  # only inside the 0.1 .. 1.0 gap


def _marker(ns, typ, mid, pts, action=0, frame="world_nwu"):
    return SimpleNamespace(ns=ns, type=typ, id=mid, action=action, header=SimpleNamespace(frame_id=frame),
                           points=[SimpleNamespace(x=x, y=y, z=0.05) for x, y in pts])


def test_existing_marker_conversion():
    h = bboxes_to_heights([1.0, -4.2, -2.6, -5.9, -5.2, 2.65, 7.0, 0, 1, 0, 1, 3.5])
    assert h == {1: 2.65, 7: 3.5}
    with pytest.raises(ValueError):
        bboxes_to_heights([1.0, 2.0])
    sq = [(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)]  # closed LINE_STRIP
    ms = [_marker("", 0, 0, [], action=3), _marker("polygon", 4, 7, sq), _marker("polygon_labels", 9, 7, [(1, 1)]),
          _marker("polygon", 4, 9, [(0, 0), (1, 0), (0, 0)]), _marker("polygon", 4, 8, sq)]
    trees, frame, cnt = markers_to_trees(ms, h, fallback=[{8: 1.25}])
    assert frame == "world_nwu" and [t.id for t in trees] == [7, 9, 8]
    t7, t9, t8 = trees
    assert len(t7.outline) == 4 and t7.centre == pytest.approx((1.0, 1.0)) and t7.height == 3.5 and not t7.sprayed
    assert cnt["degenerate_outline"] == 1 and t9.height is None and cnt["no_height"] == 1
    assert t8.height == 1.25 and cnt["height_from_neighbour_message"] == 1


def test_mount_pitch_helper_matches_tf_static_form():
    from treemapper_tools.real_frames import MOUNT_PITCH_DEG, mount_pitch_to_body_cam
    np.testing.assert_allclose(mount_pitch_to_body_cam(T_MOUNT[0], 75.0), mount_to_body_cam(*T_MOUNT), atol=1e-12)
    assert off_nadir_deg(mount_pitch_to_body_cam(T_MOUNT[0], MOUNT_PITCH_DEG)) == pytest.approx(90 - MOUNT_PITCH_DEG)
    assert off_nadir_deg(mount_pitch_to_body_cam(T_MOUNT[0], 90.0)) == pytest.approx(0.0, abs=1e-6)
