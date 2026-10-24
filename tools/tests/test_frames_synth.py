"""Input contract round trip, bag-reader geometry helpers, and synthetic generator reproducibility."""
import filecmp
import math
import os

import numpy as np

from treemapper_tools.bag_frames import rot_to_quat, static_to_body_cam
from treemapper_tools.contract import read_ground_truth, read_mission
from treemapper_tools.frames import Calibration, Frame, FrameWriter, backproject, quat_to_rot, read_frames
from treemapper_tools import synth


def test_frame_file_round_trip(tmp_path):
    cal = Calibration(width=16, height=8, fx=10.0, fy=10.0, cx=8.0, cy=4.0, baseline=0.1, stride=4)
    gh, gw = cal.grid_shape
    assert (gh, gw) == (2, 4)
    disp = np.array([[1.0, np.nan, 2.5, 3.0], [0.0, 4.25, 5.0, 6.0]], np.float32)
    boxes = np.array([[1, 2, 3, 4, 0.9, 0]], np.float32)
    p = str(tmp_path / "f.bin")
    w = FrameWriter(p, cal, {"k": 1})
    w.write(Frame(1.0, 0.99, (1, 2, 3, 0, 0, 0, 1), True, "SCAN", 7, (1.5, -2.0), boxes, disp))
    w.write(Frame(2.0, 2.0, (0, 0, 0, 0, 0, 0, 1), False, "", -1, (math.nan, math.nan), np.zeros((0, 6)), None))
    w.close()
    cal2, meta, it = read_frames(p)
    fs = list(it)
    assert meta == {"k": 1} and cal2.fx == 10.0 and len(fs) == 2
    a, b = fs
    assert a.phase == "SCAN" and a.target_id == 7 and a.pose_valid and a.target_xy == (1.5, -2.0)
    np.testing.assert_allclose(a.boxes, boxes)
    expect = np.where(disp > 0, disp, np.nan)  # 0 and NaN are both "invalid"
    np.testing.assert_allclose(a.disparity, expect, atol=1 / 256.0)
    assert b.disparity is None and b.phase == "" and not b.pose_valid and math.isnan(b.target_xy[0])


def test_spawn_static_transform_gives_mount():
    # Isaac: tf_static world->camera = spawn body pose * mount; the reader must recover the mount.
    mount_R = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)
    mount_t = np.array([0.15, 0.08, 0.0])
    body0 = [0.1, -0.2, 0.06] + [0.0, -0.0294, 0.0, math.sqrt(1 - 0.0294 ** 2)]
    Rwb = quat_to_rot(body0[3:])
    Rwc = Rwb @ mount_R
    twc = np.array(body0[:3]) + Rwb @ mount_t
    T = static_to_body_cam(list(twc) + rot_to_quat(Rwc), body0)
    np.testing.assert_allclose(T[:3], mount_t, atol=1e-9)
    np.testing.assert_allclose(quat_to_rot(T[3:]), mount_R, atol=1e-9)


def test_backproject_nadir_ground():
    cal = Calibration(width=100, height=80, fx=100.0, fy=100.0, cx=50.0, cy=40.0, baseline=0.1, stride=1,
                      T_body_cam=[0, 0, 0, -0.70710678, 0.70710678, 0, 0])
    P = backproject(cal, np.array([50.0, 70.0]), np.array([40.0, 40.0]), np.array([1.0, 1.0]), (0, 0, 10, 0, 0, 0, 1))
    np.testing.assert_allclose(P[:, 2], [0.0, 0.0], atol=1e-9)  # range 10 m from 10 m altitude: ground
    np.testing.assert_allclose(P[1, :2], [0.0, -2.0], atol=1e-9)  # image right = body -y


TINY = dict(rows=1, per_row=2, n_spray=1, frame_hz=2.0, spacing=(5.0, 0.1))


def test_synth_reproducible_and_contract(tmp_path):
    a, b = str(tmp_path / "a"), str(tmp_path / "b")
    synth.generate(a, "hard", 3, TINY)
    synth.generate(b, "hard", 3, TINY)
    for f in ("frames.bin", "ground_truth.json", "mission.json", "camera_info.json"):
        assert filecmp.cmp(os.path.join(a, f), os.path.join(b, f), shallow=False), f
    gt = read_ground_truth(os.path.join(a, "ground_truth.json"))
    assert 1 <= len(gt) <= 2 and all(math.isfinite(g.height) for g in gt)
    m = read_mission(os.path.join(a, "mission.json"))
    seq = [p for _, p in m.phases]
    assert seq[0] == "IDLE" and "SCAN" in seq and "HOPPING" in seq and "SPRAYING" in seq and seq[-1] == "IDLE"
    cal, meta, it = read_frames(os.path.join(a, "frames.bin"))
    fs = list(it)
    awake = [f for f in fs if f.disparity is not None]
    assert awake and all(f.phase in ("SCAN_APPROACH", "SCAN", "PLANNING", "HOPPING") for f in awake)
    assert any(not f.pose_valid for f in fs) and any(f.pose_valid for f in fs)  # start-up hold is exercised
    assert all(f.t <= g.t for f, g in zip(fs, fs[1:]))
    # a different seed gives a different world
    c = str(tmp_path / "c")
    synth.generate(c, "hard", 4, TINY)
    assert not filecmp.cmp(os.path.join(a, "frames.bin"), os.path.join(c, "frames.bin"), shallow=False)
