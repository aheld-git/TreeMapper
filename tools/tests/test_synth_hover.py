"""D40: the spray hover (and the transit and descent leading to it) clears every canopy in the drone's footprint,
and nothing else in the generator changed."""
import copy
import hashlib
import math
import os

import numpy as np
import pytest

from treemapper_tools.frames import read_frames
from treemapper_tools import synth


def _plans(level, seed, overrides=None):
    """The legacy and the fixed plan exactly as ``generate`` builds them (same RNG, same timeline)."""
    p = dict(synth.LEVELS[level], **(overrides or {}))
    rng = np.random.default_rng([seed, list(synth.LEVELS).index(level)])
    world = synth.World(rng, p)
    counts = []
    plan_rng = copy.deepcopy(rng)
    pts0, dt = synth.plan_mission(rng, world, p, legacy=True, counts=counts)
    pts, _ = synth.plan_mission(plan_rng, world, p, legacy=False, counts=counts)
    return world, p, pts0, pts, dt


@pytest.mark.parametrize("level", ["easy", "medium", "hard"])
@pytest.mark.parametrize("seed", [100, 103, 200])
def test_no_canopy_in_footprint_above_drone_minus_clearance(level, seed):
    world, p, pts0, pts, dt = _plans(level, seed)
    P = synth.kinematics(pts, dt)[0]  # the TRUE (smoothed) flown path, as written to mission.json
    checked = 0
    hover_xy = None
    for i, q in enumerate(pts):
        if q[3] == "SPRAYING":
            hover_xy = q[6]
        # descent, hover and the climb-out above the last hover (the rest of RETURN is the flight home and landing)
        in_scope = q[3] in ("HOPPING", "SPRAYING") or (
            q[3] == "RETURN" and hover_xy is not None and math.hypot(q[0] - hover_xy[0], q[1] - hover_xy[1]) < 0.01)
        if not in_scope:
            continue
        x, y, z = P[i]
        top = world.footprint_top(x, y, synth.FOOTPRINT_R)
        assert top + synth.CLEARANCE <= z + 1e-6, (q[3], i * dt, z, top)
        checked += 1
    assert checked > 500  # the descent / hover windows really were examined


@pytest.mark.parametrize("level", ["easy", "medium", "hard"])
def test_fix_only_raises_altitude_on_the_same_timeline(level):
    world, p, pts0, pts, dt = _plans(level, 101)
    assert len(pts) == len(pts0)
    for a, b in zip(pts0, pts):
        assert a[0] == b[0] and a[1] == b[1]          # horizontal path
        assert a[3:6] == b[3:6]                         # phase, burst, target
        np.testing.assert_array_equal(a[6], b[6])       # target centre (NaN-safe)
        assert b[2] >= a[2] - 1e-9                      # never lower than the legacy plan
        if a[3] in ("IDLE", "SCAN_APPROACH", "SCAN", "PLANNING"):
            assert a[2] == b[2]                        # scan phase untouched


def test_legacy_hover_flaw_exists_and_is_fixed():
    # a world with tall trees: the legacy hover tops out below a neighbour's canopy, the fixed one clears it
    world, p, pts0, pts, dt = _plans("hard", 100)
    P0 = synth.kinematics(pts0, dt)[0]; P1 = synth.kinematics(pts, dt)[0]
    bad0 = bad1 = 0
    for i, q in enumerate(pts):
        if q[3] == "SPRAYING":
            top = world.footprint_top(P0[i, 0], P0[i, 1], synth.FOOTPRINT_R)
            bad0 += P0[i, 2] - synth.CLEARANCE < top - 1e-6
            bad1 += P1[i, 2] - synth.CLEARANCE < top - 1e-6
    assert bad1 == 0 and bad0 > 0


# ---- nothing else changed: the legacy path reproduces the pre-fix generator bit for bit -----------------------------

TALL = dict(rows=1, per_row=4, n_spray=2, frame_hz=2.0, spacing=(4.5, 0.1), top=(6.0, 9.0))
# sha256 over the decoded frames, computed with the PRE-FIX generator (git 89317b2) for hard / seed 5 / TALL.
PIN_PREFIX = "ec2b81cc8d12e6b6e976adadae7ee08962e7bae8925a2a04d42259fa65ff1d7a"
PIN_FIXED = "05a9d52b6311032d8f31821a51a46cd841f613c7bc3ee0e1acef1b588b31378b"


def _frames(d):
    return list(read_frames(os.path.join(d, "frames.bin"))[2])


def _digest(frames):
    h = hashlib.sha256()
    for f in frames:
        h.update(repr((f.t, f.stamp, tuple(f.pose), f.pose_valid, f.phase, f.target_id)).encode())
        h.update(f.boxes.tobytes())
        h.update(b"-" if f.disparity is None else np.nan_to_num(f.disparity, nan=-1).tobytes())
    return h.hexdigest()


def test_legacy_flag_is_the_prefix_generator_and_fix_only_touches_moved_frames(tmp_path):
    a, b = str(tmp_path / "legacy"), str(tmp_path / "fixed")
    synth.generate(a, "hard", 5, dict(TALL, legacy_hover=True))
    meta = synth.generate(b, "hard", 5, TALL)
    fa, fb = _frames(a), _frames(b)
    assert _digest(fa) == PIN_PREFIX      # bit-identical to the generator before D40
    assert _digest(fb) == PIN_FIXED       # the intended change, pinned
    assert meta["hover_clearance"]["moved_samples"] > 0
    assert len(fa) == len(fb) and [(f.t, f.phase, f.target_id, f.pose_valid) for f in fa] == \
        [(f.t, f.phase, f.target_id, f.pose_valid) for f in fb]
    same = differ = 0
    for x, y in zip(fa, fb):
        if _digest([x]) == _digest([y]):
            same += 1
            continue
        differ += 1
        assert x.phase in ("HOPPING", "SPRAYING", "RETURN")           # only the hop windows
        assert x.pose[:2] == y.pose[:2] and x.pose[3:] == y.pose[3:]  # same noisy x, y and attitude
        assert y.pose[2] >= x.pose[2]                                 # only higher
    assert same > 0 and differ > 0
    for n in ("ground_truth.json", "camera_info.json"):
        with open(os.path.join(a, n), "rb") as f1, open(os.path.join(b, n), "rb") as f2:
            assert f1.read() == f2.read()
