import json
import math

import numpy as np
import pytest

from isaac.nozzle_model import Deposition, NozzleGate, cone_directions


def test_cone_within_half_angle():
    half = math.radians(20.0)
    d = cone_directions(5000, half, seed=1)
    assert d.shape == (5000, 3)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    ang = np.arccos(np.clip(-d[:, 2], -1.0, 1.0))
    assert np.max(ang) <= half + 1e-9
    assert np.all(d[:, 2] < 0.0)
    mean = d.mean(axis=0)
    assert abs(mean[0]) < 0.01 and abs(mean[1]) < 0.01
    # E[cos theta] for a uniform solid-angle cone is (1 + cos(half)) / 2
    assert mean[2] == pytest.approx(-(1.0 + math.cos(half)) / 2.0, abs=0.002)


def test_cone_uniform_in_solid_angle():
    half = math.radians(30.0)
    d = cone_directions(20000, half, seed=3)
    cos_t = -d[:, 2]
    # cos(theta) uniform on [cos(half), 1]: fraction in the lower half of that interval is 1/2
    mid = (1.0 + math.cos(half)) / 2.0
    assert np.mean(cos_t < mid) == pytest.approx(0.5, abs=0.02)
    # azimuth uniform: the four quadrants are equally filled
    phi = np.arctan2(d[:, 1], d[:, 0])
    counts = np.histogram(phi, bins=4, range=(-math.pi, math.pi))[0]
    assert np.all(np.abs(counts / 20000.0 - 0.25) < 0.02)


def test_cone_is_seeded():
    a = cone_directions(16, 0.3, seed=7)
    b = cone_directions(16, 0.3, seed=7)
    c = cone_directions(16, 0.3, seed=8)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)
    assert np.array_equal(cone_directions(16, 0.3), cone_directions(16, 0.3, seed=0))


def test_cone_zero_half_angle_is_straight_down():
    d = cone_directions(4, 0.0)
    assert np.allclose(d, [[0.0, 0.0, -1.0]] * 4)


def test_deposition_conserves_volume():
    dep = Deposition(flow_lps=0.075)
    hits = [3] * 32 + [None] * 32
    for _ in range(240):
        dep.add(hits, 1.0 / 60.0)
    out = dep.to_json()
    assert out["emitted"] == pytest.approx(0.3, abs=1e-9)
    assert out["trees"]["3"] + out["ground"] == pytest.approx(out["emitted"], abs=1e-9)
    assert out["trees"]["3"] == pytest.approx(0.15, abs=1e-9)
    assert out["ground"] == pytest.approx(0.15, abs=1e-9)
    assert out["open_time"] == pytest.approx(4.0, abs=1e-9)
    assert dep.open_time == pytest.approx(4.0, abs=1e-9)


def test_deposition_multiple_trees_and_json_keys():
    dep = Deposition(flow_lps=1.0)
    dep.add([1, 2, 2, None], 0.5)
    out = dep.to_json()
    assert out["trees"] == {"1": pytest.approx(0.125), "2": pytest.approx(0.25)}
    assert out["ground"] == pytest.approx(0.125)
    assert out["emitted"] == pytest.approx(0.5)
    assert set(out) == {"trees", "ground", "emitted", "open_time"}
    json.dumps(out)  # must be serialisable as is


def test_deposition_accepts_numpy_ids_and_empty():
    dep = Deposition(flow_lps=1.0)
    dep.add([np.int64(5), np.int32(5)], 1.0)
    dep.add([], 1.0)
    out = dep.to_json()
    json.dumps(out)
    assert out["trees"]["5"] == pytest.approx(1.0)
    assert out["emitted"] == pytest.approx(2.0)
    assert out["trees"]["5"] + out["ground"] == pytest.approx(out["emitted"])


def test_deposition_starts_empty():
    out = Deposition(0.1).to_json()
    assert out == {"trees": {}, "ground": 0.0, "emitted": 0.0, "open_time": 0.0}


def test_gate_closes_after_timeout():
    g = NozzleGate()
    g.on_command(True, 0.0)
    assert g.is_open(0.0)
    assert g.is_open(0.5)
    assert not g.is_open(0.51)


def test_gate_closed_before_any_command_and_follows_commands():
    g = NozzleGate(timeout=0.5)
    assert not g.is_open(0.0)
    g.on_command(True, 1.0)
    assert g.is_open(1.2)
    g.on_command(False, 1.3)
    assert not g.is_open(1.31)
    g.on_command(True, 2.0)
    assert g.is_open(2.4)
    assert not g.is_open(2.6)
