import copy
import json
import math
import os

import numpy as np
import pytest

from isaac import layout

SPECS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "specs")


def _spec(**rows):
    spec = {
        "seed": 3,
        "assets_dir": "/assets/trees",
        "field": {"polygon": [[-5, -5], [60, -5], [60, 50], [-5, 50]]},
        "rows": {"count": 8, "trees_per_row": 12, "row_spacing": 6.0, "tree_spacing": 4.5,
                 "heading_rad": 0.0, "origin": [0.0, 0.0], "jitter": 0.5, "missing_fraction": 0.0},
        "species": {"orange": 0.5, "lemon": 0.5},
        "scale_range": [0.8, 1.2],
        "terrain": {"slope_rad": 0.03, "slope_dir_rad": 0.7},
        "drone": {"start": [-3.0, -3.0, 0.0], "home": [-3.0, -3.0]},
    }
    spec["rows"].update(rows)
    return spec


def test_seeded_and_reproducible():
    a = layout.plan_layout(_spec(missing_fraction=0.1))
    b = layout.plan_layout(copy.deepcopy(_spec(missing_fraction=0.1)))
    assert a == b and len(a) > 0
    other = _spec(missing_fraction=0.1)
    other["seed"] = 4
    assert layout.plan_layout(other) != a


def test_counts_and_spacing():
    spec = _spec(missing_fraction=0.0)
    trees = layout.plan_layout(spec)
    assert len(trees) == 96
    assert [t.id for t in trees] == list(range(1, 97))
    jit = spec["rows"]["jitter"]
    # row-major: 12 trees per row; along-row neighbours are 4.5 +- 2*jitter apart
    for r in range(8):
        for c in range(11):
            p, q = trees[r * 12 + c], trees[r * 12 + c + 1]
            assert abs((q.x - p.x) - 4.5) <= 2 * jit + 1e-9
            assert abs(q.y - p.y) <= 2 * jit + 1e-9
    for r in range(7):
        for c in range(12):
            p, q = trees[r * 12 + c], trees[(r + 1) * 12 + c]
            assert abs((q.y - p.y) - 6.0) <= 2 * jit + 1e-9
    for t in trees:
        assert 0.8 <= t.scale <= 1.2
        assert 0.0 <= t.yaw < 2 * math.pi
        assert t.species in ("orange", "lemon")
        assert t.z_ground == pytest.approx(
            math.tan(0.03) * (t.x * math.cos(0.7) + t.y * math.sin(0.7)))
        assert t.z_ground == pytest.approx(layout.ground_z(spec, t.x, t.y))


def test_heading_rotates_rows():
    spec = _spec(missing_fraction=0.0, jitter=0.0, heading_rad=math.pi / 2)
    trees = layout.plan_layout(spec)
    # row direction is +y: first row runs along y at x = 0
    assert trees[1].x == pytest.approx(0.0, abs=1e-9)
    assert trees[1].y == pytest.approx(4.5)
    assert trees[12].x == pytest.approx(-6.0)


def test_missing_fraction():
    n = 0
    for seed in range(10):
        spec = _spec(missing_fraction=0.08)
        spec["seed"] = seed
        n += len(layout.plan_layout(spec))
    frac = 1.0 - n / (96.0 * 10)
    assert 0.04 <= frac <= 0.12


def test_missing_keeps_slot_and_ids_dense():
    spec = _spec(missing_fraction=0.3, jitter=0.0)
    trees = layout.plan_layout(spec)
    assert [t.id for t in trees] == list(range(1, len(trees) + 1))
    assert len(trees) < 96
    # a dropped tree leaves a gap: some neighbour spacing along a row is a multiple of 4.5
    gaps = [round((b.x - a.x) / 4.5) for a, b in zip(trees, trees[1:]) if abs(b.y - a.y) < 1e-9]
    assert max(gaps) >= 2


@pytest.mark.parametrize("path,key", [
    (("seed",), "seed"),
    (("rows", "tree_spacing"), "rows.tree_spacing"),
    (("rows", "trees_per_row"), "rows.trees_per_row"),
    (("terrain",), "terrain"),
    (("field", "polygon"), "field.polygon"),
])
def test_bad_spec_names_key(tmp_path, path, key):
    spec = _spec()
    d = spec
    for k in path[:-1]:
        d = d[k]
    del d[path[-1]]
    f = tmp_path / "bad.json"
    f.write_text(json.dumps(spec))
    with pytest.raises(ValueError, match=key.replace(".", r"\.")):
        layout.load_spec(str(f))


def test_bad_value_rejected(tmp_path):
    spec = _spec(missing_fraction=1.5)
    f = tmp_path / "bad.json"
    f.write_text(json.dumps(spec))
    with pytest.raises(ValueError, match=r"rows\.missing_fraction"):
        layout.load_spec(str(f))


@pytest.mark.parametrize("name,n_rows,per_row", [
    ("3x3_smoke", 3, 3), ("5x5_tropical", 5, 5), ("rows_8x12", 8, 12)])
def test_shipped_specs(name, n_rows, per_row):
    spec = layout.load_spec(os.path.join(SPECS, name + ".json"))
    assert spec["rows"]["count"] == n_rows and spec["rows"]["trees_per_row"] == per_row
    assert spec["assets_dir"] == "/assets/trees"
    trees = layout.plan_layout(spec)
    assert 0 < len(trees) <= n_rows * per_row
    xs = np.array([t.x for t in trees])
    ys = np.array([t.y for t in trees])
    poly = np.array(spec["field"]["polygon"], dtype=float)
    # polygon is an axis-aligned box 5 m beyond the outermost (nominal) tree slots
    r = spec["rows"]
    assert poly[:, 0].min() == pytest.approx(r["origin"][0] - 5.0)
    assert poly[:, 1].min() == pytest.approx(r["origin"][1] - 5.0)
    assert poly[:, 0].max() == pytest.approx(r["origin"][0] + (per_row - 1) * r["tree_spacing"] + 5.0)
    assert poly[:, 1].max() == pytest.approx(r["origin"][1] + (n_rows - 1) * r["row_spacing"] + 5.0)
    assert xs.min() >= poly[:, 0].min() and xs.max() <= poly[:, 0].max()
    assert ys.min() >= poly[:, 1].min() and ys.max() <= poly[:, 1].max()
    # drone home lies outside the tree rows (beyond every tree incl. jitter)
    hx, hy = spec["drone"]["home"]
    assert hx < xs.min() - 1.0 and hy < ys.min() - 1.0
    assert (spec["drone"]["start"][0], spec["drone"]["start"][1]) == (hx, hy)


def test_shipped_spec_params():
    d = {n: layout.load_spec(os.path.join(SPECS, n + ".json"))
         for n in ("3x3_smoke", "5x5_tropical", "rows_8x12")}
    assert d["3x3_smoke"]["seed"] == 1 and d["3x3_smoke"]["rows"]["row_spacing"] == 6.0
    assert d["5x5_tropical"]["seed"] == 1001 and d["5x5_tropical"]["rows"]["jitter"] == 1.5
    assert d["5x5_tropical"]["rows"]["row_spacing"] == 7.0
    r = d["rows_8x12"]["rows"]
    assert (d["rows_8x12"]["seed"], r["row_spacing"], r["tree_spacing"], r["missing_fraction"]) == (7, 6.0, 4.5, 0.08)
    assert d["rows_8x12"]["terrain"]["slope_rad"] == 0.03


def test_camera_json():
    c = json.load(open(os.path.join(SPECS, "camera.json")))
    for side in ("left", "right"):
        assert c[side]["width"] == 640 and c[side]["height"] == 400
        assert len(c[side]["K"]) == 9 and len(c[side]["P"]) == 12
    assert c["left"]["P"][3] == 0.0 and c["right"]["P"][3] < -1000
    assert len(c["mount"]) == 7
    assert c["camera_frame_is_optical"] is True
