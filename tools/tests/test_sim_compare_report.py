"""sim_compare_report: statistics, the noise-floor verdict, the miss classes, and one end-to-end report on two tiny
synthetic batches (no bag, no Isaac)."""
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from treemapper_tools import mission_score as ms
from treemapper_tools import sim_compare_report as rep


def sq(cx, cy, h=1.5):
    return [[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]]


# ----------------------------------------------------------------------------- pure functions

def test_stats_match_compare_runs_definition():
    s = rep.stats([3.0, 1.0, 2.0], "max")
    assert s["mean"] == pytest.approx(2.0) and s["worst"] == 3.0 and s["spread"] == 2.0
    assert rep.stats([3.0, 1.0, None], "min")["worst"] == 1.0
    assert rep.stats([], "max")["mean"] is None


def test_verdict_uses_the_larger_spread_as_noise_floor():
    a = rep.stats([100.0, 110.0], "max")          # corridor: mission time, spread 10
    b = rep.stats([200.0, 204.0], "max")          # field_first: spread 4
    d, floor, txt = rep.verdict(a, b, "max", "corridor", "field_first")
    assert d == pytest.approx(97.0) and floor == pytest.approx(10.0) and txt == "corridor better"
    b2 = rep.stats([108.0, 112.0], "max")         # diff 5 <= floor 10
    assert rep.verdict(a, b2, "max", "corridor", "field_first")[2] == "no measurable difference"
    one = rep.stats([0.9999997, 0.9999997], "min")          # zero spread: 1 % of the mean is the floor
    assert rep.verdict(one, rep.stats([0.9999999], "min"), "min", "corridor", "field_first")[2] == \
        "no measurable difference"
    hi = rep.stats([0.9, 0.95], "min")            # bad direction min: larger is better
    lo = rep.stats([0.5, 0.52], "min")
    assert rep.verdict(lo, hi, "min", "corridor", "field_first")[2] == "field_first better"
    pos, neg = rep.stats([0.69, 0.70], "absmax"), rep.stats([-0.26, -0.25], "absmax")   # a bias: closer to 0 wins
    assert neg["worst"] == -0.26 and rep.verdict(pos, neg, "absmax", "corridor", "field_first")[2] == "field_first better"
    assert rep.verdict(neg, pos, "absmax", "field_first", "corridor")[2] == "field_first better"
    assert rep.verdict(rep.stats([0.3, 0.32], "absmax"), rep.stats([-0.31, -0.33], "absmax"), "absmax", "c", "f")[2] == \
        "no measurable difference"


def test_footprint_radius_at_nominal_spray_altitude():
    cfg = {"spray_clearance": "2.5"}
    spec = {"nozzle": {"cone_half_angle_deg": 20, "offset_body": [0, 0, -0.3]}}
    r, above, half = rep.nominal_footprint(cfg, spec)
    assert above == pytest.approx(2.2) and half == 20.0
    assert r == pytest.approx(2.2 * math.tan(math.radians(20.0)))   # 0.80 m


@pytest.mark.parametrize("facts, cls", [
    ({"aimed": False, "counter": "target_height_implausible", "published": True}, "d"),
    ({"aimed": False, "counter": None, "split_or_merged": True, "published": True}, "b"),
    ({"aimed": False, "counter": None, "published": False}, "e"),
    ({"aimed": True, "split_or_merged": True, "map_err": 1.5}, "b"),
    ({"aimed": True, "map_err": 1.2, "crown_r": 1.5, "r_fp_actual": 0.8}, "a"),
    ({"aimed": True, "map_err": 0.3, "crown_r": 1.5, "r_fp_actual": 2.1, "height_err": 3.5}, "e"),
    ({"aimed": True, "map_err": 0.3, "crown_r": 1.5, "r_fp_actual": 0.8}, "e"),
])
def test_classify_precedence(facts, cls):
    assert rep.classify(facts, 0.8)[0] == cls


def test_classify_notes_name_the_reason():
    assert "height over-estimate" in rep.classify({"aimed": True, "map_err": 0.3, "crown_r": 1.5,
                                                    "r_fp_actual": 2.1, "height_err": 3.5}, 0.8)[1]
    assert rep.classify({"aimed": False, "published": False}, 0.8)[1] == "never published by TreeMapper"


# ----------------------------------------------------------------------------- synthetic batches

TRUTH = [{"id": 1, "centre": [0.0, 0.0], "outline": sq(0, 0), "top_z": 3.0},
         {"id": 2, "centre": [10.0, 0.0], "outline": sq(10, 0), "top_z": 3.0},
         {"id": 3, "centre": [20.0, 0.0], "outline": sq(20, 0), "top_z": 3.0}]


def _flat(mt, on_target, n_sprayed):
    f = {k: 0.0 for k in ms.COMPARE_METRICS}
    f.update({"mission_time_s": mt, "path_length_m": mt * 2, "on_target_fraction": on_target,
              "n_sprayed": n_sprayed, "n_missed": 3 - n_sprayed, "clearance_min_m": 2.0, "rtf": 1.0})
    return f


def make_run(d, strategy, mt, on_target):
    """Tree 1 sprayed; tree 2 aimed at entry 11, 1.5 m off its centre (30 % of the dose); tree 3's entry 12 published
    at 9.5 m (> home 0 + 7 + 2), never targeted, counter target_height_implausible."""
    os.makedirs(d)
    spec = {"rows": {"heading_rad": 0.0}, "nozzle": {"flow_lps": 0.075, "cone_half_angle_deg": 20,
                                                     "offset_body": [0, 0, -0.3]}}
    json.dump({"spec": spec, "spec_file": "/specs/tiny.json", "strategy": strategy, "git_commit": "abcdef0123",
               "git_dirty_files": 0, "seed": 5, "detector_model": "/m/det.onnx", "cfg_overrides": {"strategy": strategy}},
              open(os.path.join(d, "run.json"), "w"))
    json.dump({"frame": "world", "trees": TRUTH}, open(os.path.join(d, "truth.json"), "w"))
    json.dump({"rtf": 1.0, "sim_s": mt, "wall_s": mt}, open(os.path.join(d, "rtf.json"), "w"))
    open(os.path.join(d, "corridor_nav.cfg"), "w").write(
        "strategy = %s\nspray_clearance = 2.5\nexpected_canopy_top_max = 7.0\ntarget_height_margin = 2.0\n" % strategy)
    poses = [[t * 0.5, -5.0 + t * 0.5 * 4.0, -6.0, 15.0] for t in range(20)]          # scan along y = -6
    poses[0][3] = 0.0                                                                   # home on the ground
    poses += [[12.0 + i * 0.5, 0.0, 0.0, 5.5] for i in range(4)]                        # dose tree 1
    poses += [[15.0 + i * 0.5, 10.0, -1.5, 5.5] for i in range(4)]                      # dose entry 11
    json.dump({"phases": [[0, "IDLE"], [1, "SCAN"], [10, "PLANNING"], [11, "HOPPING"], [12, "SPRAYING"],
                          [14, "HOPPING"], [15, "SPRAYING"], [17, "RETURN"]],
               "targets": [[0, -1], [11, 10], [14.5, 11], [17, -1]], "pose": poses},
              open(os.path.join(d, "mission.json"), "w"))
    with open(os.path.join(d, "map.jsonl"), "w") as f:
        for t in (2.0, 5.0, 13.0):
            trees = [{"id": 10, "outline": sq(0, 0), "centre": [0.0, 0.0], "height": 3.0, "sprayed": t > 14},
                     {"id": 11, "outline": sq(10, -1.5), "centre": [10.0, -1.5], "height": 3.0, "sprayed": False},
                     {"id": 12, "outline": sq(20, 0), "centre": [20.0, 0.0], "height": 9.5 if t < 10 else 3.2,
                      "sprayed": False}]
            f.write(json.dumps({"t": t, "frame": "world", "trees": trees}) + "\n")
    card = {"run": {"dir": d, "strategy": strategy, "n_truth_trees": 3}, "completed": True, "all_pass": True,
            "flat": _flat(mt, on_target, 1),
            "efficiency": {"mission_time_s": mt, "time_split": {"t_start": 1.0, "groups": {"scan": 9.0}}},
            "spray": {"missed_ids": [2, 3], "events": [
                {"event": 0, "target": 1, "intended_l": 0.15, "trees_l": {"1": 0.14}},
                {"event": 1, "target": 2, "intended_l": 0.15, "trees_l": {"2": 0.045}}], "unmatched_events": 0},
            "safety": {"sprayed_tree_targeted_again": 0},
            "guards": {"counters": {"target_height_implausible": 1}, "holds": {}},
            "map": {"final": {"detection": {"recall": 1.0}, "geometry": {"per_tree": [
                {"truth_id": 1, "map_id": 10, "iou": 0.8}, {"truth_id": 2, "map_id": 11, "iou": 0.3},
                {"truth_id": 3, "map_id": 12, "iou": 0.7}]}},
                    "stability": {"truth_trees_ever_split": [], "truth_trees_ever_in_a_merged_entry": [],
                                  "max_height_drift_since_lock_m": 6.3}, "safety": {}}}
    json.dump(card, open(os.path.join(d, "mission_score.json"), "w"))


@pytest.fixture
def batches(tmp_path):
    a, b = str(tmp_path / "tiny_corridor"), str(tmp_path / "tiny_field_first")
    make_run(os.path.join(a, "run_1"), "corridor", 100.0, 0.70)
    make_run(os.path.join(a, "run_2"), "corridor", 110.0, 0.72)
    make_run(os.path.join(b, "run_1"), "field_first", 200.0, 0.71)
    make_run(os.path.join(b, "run_2"), "field_first", 204.0, 0.74)
    return [a, b]


def test_build_groups_by_spec_and_strategy(batches):
    res = rep.build(batches)
    assert sorted(res["groups"]) == [("tiny", "corridor"), ("tiny", "field_first")]
    mt = res["groups"][("tiny", "corridor")]["mission_time_s"]
    assert mt["mean"] == pytest.approx(105.0) and mt["spread"] == pytest.approx(10.0) and mt["worst"] == 110.0
    assert res["groups"][("tiny", "field_first")]["target_height_implausible"]["mean"] == 1.0
    assert res["setup"][0]["commits"] == ["abcdef0"]


def test_build_classifies_each_missed_tree(batches):
    res = rep.build(batches)
    assert len(res["misses"]) == 8                           # trees 2 and 3 in each of 4 runs
    by_tree = {}
    for m in res["misses"]:
        by_tree.setdefault(m["truth_id"], set()).add(m["class"])
    assert by_tree == {2: {"a"}, 3: {"d"}}
    m2 = next(m for m in res["misses"] if m["truth_id"] == 2)
    assert m2["map_err"] == pytest.approx(1.5) and m2["frac"] == pytest.approx(0.3)
    assert m2["nozzle_above_top"] == pytest.approx(2.2) and m2["first_offset"] == pytest.approx(6.0)
    m3 = next(m for m in res["misses"] if m["truth_id"] == 3)
    assert "target_height_implausible" in m3["note"] and "9.50" in m3["note"]


def test_patterns_see_error_toward_first_track(batches):
    p = rep.build(batches)["patterns"]["tiny"]["all"]
    assert p["far"]["n"] == 8 and p["big_err"] == 4 and p["big_err_toward_first_track"] == 4


def test_markdown_and_cli(batches, tmp_path):
    out, js = str(tmp_path / "r.md"), str(tmp_path / "r.json")
    assert rep.main(batches + ["--out", out, "--json", js]) == 0
    md = open(out).read()
    for s in ("## Setup", "## tiny", "## Missed trees", "corridor better", "no measurable difference",
              "| tiny | corridor | 2 | 0 | 0 | 2 | 0 | 4 |"):
        assert s in md, s
    doc = json.load(open(js))
    assert "tiny/corridor" in doc["groups"] and len(doc["misses"]) == 8
