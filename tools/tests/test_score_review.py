"""Regression tests for the scorer review (2026-10-24): one test per confirmed defect.

Each case is adapted from the reviewer's adversarial scripts (/data/work/review_scorer/adv*.py)
and failed on the scorer before the fix.
"""
import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from treemapper_tools.contract import (GroundTruthList, GroundTruthTree, Mission, Snapshot, Tree,
                                       read_ground_truth, read_mission, write_ground_truth)
from treemapper_tools.score import ScoreConfig, descent_kinds, main, match_one_to_one, score


def sq(cx, cy, h=2.0):
    return [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]


def g(i, cx, cy, h=2.0, H=4.0):
    return GroundTruthTree(i, sq(cx, cy, h), (cx, cy), h, H)


def t(i, cx, cy, h=2.0, H=4.0, sp=False, outline=None, centre=None):
    return Tree(i, outline or sq(cx, cy, h), centre or (cx, cy), H, sp)


GT = [g(0, 0, 0), g(1, 6, 0), g(2, 12, 0)]
SCAN = Mission(phases=[(0.0, "SCAN")], targets=[(0.0, -1)])


def _spray_mission():
    """scan, hop to tree 10 at (0,0), spray 20..24, climb out vertically, leave at t=27."""
    phases = [(0.0, "SCAN"), (9.0, "PLANNING"), (10.0, "HOPPING"), (20.0, "SPRAYING"), (24.0, "HOPPING")]
    targets = [(0.0, -1), (10.0, 10), (24.0, 11)]
    pose = [(0.0, 20.0, 0.0, 15.0), (10.0, 20.0, 0.0, 11.0), (15.0, 0.0, 0.0, 11.0), (18.0, 0.0, 0.0, 8.0),
            (20.0, 0.0, 0.0, 7.0), (24.0, 0.0, 0.0, 7.0), (25.0, 0.05, 0.0, 9.0), (26.0, 0.1, 0.0, 10.8),
            (27.0, 3.0, 0.0, 10.8), (30.0, 6.0, 0.0, 10.8)]
    return Mission(phases, targets, pose)


# ----------------------------------------------------------------------------- 1. rate-independent stability

def _slow_shrink(rate_hz):
    snaps, h, n = [], 2.0, int(100 * rate_hz)
    f = (0.985 ** (1.0 / rate_hz)) ** 0.5           # 1.5 % area per second, whatever the rate
    for k in range(n):
        snaps.append(Snapshot(k / rate_hz, [t(10, 0, 0, h=h)]))
        h *= f
    return snaps


def test_slow_shrink_is_a_retreat_and_rate_independent():
    a = score(_slow_shrink(1.0), GT, SCAN)["stability"]
    b = score(_slow_shrink(5.0), GT, SCAN)["stability"]
    assert a["geometry_retreats"] >= 1 and a["max_lost_fraction"] > 0.7
    assert b["max_lost_fraction"] == pytest.approx(a["max_lost_fraction"], abs=0.01)
    assert a["per_id"]["10"]["max_lost_fraction"] > 0.7


def test_slow_drift_is_reported_and_windowed_jump_fires():
    snaps = [Snapshot(float(k) * 0.2, [t(10, 0.5 * k, 0, H=4.0 + 0.9 * k)]) for k in range(10)]
    st = score(snaps, GT, SCAN)["stability"]
    # 0.5 m / 0.9 m per 0.2 s step: no single step exceeds 1 m, but within 1 s it does
    assert st["centre_jumps"] >= 1 and st["height_jumps"] >= 1
    assert st["per_id"]["10"]["centre_drift_from_first_m"] == pytest.approx(4.5)
    assert st["max_height_drift_from_first_m"] == pytest.approx(8.1)


def test_drift_since_lock_reported():
    m = Mission(phases=[(0.0, "SCAN"), (5.0, "PLANNING")], targets=[(0.0, -1)])
    snaps = [Snapshot(float(k), [t(10, 0.3 * k, 0)]) for k in range(10)]   # 0.3 m/s: never a jump
    st = score(snaps, GT, m)["stability"]
    assert st["centre_jumps"] == 0
    assert st["per_id"]["10"]["centre_drift_since_lock_m"] == pytest.approx(0.3 * 4)
    assert st["per_id"]["10"]["centre_drift_from_first_m"] == pytest.approx(0.3 * 9)


# ----------------------------------------------------------------------------- 2. sprayed-tree protection

def test_sprayed_vanish_recreate_respray_is_caught():
    snaps = [Snapshot(1.0, [t(10, 0, 0, sp=True)]), Snapshot(2.0, []), Snapshot(3.0, []),
             Snapshot(4.0, [t(40, 0, 0)]), Snapshot(5.0, [t(40, 0, 0, sp=True)])]
    r = score(snaps, GT, SCAN)
    sa = r["safety"]
    assert sa["sprayed_tree_change_kinds"]["vanished"] == 1
    assert sa["new_tree_inside_sprayed"] == 1
    assert r["spray"]["truth_trees_sprayed_twice"] == 1
    assert r["spray"]["sprayed_by"]["0"] == [10, 40]


def test_sprayed_flag_cleared_is_a_violation():
    sa = score([Snapshot(1.0, [t(10, 0, 0, sp=True)]), Snapshot(2.0, [t(10, 0, 0)])], GT, SCAN)["safety"]
    assert sa["sprayed_tree_change_kinds"]["flag_cleared"] == 1 and sa["total_violations"] == 1


def test_sprayed_centre_move_with_same_outline_is_a_violation():
    sa = score([Snapshot(1.0, [t(10, 0, 0, sp=True)]),
                Snapshot(2.0, [t(10, 0, 0, sp=True, centre=(0.9, 0))])], GT, SCAN)["safety"]
    assert sa["sprayed_tree_changes"] == 1


def test_sprayed_vanish_and_reappear_changed_is_a_violation():
    sa = score([Snapshot(1.0, [t(10, 0, 0, sp=True)]), Snapshot(2.0, []),
                Snapshot(3.0, [t(10, 0, 0, h=3, sp=True)])], GT, SCAN)["safety"]
    kinds = [e["kind"] for e in sa["sprayed_tree_change_events"]]
    assert kinds == ["vanished", "changed"]


def test_sprayed_slow_change_against_record_not_previous_frame():
    snaps = [Snapshot(1.0, [t(10, 0, 0, sp=True)])] + \
            [Snapshot(1.0 + k, [t(10, 0, 0, h=2.0 + 0.0001 * k, sp=True)]) for k in range(1, 30)]
    sa = score(snaps, GT, SCAN)["safety"]
    assert sa["sprayed_tree_changes"] == 1          # each step < eps, the drift from the record is not


def test_spray_outcome_uses_centre_at_spray_time_over_the_whole_stream():
    # sprayed over tree 0, later drifts into tree 1 and the final map shows it there
    snaps = [Snapshot(1.0, [t(10, 0, 0, sp=True)]), Snapshot(2.0, [t(10, 6, 0, sp=True)])]
    sp = score(snaps, GT, SCAN)["spray"]
    assert sp["sprayed_by"] == {"0": [10]}


def test_new_tree_overlapping_sprayed_outline_is_caught():
    half = Tree(10, [(-2, -2), (0.5, -2), (0.5, 2), (-2, 2)], (-0.75, 0), 4.0, True)
    other = Tree(40, [(-0.5, -2), (2, -2), (2, 2), (-0.5, 2)], (0.75, 0), 4.0, False)  # centre outside, 40 % inside
    sa = score([Snapshot(1.0, [half]), Snapshot(2.0, [half, other])], GT, SCAN)["safety"]
    assert sa["new_tree_inside_sprayed"] == 1


# ----------------------------------------------------------------------------- 3. descent window includes the climb-out

def test_post_spray_climb_is_judged():
    m = _spray_mission()
    base = [t(10, 0, 0, sp=True), t(11, 6, 0), t(12, 12, 0)]
    s1 = Snapshot(24.5, base)
    s2 = Snapshot(25.5, [t(10, 0, 0, sp=True), t(11, 6, 0), t(12, 12, 0, h=2.3)])  # climbing, 12 grows
    s3 = Snapshot(29.0, [t(10, 0, 0, sp=True), t(11, 6, 0), t(12, 12, 0, h=2.5)])  # left the hover point
    kinds = descent_kinds([s1, s2, s3], m, ScoreConfig())
    assert kinds == ["climb", "climb", ""]
    sa = score([s1, s2, s3], GT, m)["safety"]
    assert sa["nontarget_changes_in_descent"] == 1
    assert sa["nontarget_change_events"][0]["window"] == "climb"


def test_climb_judged_in_scan_approach_and_return_with_no_target():
    m = _spray_mission()
    m = Mission(m.phases[:-1] + [(24.0, "RETURN")], [(0.0, -1), (10.0, 10), (24.0, -1)], m.pose)
    s1 = Snapshot(24.5, [t(10, 0, 0, sp=True), t(11, 6, 0)])
    s2 = Snapshot(25.5, [t(10, 0, 0, sp=True), t(11, 6, 0, h=2.3)])
    sa = score([s1, s2], GT, m)["safety"]
    assert sa["nontarget_changes_in_descent"] == 1


# ----------------------------------------------------------------------------- 4. missing inputs are n/a

def test_phases_without_targets_make_descent_na():
    m = Mission(phases=[(0.0, "SCAN"), (5.0, "PLANNING"), (6.0, "HOPPING"), (10.0, "SPRAYING")], targets=[], pose=[])
    snaps = [Snapshot(float(k), [t(10, 0, 0), t(11, 6, 0, h=2.0 + 0.1 * k), t(12, 12, 0)]) for k in range(1, 15)]
    sa = score(snaps, GT, m)["safety"]
    assert sa["nontarget_changes_in_descent"] is None and "descent_note" in sa


def test_no_ground_truth_mode():
    snaps = [Snapshot(1.0, [t(10, 0, 0)]), Snapshot(2.0, []), Snapshot(3.0, [t(10, 30, 0)])]
    r = score(snaps, None, SCAN)
    assert r["mode"] == "no ground truth"
    assert "timeliness" not in r and "spray" not in r
    assert r["stability"]["id_switches"] is None and r["stability"]["id_reuse"] == 1
    assert r["safety"]["total_violations"] == 0


def test_cli_without_gt(tmp_path):
    p = tmp_path / "m.jsonl"
    p.write_text("\n".join(json.dumps(Snapshot(float(k), [t(10, 0, 0)]).to_json()) for k in range(3)) + "\n")
    out = tmp_path / "r.json"
    main(["--map", str(p), "--out-json", str(out), "--out-md", str(tmp_path / "r.md")])
    r = json.loads(out.read_text())
    assert r["mode"] == "no ground truth" and r["safety"]["total_violations"] == 0


# ----------------------------------------------------------------------------- 5. invalid entries

def test_degenerate_outline_is_invalid_and_a_full_retreat():
    snaps = [Snapshot(1.0, [t(10, 0, 0)]), Snapshot(2.0, [t(10, 0, 0, outline=[(0, 0), (1, 0)])])]
    r = score(snaps, GT, SCAN)
    assert r["safety"]["invalid_entry_frames"] == 1
    assert r["safety"]["invalid_entry_reasons"] == {"too_few_vertices": 1}
    assert r["stability"]["max_lost_fraction"] == pytest.approx(1.0)
    assert r["final"]["detection"]["invalid_entries"] == 1 and r["final"]["detection"]["false_trees"] == 0


def test_nan_centre_is_invalid():
    snaps = [Snapshot(1.0, [t(10, 0, 0)]), Snapshot(2.0, [t(10, 0, 0, centre=(float("nan"), float("nan")))])]
    r = score(snaps, GT, SCAN)
    assert r["safety"]["invalid_entry_frames"] == 1 and r["safety"]["total_violations"] == 1
    assert r["final"]["detection"]["matched"] == 0      # unusable entry cannot match


def test_zero_area_outline_is_invalid():
    snaps = [Snapshot(1.0, [t(10, 0, 0, outline=[(0, 0), (1, 0), (2, 0)])])]
    assert score(snaps, GT, SCAN)["safety"]["invalid_entry_reasons"] == {"zero_area": 1}


# ----------------------------------------------------------------------------- 6. undefined heights in quality metrics

def test_undefined_height_is_charged_worst_case():
    good = score([Snapshot(1.0, [t(10, 0, 0, H=4.1), t(11, 6, 0, H=7.0), t(12, 12, 0, H=7.0)])], GT, SCAN)
    bad = score([Snapshot(1.0, [t(10, 0, 0, H=4.1), t(11, 6, 0, H=None), t(12, 12, 0, H=None)])], GT, SCAN)
    gb = bad["final"]["geometry"]
    assert gb["n_undefined_height"] == 2
    assert gb["height_rmse_m"] > good["final"]["geometry"]["height_rmse_m"]
    assert gb["height_max_abs_m"] == pytest.approx(ScoreConfig().undefined_height_error_m)


def test_height_jump_through_undefined_frame():
    st = score([Snapshot(1.0, [t(10, 0, 0, H=4.0)]), Snapshot(2.0, [t(10, 0, 0, H=None)]),
                Snapshot(3.5, [t(10, 0, 0, H=9.0)])], GT, SCAN)["stability"]
    assert st["height_jumps"] == 1 and st["height_jump_events"][0]["dh_m"] == pytest.approx(5.0)


# ----------------------------------------------------------------------------- 7. timeliness cannot be gamed

def test_giant_polygon_cannot_buy_negative_latency():
    cam = {"left": {"K": [100.0, 0, 50, 0, 100.0, 50, 0, 0, 1], "width": 100, "height": 100}}
    pose = [(float(k), -40.0 + 2.0 * k, 0.0, 11.0) for k in range(40)]
    m = Mission(phases=[(0.0, "SCAN")], targets=[(0.0, -1)], pose=pose)
    giant = Tree(99, [(-3, -3), (15, -3), (15, 3), (-3, 3)], (6, 0), 4.0)
    ti = score([Snapshot(float(k), [giant]) for k in range(40)], GT, m, cam)["timeliness"]
    assert len(ti["trees_never_published"]) == 2          # one entry matches at most one tree
    assert ti["negative_latency_trees"] and ti["latency_mean_s"] >= 0.0


def test_undefined_height_entry_is_not_published_for_timeliness():
    snaps = [Snapshot(1.0, [t(10, 0, 0, H=None)]), Snapshot(2.0, [t(10, 0, 0)])]
    ti = score(snaps, [g(0, 0, 0)], SCAN)["timeliness"]
    assert ti["per_tree"][0]["first_published_t"] == 2.0


# ----------------------------------------------------------------------------- 8. time-weighted per-frame means

def test_per_frame_is_time_weighted():
    snaps = [Snapshot(float(k), []) for k in range(50)] + \
            [Snapshot(50 + k * 0.02, [t(10, 0, 0), t(11, 6, 0), t(12, 12, 0)]) for k in range(500)]
    pf = score(snaps, GT, SCAN)["per_frame"]["detection"]["recall"]["mean"]
    assert pf == pytest.approx(10.0 / 60.0, abs=0.01)


def test_per_frame_window_is_first_scan_to_return():
    m = Mission(phases=[(0.0, "IDLE"), (10.0, "SCAN"), (20.0, "RETURN")], targets=[(0.0, -1)])
    snaps = [Snapshot(float(k), [t(10, 0, 0), t(11, 6, 0), t(12, 12, 0)] if 10 <= k < 20 else [])
             for k in range(30)]
    pf = score(snaps, GT, m)["per_frame"]
    assert pf["detection"]["recall"]["mean"] == pytest.approx(1.0)
    assert pf["n_samples"] == 50


# ----------------------------------------------------------------------------- 9. recycled IDs

def test_recycled_id_after_scan_is_a_new_tree():
    m = Mission(phases=[(0.0, "SCAN"), (5.0, "PLANNING"), (6.0, "HOPPING")], targets=[(0.0, -1)])
    snaps = [Snapshot(1.0, [t(10, 0, 0), t(11, 6, 0), t(50, 40, 40)]), Snapshot(3.0, [t(10, 0, 0), t(11, 6, 0)]),
             Snapshot(7.0, [t(10, 0, 0), t(11, 6, 0), t(50, 12, 0)])]
    r = score(snaps, GT, m)
    assert r["stability"]["id_reuse"] == 1
    assert r["safety"]["new_trees_after_scan"] == 1
    assert r["safety"]["new_tree_after_scan_events"][0]["kind"] == "reuse"


def test_reappearance_in_place_is_not_reuse():
    snaps = [Snapshot(1.0, [t(10, 0, 0)]), Snapshot(2.0, []), Snapshot(3.0, [t(10, 0.2, 0)])]
    st = score(snaps, None, SCAN)["stability"]
    assert st["id_reappearances"] == 1 and st["id_reuse"] == 0


# ----------------------------------------------------------------------------- 10. Hungarian ignores sub-threshold IoU

def test_hungarian_zeroes_sub_threshold_iou():
    iou = np.array([[0.20, 0.09], [0.25, 0.10]])
    assert match_one_to_one(iou, 0.10) == {0: 0, 1: 1}


# ----------------------------------------------------------------------------- 11. frame check

def test_frame_mismatch_raises():
    with pytest.raises(ValueError, match="frame"):
        score([Snapshot(1.0, [t(10, 0, 0)], frame="world_nwu")], GT, SCAN)


def test_frame_from_ground_truth_file(tmp_path):
    p = str(tmp_path / "gt.json")
    write_ground_truth(p, GT, frame="map")
    gt = read_ground_truth(p)
    assert isinstance(gt, GroundTruthList) and gt.frame == "map"
    score([Snapshot(1.0, [t(10, 0, 0)], frame="map")], gt, SCAN)
    with pytest.raises(ValueError):
        score([Snapshot(1.0, [t(10, 0, 0)], frame="world")], gt, SCAN)


def test_no_gt_mode_requires_one_frame():
    with pytest.raises(ValueError):
        score([Snapshot(1.0, [], frame="a"), Snapshot(2.0, [], frame="b")], None, SCAN)


# ----------------------------------------------------------------------------- 12. duplicate IDs

def test_duplicate_ids_are_a_violation():
    snaps = [Snapshot(float(k), [t(10, 0, 0), t(10, 6, 0), t(12, 12, 0)]) for k in range(5)]
    sa = score(snaps, GT, SCAN)["safety"]
    assert sa["duplicate_id_frames"] == 5 and sa["total_violations"] == 5


def test_duplicate_sprayed_id_change_is_caught():
    s1 = Snapshot(1.0, [t(10, 0, 0, sp=True), t(10, 6, 0, sp=True)])
    s2 = Snapshot(2.0, [t(10, 0, 0, sp=True), t(10, 6, 0, h=2.5, sp=True)])   # only the 2nd copy changes
    sa = score([s1, s2], GT, SCAN)["safety"]
    assert sa["sprayed_tree_changes"] == 1


def test_duplicate_nontarget_change_in_descent_is_caught():
    m = _spray_mission()
    s1 = Snapshot(21.0, [t(10, 0, 0), t(11, 6, 0), t(11, 12, 0)])
    s2 = Snapshot(22.0, [t(10, 0, 0), t(11, 6, 0), t(11, 12, 0, h=2.5)])
    sa = score([s1, s2], GT, m)["safety"]
    assert sa["nontarget_changes_in_descent"] == 1


# ----------------------------------------------------------------------------- 13. input hygiene

def test_read_mission_sorts(tmp_path):
    p = tmp_path / "m.json"
    p.write_text(json.dumps({"phases": [[6.0, "HOPPING"], [0.0, "SCAN"]], "targets": [[0.0, -1]], "pose": []}))
    m = read_mission(str(p))
    assert [ph for _, ph in m.phases] == ["SCAN", "HOPPING"]
    sa = score([Snapshot(1.0, [t(10, 0, 0)]), Snapshot(7.0, [t(10, 0, 0), t(11, 6, 0)])], GT, m)["safety"]
    assert sa["new_trees_after_scan"] == 1


def test_unsorted_mission_passed_to_score_directly():
    m = Mission(phases=[(6.0, "HOPPING"), (0.0, "SCAN")], targets=[(0.0, -1)])
    sa = score([Snapshot(1.0, [t(10, 0, 0)]), Snapshot(7.0, [t(10, 0, 0), t(11, 6, 0)])], GT, m)["safety"]
    assert sa["new_trees_after_scan"] == 1


def test_degenerate_gt_outline_does_not_crash():
    gt = [g(0, 0, 0), GroundTruthTree(1, [(6, 0), (7, 0)], (6, 0), 0, 4.0)]
    r = score([Snapshot(1.0, [t(10, 0, 0)])], gt, SCAN)
    assert r["gt_repaired_ids"] == [1]
    with pytest.raises(ValueError):
        score([Snapshot(1.0, [])], [GroundTruthTree(2, [], (float("nan"), 0.0), 0.0, 4.0)], SCAN)


# ----------------------------------------------------------------------------- concerns: new safety / stability metrics

def test_locked_id_vanished():
    m = Mission(phases=[(0.0, "SCAN"), (5.0, "PLANNING"), (6.0, "HOPPING")], targets=[(0.0, -1), (6.0, 11)])
    snaps = [Snapshot(1.0, [t(10, 0, 0), t(11, 6, 0), t(12, 12, 0)]), Snapshot(2.0, [t(10, 0, 0), t(11, 6, 0)]),
             Snapshot(7.0, [t(10, 0, 0), t(11, 6, 0)]), Snapshot(8.0, [t(10, 0, 0)])]
    sa = score(snaps, GT, m)["safety"]
    assert sa["locked_id_vanished"] == 1 and sa["locked_id_vanished_targets"] == 1   # 12 vanished in SCAN: fine


def test_target_height_range_during_descent():
    m = _spray_mission()
    snaps = [Snapshot(16.0, [t(10, 0, 0, H=4.0)]), Snapshot(17.0, [t(10, 0, 0, H=6.5)]),
             Snapshot(21.0, [t(10, 0, 0, H=5.0)])]
    sa = score(snaps, GT, m)["safety"]
    assert sa["target_height_range_in_descent_m"] == {"10": 2.5}
    assert sa["nontarget_changes_in_descent"] == 0


def test_implausible_heights():
    sa = score([Snapshot(1.0, [t(10, 0, 0, H=0.0), t(11, 6, 0, H=-3.0)])], GT, SCAN)["safety"]
    assert sa["implausible_height_frames"] == 1 and sa["total_violations"] == 1
    m = Mission(phases=[(0.0, "SCAN")], targets=[(0.0, -1)], pose=[(0.0, 0.0, 0.0, 6.0)])
    sa = score([Snapshot(1.0, [t(10, 0, 0, H=7.0)])], GT, m)["safety"]
    assert sa["implausible_height_frames"] == 1


def test_recall_in_view_denominator():
    cam = {"left": {"K": [100.0, 0, 50, 0, 100.0, 50, 0, 0, 1], "width": 100, "height": 100}}
    pose = [(0.0, 0.0, 0.0, 11.0)]                       # footprint radius ~4.95 m: sees tree 0 and 1 only
    m = Mission(phases=[(0.0, "SCAN")], targets=[(0.0, -1)], pose=pose)
    d = score([Snapshot(1.0, [t(10, 0, 0)])], GT, m, cam)["final"]["detection"]
    assert d["recall"] == pytest.approx(1 / 3)
    assert d["n_in_view"] == 1 and d["recall_in_view"] == pytest.approx(1.0)


def test_constant_gt_height_is_flagged_unreliable(tmp_path):
    r = score([Snapshot(1.0, [t(10, 0, 0)])], GT, SCAN)
    assert r["gt_height_reliable"] is False and "constant fallback" in r["gt_height_note"]
    gt2 = [g(0, 0, 0, H=4.0), g(1, 6, 0, H=5.0), g(2, 12, 0, H=4.5)]
    assert score([Snapshot(1.0, [t(10, 0, 0)])], gt2, SCAN)["gt_height_reliable"] is True
    p = tmp_path / "gt.json"
    p.write_text(json.dumps({"frame": "world", "height_reliable": False,
                             "trees": [{"id": 0, "outline": sq(0, 0), "centre": [0, 0], "radius": 2, "height": 4.0}]}))
    assert score([Snapshot(1.0, [])], read_ground_truth(str(p)), SCAN)["gt_height_reliable"] is False
