"""Hand-made cases for every scorer metric family, including each safety metric firing."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from treemapper_tools.contract import (GroundTruthTree, Mission, Snapshot, Tree, nwu_to_world,
                                       read_snapshots, write_snapshots)
from treemapper_tools.score import ScoreConfig, descent_mask, score


def sq(cx, cy, h=2.0):
    """Axis-aligned square of half-size h centred at (cx, cy)."""
    return [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]


def gtree(i, cx, cy, h=2.0, height=4.0):
    return GroundTruthTree(i, sq(cx, cy, h), (cx, cy), h, height)


def tree(i, cx, cy, h=2.0, height=4.0, sprayed=False, outline=None):
    return Tree(i, outline or sq(cx, cy, h), (cx, cy), height, sprayed)


GT = [gtree(0, 0, 0), gtree(1, 6, 0), gtree(2, 12, 0)]


def perfect(t=1.0, **kw):
    return Snapshot(t, [tree(10, 0, 0), tree(11, 6, 0), tree(12, 12, 0)], **kw)


def scan_mission(t_end=100.0):
    return Mission(phases=[(0.0, "SCAN")], targets=[(0.0, -1)])


# ----------------------------------------------------------------------------- detection / geometry

def test_perfect_map_scores_perfectly():
    r = score([perfect(1.0), perfect(2.0)], GT, scan_mission())
    d, g = r["final"]["detection"], r["final"]["geometry"]
    assert (d["recall"], d["precision"], d["missed"], d["false_trees"]) == (1.0, 1.0, 0, 0)
    assert d["merged_entries"] == d["split_trees"] == 0
    assert g["mean_iou"] == pytest.approx(1.0)
    assert g["centre_rmse_m"] == pytest.approx(0.0)
    assert g["height_rmse_m"] == pytest.approx(0.0)
    assert g["spill"] == pytest.approx(0.0)
    assert r["safety"]["total_violations"] == 0
    assert r["stability"]["id_switches"] == 0


def test_missed_tree_unseen():
    s = Snapshot(1.0, [tree(10, 0, 0), tree(11, 6, 0)])
    d = score([s], GT)["final"]["detection"]
    assert d["missed"] == 1 and d["missed_unseen"] == 1 and d["missed_ids"] == [2]
    assert d["recall"] == pytest.approx(2 / 3)


def test_wrong_merge_of_neighbours():
    merged = Tree(10, [(-2, -2), (8, -2), (8, 2), (-2, 2)], (3, 0), 4.0)
    s = Snapshot(1.0, [merged, tree(12, 12, 0)])
    d = score([s], GT)["final"]["detection"]
    assert d["merged_entries"] == 1 and d["wrong_merges"] == 1 and d["trees_in_merged_entries"] == 2
    assert d["missed"] == 1 and d["missed_absorbed"] == 1


def test_split_tree_fragment_and_duplicate():
    frag = tree(20, 1.0, 1.0, h=0.5)          # small piece of tree 0 -> fragment
    dup = tree(21, 6.1, 0.0)                  # near-copy of tree 1 -> duplicate
    s = Snapshot(1.0, perfect().trees + [frag, dup])
    d = score([s], GT)["final"]["detection"]
    assert d["split_trees"] == 2 and d["fragments"] == 1 and d["duplicates"] == 1
    assert d["false_trees"] == 0


def test_false_tree():
    s = Snapshot(1.0, perfect().trees + [tree(30, 50, 50)])
    d = score([s], GT)["final"]["detection"]
    assert d["false_trees"] == 1 and d["false_tree_ids"] == [30]
    assert d["precision"] == pytest.approx(0.75)


def test_geometry_errors_and_spill():
    big = tree(10, 0.5, 0.0, h=2.0, height=5.0)      # shifted 0.5 m, 1 m too tall
    wide = Tree(11, [(4, -2), (9, -2), (9, 2), (4, 2)], (6.5, 0), 4.0)  # 1 m into the 7..10 gap
    s = Snapshot(1.0, [big, wide, tree(12, 12, 0)])
    g = score([s], GT)["final"]["geometry"]
    per = {p["truth_id"]: p for p in g["per_tree"]}
    assert per[0]["centre_err_m"] == pytest.approx(0.5)
    assert per[0]["height_err_m"] == pytest.approx(1.0)
    assert per[0]["iou"] == pytest.approx(14 / 18, abs=1e-3)
    # outside-all-canopies area: big 2 m^2 (x 2..2.5) ... plus wide 4 m^2 (x 8..9); total area 16+20+16
    assert g["spill"] == pytest.approx((2.0 + 4.0) / 52.0, abs=1e-6)


def test_neighbour_spill():
    into = Tree(10, [(-2, -2), (5, -2), (5, 2), (-2, 2)], (1.5, 0), 4.0)  # reaches 1 m into tree 1
    s = Snapshot(1.0, [into, tree(11, 6, 0), tree(12, 12, 0)])
    g = score([s], GT)["final"]["geometry"]
    per = {p["truth_id"]: p for p in g["per_tree"]}
    assert per[0]["neighbour_spill"] == pytest.approx(4.0 / 28.0, abs=1e-3)


# ----------------------------------------------------------------------------- stability

def test_id_switch():
    a = perfect(1.0)
    b = Snapshot(2.0, [tree(99, 0, 0), tree(11, 6, 0), tree(12, 12, 0)])
    st = score([a, b], GT)["stability"]
    assert st["id_switches"] == 1 and st["id_switch_events"][0]["truth_id"] == 0


def test_id_reuse_and_meaning_change():
    s1 = Snapshot(1.0, [tree(10, 0, 0)])
    s2 = Snapshot(2.0, [])
    s3 = Snapshot(3.0, [tree(10, 12, 0)])     # same ID, different tree
    st = score([s1, s2, s3], GT)["stability"]
    assert st["id_reappearances"] == 1 and st["id_reuse"] == 1
    assert st["id_meaning_changes"] == 1


def test_shrink_jump_and_height_jump():
    s1 = Snapshot(1.0, [tree(10, 0, 0, h=2.0, height=4.0)])
    s2 = Snapshot(2.0, [tree(10, 0, 0, h=1.5, height=4.0)])       # shrinks
    s3 = Snapshot(3.0, [tree(10, 1.6, 0, h=1.5, height=6.0)])     # jumps 1.6 m, height +2
    st = score([s1, s2, s3], GT)["stability"]
    # retreat is measured against the ID's historic outline union and counted once per onset:
    # s2 loses 44 % of it, s3 stays below it (still retreating) -> one onset
    assert st["geometry_retreats"] == 1
    # s3 (x 0.1..3.1) keeps 1.9 x 3 m^2 of the 16 m^2 union
    assert st["max_lost_fraction"] == pytest.approx(1 - 5.7 / 16.0, abs=1e-3)
    assert st["centre_jumps"] == 1 and st["height_jumps"] == 1


def test_growth_is_not_a_retreat():
    s1 = Snapshot(1.0, [tree(10, 0, 0, h=1.0)])
    s2 = Snapshot(2.0, [tree(10, 0, 0, h=2.0)])
    assert score([s1, s2], GT)["stability"]["geometry_retreats"] == 0


# ----------------------------------------------------------------------------- safety

def test_undefined_height_fires():
    bad = Snapshot(2.0, [tree(10, 0, 0, height=None), tree(11, 6, 0), tree(12, 12, 0)])
    sa = score([perfect(1.0), bad], GT, scan_mission())["safety"]
    assert sa["undefined_height_frames"] == 1 and sa["total_violations"] == 1


def test_nan_height_round_trips_as_undefined(tmp_path):
    p = str(tmp_path / "m.jsonl")
    write_snapshots(p, [Snapshot(1.0, [tree(10, 0, 0, height=float("nan"))])])
    s = read_snapshots(p)
    assert s[0].trees[0].height is None
    assert score(s, GT)["safety"]["undefined_height_frames"] == 1


def test_new_tree_after_scan_fires_only_when_locked():
    m = Mission(phases=[(0.0, "SCAN"), (5.0, "PLANNING"), (6.0, "HOPPING")], targets=[(0.0, -1)])
    s1 = Snapshot(1.0, [tree(10, 0, 0)])
    s2 = Snapshot(3.0, [tree(10, 0, 0), tree(11, 6, 0)])               # new during SCAN: fine
    s3 = Snapshot(7.0, [tree(10, 0, 0), tree(11, 6, 0), tree(12, 12, 0)])  # new during HOPPING
    sa = score([s1, s2, s3], GT, m)["safety"]
    assert sa["new_trees_after_scan"] == 1
    assert sa["new_tree_after_scan_events"][0]["id"] == 12


def _descent_mission():
    # scan, then hop toward tree 10 at (0, 0); spray starts at t=20 with the drone over it.
    phases = [(0.0, "SCAN"), (9.0, "PLANNING"), (10.0, "HOPPING"), (20.0, "SPRAYING"), (24.0, "HOPPING")]
    targets = [(0.0, -1), (10.0, 10)]
    pose = [(0.0, 20.0, 0.0, 15.0), (10.0, 20.0, 0.0, 11.0), (15.0, 0.0, 0.0, 11.0),
            (18.0, 0.0, 0.0, 8.0), (20.0, 0.0, 0.0, 7.0), (24.0, 0.0, 0.0, 9.0)]
    return Mission(phases, targets, pose)


def test_nontarget_change_in_descent_fires():
    m = _descent_mission()
    base = [tree(10, 0, 0), tree(11, 6, 0), tree(12, 12, 0)]
    s1 = Snapshot(16.0, base)
    s2 = Snapshot(17.0, [tree(10, 0, 0, h=2.2), tree(11, 6, 0, h=2.3), tree(12, 12, 0)])  # target + 11 grow
    sa = score([s1, s2], GT, m)["safety"]
    assert sa["nontarget_changes_in_descent"] == 1
    assert sa["nontarget_change_events"][0]["id"] == 11


def test_target_change_and_transit_growth_are_allowed():
    m = _descent_mission()
    s1 = Snapshot(11.0, [tree(10, 0, 0), tree(11, 6, 0), tree(12, 12, 0)])
    s2 = Snapshot(12.0, [tree(10, 0, 0), tree(11, 6, 0, h=2.3), tree(12, 12, 0)])   # transit, far from hover
    s3 = Snapshot(16.0, [tree(10, 0, 0), tree(11, 6, 0, h=2.3), tree(12, 12, 0)])
    s4 = Snapshot(17.0, [tree(10, 0, 0, h=2.4), tree(11, 6, 0, h=2.3), tree(12, 12, 0)])  # only target grows
    sa = score([s1, s2, s3, s4], GT, m)["safety"]
    assert sa["nontarget_changes_in_descent"] == 0
    mask = descent_mask([s1, s2, s3, s4], m, ScoreConfig())
    assert mask == [False, False, True, True]


def test_nontarget_vanish_and_appear_in_descent_fire():
    m = _descent_mission()
    s1 = Snapshot(21.0, [tree(10, 0, 0), tree(11, 6, 0)])
    s2 = Snapshot(22.0, [tree(10, 0, 0), tree(12, 12, 0)])    # 11 vanished, 12 appeared (SPRAYING)
    sa = score([s1, s2], GT, m)["safety"]
    kinds = sorted(e["kind"] for e in sa["nontarget_change_events"])
    assert kinds == ["appeared", "vanished"]


def test_sprayed_tree_change_fires():
    s1 = Snapshot(1.0, [tree(10, 0, 0, sprayed=True)])
    s2 = Snapshot(2.0, [tree(10, 0, 0, h=2.5, sprayed=True)])
    sa = score([s1, s2], GT)["safety"]
    assert sa["sprayed_tree_changes"] == 1


def test_spray_flag_flip_alone_is_not_a_change():
    s1 = Snapshot(1.0, [tree(10, 0, 0)])
    s2 = Snapshot(2.0, [tree(10, 0, 0, sprayed=True)])
    s3 = Snapshot(3.0, [tree(10, 0, 0, sprayed=True)])
    assert score([s1, s2, s3], GT)["safety"]["sprayed_tree_changes"] == 0


def test_new_tree_inside_sprayed_fires():
    s1 = Snapshot(1.0, [tree(10, 0, 0, sprayed=True)])
    s2 = Snapshot(2.0, [tree(10, 0, 0, sprayed=True), tree(40, 0.5, 0.5, h=0.5)])
    sa = score([s1, s2], GT)["safety"]
    assert sa["new_tree_inside_sprayed"] == 1


def test_no_mission_info_gives_na_for_phase_metrics():
    sa = score([perfect()], GT)["safety"]
    assert sa["new_trees_after_scan"] is None and sa["nontarget_changes_in_descent"] is None


# ----------------------------------------------------------------------------- spray, timeliness, cost, contract

def test_spray_outcome_uses_entry_centre():
    merged = Tree(10, [(-2, -2), (8, -2), (8, 2), (-2, 2)], (3, 0), 4.0, True)  # centre in the gap
    s = Snapshot(1.0, [merged, tree(12, 12, 0, sprayed=True), tree(13, 12.3, 0, h=1, sprayed=True)])
    sp = score([s], GT)["final"]["spray"]
    assert sp["truth_trees_sprayed"] == 1 and sp["truth_trees_sprayed_twice"] == 1
    assert sp["orphan_sprayed_entries"] == 1


def test_timeliness_latency():
    cam = {"left": {"K": [100.0, 0, 50, 0, 100.0, 50, 0, 0, 1], "width": 100, "height": 100}}
    # half-diagonal per metre = hypot(0.5, 0.5) = 0.707; range = 11 - 4 = 7 -> 4.95 m
    pose = [(float(t), -20.0 + 2.0 * t, 0.0, 11.0) for t in range(0, 20)]
    m = Mission(phases=[(0.0, "SCAN")], targets=[(0.0, -1)], pose=pose)
    snaps = [Snapshot(float(t), [tree(10, 0, 0)] if t >= 10 else []) for t in range(0, 20)]
    ti = score(snaps, [gtree(0, 0, 0)], m, cam)["timeliness"]
    # x(t) = -20 + 2t is within 4.95 m of 0 first at t = 8 (x = -4)
    assert ti["per_tree"][0]["first_in_view_t"] == 8.0
    assert ti["per_tree"][0]["latency_s"] == pytest.approx(2.0)


def test_cost_from_stage_ms():
    snaps = [Snapshot(float(t), [], stage_ms={"fuse": float(t)}) for t in range(1, 11)]
    co = score(snaps, GT)["cost"]
    assert co["available"] and co["stages"]["fuse"]["max_ms"] == 10.0


def test_nwu_conversion():
    assert nwu_to_world(1.0, 2.0) == (-2.0, 1.0)


def test_switches_split_by_lock_and_vanished_target_flagged():
    m = Mission(phases=[(0.0, "SCAN"), (5.0, "PLANNING"), (6.0, "HOPPING")], targets=[(0.0, -1), (6.0, 11)])
    s1 = Snapshot(1.0, [tree(10, 0, 0), tree(11, 6, 0)])
    s2 = Snapshot(2.0, [tree(20, 0, 0), tree(11, 6, 0)])       # switch before lock
    s3 = Snapshot(7.0, [tree(30, 0, 0), tree(11, 6, 0)])       # switch after lock
    s4 = Snapshot(8.0, [tree(30, 0, 0)])                       # target 11 vanishes
    st = score([s1, s2, s3, s4], GT, m)["stability"]
    assert st["id_switches"] == 2 and st["id_switches_after_lock"] == 1
    assert st["ids_vanished_while_held_by_navigation"] == [11]


def test_empty_stream_does_not_crash():
    r = score([], GT)
    assert r["n_snapshots"] == 0 and r["safety"]["total_violations"] == 0
