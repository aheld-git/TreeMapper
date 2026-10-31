"""Mission scorer: pure functions (no bag) plus one end-to-end run on a tiny synthetic run directory."""
import glob
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from treemapper_tools import mission_score as ms
from treemapper_tools.contract import Snapshot, Tree


def sq(cx, cy, h=2.0):
    return [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]


def truth_tree(i, cx, cy, top=6.0, h=2.0):
    return {"id": i, "species": "orange", "centre": [cx, cy], "outline": [list(p) for p in sq(cx, cy, h)],
            "radius": h, "top_z": top, "ground_z": 0.0}


# ----------------------------------------------------------------------------- events and deposits

def test_dose_events_pairs_open_close():
    spray = [(0.0, False), (1.0, True), (1.05, True), (1.1, False), (2.0, False), (3.0, True), (3.05, False)]
    assert ms.dose_events(spray) == [(1.0, 1.1), (3.0, 3.05)]
    # still open at the end of the bag: closed at the last sample
    assert ms.dose_events([(0.0, True), (0.5, True)]) == [(0.0, 0.5)]
    assert ms.dose_events([]) == []


def test_per_event_deposit_differences_cumulative():
    series = [(0.0, {1: 0.0}), (1.0, {1: 0.0}), (1.5, {1: 0.05, ms.GROUND: 0.01}), (2.0, {1: 0.1, ms.GROUND: 0.02}),
              (2.5, {1: 0.1, ms.GROUND: 0.02}), (4.0, {1: 0.1, 2: 0.03, ms.GROUND: 0.02})]
    d = ms.per_event_deposit([(1.2, 1.9), (3.5, 3.8)], series)
    assert d[0] == pytest.approx({1: 0.1, ms.GROUND: 0.02})   # cumulative at 2.4 (the 2.0 sample) minus at 1.2
    assert d[1] == pytest.approx({2: 0.03})                    # cumulative at 4.3 (the 4.0 sample) minus at 3.5
    assert ms.per_event_deposit([(2.6, 3.0)], series) == [{2: 0.03}]   # first sample at/after 3.0 is the 4.0 one


def test_deposit_landing_after_half_a_second_is_counted():
    # the only sample at or after the event end is 0.8 s later (a slow 2 Hz publisher): the old t1 + 0.5 s cut missed it
    series = [(0.0, {1: 0.0}), (10.0, {1: 0.0}), (10.9, {1: 0.09, ms.GROUND: 0.01}), (11.4, {1: 0.09, ms.GROUND: 0.01})]
    assert ms.per_event_deposit([(10.0, 10.1)], series)[0] == pytest.approx({1: 0.09, ms.GROUND: 0.01})


def test_close_events_do_not_share_a_deposit_so_no_false_double():
    # events 0.3 s apart; tree 2's cumulative already shows 0.08 L in a sample taken 0.1 s into event B. A claims
    # its deposit on tree 1 only; tree 2 belongs to B alone (the old window end t1 + 0.5 s ran into B and gave A 80 %).
    series = [(10.0, {}), (10.45, {1: 0.1}), (10.8, {1: 0.1, 2: 0.08}), (11.5, {1: 0.1, 2: 0.1})]
    events = [(10.0, 10.4), (10.7, 11.1)]
    deps = ms.per_event_deposit(events, series)
    assert deps[0] == pytest.approx({1: 0.1}) and deps[1] == pytest.approx({2: 0.1})
    r = ms.spray_metrics(deps, [1, 2], [0.1, 0.1], targets=[1, 2])
    assert r["double_sprayed_ids"] == [] and r["sprayed_ids"] == [1, 2]
    # the first sample after A's end comes after B's start: A's window is clipped to the latest sample before B's start
    deps2 = ms.per_event_deposit(events, [(10.0, {}), (10.8, {1: 0.1, 2: 0.08}), (11.5, {1: 0.1, 2: 0.1})])
    assert deps2[0] == {} and deps2[1] == pytest.approx({1: 0.1, 2: 0.1})


def test_no_sample_after_the_last_event_uses_final_else_error():
    series = [(0.0, {1: 0.0}), (5.0, {1: 0.05})]
    errs = []
    d = ms.per_event_deposit([(4.0, 4.5), (9.0, 9.5)], series, final={1: 0.2}, errors=errs)
    assert d[0] == pytest.approx({1: 0.05}) and d[1] == pytest.approx({1: 0.15}) and errs == []
    errs = []
    d = ms.per_event_deposit([(4.0, 4.5), (9.0, 9.5)], series[:1], final=None, errors=errs)
    assert len(errs) == 2 and d == [{}, {}]
    errs = []   # a non-last event with no sample at or after its end is an error even when `final` exists
    ms.per_event_deposit([(1.0, 6.0), (7.0, 7.5)], series, final={1: 0.2}, errors=errs)
    assert len(errs) == 1 and "event 0" in errs[0]
    errs = []
    ms.per_event_deposit([(1.0, 6.0), (7.0, 7.5)], series, final=None, errors=errs)
    assert len(errs) == 2


def test_double_spray_detected():
    # tree 2 gets 0.1 L in event 1 and 0.08 L in event 3, 0.3 L intended each -> 33 % and 27 %: double
    deps = [{2: 0.1, ms.GROUND: 0.2}, {1: 0.28}, {2: 0.08, ms.GROUND: 0.22}]
    r = ms.spray_metrics(deps, [1, 2, 3], 0.3)
    assert r["double_sprayed_ids"] == [2] and r["n_double_sprayed"] == 1
    assert r["sprayed_ids"] == [1]
    assert r["missed_ids"] == [2, 3]


def test_missed_below_half():
    r = ms.spray_metrics([{1: 0.14, 2: 0.15, 3: 0.3}], [1, 2, 3], 0.3)
    assert r["sprayed_ids"] == [2, 3]            # 0.15 / 0.3 is exactly half: sprayed
    assert r["missed_ids"] == [1] and r["n_missed"] == 1
    assert r["double_sprayed_ids"] == []


def test_on_target_fraction():
    deps = [{1: 0.2, ms.GROUND: 0.1}, {2: 0.05, ms.GROUND: 0.25}, {ms.GROUND: 0.3}]
    r = ms.spray_metrics(deps, [1, 2], [0.3, 0.3, 0.3], targets=[1, 2, None])
    assert r["on_target_l"] == pytest.approx(0.25)
    assert r["on_target_fraction"] == pytest.approx(0.25 / 0.9)
    assert r["off_target_l"] == pytest.approx(0.65)
    assert r["unmatched_events"] == 1
    assert r["ground_l"] == pytest.approx(0.65)
    assert r["resprayed_events"] == 0


def test_sprayed_tree_targeted_again_counted():
    deps = [{1: 0.3}, {1: 0.1, ms.GROUND: 0.2}]
    r = ms.spray_metrics(deps, [1], 0.3, targets=[1, 1])
    assert r["resprayed_events"] == 1


def test_clearance_min_uses_truth_outline():
    trees = [truth_tree(0, 0.0, 0.0, top=6.0)]
    # over the tree top at z 7.2 -> 1.2; 3 m outside the outline (radius 0.8) is ignored even though it is lower
    poses = [(0.0, 0.0, 0.0, 7.2), (1.0, 5.0, 0.0, 1.0)]
    assert ms.clearance_min(poses, trees, 0.8) == pytest.approx(1.2)
    v, t = ms.clearance_min_at(poses + [(2.0, 2.5, 0.0, 5.0)], trees, 0.8)   # 0.5 m outside the edge: within radius
    assert v == pytest.approx(-1.0) and t == 2.0
    assert ms.clearance_min([(0.0, 20.0, 20.0, 1.0)], trees, 0.8) == float("inf")


def test_nozzle_outside_dose_counted():
    phases = [(0.0, "SCAN"), (10.0, "SPRAYING"), (12.0, "HOPPING")]
    spray = [(5.0, False), (9.0, True), (11.0, True), (11.5, True), (13.0, True), (14.0, False)]
    assert ms.nozzle_outside_dose(spray, phases) == 2
    assert ms.nozzle_outside_dose([(1.0, True)], []) == 1


def test_time_split_groups_and_window():
    phases = [(0.0, "IDLE"), (1.0, "SCAN_APPROACH"), (3.0, "SCAN"), (10.0, "HOPPING"), (14.0, "SPRAYING"),
              (20.0, "RETURN"), (25.0, "IDLE")]
    states = [(0.0, "IDLE"), (1.0, "TAKEOFF"), (3.0, "SCAN_OUT"), (8.0, "SCAN_SETTLE"), (10.0, "GLIDE"),
              (13.0, "HOLD"), (14.0, "DOSE"), (16.0, "RETURN"), (24.0, "DONE")]
    r = ms.time_split(phases, states, t_end=30.0)
    assert r["mission_time_s"] == pytest.approx(23.0) and r["t_start"] == 1.0 and r["t_end"] == 24.0
    g = r["groups"]
    assert (g["scan"], g["settle"], g["glide"], g["hold"], g["dose"]) == pytest.approx((5.0, 2.0, 3.0, 1.0, 2.0))
    assert g["transit"] == pytest.approx(2.0 + 8.0)
    assert r["by_phase"]["SPRAYING"] == pytest.approx(6.0)
    assert ms.time_split([], [], 5.0)["mission_time_s"] == 0.0


def test_cost_metrics_mean_p99():
    r = ms.cost_metrics({"step": [float(i) for i in range(1, 101)], "sgbm": [10.0, 30.0], "detector": []})
    assert r["step"]["mean_ms"] == pytest.approx(50.5)
    assert r["step"]["p99_ms"] == pytest.approx(99.01)
    assert r["step"]["n"] == 100
    assert r["sgbm"]["mean_ms"] == pytest.approx(20.0)
    assert r["detector"]["mean_ms"] is None


def test_match_target_outline_then_nearest_then_none():
    gt = [truth_tree(0, 0.0, 0.0), truth_tree(1, 10.0, 0.0, h=1.0)]
    snap = [Tree(5, sq(0.5, 0.5, 0.5), (0.5, 0.5), 5.0), Tree(6, sq(13.0, 0.0, 0.5), (4.0, 0.0), 5.0),
            Tree(7, sq(0, 0, 0.5), (30.0, 30.0), 5.0), Tree(8, sq(0, 0, 0.5), (11.5, 0.0), 5.0)]
    assert ms.match_target_to_truth(5, snap, gt) == 0         # centre inside the outline
    assert ms.match_target_to_truth(6, snap, gt) is None      # 4 m from tree 0, 6 m from tree 1, outside both
    assert ms.match_target_to_truth(7, snap, gt) is None
    assert ms.match_target_to_truth(8, snap, gt) == 1         # outside the outline (x 9..11), 1.5 m from the centre
    assert ms.match_target_to_truth(99, snap, gt) is None


def test_compare_groups_by_strategy_and_reports_worst_direction():
    def card(strategy, **kw):
        flat = {k: None for k in ms.COMPARE_METRICS}
        flat.update(kw)
        return {"run": {"strategy": strategy}, "flat": flat}
    cards = [card("corridor", mission_time_s=100.0, n_sprayed=24, clearance_min_m=1.5),
             card("corridor", mission_time_s=120.0, n_sprayed=22, clearance_min_m=0.9),
             card("field_first", mission_time_s=200.0, n_sprayed=25, clearance_min_m=2.0)]
    r = ms.compare_runs(cards)
    c = r["corridor"]
    assert c["mission_time_s"]["mean"] == pytest.approx(110.0)
    assert c["mission_time_s"]["worst"] == 120.0 and c["mission_time_s"]["spread"] == pytest.approx(20.0)
    assert c["n_sprayed"]["worst"] == 22 and c["clearance_min_m"]["worst"] == pytest.approx(0.9)
    assert r["field_first"]["mission_time_s"]["worst"] == 200.0
    assert r["field_first"]["mission_time_s"]["spread"] == 0.0
    assert r["corridor"]["rtf"]["mean"] is None


# ----------------------------------------------------------------------------- end to end

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _msg_classes():
    """Build the repo's custom message classes from their .msg files (no catkin build needed)."""
    genpy_dynamic = pytest.importorskip("genpy.dynamic")

    def text(path):
        with open(path) as f:
            return f.read()

    def find(pkg, name):
        p = os.path.join(REPO, "ros", pkg, "msg", name + ".msg")
        if os.path.exists(p):
            return p
        return glob.glob("/opt/ros/*/share/%s/msg/%s.msg" % (pkg, name))[0]

    def cat(main, deps):
        out = text(find(*main.split("/")))
        for d in deps:
            out += "\n" + "=" * 80 + "\nMSG: %s\n" % d + text(find(*d.split("/")))
        return out

    classes = {}
    classes.update(genpy_dynamic.generate_dynamic("treemapper_msgs/StageTiming", cat(
        "treemapper_msgs/StageTiming", ["std_msgs/Header"])))
    classes.update(genpy_dynamic.generate_dynamic("treemapper_msgs/TreeMap", cat(
        "treemapper_msgs/TreeMap", ["treemapper_msgs/TreeEntry", "geometry_msgs/Point", "std_msgs/Header"])))
    classes.update(genpy_dynamic.generate_dynamic("corridor_nav_msgs/NavStatus", cat(
        "corridor_nav_msgs/NavStatus", ["std_msgs/Header"])))
    return classes


def _build_run(root, flow=0.1, deposition=True, tf=True, spray=True, last_state="DONE"):
    rospy = pytest.importorskip("rospy")
    rosbag = pytest.importorskip("rosbag")
    from geometry_msgs.msg import Point, TransformStamped
    from std_msgs.msg import Bool, Int32, String
    from tf2_msgs.msg import TFMessage
    C = _msg_classes()
    T = lambda s: rospy.Time.from_sec(s)

    trees = [truth_tree(0, 0.0, 0.0, top=6.0), truth_tree(1, 8.0, 0.0, top=6.0)]
    os.makedirs(root)
    from treemapper_tools import contract
    json.dump({"frame": "world", "trees": trees}, open(os.path.join(root, "truth.json"), "w"))
    nozzle = {} if flow is None else {"flow_lps": flow}
    json.dump({"spec": {"nozzle": nozzle, "drone": {"home": [0, 0]}}, "strategy": "corridor",
               "git_commit": "abc1234", "seed": 7}, open(os.path.join(root, "run.json"), "w"))
    json.dump({"sim_s": 40.0, "wall_s": 80.0, "rtf": 0.5}, open(os.path.join(root, "rtf.json"), "w"))
    # final cumulative deposition: event 1 (2 s, 0.2 L): 0.18 on tree 0; event 2 (2 s, 0.2 L): 0.05 on tree 1
    json.dump({"trees": {"0": 0.18, "1": 0.05}, "ground": 0.17, "emitted": 0.4, "open_time": 4.0},
              open(os.path.join(root, "deposition.json"), "w"))

    def entry(i, cx, cy):
        e = C["treemapper_msgs/TreeEntry"]()
        e.id = i
        e.outline = [Point(x, y, 0.0) for x, y in sq(cx, cy, 1.8)]
        e.centre = Point(cx, cy, 0.0)
        e.height = 6.0
        return e

    with rosbag.Bag(os.path.join(root, "mission.bag"), "w") as bag:
        tm = C["treemapper_msgs/TreeMap"]()
        tm.header.stamp = T(5.0)
        tm.header.frame_id = "world"
        tm.trees = [entry(5, 0.1, 0.1), entry(6, 8.2, -0.1)]
        bag.write("/treemapper/map", tm, T(5.0))
        # TF: 10 Hz, z = 8 flying over both trees, one dip to 7.2 over tree 0 at t = 11
        for k in range(0, 400 if tf else 0):
            t = k * 0.1
            x = -1.0 if t < 10 else (0.0 if t < 15 else 8.0)
            m = TFMessage()
            tr = TransformStamped()
            tr.header.stamp = T(t)
            tr.header.frame_id = "world"
            tr.child_frame_id = "body"
            tr.transform.translation.x, tr.transform.translation.z = x, 7.2 if abs(t - 11.0) < 1e-6 else 8.0
            tr.transform.rotation.w = 1.0
            other = TransformStamped()
            other.header.stamp = T(t)
            other.header.frame_id = "world"
            other.child_frame_id = "camera"       # ignored
            other.transform.translation.z = -50.0
            other.transform.rotation.w = 1.0
            m.transforms = [tr, other]
            bag.write("/tf", m, T(t))
        for t, s in [(0.0, "IDLE"), (1.0, "SCAN"), (9.0, "HOPPING"), (10.0, "SPRAYING"), (12.5, "HOPPING"),
                     (20.0, "SPRAYING"), (22.5, "RETURN")]:
            bag.write("/mission_state", String(s), T(t))
        bag.write("/spray_target", Int32(-1), T(1.0))
        bag.write("/spray_target", Int32(5), T(9.9))
        bag.write("/spray_target", Int32(6), T(19.9))
        for k in range(0, 800):                     # 20 Hz; open 10..12 and 20..22
            t = k * 0.05
            bag.write("/corridor_nav/spray", Bool(spray and ((10.0 <= t < 12.0) or (20.0 <= t < 22.0))), T(t))
        if deposition == "garbage":
            bag.write("/sim/deposition", String("{not json"), T(0.0))
        for k in range(0, 80 if deposition else 0):  # 2 Hz cumulative deposition
            t = k * 0.5
            f1 = min(max((t - 10.0) / 2.0, 0.0), 1.0)
            f2 = min(max((t - 20.0) / 2.0, 0.0), 1.0)
            d = {"trees": {"0": 0.18 * f1, "1": 0.05 * f2}, "ground": 0.02 * f1 + 0.15 * f2, "emitted": 0.2 * f1 + 0.2 * f2,
                 "open_time": 2 * f1 + 2 * f2}
            bag.write("/sim/deposition", String(json.dumps(d)), T(t))
        for t, s in [(0.0, "IDLE"), (1.0, "TAKEOFF"), (3.0, "SCAN_OUT"), (9.0, "GLIDE"), (10.0, "DOSE"),
                     (12.5, "GLIDE"), (20.0, "DOSE"), (22.5, "RETURN"), (30.0, last_state)]:
            st = C["corridor_nav_msgs/NavStatus"]()
            st.header.stamp = T(t)
            st.state = s
            st.trees_sprayed = 2 if t >= 22.5 else 0
            st.reasons, st.counts = ["glide_climbs_inserted", "hold_map_stale", "guard_triggers"], [1, 3, 2]
            bag.write("/corridor_nav/status", st, T(t))
        for topic, stage, vals in [("/corridor_nav/timing", "step", [1.0, 2.0, 3.0]),
                                   ("/stereo_sgbm/timing", "sgbm", [20.0, 40.0]),
                                   ("/detector/timing", "detector", [50.0])]:
            for k, v in enumerate(vals):
                st = C["treemapper_msgs/StageTiming"]()
                st.header.stamp = T(1.0 + k)
                st.stages, st.ms = [stage], [v]
                bag.write(topic, st, T(1.0 + k))
    return root


def test_end_to_end_synthetic_run(tmp_path, capsys):
    root = _build_run(str(tmp_path / "run_1"))
    out = str(tmp_path / "card.md")
    assert ms.main(["--run", root, "--out", out]) == 0
    card = json.load(open(os.path.join(root, "mission_score.json")))
    for f in ("map.jsonl", "mission.json", "ground_truth.json"):
        assert os.path.exists(os.path.join(root, f)), f
    assert card["errors"] == [] and card["end_state"] == "DONE" and card["completed"] is True
    assert card["all_pass"] is True
    assert card["efficiency"]["dose_events"] == 2
    assert card["efficiency"]["litres_emitted"] == pytest.approx(0.4)
    assert card["efficiency"]["mission_time_s"] == pytest.approx(29.0)     # 1.0 .. DONE at 30.0
    assert card["efficiency"]["glide_climbs_inserted"] == 1
    sp = card["spray"]
    assert sp["sprayed_ids"] == [0] and sp["missed_ids"] == [1] and sp["double_sprayed_ids"] == []
    assert sp["unmatched_events"] == 0
    assert sp["on_target_fraction"] == pytest.approx((0.18 + 0.05) / 0.4)
    sf = card["safety"]
    assert sf["clearance_min_m"] == pytest.approx(1.2) and sf["clearance_min_t"] == pytest.approx(11.0)
    assert sf["nozzle_outside_dose"] == 0 and sf["all_pass"] is True
    assert card["guards"]["holds"] == {"hold_map_stale": 3} and card["guards"]["guard_triggers"] == 2
    c = card["cost"]
    assert c["step"]["mean_ms"] == pytest.approx(2.0) and c["sgbm"]["mean_ms"] == pytest.approx(30.0)
    assert c["detector"]["p99_ms"] == pytest.approx(50.0) and c["rtf"] == 0.5
    assert card["map"]["final"]["detection"]["matched"] == 2
    md = open(out).read()
    assert "Minimum clearance" in md and "PASS" in md and "Detection" in md
    # a second run in the same strategy gives the comparison
    root2 = _build_run(str(tmp_path / "run_2"))
    assert ms.main(["--run", root, "--compare", root2]) == 0
    printed = capsys.readouterr().out
    assert "Mission comparison" in printed and "corridor (2 runs)" in printed


# ----------------------------------------------------------------------------- failure modes (end to end)

def _run_cli(root, capsys):
    rc = ms.main(["--run", root])
    cap = capsys.readouterr()
    return rc, json.load(open(os.path.join(root, "mission_score.json"))), cap


def test_missing_deposition_series_is_an_error_and_exits_nonzero(tmp_path, capsys):
    rc, card, cap = _run_cli(_build_run(str(tmp_path / "r"), deposition=False), capsys)
    assert rc == 2
    assert any("no /sim/deposition samples" in e for e in card["errors"])
    assert card["all_pass"] is False and card["safety"]["all_pass"] is False
    assert "## ERRORS" in cap.out and "no /sim/deposition samples" in cap.out
    assert "mission_score: FAIL" in cap.err


def test_unparseable_deposition_sample_is_counted(tmp_path, capsys):
    rc, card, _ = _run_cli(_build_run(str(tmp_path / "r"), deposition="garbage"), capsys)
    assert rc == 2 and any("1 unparseable /sim/deposition" in e for e in card["errors"])


def test_flow_zero_or_missing_is_an_error(tmp_path, capsys):
    for i, flow in enumerate((0.0, None)):
        rc, card, _ = _run_cli(_build_run(str(tmp_path / ("r%d" % i)), flow=flow), capsys)
        assert rc == 2 and any("flow_lps" in e for e in card["errors"]) and card["all_pass"] is False


def test_no_tf_is_an_error_and_clearance_does_not_pass(tmp_path, capsys):
    rc, card, _ = _run_cli(_build_run(str(tmp_path / "r"), tf=False), capsys)
    assert rc == 2 and any("no TF pose samples" in e for e in card["errors"])
    assert card["safety"]["clearance_pass"] is False and card["safety"]["clearance_min_m"] is None
    assert card["all_pass"] is False


def test_zero_dose_events_is_an_error(tmp_path, capsys):
    rc, card, _ = _run_cli(_build_run(str(tmp_path / "r"), spray=False), capsys)
    assert rc == 2 and any("no dose events" in e for e in card["errors"]) and card["all_pass"] is False


def test_deposition_json_missing_or_inconsistent_is_an_error(tmp_path, capsys):
    root = _build_run(str(tmp_path / "a"))
    os.remove(os.path.join(root, "deposition.json"))
    rc, card, _ = _run_cli(root, capsys)
    assert rc == 2 and any("deposition.json is missing" in e for e in card["errors"])
    root = _build_run(str(tmp_path / "b"))
    json.dump({"trees": {"0": 0.18, "1": 0.25}, "ground": 0.17, "emitted": 0.6, "open_time": 4.0},
              open(os.path.join(root, "deposition.json"), "w"))
    rc, card, _ = _run_cli(root, capsys)
    assert rc == 2 and any("deposition inconsistent" in e and "tree 1" in e for e in card["errors"])


def test_abort_end_state_fails_and_is_excluded_from_time_metrics(tmp_path, capsys):
    rc, card, cap = _run_cli(_build_run(str(tmp_path / "abort"), last_state="ABORT"), capsys)
    assert card["end_state"] == "ABORT" and card["completed"] is False
    assert card["errors"] == [] and card["all_pass"] is False and rc == 2
    assert "NOT completed" in cap.out and "end state ABORT" in cap.err
    ok = _build_run(str(tmp_path / "ok"))
    assert ms.main(["--run", ok, "--compare", os.path.join(str(tmp_path), "abort")]) == 2
    out = capsys.readouterr().out
    assert "Completed (ended in DONE): 1 of 2" in out and "`abort`" in out


def test_compare_excludes_incomplete_runs_from_time_metrics():
    def card(name, completed, t, n):
        flat = {k: None for k in ms.COMPARE_METRICS}
        flat.update(mission_time_s=t, n_sprayed=n)
        return {"run": {"strategy": "corridor", "dir": "/x/" + name}, "completed": completed, "flat": flat}
    r = ms.compare_runs([card("a", True, 100.0, 10), card("b", False, 5.0, 3), card("c", True, 120.0, 9)])["corridor"]
    assert r["_runs"] == {"n": 3, "completed": 2, "incomplete": ["b"]}
    assert r["mission_time_s"]["mean"] == pytest.approx(110.0) and r["mission_time_s"]["worst"] == 120.0
    assert r["n_sprayed"]["worst"] == 3 and r["n_sprayed"]["n"] == 3     # non-time metrics keep every run
