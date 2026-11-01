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


# ----------------------------------------------------------------------------- dose episodes (controller ruling I1)

def snap(t, sprayed_ids=(), ids=(5, 6)):
    return Snapshot(t, [Tree(i, sq(0.0, 0.0, 1.0), (0.0, 0.0), 5.0, i in sprayed_ids) for i in ids])


def test_episode_is_one_spraying_interval_with_its_nozzle_intervals():
    phases = [(0.0, "HOPPING"), (10.0, "SPRAYING"), (14.0, "HOPPING"), (20.0, "SPRAYING"), (24.0, "RETURN")]
    targets = [(0.0, 5), (19.0, 6)]
    # a guard climb closes the nozzle 11.0 .. 12.0 inside the first episode: two nozzle intervals, one episode
    spray = [(9.9, False), (10.1, True), (11.0, False), (12.0, True), (13.9, False), (20.1, True), (23.9, False)]
    eps = ms.dose_episodes(phases, targets, spray, t_end=30.0)
    assert [(e.start, e.end, e.tm_id) for e in eps] == [(10.0, 14.0, 5), (20.0, 24.0, 6)]
    assert eps[0].intervals == [(10.1, 11.0), (12.0, 13.9)]
    assert eps[0].spraying_s == pytest.approx(4.0) and eps[0].open_s == pytest.approx(2.8)
    assert not any(e.orphan for e in eps)
    # a /spray_target change inside a SPRAYING interval starts a new episode; an open nozzle outside SPRAYING is an
    # orphan unit of its own (nozzle_outside_dose flags it as a safety failure)
    eps = ms.dose_episodes([(0.0, "SPRAYING"), (8.0, "HOPPING")], [(0.0, 5), (4.0, 6)],
                           [(0.5, True), (3.0, False), (4.5, True), (7.5, False), (9.0, True), (9.5, False)], t_end=10.0)
    assert [(e.start, e.end, e.tm_id, e.orphan) for e in eps] == [(0.0, 4.0, 5, False), (4.0, 8.0, 6, False),
                                                                  (9.0, 9.5, 6, True)]


def test_guard_interruption_is_one_episode_not_a_double_nor_targeted_again():
    phases = [(0.0, "HOPPING"), (10.0, "SPRAYING"), (14.0, "HOPPING")]
    spray = [(10.1, True), (11.0, False), (12.0, True), (13.9, False)]
    eps = ms.dose_episodes(phases, [(0.0, 5)], spray, t_end=20.0)
    assert len(eps) == 1
    series = [(9.0, {}), (11.5, {1: 0.09}), (14.5, {1: 0.26, ms.GROUND: 0.02})]   # 2.8 s open at 0.1 L/s
    deps = ms.per_event_deposit([e.window for e in eps], series)
    flow = 0.1
    r = ms.spray_metrics(deps, [1], [flow * e.spraying_s for e in eps], targets=[1],
                         emitted_l_per_event=[flow * e.open_s for e in eps])
    assert r["sprayed_ids"] == [1] and r["double_sprayed_ids"] == [] and r["n_events"] == 1
    assert r["intended_l_total"] == pytest.approx(0.4)              # 4 s of SPRAYING; 0.26 / 0.4 >= 50 %
    assert r["on_target_fraction"] == pytest.approx(0.26 / 0.28)
    assert ms.targeted_again(eps, [snap(5.0)]) == []


def test_retry_on_an_unsprayed_id_is_not_targeted_again_but_a_sprayed_id_is():
    # dose invalid -> retry: two SPRAYING episodes on id 5, no sprayed flag in between: a legitimate retry
    phases = [(10.0, "SPRAYING"), (14.0, "HOPPING"), (16.0, "SPRAYING"), (20.0, "HOPPING")]
    eps = ms.dose_episodes(phases, [(0.0, 5)], [(10.5, True), (13.5, False), (16.5, True), (19.5, False)], t_end=30.0)
    assert len(eps) == 2
    assert ms.targeted_again(eps, [snap(5.0), snap(15.0)]) == []
    # the map marked 5 sprayed after the first episode: the second is targeted again
    assert ms.targeted_again(eps, [snap(5.0), snap(15.0, sprayed_ids=(5,))]) == [1]
    # already sprayed in the latest map before the very first episode
    assert ms.targeted_again(eps[:1], [snap(5.0, sprayed_ids=(5,))]) == [0]
    # a map that showed the flag after the first episode, even if a later snapshot before the retry lost it
    assert ms.targeted_again(eps, [snap(14.5, sprayed_ids=(5,)), snap(15.5)]) == [1]
    # a sprayed flag on another id does not count; a map after the episode start is not "before"
    assert ms.targeted_again(eps, [snap(15.0, sprayed_ids=(6,)), snap(17.0, sprayed_ids=(5,))]) == []


def test_second_dose_on_a_completed_id_within_one_map_period_is_targeted_again():
    # id 5 gets a full dose (3.8 of 4 s open), then another episode, with no map snapshot in between (one map period)
    phases = [(10.0, "SPRAYING"), (14.0, "HOPPING"), (16.0, "SPRAYING"), (20.0, "HOPPING")]
    spray = [(10.1, True), (13.9, False), (16.1, True), (19.9, False)]
    eps = ms.dose_episodes(phases, [(0.0, 5)], spray, t_end=30.0)
    assert len(eps) == 2
    assert ms.targeted_again(eps, [snap(5.0), snap(30.0)]) == [1]
    assert ms.targeted_again(eps, []) == [1]
    # a different id in the second episode is not a second dose
    eps2 = ms.dose_episodes(phases, [(0.0, 5), (15.0, 6)], spray, t_end=30.0)
    assert ms.targeted_again(eps2, [snap(5.0), snap(30.0)]) == []
    # the first dose was invalid (1.0 of 4 s open): the navigator retries it, which is not a second dose
    short = [(10.1, True), (11.1, False), (16.1, True), (19.9, False)]
    eps3 = ms.dose_episodes(phases, [(0.0, 5)], short, t_end=30.0)
    assert ms.targeted_again(eps3, [snap(5.0), snap(30.0)]) == []
    # sprayed in a snapshot earlier than the later episode's start, whichever snapshot of those it is
    assert ms.targeted_again(eps3, [snap(5.0, sprayed_ids=(5,)), snap(12.0), snap(15.0)]) == [0, 1]


def test_hold_inside_spraying_is_not_intended_dose():
    # final review I2: HOLD keeps /mission_state SPRAYING (spec 4.4) with the nozzle closed; NavStatus shows it. The
    # intended dose (flow x SPRAYING duration, 16A ruling) and the dose validity are judged on the non-hold time.
    status = [(0.0, "IDLE"), (9.0, "GLIDE"), (10.0, "DOSE"), (11.0, "HOLD"), (12.0, "DOSE"), (14.5, "GLIDE"),
              (16.0, "DOSE"), (19.0, "HOLD"), (19.5, "GLIDE"), (25.0, "HOLD"), (26.0, "RETURN")]
    holds = ms.hold_intervals(status, t_end=30.0)
    assert holds == [(11.0, 12.0), (19.0, 19.5), (25.0, 26.0)]
    assert ms.hold_intervals([(0.0, "DOSE"), (3.0, "HOLD")], t_end=4.0) == [(3.0, 4.0)]   # open at the end: t_end
    phases = [(9.0, "HOPPING"), (10.0, "SPRAYING"), (14.5, "HOPPING"), (16.0, "SPRAYING"), (19.25, "HOPPING")]
    # open 0.9 s, HOLD 1 s (nozzle closed), open 2.5 s: 3.4 s of 3.5 s non-hold SPRAYING
    spray = [(10.1, True), (11.0, False), (12.0, True), (14.5, False), (16.1, True), (19.0, False)]
    eps = ms.dose_episodes(phases, [(0.0, 5)], spray, t_end=30.0, holds=holds)
    assert [(e.start, e.end) for e in eps] == [(10.0, 14.5), (16.0, 19.25)]
    assert eps[0].hold_s == pytest.approx(1.0) and eps[0].spraying_s == pytest.approx(3.5)
    assert eps[1].hold_s == pytest.approx(0.25) and eps[1].spraying_s == pytest.approx(3.0)   # only the overlap
    flow = 0.1
    assert [flow * e.spraying_s for e in eps] == [pytest.approx(0.35), pytest.approx(0.30)]
    assert eps[0].dose_complete()                     # 3.4 / 3.5 >= 0.8 (3.4 / 4.5 would not be)
    assert ms.targeted_again(eps, []) == [1]          # so the second episode on id 5 is a second dose
    # without hold information nothing changes
    plain = ms.dose_episodes(phases, [(0.0, 5)], spray, t_end=30.0)
    assert plain[0].hold_s == 0.0 and plain[0].spraying_s == pytest.approx(4.5)
    assert not plain[0].dose_complete()
    # an orphan unit keeps its open time
    orphan = ms.dose_episodes([(0.0, "HOPPING")], [(0.0, 5)], [(1.0, True), (2.0, False)], t_end=5.0,
                              holds=[(0.0, 5.0)])
    assert orphan[0].orphan and orphan[0].spraying_s == pytest.approx(1.0)


def _repo_cfg_value(key):
    path = os.path.join(os.path.dirname(__file__), "..", "..", "config", "corridor_nav.cfg")
    for line in open(path):
        line = line.split("#", 1)[0].strip()
        if "=" in line and line.split("=", 1)[0].strip() == key:
            return float(line.split("=", 1)[1])
    raise KeyError(key)


def test_dose_valid_frac_constant_equals_the_shipped_navigator_cfg():
    assert ms.DOSE_VALID_FRAC == _repo_cfg_value("dose_valid_frac")


def test_dose_valid_frac_is_read_from_the_run_cfg(tmp_path):
    d = str(tmp_path)
    assert ms.dose_valid_frac(d) == ms.DOSE_VALID_FRAC                     # no cfg in the run dir: the constant
    with open(os.path.join(d, ms.RUN_CFG), "w") as f:
        f.write("# run cfg\nstrategy = corridor\ndose_valid_frac = 0.6   # sim\ncarrot_lead_s = 1.2\n")
    assert ms.dose_valid_frac(d) == pytest.approx(0.6)
    with open(os.path.join(d, ms.RUN_CFG), "w") as f:
        f.write("strategy = corridor\n")                                    # key absent: the constant
    assert ms.dose_valid_frac(d) == ms.DOSE_VALID_FRAC
    for bad in ("abc", "nan", "0", "1.5"):
        with open(os.path.join(d, ms.RUN_CFG), "w") as f:
            f.write("dose_valid_frac = %s\n" % bad)
        with pytest.raises(ValueError):
            ms.dose_valid_frac(d)
    # the threshold decides whether a 3.0 of 4 s first dose was valid, i.e. whether the second episode is a re-dose
    phases = [(10.0, "SPRAYING"), (14.0, "HOPPING"), (16.0, "SPRAYING"), (20.0, "HOPPING")]
    eps = ms.dose_episodes(phases, [(0.0, 5)], [(10.0, True), (13.0, False), (16.1, True), (19.9, False)], t_end=30.0)
    assert ms.targeted_again(eps, []) == []                 # 75 % < 0.8: a retry
    assert ms.targeted_again(eps, [], 0.7) == [1]           # 75 % >= 0.7: a second dose


def test_short_burst_in_a_long_episode_is_not_sprayed():
    # 0.2 s of nozzle in a 4 s SPRAYING episode: all of it on tree 1, but 5 % of the intended dose
    eps = ms.dose_episodes([(10.0, "SPRAYING"), (14.0, "HOPPING")], [(0.0, 5)], [(10.0, True), (10.2, False)],
                           t_end=20.0)
    flow = 0.1
    deps = ms.per_event_deposit([e.window for e in eps], [(9.0, {}), (15.0, {1: 0.02})])
    r = ms.spray_metrics(deps, [1], [flow * e.spraying_s for e in eps], targets=[1],
                         emitted_l_per_event=[flow * e.open_s for e in eps])
    assert r["intended_l_total"] == pytest.approx(0.4) and r["emitted_l_total"] == pytest.approx(0.02)
    assert r["sprayed_ids"] == [] and r["missed_ids"] == [1]
    assert r["on_target_fraction"] == pytest.approx(1.0)


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


def _build_run(root, flow=0.1, deposition=True, tf=True, spray=True, last_state="DONE", counters=None,
               extra_phases=(), extra_status=()):
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
        for t, s in sorted([(0.0, "IDLE"), (1.0, "SCAN"), (9.0, "HOPPING"), (10.0, "SPRAYING"), (12.5, "HOPPING"),
                            (20.0, "SPRAYING"), (22.5, "RETURN")] + list(extra_phases)):
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
        for t, s in sorted([(0.0, "IDLE"), (1.0, "TAKEOFF"), (3.0, "SCAN_OUT"), (9.0, "GLIDE"), (10.0, "DOSE"),
                            (12.5, "GLIDE"), (20.0, "DOSE"), (22.5, "RETURN"), (30.0, last_state)] + list(extra_status)):
            st = C["corridor_nav_msgs/NavStatus"]()
            st.header.stamp = T(t)
            st.state = s
            st.trees_sprayed = 2 if t >= 22.5 else 0
            cn = {"glide_climbs_inserted": 1, "hold_map_stale": 3, "guard_triggers": 2}
            cn.update(counters or {})
            st.reasons, st.counts = list(cn), list(cn.values())
            st.trees_remaining = 0 if t >= 22.5 else 2
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
    assert card["spray"]["n_events"] == 2 and card["spray"]["intended_l_total"] == pytest.approx(0.5)  # 2 x 2.5 s SPRAYING
    assert card["spray"]["nav_trees_remaining"] == 0
    assert card["safety"]["sprayed_tree_targeted_again"] == 0
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
    assert "Completed (DONE, no abort or hold timeout, never ERROR): 1 of 2" in out and "`abort`" in out


def test_abort_or_hold_timeout_or_error_phase_is_not_completed_even_if_it_ends_in_done(tmp_path, capsys):
    # an airborne ABORT shows ERROR for one tick, then RETURN -> LAND -> DONE; the 2 Hz NavStatus never says ABORT
    rc, card, cap = _run_cli(_build_run(str(tmp_path / "a"), counters={"aborts": 1},
                                        extra_phases=[(15.0, "ERROR"), (15.05, "RETURN"), (19.9, "HOPPING")]), capsys)
    assert card["end_state"] == "DONE" and card["completed"] is False
    assert card["all_pass"] is False and card["safety"]["all_pass"] is False and rc == 2
    assert any("aborts 1" in m for m in card["fail_reasons"]) and any("ERROR" in m for m in card["fail_reasons"])
    assert "NOT completed" in cap.out
    rc, card, _ = _run_cli(_build_run(str(tmp_path / "b"), counters={"hold_timeouts": 1}), capsys)
    assert card["completed"] is False and rc == 2 and any("hold_timeouts 1" in m for m in card["fail_reasons"])
    rc, card, _ = _run_cli(_build_run(str(tmp_path / "c"), extra_phases=[(15.0, "ERROR"), (15.05, "HOPPING")]), capsys)
    assert card["completed"] is False and rc == 2


def test_end_to_end_hold_inside_spraying_is_excluded_from_the_intended_dose(tmp_path, capsys):
    # NavStatus HOLD 20.5 .. 21.0 inside the second SPRAYING interval (20.0 .. 22.5): 2.5 + 2.0 s intended at 0.1 L/s
    root = _build_run(str(tmp_path / "run_hold"), extra_status=[(20.5, "HOLD"), (21.0, "DOSE")])
    rc, card, _ = _run_cli(root, capsys)
    assert rc == 0 and card["errors"] == []
    assert card["spray"]["intended_l_total"] == pytest.approx(0.45)


def test_compare_excludes_incomplete_runs_from_time_metrics():
    def card(name, completed, t, n):
        flat = {k: None for k in ms.COMPARE_METRICS}
        flat.update(mission_time_s=t, n_sprayed=n)
        return {"run": {"strategy": "corridor", "dir": "/x/" + name}, "completed": completed, "flat": flat}
    r = ms.compare_runs([card("a", True, 100.0, 10), card("b", False, 5.0, 3), card("c", True, 120.0, 9)])["corridor"]
    assert r["_runs"] == {"n": 3, "completed": 2, "incomplete": ["b"]}
    assert r["mission_time_s"]["mean"] == pytest.approx(110.0) and r["mission_time_s"]["worst"] == 120.0
    assert r["n_sprayed"]["worst"] == 3 and r["n_sprayed"]["n"] == 3     # non-time metrics keep every run
