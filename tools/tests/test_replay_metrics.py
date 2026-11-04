"""Replay metrics: per-entry height and centre error against truth at the lock and at each dose (pure, on a
synthetic run: no bag, no replay binary)."""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from treemapper_tools import replay_metrics as rm


def sq(cx, cy, h=1.5):
    return [[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]]


def truth_tree(i, cx, cy, top=4.0, key="top_z"):
    return {"id": i, "centre": [cx, cy], "outline": sq(cx, cy), "radius": 1.5, key: top}


def write_run(tmp_path, truth, snaps, phases, targets, poses, name="run"):
    """snaps: [(t, [(id, cx, cy, height), ...]), ...]; returns (map, truth, mission) paths."""
    d = tmp_path / name
    d.mkdir()
    with open(d / "map.jsonl", "w") as f:
        for t, trees in snaps:
            f.write(json.dumps({"t": t, "frame": "world", "trees": [
                {"id": i, "outline": sq(cx, cy, 1.0), "centre": [cx, cy], "height": h, "sprayed": False}
                for i, cx, cy, h in trees]}) + "\n")
    (d / "truth.json").write_text(json.dumps({"frame": "world", "trees": truth}))
    (d / "mission.json").write_text(json.dumps({"phases": phases, "targets": targets, "pose": poses}))
    return str(d / "map.jsonl"), str(d / "truth.json"), str(d / "mission.json")


def track(t0, t1, y=0.0, z=8.0, dt=0.5):
    """Drone flying along +x at 1 m/s on the line y: pose = (t, x, y, z)."""
    n = int(round((t1 - t0) / dt))
    return [[t0 + k * dt, t0 + k * dt, y, z] for k in range(n + 1)]


def by_id(res):
    return {e["id"]: e for e in res["entries"]}


# Truth A (id 1) at (5, 6), B (id 2) at (15, 3), both top 4.0. The drone flies along x at y = 0, so A is 6.0 m across
# track and B 3.0 m. Entry 7 = A, entry 8 = B.
TRUTH = [truth_tree(1, 5.0, 6.0), truth_tree(2, 15.0, 3.0)]
PHASES = [[0.0, "SCAN"], [10.0, "PLANNING"], [11.0, "HOPPING"], [14.0, "SPRAYING"], [18.0, "HOPPING"],
          [20.0, "SPRAYING"], [24.0, "HOPPING"]]
TARGETS = [[0.0, -1], [11.0, 7], [18.0, 8], [24.0, -1]]


def basic_run(tmp_path, a_dose_h=4.5, b_dose_c=(15.4, 3.0), extra=(), name="run"):
    snaps = [
        (3.0, [(7, 5.9, 6.0, 6.0)]),                                       # A first published here: poor centre / height
        (8.0, [(7, 5.2, 6.0, 4.5), (8, 15.0, 3.0, 3.0)]),                  # lock snapshot (PLANNING starts at 10)
        (12.0, [(7, 5.2, 6.0, a_dose_h), (8, b_dose_c[0], b_dose_c[1], 3.0)] + list(extra)),
        (16.0, [(7, 5.2, 6.0, 4.7), (8, b_dose_c[0], b_dose_c[1], 3.0)]),  # after A's dose, before B's
    ]
    return write_run(tmp_path, TRUTH, snaps, PHASES, TARGETS, track(0, 26), name)


def test_heights_and_centres_at_lock_and_dose(tmp_path):
    res = rm.compute(*basic_run(tmp_path))
    a = by_id(res)[7]
    assert a["truth_id"] == 1
    assert a["first_pub_t"] == 3.0
    assert a["lock"]["dh"] == pytest.approx(0.5)      # snapshot at 8.0 (latest <= the PLANNING start at 10.0)
    assert a["lock"]["dc"] == pytest.approx(0.2)
    assert len(a["doses"]) == 1
    d = a["doses"][0]
    assert d["t"] == pytest.approx(14.0)
    assert d["dh"] == pytest.approx(0.5)              # snapshot at 12.0, not the later 16.0 one (4.7)
    assert d["dc"] == pytest.approx(0.2)
    b = by_id(res)[8]                                 # B: lock at 10 from the 8.0 snapshot, dose at 20 from the 16.0 one
    assert b["lock"]["dh"] == pytest.approx(-1.0)
    assert b["doses"][0]["dc"] == pytest.approx(0.4)


def test_a1_counts_over_1p5(tmp_path):
    res = rm.compute(*basic_run(tmp_path))
    assert res["a1"]["n_over_1p5"] == 0
    assert res["a1"]["max_dh_at_lock"] == pytest.approx(0.5)
    assert res["a1"]["max_dh_at_dose"] == pytest.approx(0.5)
    res = rm.compute(*basic_run(tmp_path, a_dose_h=6.0, name="run2"))        # +2.0 m at A's dose
    assert res["a1"]["n_over_1p5"] == 1
    assert res["a1"]["max_dh_at_dose"] == pytest.approx(2.0)
    assert res["a1"]["max_dh_at_lock"] == pytest.approx(0.5)
    # exactly 1.5 is not over
    res = rm.compute(*basic_run(tmp_path, a_dose_h=5.5, name="run3"))
    assert res["a1"]["n_over_1p5"] == 0


def test_a2_under_estimate_side(tmp_path):
    res = rm.compute(*basic_run(tmp_path))
    # B is 1.0 m below its top at the lock and at its dose; A never is
    assert res["a2"]["worst_dh"] == pytest.approx(-1.0)
    assert res["a2"]["share_below_0p3"] == pytest.approx(0.5)     # 1 of 2 matched entries
    assert res["a2"]["n_entries"] == 2


def test_a3_far_near_split(tmp_path):
    res = rm.compute(*basic_run(tmp_path))
    e = by_id(res)
    assert abs(e[7]["first_pub_offset_m"]) == pytest.approx(6.0)
    assert abs(e[8]["first_pub_offset_m"]) == pytest.approx(3.0)
    far, near = res["a3"]["far"], res["a3"]["near"]
    assert far["n"] == 1 and near["n"] == 1
    assert far["mean_dc_at_dose"] == pytest.approx(0.2)
    assert near["mean_dc_at_dose"] == pytest.approx(0.4)
    assert far["share_over_0p8"] == 0.0
    # B's dose centre 0.9 m off: near, over 0.8
    res = rm.compute(*basic_run(tmp_path, b_dose_c=(15.9, 3.0), name="run2"))
    assert res["a3"]["near"]["share_over_0p8"] == 1.0
    assert res["a3"]["near"]["mean_dc_at_dose"] == pytest.approx(0.9)


def test_a3_split_is_at_five_metres(tmp_path):
    truth = [truth_tree(1, 5.0, 5.0), truth_tree(2, 15.0, 4.99)]
    snaps = [(3.0, [(7, 5.0, 5.0, 4.0)]), (8.0, [(7, 5.0, 5.0, 4.0), (8, 15.0, 4.99, 4.0)])]
    res = rm.compute(*write_run(tmp_path, truth, snaps, PHASES, TARGETS, track(0, 26)))
    e = by_id(res)
    assert abs(e[7]["first_pub_offset_m"]) == pytest.approx(5.0)
    assert abs(e[8]["first_pub_offset_m"]) == pytest.approx(4.99)
    assert res["a3"]["far"]["n"] == 1 and res["a3"]["near"]["n"] == 1


def test_unmatched_entry_excluded(tmp_path):
    # entry 9 sits 3 m from A's centre, outside its outline and > 2 m from every truth centre
    res = rm.compute(*basic_run(tmp_path, extra=[(9, 5.0, 9.0, 4.0)]))
    assert 9 not in by_id(res)
    assert sorted(by_id(res)) == [7, 8]
    assert res["unmatched_entries"] == [9]


def test_identity_is_the_first_match_and_survives_a_later_drift(tmp_path):
    # entry 7 first sits on A, then its centre drifts 2.3 m away (outside the 2 m match radius): still A's entry,
    # and its dose error is the full 2.3 m (a pure match at the dose would drop it)
    snaps = [(3.0, [(7, 5.5, 6.0, 4.0)]), (8.0, [(7, 5.0, 8.3, 4.0)]), (12.0, [(7, 5.0, 8.3, 4.0)])]
    res = rm.compute(*write_run(tmp_path, TRUTH, snaps, PHASES, TARGETS, track(0, 26)))
    e = by_id(res)[7]
    assert e["truth_id"] == 1
    assert e["lock"]["dc"] == pytest.approx(2.3)
    assert e["doses"][0]["dc"] == pytest.approx(2.3)


def test_undefined_height_is_none_not_an_error(tmp_path):
    snaps = [(3.0, [(7, 5.2, 6.0, None)]), (12.0, [(7, 5.2, 6.0, None)])]
    res = rm.compute(*write_run(tmp_path, TRUTH, snaps, PHASES, TARGETS, track(0, 26)))
    e = by_id(res)[7]
    assert e["lock"]["dh"] is None and e["lock"]["dc"] == pytest.approx(0.2)
    assert e["doses"][0]["dh"] is None
    assert res["a1"]["max_dh_at_dose"] is None


# ----------------------------------------------------------------------------- the lock rule (corridor mode)

def corridor_run(tmp_path, name="corr"):
    """Two corridors: PLANNING at 10 then PLANNING at 30. Entry 7 (truth 1) published at 3 and dosed at 14; entry 8
    (truth 2) published at 22 (after the first lock) and dosed at 34; entry 10 (truth 3) published at 25, never
    dosed; entry 11 (truth 4) published at 33, after the last PLANNING, never dosed; entry 12 (truth 5) published at 5,
    never dosed. Each snapshot carries a distinct height so the test can tell which snapshot a value came from."""
    truth = [truth_tree(i, 5.0 + 10.0 * i, 6.0) for i in range(1, 6)]
    x = lambda i: 5.0 + 10.0 * i
    snaps = [
        (3.0, [(7, x(1), 6.0, 4.1)]),
        (5.0, [(7, x(1), 6.0, 4.2), (12, x(5), 6.0, 5.1)]),
        (9.0, [(7, x(1), 6.0, 4.3), (12, x(5), 6.0, 5.2)]),                        # lock 1 (PLANNING at 10) sees 7 and 12
        (12.0, [(7, x(1), 6.0, 4.4), (12, x(5), 6.0, 5.3)]),
        (22.0, [(7, x(1), 6.0, 4.5), (12, x(5), 6.0, 5.4), (8, x(2), 6.0, 4.6)]),
        (25.0, [(7, x(1), 6.0, 4.5), (12, x(5), 6.0, 5.5), (8, x(2), 6.0, 4.7), (10, x(3), 6.0, 4.8)]),
        (29.0, [(7, x(1), 6.0, 4.5), (12, x(5), 6.0, 5.6), (8, x(2), 6.0, 4.9), (10, x(3), 6.0, 5.0)]),   # lock 2 (30)
        (33.0, [(7, x(1), 6.0, 4.5), (12, x(5), 6.0, 5.7), (8, x(2), 6.0, 5.2), (10, x(3), 6.0, 5.1),
                (11, x(4), 6.0, 4.0)]),
    ]
    phases = [[0.0, "SCAN"], [10.0, "PLANNING"], [11.0, "HOPPING"], [14.0, "SPRAYING"], [18.0, "HOPPING"],
              [20.0, "SCAN"], [30.0, "PLANNING"], [31.0, "HOPPING"], [34.0, "SPRAYING"], [38.0, "HOPPING"]]
    targets = [[0.0, -1], [11.0, 7], [18.0, -1], [31.0, 8], [38.0, -1]]
    return write_run(tmp_path, truth, snaps, phases, targets, track(0, 40), name)


def test_lock_is_the_last_planning_before_the_first_dose(tmp_path):
    res = rm.compute(*corridor_run(tmp_path))
    e = by_id(res)
    # entry 8 is dosed at 34: its lock is the PLANNING at 30 (snapshot 29.0, height 4.9), not the first one at 10
    assert e[8]["lock"]["dh"] == pytest.approx(4.9 - 4.0)
    assert e[8]["lock"]["t"] == pytest.approx(30.0)
    assert e[7]["lock"]["dh"] == pytest.approx(4.3 - 4.0)
    assert e[7]["lock"]["t"] == pytest.approx(10.0)


def test_lock_of_an_undosed_entry_is_the_first_planning_after_its_publication(tmp_path):
    res = rm.compute(*corridor_run(tmp_path))
    e = by_id(res)
    assert e[10]["lock"]["t"] == pytest.approx(30.0)               # published 25 -> next PLANNING is 30
    assert e[10]["lock"]["dh"] == pytest.approx(5.0 - 4.0)         # snapshot 29.0
    assert e[12]["lock"]["t"] == pytest.approx(10.0)               # published 5 -> PLANNING at 10
    assert e[12]["lock"]["dh"] == pytest.approx(5.2 - 4.0)
    assert e[10]["doses"] == [] and e[12]["doses"] == []


def test_entry_published_after_the_last_planning_has_no_lock(tmp_path):
    res = rm.compute(*corridor_run(tmp_path))
    e = by_id(res)[11]
    assert e["lock"]["dh"] is None and e["lock"]["dc"] is None and e["lock"]["t"] is None
    # ... and is left out of the lock summaries: the largest lock dh is entry 12's 1.2 (5.2 - 4.0)
    assert res["a1"]["max_dh_at_lock"] == pytest.approx(1.2)


def test_field_first_has_the_single_lock(tmp_path):
    # one PLANNING: every entry (dosed or not) locks there
    snaps = [(3.0, [(7, 5.0, 6.0, 4.1)]), (9.0, [(7, 5.0, 6.0, 4.3), (8, 15.0, 3.0, 4.2)]),
             (12.0, [(7, 5.0, 6.0, 4.9), (8, 15.0, 3.0, 4.9)])]
    res = rm.compute(*write_run(tmp_path, TRUTH, snaps, PHASES, TARGETS, track(0, 26)))
    e = by_id(res)
    assert e[7]["lock"]["t"] == pytest.approx(10.0) and e[8]["lock"]["t"] == pytest.approx(10.0)
    assert e[7]["lock"]["dh"] == pytest.approx(0.3) and e[8]["lock"]["dh"] == pytest.approx(0.2)


# ----------------------------------------------------------------------------- inputs

def test_targets_file_replaces_the_missions_target_timeline(tmp_path):
    # a replay map has its own ids: the replay's --out-targets timeline names the entries that were dosed
    paths = basic_run(tmp_path)
    tpath = tmp_path / "targets.json"
    tpath.write_text(json.dumps([[0.0, -1], [11.0, 8], [18.0, 7], [24.0, -1]]))      # swapped
    res = rm.compute(*paths, targets_json=str(tpath))
    e = by_id(res)
    assert e[8]["doses"][0]["t"] == pytest.approx(14.0)
    assert e[7]["doses"][0]["t"] == pytest.approx(20.0)
    # a replay target timeline that never resolved the first dose: it has no entry, and the count says so
    tpath.write_text(json.dumps([[0.0, -1], [18.0, 8], [24.0, -1]]))
    res = rm.compute(*paths, targets_json=str(tpath))
    assert res["n_dose_episodes"] == 2 and res["n_dose_episodes_without_target"] == 1
    assert res["n_dose_episodes_attached"] == 1 and res["n_dose_episodes_unattached"] == 0
    assert by_id(res)[7]["doses"] == []


def test_truth_file_in_contract_form_uses_height_key(tmp_path):
    truth = [truth_tree(1, 5.0, 6.0, key="height"), truth_tree(2, 15.0, 3.0, key="height")]
    d = tmp_path / "c"
    d.mkdir()
    paths = basic_run(tmp_path, name="base")
    (d / "gt.json").write_text(json.dumps({"frame": "world", "trees": truth}))
    res = rm.compute(paths[0], str(d / "gt.json"), paths[2])
    assert by_id(res)[7]["lock"]["dh"] == pytest.approx(0.5)


def test_heading_override_sets_the_across_track_axis(tmp_path):
    paths = basic_run(tmp_path)
    # across axis = (-sin h, cos h); h = pi/2 makes it -x: A (x = 5) is now 5 - drone x away, drone x = 3 at t = 3
    res = rm.compute(*paths, heading_rad=1.5707963267948966)
    assert by_id(res)[7]["first_pub_offset_m"] == pytest.approx(-(5.0 - 3.0))


def test_run_json_next_to_the_mission_gives_the_heading(tmp_path):
    paths = basic_run(tmp_path)
    (tmp_path / "run" / "run.json").write_text(json.dumps({"spec": {"rows": {"heading_rad": 1.5707963267948966}}}))
    assert by_id(rm.compute(*paths))[7]["first_pub_offset_m"] == pytest.approx(-2.0)


def test_no_poses_gives_unknown_offset(tmp_path):
    snaps = [(3.0, [(7, 5.2, 6.0, 4.5)])]
    res = rm.compute(*write_run(tmp_path, TRUTH, snaps, PHASES, TARGETS, []))
    assert by_id(res)[7]["first_pub_offset_m"] is None
    assert res["a3"]["far"]["n"] == 0 and res["a3"]["near"]["n"] == 0


# ----------------------------------------------------------------------------- every dose episode is accounted for

def test_every_dose_episode_is_attached_without_target_or_unattached(tmp_path):
    res = rm.compute(*basic_run(tmp_path))
    assert (res["n_dose_episodes"], res["n_dose_episodes_attached"], res["n_dose_episodes_without_target"],
            res["n_dose_episodes_unattached"]) == (2, 2, 0, 0)
    assert res["unattached_share"] == 0.0 and rm.problems(res) == []


def test_target_id_absent_from_the_map_is_unattached(tmp_path):
    paths = basic_run(tmp_path)
    tpath = tmp_path / "t.json"
    tpath.write_text(json.dumps([[0.0, -1], [11.0, 99], [18.0, 8], [24.0, -1]]))      # 99 is in no snapshot
    res = rm.compute(*paths, targets_json=str(tpath))
    assert res["n_dose_episodes_unattached"] == 1 and res["n_dose_episodes_attached"] == 1
    assert res["unattached_breakdown"] == {"id_not_in_map": 1, "entry_unmatched": 0, "entry_not_in_snapshot": 0}
    assert res["unattached_share"] == pytest.approx(0.5)


def test_target_entry_unmatched_to_truth_and_entry_missing_from_the_snapshot_are_unattached(tmp_path):
    # entry 9 (3 m off every truth) is targeted at 14; entry 8 is targeted at 20 but gone from the snapshots <= 20
    snaps = [(3.0, [(7, 5.2, 6.0, 4.5), (9, 5.0, 9.0, 4.0), (8, 15.0, 3.0, 4.0)]), (12.0, [(7, 5.2, 6.0, 4.5), (9, 5.0, 9.0, 4.0)])]
    tpath = tmp_path / "t.json"
    tpath.write_text(json.dumps([[0.0, -1], [11.0, 9], [18.0, 8], [24.0, -1]]))
    res = rm.compute(*write_run(tmp_path, TRUTH, snaps, PHASES, TARGETS, track(0, 26)), targets_json=str(tpath))
    assert res["unattached_breakdown"] == {"id_not_in_map": 0, "entry_unmatched": 1, "entry_not_in_snapshot": 1}
    assert res["n_dose_episodes_unattached"] == 2 and res["n_dose_episodes_attached"] == 0


def test_unattached_share_over_the_threshold_is_flagged(tmp_path):
    paths = basic_run(tmp_path)
    tpath = tmp_path / "t.json"
    tpath.write_text(json.dumps([[0.0, -1], [11.0, 98], [18.0, 99], [24.0, -1]]))     # a replay scored with foreign ids
    res = rm.compute(*paths, targets_json=str(tpath))
    assert res["unattached_share"] == 1.0
    msgs = rm.problems(res, replay=True)
    assert len(msgs) == 1 and "2 of 2" in msgs[0] and "--out-targets" in msgs[0]
    # at the threshold it is fine, above it is not
    ok = dict(res, unattached_share=rm.UNATTACHED_MAX_SHARE)
    assert rm.problems(ok) == []
    assert rm.problems(dict(ok, unattached_share=rm.UNATTACHED_MAX_SHARE + 1e-9)) != []
    # the CLI writes the problem into its output and exits 3
    out_json = str(tmp_path / "bad.json")
    assert rm.main(["--map", paths[0], "--truth", paths[1], "--mission", paths[2], "--targets", str(tpath),
                    "--replay", "--json", out_json, "--md", str(tmp_path / "bad.md")]) == 3
    assert json.load(open(out_json))["problems"]
    assert "NOT TRUSTWORTHY" in open(tmp_path / "bad.md").read()


def test_replay_without_targets_fails(tmp_path, capsys):
    paths = basic_run(tmp_path)
    with pytest.raises(ValueError, match="out-targets"):
        rm.compute(*paths, replay=True)
    with pytest.raises(SystemExit) as ex:
        rm.main(["--map", paths[0], "--truth", paths[1], "--mission", paths[2], "--replay"])
    assert ex.value.code == 2
    assert "--out-targets" in capsys.readouterr().err
    assert rm.compute(*paths)["a1"]["n_over_1p5"] == 0          # closed loop: no targets file needed


# ----------------------------------------------------------------------------- report

def test_compare_and_cli(tmp_path, capsys):
    before = rm.compute(*basic_run(tmp_path, a_dose_h=6.0, name="b"))
    after = rm.compute(*basic_run(tmp_path, a_dose_h=4.5, name="a"))
    md = rm.compare(before, after)
    assert "max dh at dose" in md and "2.000" in md and "0.500" in md
    assert "n over 1.5 m" in md
    assert "A2 entries (denominator) | 2 | 2 | +0" in md
    m, t, j = basic_run(tmp_path, name="cli")
    out_json, out_md = str(tmp_path / "o.json"), str(tmp_path / "o.md")
    assert rm.main(["--map", m, "--truth", t, "--mission", j, "--json", out_json, "--md", out_md]) == 0
    assert json.load(open(out_json))["a1"]["n_over_1p5"] == 0
    assert "A1" in open(out_md).read()
