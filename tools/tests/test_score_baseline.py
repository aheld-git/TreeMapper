"""Pins the existing system's baseline scorecard (recorded Isaac mission 2026-10-23-15-34-29).

Every later architecture decision is judged against this bar, so a scorer change that moves
any of these numbers must be deliberate: update the pin, results/baseline_existing_system*,
and docs/DECISIONS.md together. Skipped when the extracted recording is not mounted.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from treemapper_tools.contract import read_ground_truth, read_mission, read_snapshots
from treemapper_tools.score import score

E = os.environ.get("TM_EXTRACTED", "/data/work/extracted/isaac_2026-10-23-15-34-29")
NEEDED = ("map_existing.jsonl", "ground_truth.json", "mission.json", "camera_info.json")


@pytest.fixture(scope="module")
def res():
    if not all(os.path.isfile(os.path.join(E, f)) for f in NEEDED):
        pytest.skip("extracted Isaac recording not available at %s" % E)
    import json
    with open(os.path.join(E, "camera_info.json")) as f:
        cam = json.load(f)
    return score(read_snapshots(os.path.join(E, "map_existing.jsonl")), read_ground_truth(os.path.join(E, "ground_truth.json")),
                 read_mission(os.path.join(E, "mission.json")), cam)


def test_baseline_final_map(res):
    d, g = res["final"]["detection"], res["final"]["geometry"]
    assert (d["n_truth"], d["n_published"], d["matched"]) == (9, 7, 7)
    assert d["missed_ids"] == [0, 3] and d["merged_entry_ids"] == {"7": [3, 4], "1": [0, 1]}
    assert d["recall"] == pytest.approx(7 / 9) and d["precision"] == pytest.approx(1.0)
    assert g["mean_iou"] == pytest.approx(0.629, abs=0.002)
    assert g["centre_rmse_m"] == pytest.approx(1.355, abs=0.005)
    assert g["height_rmse_m"] == pytest.approx(1.460, abs=0.005)
    assert g["spill"] == pytest.approx(0.164, abs=0.002)
    assert res["gt_height_reliable"] is False          # constant 4.0 m fallback truth


def test_baseline_per_frame(res):
    pf = res["per_frame"]
    assert pf["n_samples"] == 880
    assert pf["detection"]["recall"]["mean"] == pytest.approx(0.580, abs=0.003)
    assert pf["detection"]["recall_in_view"]["mean"] == pytest.approx(0.710, abs=0.003)
    assert pf["geometry"]["mean_iou"]["mean"] == pytest.approx(0.596, abs=0.003)


def test_baseline_safety(res):
    sa = res["safety"]
    assert sa["total_violations"] == 5
    assert (sa["undefined_height_frames"], sa["invalid_entry_frames"], sa["duplicate_id_frames"],
            sa["implausible_height_frames"], sa["new_trees_after_scan"], sa["new_tree_inside_sprayed"]) == (0,) * 6
    assert sa["sprayed_tree_changes"] == 1 and sa["sprayed_tree_change_events"][0]["id"] == 7
    assert sa["locked_id_vanished"] == 1 and sa["locked_id_vanished_events"][0]["id"] == 8
    assert sa["nontarget_changes_in_descent"] == 3
    ev = sa["nontarget_change_events"]
    assert [(e["id"], e["window"]) for e in ev] == [(7, "climb"), (1, "climb"), (1, "climb")]
    assert ev[0]["t"] == pytest.approx(145.976, abs=0.01)
    assert sa["descent_frames_by_kind"] == {"approach": 230, "spraying": 140, "climb": 183}


def test_baseline_stability_spray_timeliness(res):
    st, sp, ti = res["stability"], res["spray"], res["timeliness"]
    assert (st["id_switches"], st["id_switches_after_lock"], st["id_reuse"], st["geometry_retreats"]) == (1, 0, 0, 0)
    assert (st["centre_jumps"], st["height_jumps"]) == (8, 19)
    assert st["max_height_drift_since_lock_m"] == pytest.approx(4.21, abs=0.01)
    assert st["ids_vanished_while_held_by_navigation"] == [8]
    assert (sp["truth_trees_sprayed"], sp["truth_trees_sprayed_twice"], sp["orphan_sprayed_entries"]) == (7, 0, 0)
    assert ti["trees_never_published"] == [] and ti["negative_latency_trees"] == []
    assert ti["latency_scan_mean_s"] == pytest.approx(1.19, abs=0.02)
    assert ti["latency_mean_s"] == pytest.approx(13.39, abs=0.05)
