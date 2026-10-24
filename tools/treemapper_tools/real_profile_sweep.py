"""Small configuration sweep of a mapper on the real flight, judged with no-ground-truth criteria only.

    python3 -m treemapper_tools.real_profile_sweep --dataset /data/work/extracted/real_farm \
        --replay /data/work/eval/m4c/build_960ccb7/treemapper_replay --system cand_obj \
        --crowns results/real_flight_crowns.json --out /data/work/eval/m4c/sweep --out-json results/real_flight_profile_sweep.json

Per configuration (``--config`` strings in CONFIGS, or ``--configs-file``):
* crown correspondence with the stereo crown peaks (``real_score.crown_correspondence``): outlines without a
  peak (fragments / false trees; target ~0), with 2+ peaks (merged), peaks in no outline (missed);
* fragment areas (outlines without a peak) and the outline area distribution;
* stability (no GT): centre / height jumps, max centre drift from first, IDs vanished, retreats;
* safety counts (undefined / invalid / duplicate; implausible split as in real_score);
* agreement: final trees overlapping a final tree of each reference map (other systems, not truth).
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np

from .contract import read_mission, read_snapshots
from . import score as scoring
from .real_score import agreement, crown_correspondence, poly_area, pq, run_replay

CONFIGS = [
    ("default", ""),
    ("prom1.0", "min_prominence=1.0"),
    ("prom1.5", "min_prominence=1.5"),
    ("basin8", "min_basin_cells=8"),
    ("basin12", "min_basin_cells=12"),
    ("prom1.0_basin8", "min_prominence=1.0;min_basin_cells=8"),
    ("prom1.0_basin8_r3", "min_prominence=1.0;min_basin_cells=8;match_radius=3.0;merge_centre_dist=2.5"),
    ("prom1.5_basin12_r3", "min_prominence=1.5;min_basin_cells=12;match_radius=3.0;merge_centre_dist=2.5"),
    ("prom1.0_basin8_r3_a6", "min_prominence=1.0;min_basin_cells=8;match_radius=3.0;merge_centre_dist=2.5;min_publish_area=6.0"),
    ("prom1.0_basin8_r3_a8", "min_prominence=1.0;min_basin_cells=8;match_radius=3.0;merge_centre_dist=2.5;min_publish_area=8.0"),
    ("prom1.5_basin8_r3_a8", "min_prominence=1.5;min_basin_cells=8;match_radius=3.0;merge_centre_dist=2.5;min_publish_area=8.0"),
    ("basin12_r3_a8", "min_basin_cells=12;match_radius=3.0;merge_centre_dist=2.5;min_publish_area=8.0"),
]

# round 2 (after round 1 left >= 16 peakless outlines): instance-level knobs on top of round 1's best
_P = "min_prominence=1.5;min_basin_cells=8;match_radius=3.0;merge_centre_dist=2.5;min_publish_area=8.0"
CONFIGS2 = [
    ("P_mergeov0.3", _P + ";merge_overlap=0.3"),
    ("P_noseedtrunc", _P + ";seed_truncated=false"),
    ("P_seedov0.1", _P + ";seed_overlap=0.1"),
    ("P_cell0.6", _P + ";cell=0.6"),
    ("P_all", _P + ";merge_overlap=0.3;seed_truncated=false;seed_overlap=0.1;match_reach=4.5"),
    ("P1.0_all", _P.replace("min_prominence=1.5", "min_prominence=1.0")
     + ";merge_overlap=0.3;seed_truncated=false;seed_overlap=0.1;match_reach=4.5"),
]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--replay", required=True)
    ap.add_argument("--system", default="cand_obj")
    ap.add_argument("--crowns", required=True, help="real_flight_crowns.json (crown peaks)")
    ap.add_argument("--refs", default="", help="label=map.jsonl,... reference maps for the agreement column")
    ap.add_argument("--only", default="", help="comma list of config labels to run")
    ap.add_argument("--round", type=int, default=1, help="1 = CONFIGS, 2 = CONFIGS2")
    ap.add_argument("--out", required=True)
    ap.add_argument("--out-json")
    a = ap.parse_args(argv)
    crowns = [tuple(c) for c in json.load(open(a.crowns))["crowns"]]
    mission = read_mission(os.path.join(a.dataset, "mission.json"))
    refs = {}
    for kv in [x for x in a.refs.split(",") if x]:
        k, p = kv.split("=", 1)
        refs[k] = read_snapshots(p)[-1].trees
    rows = []
    for label, cfg in (CONFIGS if a.round == 1 else CONFIGS2):
        if a.only and label not in a.only.split(","):
            continue
        rd = os.path.join(a.out, label)
        info = run_replay(a.replay, a.dataset, a.system, rd, cfg)
        snaps = read_snapshots(os.path.join(rd, "map.jsonl"))
        res = scoring.score(snaps, None, mission, None)
        fin = snaps[-1].trees
        cc = crown_correspondence(crowns, fin)
        from shapely.geometry import Point, Polygon
        frag = [poly_area(t.outline) for t in fin if len(t.outline) >= 3 and
                not any(Polygon(t.outline).buffer(0).contains(Point(c[0], c[1])) for c in crowns)]
        st, sf = res["stability"], res["safety"]
        ag = agreement(dict([("x", fin)] + list(refs.items())))["found_by_other"] if refs else {}
        r = {"label": label, "config": cfg, "final_trees": len(fin), "ids_ever": len(res["ids_ever_published"]),
             **{k: cc[k] for k in ("outlines_with_no_peak", "outlines_with_one_peak", "outlines_with_2plus_peaks",
                                   "peaks_in_exactly_one", "peaks_in_none", "peaks_in_2plus", "nearest_centre_m_p10_p50")},
             "fragment_area_m2_p50_max": pq(frag, (50, 100)) if frag else None,
             "area_m2_p10_p50_p90": pq([poly_area(t.outline) for t in fin]),
             "centre_jumps": st["centre_jumps"], "height_jumps": st["height_jumps"],
             "max_centre_drift_m": st["max_centre_drift_from_first_m"], "ids_vanished": len(st["ids_vanished"]),
             "retreats": st["geometry_retreats"], "id_reuse": st["id_reuse"],
             "undefined_height_frames": sf["undefined_height_frames"], "invalid_entry_frames": sf["invalid_entry_frames"],
             "duplicate_id_frames": sf["duplicate_id_frames"], "implausible_height_frames": sf["implausible_height_frames"],
             "agreement_with_refs": {k[len("x_in_"):]: v for k, v in ag.items() if k.startswith("x_in_")},
             "cost_ms": {k: round(v["mean_ms"], 3) for k, v in res["cost"].get("stages", {}).items()},
             "run_seconds": info["run_seconds"]}
        rows.append(r)
        print(json.dumps(r), flush=True)
    if a.out_json:
        with open(a.out_json, "w") as f:
            json.dump(rows, f, indent=1)


if __name__ == "__main__":
    main()
