"""Comparison report of Isaac closed-loop batches (Task 16): strategy vs strategy per spec, and why trees were missed.

    python3 -m treemapper_tools.sim_compare_report <batch dir> [<batch dir> ...] [--out report.md] [--json misses.json]

A batch directory is what ``scripts/sim_mission.sh --out DIR`` writes: ``run_1 .. run_N``, each scored already
(``mission_score.json``). Nothing is re-scored and no bag is read; per run it reads ``mission_score.json``,
``run.json``, ``truth.json``, ``mission.json``, ``map.jsonl`` (written by the scorer) and ``corridor_nav.cfg``.

Statistics per (spec, strategy): mean over runs, worst case in the bad direction, spread (max - min); the scorer's
metrics come from ``mission_score.compare_runs`` (same rules: incomplete runs are left out of the mission-wide
metrics), the further metrics below use the same definitions. Strategy A vs B: a difference of the means no larger
than the larger of the two spreads (and than 1 % of the larger mean) is "no measurable difference" (the noise floor
with 3 runs per side).

Miss analysis: every truth tree the scorer counts as missed (< 50 % of every episode's intended dose) gets one primary
class, in this precedence:
  d  navigator skip: no episode aimed at the tree, and a navigator counter explains it (``target_height_implausible``:
     a map entry of the tree was published above home + expected_canopy_top_max + target_height_margin;
     ``trees_skipped`` / ``ids_dropped_vanished``: named as probable);
  b  split / fragment / merged: the tree was ever split into >= 2 map entries or inside a merged entry;
  c  phantom: the only dose meant for the tree went to an entry that is not a displaced copy of it (none so far);
  a  the aimed map entry's centre (at the episode start) was further from the truth centre than the nominal spray
     footprint radius ``r_fp = (spray_clearance - nozzle drop) * tan(cone half-angle)``;
  e  other, with the reason: height over-estimate (the nozzle so high above the true top that the actual footprint is
     wider than the crown), never published, published but never targeted, or unexplained.
An episode the scorer could not match to a truth tree (its entry's centre outside every canopy and > 2 m from every
truth centre) counts as aimed at the nearest truth tree when it lies within that tree's crown radius + r_fp + 0.5 m,
else as a phantom episode.

Also emitted: the dose aim accuracy (drone's mean xy during each dose vs the centre of the map entry the navigator
aimed at: the navigator's accuracy, not the map's), the episodes beyond r_fp that still got >= 50 %, the height
over-estimates > 1.5 m with their along-track position, and (``--trace RUN_DIR:ID,...``) published-height traces of
chosen map entries. Cost and flight-description counters (``INFO_ONLY``) get an "info only" verdict.
"""
from __future__ import annotations

import argparse
import bisect
import glob
import json
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from treemapper_tools import mission_score as ms

MISS_FRAC = ms.SPRAYED_FRAC
REL_RESOLUTION = 0.01          # the noise floor is never below 1 % of the larger mean (RTF 0.9999997 vs 0.9999999)
HEIGHT_OVER_M = 1.0            # published height - truth top above this is reported as a contributing over-estimate
NEAR_EXTRA_M = 0.5             # unmatched episode -> nearest truth tree if within crown radius + r_fp + this
CLASSES = ("a", "b", "c", "d", "e")
CLASS_LABEL = {"a": "map centre error > spray footprint", "b": "split / fragment / merged entry",
               "c": "phantom took the dose", "d": "navigator skip", "e": "other"}

# extra metric -> (label, group, bad direction, getter on a mission_score card); "max" = larger is worse, "min" =
# smaller is worse, "absmax" = further from 0 is worse (a bias)
def _g(*path):
    def get(card):
        v = card
        for k in path:
            if not isinstance(v, dict) or k not in v:
                return None
            v = v[k]
        return v
    return get


def _count(key):
    return lambda c: (c.get("guards", {}).get("counters", {}) or {}).get(key, 0)


EXTRA_METRICS = [
    ("glide_climbs_inserted", "Glide climbs inserted", "efficiency", "max", _g("efficiency", "glide_climbs_inserted")),
    ("t_scan", "Time in scan (s)", "efficiency", "max", _g("efficiency", "time_split", "groups", "scan")),
    ("t_settle", "Time in settle (s)", "efficiency", "max", _g("efficiency", "time_split", "groups", "settle")),
    ("t_transit", "Time in transit (s)", "efficiency", "max", _g("efficiency", "time_split", "groups", "transit")),
    ("t_glide", "Time in glide (s)", "efficiency", "max", _g("efficiency", "time_split", "groups", "glide")),
    ("t_dose", "Time in dose (s)", "efficiency", "max", _g("efficiency", "time_split", "groups", "dose")),
    ("t_hold", "Time in hold (s)", "efficiency", "max", _g("efficiency", "time_split", "groups", "hold")),
    ("dose_episodes", "Dose episodes", "spray", "max", _g("efficiency", "dose_episodes")),
    ("unmatched_events", "Episodes without a matched truth tree", "spray", "max", _g("spray", "unmatched_events")),
    ("ground_l", "Litres on the ground", "spray", "max", _g("spray", "ground_l")),
    ("nav_trees_skipped", "Trees skipped (navigator)", "spray", "max", _g("spray", "nav_trees_skipped")),
    ("completed", "Completed (1 = DONE, no abort / hold timeout / ERROR)", "safety", "min",
     lambda c: 1 if c.get("completed") else 0),
    ("all_pass", "All safety rows pass (1 = yes)", "safety", "min", lambda c: 1 if c.get("all_pass") else 0),
    ("targeted_again", "Sprayed tree targeted again (episodes)", "safety", "max",
     _g("safety", "sprayed_tree_targeted_again")),
    ("guard_triggers", "Clearance-guard triggers", "guards", "max", _g("guards", "guard_triggers")),
    ("guard_blind", "Guard-blind counter", "guards", "max", _g("guards", "guard_blind")),
    ("holds", "Holds (all reasons)", "guards", "max", lambda c: sum((c.get("guards", {}).get("holds") or {}).values())),
    ("ids_reresolved", "IDs re-resolved", "guards", "max", _g("guards", "ids_reresolved")),
    ("ids_dropped_vanished", "IDs dropped (vanished)", "guards", "max", _g("guards", "ids_dropped_vanished")),
    ("late_trees", "Late trees (cleanup list)", "guards", "max", _g("guards", "late_trees")),
    ("target_height_implausible", "Targets skipped as implausibly high", "guards", "max",
     _count("target_height_implausible")),
    ("spray_point_raised", "Spray points raised (split neighbour)", "guards", "max", _count("spray_point_raised")),
    ("recall", "Map recall (final)", "map", "min", _g("map", "final", "detection", "recall")),
    ("precision", "Map precision (final)", "map", "min", _g("map", "final", "detection", "precision")),
    ("split_trees", "Split trees (final)", "map", "max", _g("map", "final", "detection", "split_trees")),
    ("false_trees", "False trees (final)", "map", "max", _g("map", "final", "detection", "false_trees")),
    ("mean_iou", "Mean IoU (final)", "map", "min", _g("map", "final", "geometry", "mean_iou")),
    ("centre_rmse_m", "Centre RMSE (m, final)", "map", "max", _g("map", "final", "geometry", "centre_rmse_m")),
    ("centre_max_m", "Centre error max (m, final)", "map", "max", _g("map", "final", "geometry", "centre_max_m")),
    ("height_bias_m", "Height bias (m, final)", "map", "absmax", _g("map", "final", "geometry", "height_bias_m")),
    ("height_rmse_m", "Height RMSE (m, final)", "map", "max", _g("map", "final", "geometry", "height_rmse_m")),
    ("spill", "Spill outside all canopies (final)", "map", "max", _g("map", "final", "geometry", "spill")),
    ("height_drift_lock", "Max height drift per ID since lock (m)", "map", "max",
     _g("map", "stability", "max_height_drift_since_lock_m")),
    ("implausible_height_frames", "Frames with an implausible height", "map", "max",
     _g("map", "safety", "implausible_height_frames")),
    ("map_violations", "Map safety violations (total)", "map", "max", _g("map", "safety", "total_violations")),
]

# scorer metric -> group, for the table layout
STD_GROUP = {"mission_time_s": "efficiency", "path_length_m": "efficiency", "seconds_per_sprayed_tree": "efficiency",
             "litres_emitted": "efficiency", "n_sprayed": "spray", "n_missed": "spray", "n_double_sprayed": "spray",
             "on_target_fraction": "spray", "off_target_l": "spray", "clearance_min_m": "safety",
             "nozzle_outside_dose": "safety", "step_mean_ms": "cost", "step_p99_ms": "cost", "sgbm_mean_ms": "cost",
             "detector_mean_ms": "cost", "rtf": "cost"}
GROUP_ORDER = ("efficiency", "spray", "safety", "guards", "map", "cost")
# metrics whose difference is reported but is not a quality finding (cost, and counters that only describe how the
# mission was flown): their verdict reads "<strategy> lower / higher (info only)"
INFO_ONLY = {"step_mean_ms", "step_p99_ms", "sgbm_mean_ms", "detector_mean_ms", "rtf", "glide_climbs_inserted",
             "dose_episodes", "guard_blind", "late_trees", "spray_point_raised"}
HEIGHT_OVER_BIG_M = 1.5        # published height - truth top above this: counted separately, with its along-track range


# ----------------------------------------------------------------------------- loading

def read_cfg(path: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for line in f:
            body = line.split("#", 1)[0]
            if "=" in body:
                k, v = (x.strip() for x in body.split("=", 1))
                out[k] = v
    return out


def run_dirs(batch_dir: str) -> List[str]:
    ds = [d for d in glob.glob(os.path.join(batch_dir, "run_*")) if os.path.exists(os.path.join(d, "mission_score.json"))]
    return sorted(ds, key=lambda d: int(d.rsplit("_", 1)[1]) if d.rsplit("_", 1)[1].isdigit() else 0)


def spec_name(run: dict) -> str:
    f = run.get("spec_file") or ""
    return os.path.splitext(os.path.basename(f))[0] if f else "spec_seed_%s" % run.get("seed")


def load_run(run_dir: str) -> dict:
    card = ms._load_json(os.path.join(run_dir, "mission_score.json"))
    run = ms._load_json(os.path.join(run_dir, "run.json"), {})
    return {"dir": run_dir, "name": os.path.basename(run_dir.rstrip("/")), "card": card, "run": run,
            "cfg": read_cfg(os.path.join(run_dir, ms.RUN_CFG)), "rtf": ms._load_json(os.path.join(run_dir, "rtf.json"), {})}


# ----------------------------------------------------------------------------- statistics

def stats(vals: Sequence[float], bad: str) -> dict:
    """Same definition as mission_score.compare_runs: mean, worst in the bad direction, spread = max - min."""
    v = [float(x) for x in vals if x is not None]
    if not v:
        return {"n": 0, "mean": None, "worst": None, "spread": None, "values": []}
    worst = max(v) if bad == "max" else (max(v, key=abs) if bad == "absmax" else min(v))
    return {"n": len(v), "mean": float(np.mean(v)), "worst": worst,
            "spread": max(v) - min(v), "values": v}


def group_stats(cards: Sequence[dict]) -> Dict[str, dict]:
    """{metric: stats} for one (spec, strategy): the scorer's COMPARE_METRICS via mission_score.compare_runs, then
    EXTRA_METRICS."""
    cmp = ms.compare_runs(cards)
    if len(cmp) != 1:
        raise ValueError("group_stats wants the runs of one strategy, got %s" % sorted(cmp))
    out = dict(next(iter(cmp.values())))
    for key, _, _, bad, get in EXTRA_METRICS:
        out[key] = stats([get(c) for c in cards], bad)
    return out


def metric_rows() -> List[Tuple[str, str, str, str]]:
    """(key, label, group, bad) of every metric in table order."""
    rows = [(k, lab, STD_GROUP[k], bad) for k, (lab, bad) in ms.COMPARE_METRICS.items()]
    rows += [(k, lab, grp, bad) for k, lab, grp, bad, _ in EXTRA_METRICS]
    return sorted(rows, key=lambda r: GROUP_ORDER.index(r[2]))


def verdict(a: dict, b: dict, bad: str, name_a: str, name_b: str) -> Tuple[Optional[float], Optional[float], str]:
    """(mean_b - mean_a, noise floor = max(spreads, 1 % of the larger |mean|), text). Not measurable when
    |diff| <= noise floor."""
    if a.get("mean") is None or b.get("mean") is None:
        return None, None, "n/a"
    d = b["mean"] - a["mean"]
    floor = max(a["spread"] or 0.0, b["spread"] or 0.0, REL_RESOLUTION * max(abs(a["mean"]), abs(b["mean"])))
    if abs(d) <= floor or abs(d) < 1e-12:
        return d, floor, "no measurable difference"
    if bad == "absmax":
        if abs(abs(b["mean"]) - abs(a["mean"])) <= floor:
            return d, floor, "no measurable difference"
        better_b = abs(b["mean"]) < abs(a["mean"])
    else:
        better_b = (d < 0) if bad == "max" else (d > 0)
    return d, floor, "%s better" % (name_b if better_b else name_a)


# ----------------------------------------------------------------------------- miss analysis: geometry

def area(outline) -> float:
    o = [(float(p[0]), float(p[1])) for p in outline]
    return abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(o, o[1:] + o[:1]))) / 2.0


def crown_radius(tree: dict) -> float:
    return math.sqrt(area(tree["outline"]) / math.pi)


def footprint_radius(height_above_top: float, half_angle_deg: float) -> float:
    return max(0.0, height_above_top) * math.tan(math.radians(half_angle_deg))


def nominal_footprint(cfg: Dict[str, str], spec: dict) -> Tuple[float, float, float]:
    """(r_fp, nozzle height above the published top, cone half-angle) at the nominal spray altitude."""
    clearance = float(cfg.get("spray_clearance", "2.5"))
    noz = spec.get("nozzle") or {}
    drop = -float((noz.get("offset_body") or [0, 0, 0])[2])
    half = float(noz.get("cone_half_angle_deg", 20.0))
    return footprint_radius(clearance - drop, half), clearance - drop, half


def nozzle_drop(spec: dict) -> float:
    return -float(((spec.get("nozzle") or {}).get("offset_body") or [0, 0, 0])[2])


class MapIndex:
    """One pass over map.jsonl: per id the first appearance (t, centre, height) and the maximum height (h, t), plus
    the latest snapshot before each query time (as {id: (centre, height)})."""

    def __init__(self, path: str, query_times: Sequence[float] = ()):
        self.first: Dict[int, Tuple[float, Tuple[float, float], Optional[float]]] = {}
        self.max_h: Dict[int, Tuple[float, float]] = {}
        self.centres: Dict[int, List[Tuple[float, float]]] = {}
        self.at: Dict[float, Dict[int, Tuple[Tuple[float, float], Optional[float]]]] = {}
        qs = sorted(set(query_times))
        qi = 0
        prev: Dict[int, Tuple[Tuple[float, float], Optional[float]]] = {}
        if not os.path.exists(path):
            return
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                d = json.loads(line)
                t = float(d["t"])
                while qi < len(qs) and qs[qi] < t:
                    self.at[qs[qi]] = prev
                    qi += 1
                cur = {}
                for tr in d.get("trees", []):
                    i = int(tr["id"])
                    c = (float(tr["centre"][0]), float(tr["centre"][1]))
                    h = tr.get("height")
                    h = float(h) if h is not None and math.isfinite(float(h)) else None
                    cur[i] = (c, h)
                    if i not in self.first:
                        self.first[i] = (t, c, h)
                    if h is not None and (i not in self.max_h or h > self.max_h[i][0]):
                        self.max_h[i] = (h, t)
                    cl = self.centres.setdefault(i, [])
                    if not cl or math.hypot(cl[-1][0] - c[0], cl[-1][1] - c[1]) > 0.25:
                        cl.append(c)
                prev = cur
        while qi < len(qs):
            self.at[qs[qi]] = prev
            qi += 1


def pose_at(poses: Sequence[Sequence[float]], times: Sequence[float], t: float) -> Optional[Sequence[float]]:
    if not poses:
        return None
    i = bisect.bisect_right(times, t)
    return poses[max(0, i - 1)]


def mean_pose(poses, times, t0: float, t1: float) -> Optional[Tuple[float, float, float]]:
    a, b = bisect.bisect_left(times, t0), bisect.bisect_right(times, t1)
    ps = poses[a:b] or ([pose_at(poses, times, t0)] if poses else [])
    if not ps:
        return None
    return (float(np.mean([p[1] for p in ps])), float(np.mean([p[2] for p in ps])), float(np.mean([p[3] for p in ps])))


def nearest_gap(tree: dict, truth: Sequence[dict]) -> float:
    """Edge-to-edge gap (centre distance - both crown radii) to the nearest other truth tree."""
    r = crown_radius(tree)
    best = math.inf
    for o in truth:
        if o["id"] == tree["id"]:
            continue
        d = math.hypot(o["centre"][0] - tree["centre"][0], o["centre"][1] - tree["centre"][1]) - r - crown_radius(o)
        best = min(best, d)
    return best


# ----------------------------------------------------------------------------- miss analysis: per run

def episode_facts(r: dict) -> List[dict]:
    """Per dose episode of a run: the aimed TreeMapper id, the truth tree it is attributed to, dose fraction on that
    tree, the entry's centre / height at the episode start, the drone during the episode, and where the drone was
    when that entry was first published."""
    card, run, cfg = r["card"], r["run"], r["cfg"]
    spec = run.get("spec") or {}
    truth = list((ms._load_json(os.path.join(r["dir"], "truth.json"), {}) or {}).get("trees", []))
    by_id = {int(t["id"]): t for t in truth}
    mis = ms._load_json(os.path.join(r["dir"], "mission.json"), {}) or {}
    phases = [(float(t), str(p)) for t, p in mis.get("phases", [])]
    targets = [(float(t), int(i)) for t, i in mis.get("targets", [])]
    poses = [tuple(float(v) for v in p) for p in mis.get("pose", [])]
    pt = [p[0] for p in poses]
    t_end = max([phases[-1][0] if phases else 0.0, pt[-1] if pt else 0.0])
    eps = ms.dose_episodes(phases, targets, [], t_end)
    ev = (card.get("spray") or {}).get("events", [])
    r["truth"], r["by_id"], r["poses"], r["pose_t"] = truth, by_id, poses, pt
    r["episodes_aligned"] = len(eps) == len(ev)
    mi = MapIndex(os.path.join(r["dir"], "map.jsonl"), [e.start for e in eps])
    r["map_index"] = mi
    r_fp, _, half = nominal_footprint(cfg, spec)
    drop = nozzle_drop(spec)
    heading = float((spec.get("rows") or {}).get("heading_rad", 0.0))
    across = (-math.sin(heading), math.cos(heading))
    along = (math.cos(heading), math.sin(heading))
    poly = (spec.get("field") or {}).get("polygon") or []
    along0 = min([p[0] * along[0] + p[1] * along[1] for p in poly]) if poly else 0.0
    t0 = (card.get("efficiency") or {}).get("time_split", {}).get("t_start") or 0.0
    mt = (card.get("efficiency") or {}).get("mission_time_s") or 1.0
    out = []
    if len(eps) != len(ev):
        return out
    for k, (e, row) in enumerate(zip(eps, ev)):
        snap = mi.at.get(e.start, {})
        ent = snap.get(e.tm_id) if e.tm_id is not None else None
        tg = row.get("target")
        matched = tg is not None
        T = by_id.get(int(tg)) if matched else None
        if T is None and ent is not None and truth:
            near = min(truth, key=lambda t: math.hypot(t["centre"][0] - ent[0][0], t["centre"][1] - ent[0][1]))
            dn = math.hypot(near["centre"][0] - ent[0][0], near["centre"][1] - ent[0][1])
            if dn <= crown_radius(near) + r_fp + NEAR_EXTRA_M:
                T = near
        f = {"k": k, "start": e.start, "end": e.end, "tm_id": e.tm_id, "truth_id": int(T["id"]) if T else None,
             "matched": matched, "phantom": T is None, "intended_l": row.get("intended_l"),
             "t_frac": (e.start - t0) / mt if mt else None}
        dp = mean_pose(poses, pt, e.start, e.end)
        if dp is not None and ent is not None:
            # dose aim error: the drone's mean xy during the dose vs the centre of the entry it aimed at (/spray_target
            # id, its centre in the latest map before the dose) - the navigator's own accuracy, independent of the map
            f["aim_err"] = math.hypot(dp[0] - ent[0][0], dp[1] - ent[0][1])
        if T is None:
            out.append(f)
            continue
        tl = row.get("trees_l") or {}
        f["frac"] = (float(tl.get(str(T["id"]), 0.0)) / row["intended_l"]) if row.get("intended_l") else 0.0
        f["crown_r"] = crown_radius(T)
        f["top"] = float(T["top_z"])
        if ent is not None:
            (cx, cy), h = ent
            ex, ey = cx - T["centre"][0], cy - T["centre"][1]
            f["map_err"] = math.hypot(ex, ey)
            f["err_across"] = ex * across[0] + ey * across[1]
            f["height_err"] = (h - f["top"]) if h is not None else None
        f["along_from_entry"] = T["centre"][0] * along[0] + T["centre"][1] * along[1] - along0
        if dp is not None:
            f["nozzle_above_top"] = dp[2] - drop - f["top"]
            f["r_fp_actual"] = footprint_radius(f["nozzle_above_top"], half)
            f["drone_truth_err"] = math.hypot(dp[0] - T["centre"][0], dp[1] - T["centre"][1])
        fi = mi.first.get(e.tm_id) if e.tm_id is not None else None
        if fi is not None:
            p = pose_at(poses, pt, fi[0])
            f["first_t"] = fi[0]
            f["first_err"] = math.hypot(fi[1][0] - T["centre"][0], fi[1][1] - T["centre"][1])
            if p is not None:
                f["first_offset"] = ((T["centre"][0] - p[1]) * across[0] + (T["centre"][1] - p[2]) * across[1])
        out.append(f)
    return out


def classify(facts: dict, r_fp: float) -> Tuple[str, str]:
    """Primary class (a..e, see the module doc) and a note, from the facts of one missed tree:
    aimed (bool), counter (str or None: the navigator counter that explains a skip), split_or_merged (bool),
    phantom_only (bool), map_err, crown_r, height_err, r_fp_actual (floats or None), published (bool)."""
    notes = []
    he, ra, cr, me = facts.get("height_err"), facts.get("r_fp_actual"), facts.get("crown_r"), facts.get("map_err")
    if he is not None and he > HEIGHT_OVER_M:
        notes.append("height +%.1f m" % he)
    if not facts.get("aimed"):
        if facts.get("counter"):
            return "d", facts["counter"]
        if facts.get("split_or_merged"):
            return "b", "never targeted; inside a split / merged entry"
        if facts.get("phantom_only"):
            return "c", "its dose went to a phantom"
        if not facts.get("published"):
            return "e", "never published by TreeMapper"
        return "e", "published, never targeted, no navigator counter explains it"
    if facts.get("split_or_merged"):
        if me is not None:
            notes.insert(0, "aimed entry %.2f m off" % me)
        return "b", "; ".join(notes)
    if me is not None and me > r_fp:
        return "a", "; ".join(["%.2f m > %.2f m" % (me, r_fp)] + notes)
    if ra is not None and cr is not None and ra > cr:
        return "e", "; ".join(["height over-estimate: footprint %.2f m > crown radius %.2f m" % (ra, cr)] + notes)
    return "e", "; ".join(["unexplained"] + notes)


def miss_rows(r: dict, facts: Sequence[dict]) -> List[dict]:
    card, cfg, spec = r["card"], r["cfg"], (r["run"].get("spec") or {})
    r_fp, _, _ = nominal_footprint(cfg, spec)
    sp = card.get("spray") or {}
    stab = ((card.get("map") or {}).get("stability") or {})
    split = set(stab.get("truth_trees_ever_split") or []) | set(stab.get("truth_trees_ever_in_a_merged_entry") or [])
    final = ((card.get("map") or {}).get("final") or {})
    per_tree = {int(p["truth_id"]): p for p in ((final.get("geometry") or {}).get("per_tree") or [])}
    counters = (card.get("guards") or {}).get("counters") or {}
    home_z = r["poses"][0][3] if r.get("poses") else 0.0
    h_lim = home_z + float(cfg.get("expected_canopy_top_max", "7")) + float(cfg.get("target_height_margin", "2"))
    mi: MapIndex = r["map_index"]
    out = []
    for tid in sp.get("missed_ids") or []:
        T = r["by_id"].get(int(tid))
        if T is None:
            continue
        aimed = [f for f in facts if f.get("truth_id") == tid and not f.get("phantom")]
        best = max(aimed, key=lambda f: f.get("frac", 0.0)) if aimed else {}
        cr = crown_radius(T)
        # entries that belonged to the tree: the final match, else any entry whose centre ever came inside its crown
        ents = set([per_tree[tid]["map_id"]] if tid in per_tree and per_tree[tid].get("map_id") is not None else [])
        for i, cl in mi.centres.items():
            if any(math.hypot(c[0] - T["centre"][0], c[1] - T["centre"][1]) <= cr for c in cl):
                ents.add(i)
        counter = None
        if not aimed:
            high = [(i, mi.max_h[i][0]) for i in sorted(ents) if i in mi.max_h and mi.max_h[i][0] > h_lim]
            if counters.get("target_height_implausible") and high:
                counter = "target_height_implausible (entry %d peaked at %.2f m > %.2f m)" % (high[0][0], high[0][1], h_lim)
            elif counters.get("trees_skipped") or sp.get("nav_trees_skipped"):
                counter = "trees_skipped (probable)"
            elif counters.get("ids_dropped_vanished"):
                counter = "ids_dropped_vanished (probable)"
        fa = {"aimed": bool(aimed), "counter": counter, "split_or_merged": tid in split, "phantom_only": False,
              "published": bool(ents), "map_err": best.get("map_err"), "crown_r": cr,
              "height_err": best.get("height_err"), "r_fp_actual": best.get("r_fp_actual")}
        cls, note = classify(fa, r_fp)
        out.append({"run": r["name"], "truth_id": tid, "class": cls, "note": note, "tm_id": best.get("tm_id"),
                    "episodes": len(aimed), "frac": best.get("frac"), "map_err": best.get("map_err"),
                    "crown_r": cr, "height_err": best.get("height_err"),
                    "nozzle_above_top": best.get("nozzle_above_top"), "r_fp_actual": best.get("r_fp_actual"),
                    "first_offset": best.get("first_offset"), "first_err": best.get("first_err"),
                    "final_iou": (per_tree.get(tid) or {}).get("iou"), "t_frac": best.get("t_frac"),
                    "gap": nearest_gap(T, r["truth"]), "r_fp": r_fp})
    return out


def pattern_stats(facts: Sequence[dict], r_fp: float, offset_split: float = 5.0) -> dict:
    """Over the aimed, attributed episodes: map error / miss share by the lateral offset of the first sighting, the
    share of large errors pointing back toward the first-sighting track, misses by mission third, and height."""
    fs = [f for f in facts if not f.get("phantom") and f.get("map_err") is not None]

    def summ(sel):
        if not sel:
            return {"n": 0}
        return {"n": len(sel), "mean_err": float(np.mean([f["map_err"] for f in sel])),
                "share_err_gt_fp": sum(f["map_err"] > r_fp for f in sel) / len(sel),
                "share_miss": sum(f.get("frac", 0.0) < MISS_FRAC for f in sel) / len(sel)}

    near = [f for f in fs if f.get("first_offset") is not None and abs(f["first_offset"]) < offset_split]
    far = [f for f in fs if f.get("first_offset") is not None and abs(f["first_offset"]) >= offset_split]
    big = [f for f in fs if f["map_err"] > r_fp and f.get("first_offset") is not None and abs(f["first_offset"]) > 0.5]
    toward = [f for f in big if f["err_across"] * f["first_offset"] < 0]
    thirds = [summ([f for f in fs if f.get("t_frac") is not None and min(2, max(0, int(f["t_frac"] * 3))) == i])
              for i in range(3)]
    hi = [f for f in fs if f.get("height_err") is not None and f["height_err"] > HEIGHT_OVER_M]
    big_h = [f for f in fs if f.get("height_err") is not None and f["height_err"] > HEIGHT_OVER_BIG_M]
    hit = [f for f in fs if f.get("frac", 0.0) >= MISS_FRAC]
    beyond_hit = [f["map_err"] for f in hit if f["map_err"] > r_fp]
    beyond_miss = [f for f in fs if f.get("frac", 0.0) < MISS_FRAC and f["map_err"] > r_fp]
    alongs = [f["along_from_entry"] for f in big_h if f.get("along_from_entry") is not None]
    return {"n": len(fs), "near": summ(near), "far": summ(far), "offset_split": offset_split,
            "beyond_fp_hit": {"n": len(beyond_hit), "min_err": min(beyond_hit) if beyond_hit else None,
                              "max_err": max(beyond_hit) if beyond_hit else None},
            "hit_ratio_max": max([f["map_err"] / f["crown_r"] for f in hit], default=None),
            "beyond_fp_miss_ratio_min": min([f["map_err"] / f["crown_r"] for f in beyond_miss], default=None),
            "height_over_big": {"n": len(big_h), "along_min": min(alongs) if alongs else None,
                                "along_max": max(alongs) if alongs else None},
            "big_err": len(big), "big_err_toward_first_track": len(toward),
            "across_share_of_big_err": (float(np.mean([abs(f["err_across"]) / f["map_err"] for f in big])) if big else None),
            "thirds": thirds, "height_over": summ(hi)}


def aim_stats(facts: Sequence[dict]) -> dict:
    """Dose aim error (drone mean xy during a dose vs the aimed entry's centre) over every episode with an entry."""
    v = sorted(f["aim_err"] for f in facts if f.get("aim_err") is not None)
    if not v:
        return {"n": 0, "median": None, "p95": None, "max": None, "share_le_0_1": None}
    return {"n": len(v), "median": float(np.percentile(v, 50)), "p95": float(np.percentile(v, 95)), "max": v[-1],
            "share_le_0_1": sum(x <= 0.1 for x in v) / len(v)}


def height_trace(run_dir: str, ids: Sequence[int], min_dh: float = 0.25) -> List[dict]:
    """Published height of the given entries over time, one row whenever the height moved >= ``min_dh`` since the last
    row (plus the first appearance and the last snapshot), with the mission phase, the target and the drone."""
    mis = ms._load_json(os.path.join(run_dir, "mission.json"), {}) or {}
    ph = [(float(t), str(p)) for t, p in mis.get("phases", [])]
    poses = [tuple(float(v) for v in p) for p in mis.get("pose", [])]
    pt = [p[0] for p in poses]
    pht = [p[0] for p in ph]
    rows: List[dict] = []
    last: Dict[int, dict] = {}
    seen: Dict[int, dict] = {}
    want = set(int(i) for i in ids)
    with open(os.path.join(run_dir, "map.jsonl")) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            for tr in d.get("trees", []):
                i = int(tr["id"])
                if i not in want or tr.get("height") is None:
                    continue
                t, h, c = float(d["t"]), float(tr["height"]), tr["centre"]
                p = pose_at(poses, pt, t)
                row = {"id": i, "t": t, "height": h, "centre": [float(c[0]), float(c[1])],
                       "phase": ph[max(0, bisect.bisect_right(pht, t) - 1)][1] if ph else None,
                       "drone": [p[1], p[2], p[3]] if p else None,
                       "dist": math.hypot(p[1] - float(c[0]), p[2] - float(c[1])) if p else None}
                seen[i] = row
                if i not in last or abs(h - last[i]["height"]) >= min_dh:
                    rows.append(row)
                    last[i] = row
    for i, row in seen.items():
        if last.get(i) is not row:
            rows.append(row)
    return sorted(rows, key=lambda r: (r["id"], r["t"]))


def trace_markdown(run_dir: str, rows: Sequence[dict]) -> str:
    L = ["### Height trace: %s" % run_dir, "", "| Entry | t (s) | Published height (m) | Phase | Drone x, y, z (m) | "
         "Drone - entry distance (m) |", "|---|---|---|---|---|---|"]
    for r in rows:
        L.append("| %d | %.2f | %.2f | %s | %s | %s |" % (
            r["id"], r["t"], r["height"], r["phase"],
            "%.1f, %.1f, %.1f" % tuple(r["drone"]) if r["drone"] else "n/a", _f(r["dist"], 1)))
    return "\n".join(L)


# ----------------------------------------------------------------------------- report

def _f(v, nd=2):
    return ms._f(v, nd)


def _signed(v, nd=2):
    return "n/a" if v is None else ("%+.*f" % (nd, v))


def build(batch_dirs: Sequence[str]) -> dict:
    runs = [load_run(d) for b in batch_dirs for d in run_dirs(b)]
    groups: Dict[Tuple[str, str], List[dict]] = {}
    for r in runs:
        groups.setdefault((spec_name(r["run"]), str(r["run"].get("strategy"))), []).append(r)
    res = {"groups": {}, "misses": [], "patterns": {}, "setup": [], "phantoms": [], "aim": {},
           "unaligned_runs": []}
    facts_by_spec: Dict[str, List[dict]] = {}
    for (spec, strat), rs in sorted(groups.items()):
        cards = [r["card"] for r in rs]
        res["groups"][(spec, strat)] = group_stats(cards)
        r_fp = nominal_footprint(rs[0]["cfg"], rs[0]["run"].get("spec") or {})
        res["setup"].append({"spec": spec, "strategy": strat, "runs": [r["name"] for r in rs],
                             "batch": os.path.dirname(rs[0]["dir"].rstrip("/")),
                             "commits": sorted(set(str(r["run"].get("git_commit"))[:7] for r in rs)),
                             "dirty": sorted(set(r["run"].get("git_dirty_files") for r in rs)),
                             "seeds": sorted(set(r["run"].get("seed") for r in rs)),
                             "detector": sorted(set(str(r["run"].get("detector_model")) for r in rs)),
                             "overrides": rs[0]["run"].get("cfg_overrides"), "isaac": rs[0]["run"].get("isaac"),
                             "rtf": [r["rtf"].get("rtf") for r in rs], "wall_s": [r["rtf"].get("wall_s") for r in rs],
                             "r_fp": r_fp, "n_truth": rs[0]["card"]["run"].get("n_truth_trees")})
        for r in rs:
            facts = episode_facts(r)
            if not r.get("episodes_aligned"):
                # the scorer's missed trees still count, under their spec and strategy, unanalysed
                for tid in (r["card"].get("spray") or {}).get("missed_ids") or []:
                    res["misses"].append({"spec": spec, "strategy": strat, "run": r["name"], "truth_id": tid,
                                          "class": "e", "note": "not analysed: the episodes in mission.json do not "
                                          "align with the events of mission_score.json"})
                res["unaligned_runs"].append("%s/%s/%s" % (spec, strat, r["name"]))
                continue
            for f in facts:
                f["spec"], f["strategy"], f["run"] = spec, strat, r["name"]
            facts_by_spec.setdefault(spec, []).extend(facts)
            for f in facts:
                if f.get("phantom"):
                    res["phantoms"].append({"spec": spec, "strategy": strat, "run": r["name"], "tm_id": f["tm_id"],
                                            "start": f["start"]})
            for m in miss_rows(r, facts):
                m.update({"spec": spec, "strategy": strat})
                res["misses"].append(m)
    all_facts = [f for fs in facts_by_spec.values() for f in fs]
    res["aim"]["all/all"] = aim_stats(all_facts)
    for spec, fs in facts_by_spec.items():
        for strat in sorted(set(f["strategy"] for f in fs)):
            res["aim"]["%s/%s" % (spec, strat)] = aim_stats([f for f in fs if f["strategy"] == strat])
    for spec, fs in facts_by_spec.items():
        g = next(s for s in res["setup"] if s["spec"] == spec)
        res["patterns"][spec] = {"all": pattern_stats(fs, g["r_fp"][0])}
        for strat in sorted(set(f["strategy"] for f in fs)):
            res["patterns"][spec][strat] = pattern_stats([f for f in fs if f["strategy"] == strat], g["r_fp"][0])
    return res


def markdown(res: dict) -> str:
    L: List[str] = []
    L += ["## Setup", "", "| Spec | Strategy | Runs | Commit (dirty files) | Seed | Detector model | RTF mean (min) | "
          "Mission wall time per run (s) | Footprint radius r_fp |", "|---|---|---|---|---|---|---|---|---|"]
    for s in res["setup"]:
        rtf = [v for v in s["rtf"] if v is not None]
        wall = [v for v in s["wall_s"] if v is not None]
        L.append("| %s | %s | %d | %s (%s) | %s | `%s` | %s (%s) | %s | %.2f m (nozzle %.1f m above the published top, "
                 "cone %.0f deg) |" % (
                     s["spec"], s["strategy"], len(s["runs"]), ", ".join(s["commits"]), ", ".join(map(str, s["dirty"])),
                     ", ".join(map(str, s["seeds"])), ", ".join(os.path.basename(d) for d in s["detector"]),
                     _f(float(np.mean(rtf)) if rtf else None, 3), _f(min(rtf) if rtf else None, 3),
                     "%.0f .. %.0f" % (min(wall), max(wall)) if wall else "n/a", s["r_fp"][0], s["r_fp"][1], s["r_fp"][2]))
    L.append("")
    specs = sorted(set(k[0] for k in res["groups"]))
    for spec in specs:
        strats = sorted(k[1] for k in res["groups"] if k[0] == spec)
        L += ["## %s" % spec, ""]
        a_name = "corridor" if "corridor" in strats else strats[0]
        others = [s for s in strats if s != a_name]
        hdr = "| Group | Metric | " + " | ".join("%s mean / worst / spread" % s for s in strats)
        if others:
            hdr += " | %s - %s (noise floor) | Verdict |" % (others[0], a_name)
        else:
            hdr += " |"
        L += [hdr, "|" + "---|" * (2 + len(strats) + (2 if others else 0))]
        for key, label, grp, bad in metric_rows():
            cells = []
            for s in strats:
                m = res["groups"][(spec, s)][key]
                cells.append("%s / %s / %s" % (_f(m["mean"]), _f(m["worst"]), _f(m["spread"])))
            line = "| %s | %s | %s" % (grp, label, " | ".join(cells))
            if others:
                d, floor, txt = verdict(res["groups"][(spec, a_name)][key], res["groups"][(spec, others[0])][key],
                                        bad, a_name, others[0])
                if key in INFO_ONLY and txt not in ("n/a", "no measurable difference"):
                    txt = "%s %s (info only)" % (others[0], "lower" if d < 0 else "higher")
                line += " | %s (%s) | %s |" % (_signed(d), _f(floor), txt)
            else:
                line += " |"
            L.append(line)
        L.append("")
    # misses
    L += ["## Missed trees", "", "Classes: " + "; ".join("(%s) %s" % (c, CLASS_LABEL[c]) for c in CLASSES) + ".", "",
          "| Spec | Strategy | (a) | (b) | (c) | (d) | (e) | Missed |", "|---|---|---|---|---|---|---|---|"]
    for spec in specs:
        for strat in sorted(k[1] for k in res["groups"] if k[0] == spec):
            ms_ = [m for m in res["misses"] if m.get("spec") == spec and m.get("strategy") == strat]
            L.append("| %s | %s | %s | %d |" % (spec, strat, " | ".join(str(sum(m["class"] == c for m in ms_))
                                                                          for c in CLASSES), len(ms_)))
    tot = res["misses"]
    L += ["| all | all | %s | %d |" % (" | ".join(str(sum(m["class"] == c for m in tot)) for c in CLASSES), len(tot)), ""]
    L += ["| Spec | Strategy | Run | Truth tree | Class | Aimed entry (episodes) | Best dose fraction | Map centre error at "
          "spray (m) | Crown radius (m) | Published - true top (m) | Nozzle above true top (m) | Actual footprint (m) | "
          "First-sighting lateral offset (m) | First-sighting centre error (m) | Final IoU | Mission time fraction | "
          "Gap to nearest crown (m) | Note |", "|" + "---|" * 18]
    for m in sorted(tot, key=lambda m: (m.get("spec", ""), m.get("strategy", ""), m.get("run", ""), m.get("truth_id") or 0)):
        L.append("| %s | %s | %s | %s | %s | %s (%s) | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            m.get("spec"), m.get("strategy"), m.get("run"), m.get("truth_id"), m["class"], m.get("tm_id"),
            m.get("episodes"), _f(m.get("frac")), _f(m.get("map_err")), _f(m.get("crown_r")), _signed(m.get("height_err")),
            _f(m.get("nozzle_above_top")), _f(m.get("r_fp_actual")), _signed(m.get("first_offset"), 1),
            _f(m.get("first_err")), _f(m.get("final_iou")), _f(m.get("t_frac")), _f(m.get("gap")), m.get("note", "")))
    L.append("")
    L += ["## Patterns over every attributed dose episode", "",
          "First-sighting lateral offset = across-track distance from the drone to the truth centre when the aimed "
          "entry was first published. 'Toward the first track' = the across-track part of the map error points back "
          "toward where the drone was at that moment.", "",
          "| Spec | Strategy | Episodes | offset < 5 m: n, mean error, share > r_fp, miss share | offset >= 5 m: same | "
          "Errors > r_fp pointing toward the first track | Across-track share of those errors | Miss share by mission "
          "third | Height over-estimate > 1 m: n, miss share |", "|---|---|---|---|---|---|---|---|---|"]

    def sm(s):
        if not s.get("n"):
            return "0"
        return "%d, %.2f m, %.0f %%, %.0f %%" % (s["n"], s["mean_err"], 100 * s["share_err_gt_fp"], 100 * s["share_miss"])

    for spec in specs:
        for strat, p in sorted(res["patterns"].get(spec, {}).items()):
            L.append("| %s | %s | %d | %s | %s | %d of %d | %s | %s | %s |" % (
                spec, strat, p["n"], sm(p["near"]), sm(p["far"]), p["big_err_toward_first_track"], p["big_err"],
                _f(p["across_share_of_big_err"]),
                " / ".join("%.0f %%" % (100 * t["share_miss"]) if t.get("n") else "n/a" for t in p["thirds"]),
                ("%d, %.0f %%" % (p["height_over"]["n"], 100 * p["height_over"]["share_miss"]))
                if p["height_over"].get("n") else "0"))
    L.append("")
    L += ["| Spec | Strategy | Episodes with err > r_fp that still got >= 50 %: n, error range | Max error / crown "
          "radius of an episode that got >= 50 % | Min error / crown radius of an episode with err > r_fp that got "
          "< 50 % | Height over-estimate > 1.5 m at spray: n, along-track distance of the tree from the field's entry "
          "edge |", "|---|---|---|---|---|---|"]
    for spec in specs:
        for strat, p in sorted(res["patterns"].get(spec, {}).items()):
            bh, ho = p["beyond_fp_hit"], p["height_over_big"]
            L.append("| %s | %s | %d, %s .. %s m | %s | %s | %d, %s .. %s m |" % (
                spec, strat, bh["n"], _f(bh["min_err"]), _f(bh["max_err"]), _f(p["hit_ratio_max"]),
                _f(p["beyond_fp_miss_ratio_min"]), ho["n"], _f(ho["along_min"], 1), _f(ho["along_max"], 1)))
    L.append("")
    L += ["## Dose aim accuracy", "",
          "Aim error = distance from the drone's mean xy during a dose episode to the centre of the map entry the "
          "navigator aimed at (its `/spray_target` id, centre in the latest map before the episode). It measures the "
          "navigator and the drone, not the map.", "",
          "| Spec / strategy | Episodes | Median (m) | p95 (m) | Max (m) | Share <= 0.1 m |", "|---|---|---|---|---|---|"]
    for key in sorted(res["aim"], key=lambda k: (k == "all/all", k)):
        a = res["aim"][key]
        L.append("| %s | %d | %s | %s | %s | %s |" % (key, a["n"], _f(a["median"], 3), _f(a["p95"], 3),
                                                      _f(a["max"], 3), "n/a" if a["share_le_0_1"] is None
                                                      else "%.1f %%" % (100 * a["share_le_0_1"])))
    L.append("")
    if res.get("unaligned_runs"):
        L += ["Runs not analysed (episodes do not align with the scorer's events; their missed trees are counted as "
              "(e)): " + ", ".join(res["unaligned_runs"]) + ".", ""]
    if res["phantoms"]:
        L += ["Phantom episodes (dose on an entry that is not a displaced copy of any truth tree): " + ", ".join(
            "%s %s %s id %s at %.0f s" % (p["spec"], p["strategy"], p["run"], p["tm_id"], p["start"])
            for p in res["phantoms"]) + ".", ""]
    return "\n".join(L)


def _jsonable(res: dict) -> dict:
    out = dict(res)
    out["groups"] = {"%s/%s" % k: v for k, v in res["groups"].items()}
    return ms._clean(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Strategy comparison and miss analysis of scored Isaac closed-loop batches.")
    ap.add_argument("batches", nargs="+", help="batch directories (run_1 .. run_N with mission_score.json)")
    ap.add_argument("--out", help="write the markdown here instead of stdout")
    ap.add_argument("--json", help="also write the numbers (groups, misses, patterns) as JSON")
    ap.add_argument("--trace", action="append", default=[], metavar="RUN_DIR:ID[,ID...]",
                    help="append the published-height trace of these map entries of one run (repeatable)")
    a = ap.parse_args(argv)
    res = build(a.batches)
    md = markdown(res)
    for spec_ in a.trace:
        run_dir, _, ids = spec_.rpartition(":")
        md += "\n\n" + trace_markdown(run_dir, height_trace(run_dir, [int(i) for i in ids.split(",") if i]))
    if a.out:
        with open(a.out, "w") as f:
            f.write(md + "\n")
    else:
        print(md)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(_jsonable(res), f, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
