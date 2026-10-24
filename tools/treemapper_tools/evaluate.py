"""One-command evaluation: run a system on datasets, score each run, aggregate, print a scorecard.

A *dataset* is a directory with ``frames.bin`` (input contract, ``frames.py``), ``ground_truth.json``,
``mission.json`` and ``camera_info.json`` (contract formats). Recorded missions may also hold
``map_existing.jsonl`` (the existing system's map; system name ``existing``).

    python3 -m treemapper_tools.evaluate --system simple --dataset /data/work/extracted/isaac_...
    python3 -m treemapper_tools.evaluate --system simple --suite /data/work/synthetic/suite_v1 [--repeat 2]

Systems: ``simple`` (C++ ``treemapper_replay --system simple``), ``existing`` (recorded map, no run), or
any other name accepted by ``treemapper_replay --system``. The replay's navigation emulator resolves hop
targets into the system's own IDs; the scorer gets ``mission.json`` with those resolved targets
(``mission_resolved.json`` in the run directory), so descent rules are judged against the IDs the system
actually received.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional

from .contract import read_ground_truth, read_mission, read_snapshots
from . import score as scoring

REPLAY = os.environ.get("TM_REPLAY", "/data/work/build/core/treemapper_replay")


def run_system(system: str, ds: str, run_dir: str) -> str:
    os.makedirs(run_dir, exist_ok=True)
    if system == "existing":
        return os.path.join(ds, "map_existing.jsonl")
    out_map = os.path.join(run_dir, "map.jsonl")
    cmd = [REPLAY, "--frames", os.path.join(ds, "frames.bin"), "--out-map", out_map,
           "--out-targets", os.path.join(run_dir, "targets.json"),
           "--out-counters", os.path.join(run_dir, "counters.json"), "--system", system]
    t0 = time.time()
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    with open(os.path.join(run_dir, "replay.log"), "w") as f:
        f.write(r.stderr)
    if r.returncode != 0:
        raise RuntimeError("replay failed (%d): %s" % (r.returncode, r.stderr[-2000:]))
    with open(os.path.join(run_dir, "run_seconds.txt"), "w") as f:
        f.write("%.2f\n" % (time.time() - t0))
    return out_map


def resolved_mission(ds: str, run_dir: str, system: str) -> Optional[str]:
    mpath = os.path.join(ds, "mission.json")
    if not os.path.exists(mpath):
        return None
    if system == "existing":
        return mpath
    with open(mpath) as f:
        m = json.load(f)
    with open(os.path.join(run_dir, "targets.json")) as f:
        m["targets"] = json.load(f)
    out = os.path.join(run_dir, "mission_resolved.json")
    with open(out, "w") as f:
        json.dump(m, f, separators=(",", ":"))
    return out


def score_dataset(system: str, ds: str, run_dir: str) -> dict:
    mp = run_system(system, ds, run_dir)
    mission = resolved_mission(ds, run_dir, system)
    cam = os.path.join(ds, "camera_info.json")
    camera = json.load(open(cam)) if os.path.exists(cam) else None
    res = scoring.score(read_snapshots(mp), read_ground_truth(os.path.join(ds, "ground_truth.json")),
                        read_mission(mission), camera)
    with open(os.path.join(run_dir, "score.json"), "w") as f:
        json.dump(res, f, indent=1, default=str)
    return res


def _g(d, *path, default=None):
    for p in path:
        if not isinstance(d, dict) or p not in d:
            return default
        d = d[p]
    return d


def _n(v) -> Optional[float]:
    if isinstance(v, list):
        return float(len(v))
    if v is None or isinstance(v, (str, dict)):
        return None
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


# (label, path into the score dict, "higher is better"?)
HEADLINE = [
    ("recall (final)", ("final", "detection", "recall"), True),
    ("precision (final)", ("final", "detection", "precision"), True),
    ("published / truth (final)", ("final", "detection", "n_published"), None),
    ("recall (per-frame mean)", ("per_frame", "detection", "recall", "mean"), True),
    ("mean IoU (final)", ("final", "geometry", "mean_iou"), True),
    ("min IoU (final)", ("final", "geometry", "min_iou"), True),
    ("merged entries (final)", ("final", "detection", "merged_entries"), False),
    ("neighbours swallowed (final)", ("final", "detection", "wrong_merges"), False),
    ("split trees (final)", ("final", "detection", "split_trees"), False),
    ("false trees (final)", ("final", "detection", "false_trees"), False),
    ("centre RMSE m (final)", ("final", "geometry", "centre_rmse_m"), False),
    ("height bias m (final)", ("final", "geometry", "height_bias_m"), None),
    ("height RMSE m (final)", ("final", "geometry", "height_rmse_m"), False),
    ("spill (final)", ("final", "geometry", "spill"), False),
    ("neighbour spill max (final)", ("final", "geometry", "neighbour_spill_max"), False),
    ("ID switches after lock", ("stability", "id_switches_after_lock"), False),
    ("geometry retreats", ("stability", "geometry_retreats"), False),
    ("centre jumps", ("stability", "centre_jumps"), False),
    ("height jumps", ("stability", "height_jumps"), False),
    ("IDs vanished while held by nav", ("stability", "ids_vanished_while_held_by_navigation"), False),
    ("SAFETY total violations", ("safety", "total_violations"), False),
    ("  undefined-height frames", ("safety", "undefined_height_frames"), False),
    ("  new trees after scan", ("safety", "new_trees_after_scan"), False),
    ("  non-target changes in descent", ("safety", "nontarget_changes_in_descent"), False),
    ("  sprayed-tree changes", ("safety", "sprayed_tree_changes"), False),
    ("  new tree inside sprayed", ("safety", "new_tree_inside_sprayed"), False),
    ("  invalid-entry frames", ("safety", "invalid_entry_frames"), False),
    ("  duplicate-ID frames", ("safety", "duplicate_id_frames"), False),
    ("  implausible-height frames", ("safety", "implausible_height_frames"), False),
    ("  locked ID vanished", ("safety", "locked_id_vanished"), False),
    ("truth trees sprayed", ("final", "spray", "truth_trees_sprayed"), True),
    ("truth trees sprayed twice", ("final", "spray", "truth_trees_sprayed_twice"), False),
    ("scan latency mean s", ("timeliness", "latency_scan_mean_s"), False),
]


HEIGHT_ROWS = ("height bias m (final)", "height RMSE m (final)")


def headline(res: dict) -> Dict[str, Optional[float]]:
    """Headline numbers. Height error rows are None when the scorer flags the truth heights unreliable
    (``gt_height_reliable`` false, e.g. the Isaac recording's constant 4.0 m fallback heights)."""
    out = {}
    reliable = res.get("gt_height_reliable", True) is not False
    for label, path, _ in HEADLINE:
        out[label] = _n(_g(res, *path)) if (reliable or label not in HEIGHT_ROWS) else None
    return out


def cost_line(res: dict) -> str:
    c = res.get("cost") or {}
    if not c.get("available"):
        return "n/a"
    return json.dumps({k: v for k, v in c.items() if k != "available"}, default=str)[:400]


def _fmt(v) -> str:
    if v is None:
        return "n/a"
    if abs(v - round(v)) < 1e-9 and abs(v) < 1e6:
        return "%d" % round(v)
    return "%.3f" % v


def aggregate(rows: List[Dict[str, Optional[float]]]) -> Dict[str, dict]:
    agg = {}
    for label, _, better in HEADLINE:
        vals = [r[label] for r in rows if r.get(label) is not None]
        if not vals:
            agg[label] = {"mean": None, "worst": None, "std": None, "n": 0}
            continue
        mean = sum(vals) / len(vals)
        std = math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals))
        worst = min(vals) if better is True else max(vals) if better is False else None
        agg[label] = {"mean": mean, "worst": worst, "std": std, "n": len(vals)}
    return agg


def evaluate_system(system: str, dsets: List[str], runs: str, repeat: int) -> dict:
    per = {}
    for ds in dsets:
        name = os.path.basename(ds)
        for k in range(repeat):
            run_dir = os.path.join(runs, system, name + ("" if repeat == 1 else "_r%d" % k))
            res = score_dataset(system, ds, run_dir)
            h = headline(res)
            per.setdefault(name, []).append({"headline": h, "cost": cost_line(res), "run_dir": run_dir})
            print("%-10s %-32s %s" % (system, name, " ".join("%s=%s" % (l.split(" (")[0], _fmt(v))
                                                         for l, v in list(h.items())[:5])), file=sys.stderr)
    agg = aggregate([r[0]["headline"] for r in per.values()])
    floor = {}
    if repeat > 1:
        for label, _, _ in HEADLINE:
            diffs = []
            for r in per.values():
                vs = [x["headline"][label] for x in r if x["headline"][label] is not None]
                if len(vs) > 1:
                    diffs.append(max(vs) - min(vs))
            floor[label] = max(diffs) if diffs else None
    return {"per_dataset": per, "aggregate": agg, "run_to_run_max_diff": floor}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--system", default="simple", help="system name, or a comma list (side-by-side)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--dataset", help="one dataset directory")
    g.add_argument("--suite", help="directory of dataset directories (e.g. a synthetic suite)")
    ap.add_argument("--runs", default="/data/work/runs", help="where run outputs go")
    ap.add_argument("--repeat", type=int, default=1, help="run every dataset N times (run-to-run noise floor)")
    ap.add_argument("--title", default=None)
    ap.add_argument("--out-md")
    ap.add_argument("--out-json")
    a = ap.parse_args(argv)
    systems = [x for x in a.system.split(",") if x]
    if a.dataset:
        dsets = [a.dataset.rstrip("/")]
    else:
        dsets = sorted(os.path.join(a.suite, d) for d in os.listdir(a.suite)
                       if os.path.exists(os.path.join(a.suite, d, "ground_truth.json")))
    results = {s: evaluate_system(s, dsets, a.runs, a.repeat) for s in systems}
    title = a.title or "%s on %s" % (a.system, a.dataset or a.suite)
    md = ["# " + title, "",
          "Datasets: %d; repeats: %d. Generated by `python3 -m treemapper_tools.evaluate` (scorer: "
          "`treemapper_tools.score`)." % (len(dsets), a.repeat), ""]
    if len(dsets) == 1:
        name = os.path.basename(dsets[0])
        md += ["| Metric | " + " | ".join(systems) + " |", "|---|" + "---|" * len(systems)]
        for label, _, _ in HEADLINE:
            md.append("| %s | " % label + " | ".join(_fmt(results[s]["per_dataset"][name][0]["headline"][label])
                                                    for s in systems) + " |")
        md.append("| cost (stage_ms) | " + " | ".join(results[s]["per_dataset"][name][0]["cost"] for s in systems) + " |")
    else:
        for s in systems:
            R = results[s]
            floor = R["run_to_run_max_diff"]
            md += ["## %s" % s, "",
                   "| Metric | mean | worst | std over datasets |" + (" run-to-run max diff |" if floor else ""),
                   "|---|---|---|---|" + ("---|" if floor else "")]
            for label, _, _ in HEADLINE:
                st = R["aggregate"][label]
                md.append("| %s | %s | %s | %s |" % (label, _fmt(st["mean"]), _fmt(st["worst"]), _fmt(st["std"]))
                          + (" %s |" % _fmt(floor.get(label)) if floor else ""))
            groups = {}
            for name, runs in R["per_dataset"].items():
                groups.setdefault(name.rsplit("_s", 1)[0], []).append(runs[0]["headline"])
            if len(groups) > 1:
                gl = list(groups)
                md += ["", "Per group (mean / worst):", "", "| Metric | " + " | ".join("%s (n=%d)" % (g, len(groups[g])) for g in gl) + " |",
                       "|---|" + "---|" * len(gl)]
                ga = {g: aggregate(groups[g]) for g in gl}
                for label, _, _ in HEADLINE:
                    md.append("| %s | " % label + " | ".join("%s / %s" % (_fmt(ga[g][label]["mean"]), _fmt(ga[g][label]["worst"]))
                                                            for g in gl) + " |")
                R["per_group"] = ga
            cols = [l for l, _, _ in HEADLINE[:12]]
            md += ["", "Per dataset:", "", "| dataset | " + " | ".join(cols) + " | safety |", "|---|" + "---|" * (len(cols) + 1)]
            for name, runs in R["per_dataset"].items():
                h = runs[0]["headline"]
                md.append("| %s | " % name + " | ".join(_fmt(h[l]) for l in cols) + " | %s |" % _fmt(h["SAFETY total violations"]))
            md.append("")
    text = "\n".join(md) + "\n"
    print(text)
    if a.out_md:
        with open(a.out_md, "w") as f:
            f.write(text)
    if a.out_json:
        with open(a.out_json, "w") as f:
            json.dump({"title": title, "systems": systems, "datasets": dsets, "repeat": a.repeat,
                       "results": results}, f, indent=1)


if __name__ == "__main__":
    main()
