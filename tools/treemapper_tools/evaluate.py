"""One-command evaluation: run a system on datasets, score each run, aggregate, print a scorecard.

A *dataset* is a directory with ``frames.bin`` (input contract, ``frames.py``), ``ground_truth.json``,
``mission.json`` and ``camera_info.json`` (contract formats). Recorded missions may also hold
``map_existing.jsonl`` (the existing system's map; system name ``existing``).

    python3 -m treemapper_tools.evaluate --system simple --dataset /data/work/extracted/isaac_...
    python3 -m treemapper_tools.evaluate --system simple --suite /data/work/synthetic/suite_v1 [--repeat 2]
    python3 -m treemapper_tools.evaluate --system simple,cand_a,cand_b --suite S [--reference simple]
                                         [--set cand_a.alpha=0.3 ...]

Systems: any name the replay runner has registered (``treemapper_replay --list-systems``: ``simple`` and
every candidate registered in ``core/src/mapper_registry.cpp``), plus ``existing`` (recorded map, no run;
recorded missions only). With two or more systems the output gets a **paired comparison**: every other
system against the reference (``--reference``, default the first system), dataset by dataset, because the
run-to-run noise is 0 and the seed-to-seed spread dominates the means. ``--set SYSTEM.key=value``
(repeatable) passes a configuration entry to that system (``treemapper_replay --config``). The replay's navigation emulator resolves hop
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


def registered_systems() -> List[str]:
    """Names the replay runner knows (``treemapper_replay --list-systems``)."""
    r = subprocess.run([REPLAY, "--list-systems"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                       universal_newlines=True)
    if r.returncode != 0:
        raise RuntimeError("%s --list-systems failed: %s" % (REPLAY, r.stderr[-500:]))
    return [x for x in r.stdout.split() if x]


def check_systems(systems: List[str], dsets: List[str]) -> None:
    """Fail early, with the list of registered names, instead of deep inside the first replay."""
    known = registered_systems()
    for s in systems:
        if s == "existing":
            missing = [d for d in dsets if not os.path.exists(os.path.join(d, "map_existing.jsonl"))]
            if missing:
                raise SystemExit("system 'existing' needs map_existing.jsonl (recorded missions only); missing in: %s"
                                 % ", ".join(os.path.basename(m) for m in missing[:3]))
        elif s not in known:
            raise SystemExit("unknown system '%s'; registered: %s (+ 'existing' for recorded missions). A new "
                             "candidate needs its TM_REGISTER_CANDIDATE line in core/src/mapper_registry.cpp and a rebuild."
                             % (s, ", ".join(known)))


def run_system(system: str, ds: str, run_dir: str, config: str = "") -> str:
    os.makedirs(run_dir, exist_ok=True)
    if system == "existing":
        return os.path.join(ds, "map_existing.jsonl")
    out_map = os.path.join(run_dir, "map.jsonl")
    cmd = [REPLAY, "--frames", os.path.join(ds, "frames.bin"), "--out-map", out_map,
           "--out-targets", os.path.join(run_dir, "targets.json"),
           "--out-counters", os.path.join(run_dir, "counters.json"), "--system", system]
    if config:
        cmd += ["--config", config]
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


def score_dataset(system: str, ds: str, run_dir: str, config: str = "") -> dict:
    mp = run_system(system, ds, run_dir, config)
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


def evaluate_system(system: str, dsets: List[str], runs: str, repeat: int, config: str = "") -> dict:
    per = {}
    for ds in dsets:
        name = os.path.basename(ds)
        for k in range(repeat):
            run_dir = os.path.join(runs, system, name + ("" if repeat == 1 else "_r%d" % k))
            res = score_dataset(system, ds, run_dir, config)
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


# Columns of the per-seed paired table (the full per-metric differences are in the JSON).
PAIRED_COLUMNS = ["recall (final)", "precision (final)", "mean IoU (final)", "neighbours swallowed (final)",
                  "false trees (final)", "centre RMSE m (final)", "height RMSE m (final)", "spill (final)",
                  "SAFETY total violations"]
_BETTER = {label: better for label, _, better in HEADLINE}


def _sd(vals: List[float]) -> Optional[float]:
    if len(vals) < 2:
        return None
    m = sum(vals) / len(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))


def _sgn(v: Optional[float]) -> str:
    if v is None:
        return "n/a"
    if abs(v) < 5e-4:
        return "0"
    return ("%+d" % round(v)) if abs(v - round(v)) < 1e-9 else "%+.3f" % v


def paired_comparison(ref: dict, other: dict) -> dict:
    """Paired differences ``other - ref`` over the datasets both ran (paired by dataset = by seed).

    ``per_dataset[name][label]`` is the difference (None if either side is n/a); ``summary[label]`` has n,
    ref/other mean over the pairs, mean/std (sample) of the differences, t = mean / (std / sqrt(n)), and
    wins / losses / ties for metrics with a direction (a win = ``other`` better than ``ref``)."""
    names = [n for n in ref["per_dataset"] if n in other["per_dataset"]]
    per = {}
    for n in names:
        a, b = ref["per_dataset"][n][0]["headline"], other["per_dataset"][n][0]["headline"]
        per[n] = {l: (b[l] - a[l]) if (a.get(l) is not None and b.get(l) is not None) else None
                  for l, _, _ in HEADLINE}
    summary = {}
    for label, _, better in HEADLINE:
        pairs = [(ref["per_dataset"][n][0]["headline"][label], other["per_dataset"][n][0]["headline"][label])
                 for n in names]
        pairs = [(a, b) for a, b in pairs if a is not None and b is not None]
        if not pairs:
            summary[label] = {"n": 0}
            continue
        d = [b - a for a, b in pairs]
        mean = sum(d) / len(d)
        sd = _sd(d)
        t = None
        if sd is not None:
            t = (mean / (sd / math.sqrt(len(d)))) if sd > 0 else (0.0 if abs(mean) < 1e-12 else float("inf") * (1 if mean > 0 else -1))
        wins = losses = ties = None
        if better is not None:
            ties = sum(1 for x in d if abs(x) < 1e-9)
            wins = sum(1 for x in d if (x > 1e-9 if better else x < -1e-9))
            losses = len(d) - ties - wins
        summary[label] = {"n": len(d), "ref_mean": sum(a for a, _ in pairs) / len(d),
                          "other_mean": sum(b for _, b in pairs) / len(d), "mean_diff": mean,
                          "std_diff": sd, "t": t, "wins": wins, "losses": losses, "ties": ties}
    return {"per_dataset": per, "summary": summary, "n_pairs": len(names)}


def paired_markdown(ref_name: str, name: str, P: dict) -> List[str]:
    md = ["## Paired: %s vs %s (difference = %s - %s, same dataset = same seed)" % (name, ref_name, name, ref_name), "",
          "%d pairs. For metrics with a direction, **win** = %s better than %s. t = mean diff / (std diff / sqrt(n)); "
          "|t| below about 2 is within seed noise." % (P["n_pairs"], name, ref_name), "",
          "| Metric | %s mean | %s mean | mean diff | std diff | t | win / loss / tie |" % (ref_name, name),
          "|---|---|---|---|---|---|---|"]
    for label, _, _ in HEADLINE:
        st = P["summary"][label]
        if not st["n"]:
            md.append("| %s | n/a | n/a | n/a | n/a | n/a | n/a |" % label)
            continue
        t = st["t"]
        md.append("| %s | %s | %s | %s | %s | %s | %s |" % (
            label, _fmt(st["ref_mean"]), _fmt(st["other_mean"]), _sgn(st["mean_diff"]),
            "n/a" if st["std_diff"] is None else _fmt(st["std_diff"]),
            "n/a" if t is None else ("inf" if math.isinf(t) else "%+.1f" % t),
            "-" if st["wins"] is None else "%d / %d / %d" % (st["wins"], st["losses"], st["ties"])))
    groups = {}
    for n in P["per_dataset"]:
        groups.setdefault(n.rsplit("_s", 1)[0], []).append(n)
    if len(groups) > 1:
        md += ["", "Mean paired difference per group:", "",
               "| group (n) | " + " | ".join(PAIRED_COLUMNS) + " |", "|---|" + "---|" * len(PAIRED_COLUMNS)]
        for g, ns in groups.items():
            cells = []
            for l in PAIRED_COLUMNS:
                v = [P["per_dataset"][n][l] for n in ns if P["per_dataset"][n][l] is not None]
                cells.append(_sgn(sum(v) / len(v)) if v else "n/a")
            md.append("| %s (%d) | " % (g, len(ns)) + " | ".join(cells) + " |")
    md += ["", "Per seed (%s - %s):" % (name, ref_name), "",
           "| dataset | " + " | ".join(PAIRED_COLUMNS) + " |", "|---|" + "---|" * len(PAIRED_COLUMNS)]
    for n, d in P["per_dataset"].items():
        md.append("| %s | " % n + " | ".join(_sgn(d[l]) for l in PAIRED_COLUMNS) + " |")
    md.append("")
    return md


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--system", default="simple", help="system name, or a comma list (side-by-side)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--dataset", help="one dataset directory")
    g.add_argument("--suite", help="directory of dataset directories (e.g. a synthetic suite)")
    ap.add_argument("--runs", default="/data/work/runs", help="where run outputs go")
    ap.add_argument("--repeat", type=int, default=1, help="run every dataset N times (run-to-run noise floor)")
    ap.add_argument("--reference", default=None, help="reference system of the paired comparison (default: the first one)")
    ap.add_argument("--set", action="append", default=[], metavar="SYSTEM.key=value",
                    help="configuration entry for a system (repeatable), passed to treemapper_replay --config")
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
    if not systems:
        raise SystemExit("--system is empty")
    ref = a.reference or systems[0]
    if ref not in systems:
        raise SystemExit("--reference %s is not in --system %s" % (ref, a.system))
    configs: Dict[str, Dict[str, str]] = {}
    for item in a.set:
        sysname, _, kv = item.partition(".")
        key, eq, val = kv.partition("=")
        if not (sysname and key and eq) or sysname not in systems:
            raise SystemExit("bad --set %r (expected SYSTEM.key=value for a system in --system)" % item)
        configs.setdefault(sysname, {})[key] = val
    check_systems(systems, dsets)
    results = {s: evaluate_system(s, dsets, a.runs, a.repeat, ";".join("%s=%s" % kv for kv in sorted(configs.get(s, {}).items())))
               for s in systems}
    paired = {s: paired_comparison(results[ref], results[s]) for s in systems if s != ref}
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
    for s_name, P in paired.items():
        md += paired_markdown(ref, s_name, P)
    text = "\n".join(md) + "\n"
    print(text)
    if a.out_md:
        with open(a.out_md, "w") as f:
            f.write(text)
    if a.out_json:
        with open(a.out_json, "w") as f:
            json.dump({"title": title, "systems": systems, "datasets": dsets, "repeat": a.repeat,
                       "reference": ref, "configs": configs, "results": results, "paired": paired}, f, indent=1)


if __name__ == "__main__":
    main()
