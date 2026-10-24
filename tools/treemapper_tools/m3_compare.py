"""M3 selection: merge per-level held-out evaluations and build the comparison tables.

    python3 -m treemapper_tools.m3_compare --eval /data/work/eval/m3_select --out-json ../results/m3_comparison.json \
        --out-md /data/work/eval/m3_select/tables.md

Input: ``holdout_{easy,medium,hard}.json`` (``treemapper_tools.evaluate`` outputs, systems simple, cand_obj,
cand_grid, reference simple) and ``isaac.json``. The pairings reuse ``evaluate.paired_comparison`` so the win /
loss semantics are those of every other scorecard; cand_grid vs cand_obj is computed from the same runs (no
re-run). Extra per-run numbers (drift, latency, cost, memory, safety events) come from each run's
``score.json`` / ``counters.json``.
"""
from __future__ import annotations

import argparse
import json
import os
from typing import Dict, List

from . import evaluate as ev

LEVELS = ("easy", "medium", "hard")
SYSTEMS = ("simple", "cand_obj", "cand_grid")
PAIRS = (("simple", "cand_obj"), ("simple", "cand_grid"), ("cand_obj", "cand_grid"))
TABLE_ROWS = ["recall (final)", "precision (final)", "recall (per-frame mean)", "mean IoU (final)", "min IoU (final)",
              "neighbours swallowed (final)", "split trees (final)", "false trees (final)", "centre RMSE m (final)",
              "height RMSE m (final)", "height bias m (final)", "spill (final)", "neighbour spill max (final)",
              "geometry retreats", "centre jumps", "height jumps", "SAFETY total violations",
              "  implausible-height frames", "truth trees sprayed", "scan latency mean s"]
PAIR_ROWS = ["recall (final)", "precision (final)", "mean IoU (final)", "neighbours swallowed (final)",
             "false trees (final)", "centre RMSE m (final)", "height RMSE m (final)", "spill (final)",
             "centre jumps", "height jumps", "SAFETY total violations", "scan latency mean s"]


SAFETY_KEYS = ("undefined_height_frames", "invalid_entry_frames", "duplicate_id_frames", "implausible_height_frames",
               "new_trees_after_scan", "locked_id_vanished", "nontarget_changes_in_descent", "sprayed_tree_changes",
               "new_tree_inside_sprayed")


def _extras(run_dir: str) -> dict:
    sc = json.load(open(os.path.join(run_dir, "score.json")))
    st, sa, cost = sc.get("stability", {}), sc.get("safety", {}), sc.get("cost", {})
    out = {"max_centre_drift_since_lock_m": st.get("max_centre_drift_since_lock_m"),
           "max_height_drift_since_lock_m": st.get("max_height_drift_since_lock_m"),
           "max_lost_fraction": st.get("max_lost_fraction"),
           "implausible_excused": sa.get("implausible_height_excused_entries"),
           "safety_nonzero": {k: sa.get(k) for k in SAFETY_KEYS if sa.get(k)},
           "height_errors": [p["height_err_m"] for p in (sc.get("final", {}).get("geometry", {}).get("per_tree") or [])
                             if p.get("height_err_m") is not None],
           "target_height_range_in_descent_max_m": sa.get("target_height_range_in_descent_max_m"),
           "implausible_events": sa.get("implausible_height_events", [])[:5],
           "stage_ms_total_mean": None, "stage_ms_total_max": None}
    if cost.get("available"):
        stages = cost["stages"]
        out["stage_ms_total_mean"] = sum(s["mean_ms"] for s in stages.values())
        out["stage_ms_total_max"] = sum(s["max_ms"] for s in stages.values())
        out["stage_ms"] = {k: round(s["mean_ms"], 3) for k, s in stages.items()}
    cp = os.path.join(run_dir, "counters.json")
    if os.path.exists(cp):
        c = json.load(open(cp))
        c = c.get("counters", c) if isinstance(c, dict) else {}
        out["grid_bytes"] = c.get("gauge_grid_bytes")
    return out


def _fmt(v, nd=3):
    if v is None:
        return "n/a"
    if isinstance(v, float) and abs(v - round(v)) > 1e-9:
        return ("%." + str(nd) + "f") % v
    return "%d" % round(v)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval", required=True)
    ap.add_argument("--out-json", required=True)
    ap.add_argument("--out-md", required=True)
    a = ap.parse_args(argv)
    res: Dict[str, dict] = {s: {"per_dataset": {}} for s in SYSTEMS}
    for L in LEVELS:
        d = json.load(open(os.path.join(a.eval, "holdout_%s.json" % L)))
        for s in SYSTEMS:
            res[s]["per_dataset"].update(d["results"][s]["per_dataset"])
    names = sorted(res["simple"]["per_dataset"])
    extras = {s: {n: _extras(res[s]["per_dataset"][n][0]["run_dir"]) for n in names} for s in SYSTEMS}
    out: dict = {"datasets": names, "levels": {}, "paired": {}, "extras": extras}
    md: List[str] = []
    # per-level mean / worst
    for L in LEVELS:
        ns = [n for n in names if n.startswith(L + "_")]
        agg = {s: ev.aggregate([res[s]["per_dataset"][n][0]["headline"] for n in ns]) for s in SYSTEMS}
        ext = {}
        for s in SYSTEMS:
            e = [extras[s][n] for n in ns]
            mx = lambda k: max((x[k] for x in e if x.get(k) is not None), default=None)
            mean = lambda k: (sum(x[k] for x in e if x.get(k) is not None) / max(1, sum(1 for x in e if x.get(k) is not None))
                              if any(x.get(k) is not None for x in e) else None)
            ext[s] = {"max_centre_drift_since_lock_m": mx("max_centre_drift_since_lock_m"),
                      "max_height_drift_since_lock_m": mx("max_height_drift_since_lock_m"),
                      "stage_ms_total_mean": mean("stage_ms_total_mean"),
                      "stage_ms_total_max": mx("stage_ms_total_max"),
                      "grid_bytes_max": mx("grid_bytes"),
                      "target_height_range_in_descent_max_m": mx("target_height_range_in_descent_max_m"),
                      "matched_heights": sum(len(x["height_errors"]) for x in e),
                      "height_err_gt_1m": sum(sum(1 for h in x["height_errors"] if abs(h) > 1.0) for x in e),
                      "height_under_gt_1m": sum(sum(1 for h in x["height_errors"] if h < -1.0) for x in e),
                      "height_err_max_abs_m": max((abs(h) for x in e for h in x["height_errors"]), default=None)}
        out["levels"][L] = {"n": len(ns), "aggregate": agg, "extras": ext}
        md += ["### %s (n = %d held-out seeds): mean / worst" % (L, len(ns)), "",
               "| Metric | " + " | ".join(SYSTEMS) + " |", "|---|" + "---|" * len(SYSTEMS)]
        for r in TABLE_ROWS:
            md.append("| %s | " % r.strip() + " | ".join("%s / %s" % (_fmt(agg[s][r]["mean"]), _fmt(agg[s][r]["worst"]))
                                                       for s in SYSTEMS) + " |")
        md.append("| max centre drift since lock m (worst seed) | " + " | ".join(_fmt(ext[s]["max_centre_drift_since_lock_m"]) for s in SYSTEMS) + " |")
        md.append("| max height drift since lock m (worst seed) | " + " | ".join(_fmt(ext[s]["max_height_drift_since_lock_m"]) for s in SYSTEMS) + " |")
        md.append("| matched trees with abs height error > 1 m (of which under-estimates) / matched | " + " | ".join(
            "%d (%d) / %d" % (ext[s]["height_err_gt_1m"], ext[s]["height_under_gt_1m"], ext[s]["matched_heights"]) for s in SYSTEMS) + " |")
        md.append("| max abs height error m (any matched tree) | " + " | ".join(_fmt(ext[s]["height_err_max_abs_m"]) for s in SYSTEMS) + " |")
        md.append("| target height range during its own descent m (worst seed) | " + " | ".join(_fmt(ext[s]["target_height_range_in_descent_max_m"]) for s in SYSTEMS) + " |")
        md.append("| cost ms / frame, mean (worst snapshot max) | " + " | ".join(
            "%s (%s)" % (_fmt(ext[s]["stage_ms_total_mean"]), _fmt(ext[s]["stage_ms_total_max"])) for s in SYSTEMS) + " |")
        md.append("")
    # paired
    for ref, oth in PAIRS:
        key = "%s_vs_%s" % (oth, ref)
        P = ev.paired_comparison(res[ref], res[oth])
        byL = {}
        for L in LEVELS:
            sub = lambda R: {"per_dataset": {n: v for n, v in R["per_dataset"].items() if n.startswith(L + "_")}}
            byL[L] = ev.paired_comparison(sub(res[ref]), sub(res[oth]))["summary"]
        out["paired"][key] = {"all": P, "by_level": byL}
        md += ["### Paired: %s vs %s (diff = %s - %s; win = %s better)" % (oth, ref, oth, ref, oth), "",
               "| Metric | all 30: mean diff (t) | all 30: W / L / T | easy: diff, W/L/T | medium: diff, W/L/T | hard: diff, W/L/T |",
               "|---|---|---|---|---|---|"]
        for r in PAIR_ROWS:
            st = P["summary"][r]
            def cell(x):
                if not x.get("n"):
                    return "n/a"
                wl = "-" if x["wins"] is None else "%d/%d/%d" % (x["wins"], x["losses"], x["ties"])
                return "%s, %s" % (ev._sgn(x["mean_diff"]), wl)
            t = st.get("t")
            md.append("| %s | %s (%s) | %s | %s | %s | %s |" % (
                r.strip(), ev._sgn(st.get("mean_diff")) if st.get("n") else "n/a",
                "n/a" if t is None else ("inf" if abs(t) == float("inf") else "%+.1f" % t),
                "-" if st.get("wins") is None else "%d / %d / %d" % (st["wins"], st["losses"], st["ties"]),
                cell(byL["easy"][r]), cell(byL["medium"][r]), cell(byL["hard"][r])))
        md.append("")
    # per-seed headline (medium + hard) for the two candidates
    cols = ["recall (final)", "precision (final)", "mean IoU (final)", "neighbours swallowed (final)",
            "false trees (final)", "height RMSE m (final)", "spill (final)", "SAFETY total violations"]
    md += ["### Per seed, cand_obj | cand_grid", "", "| dataset | " + " | ".join(c.split(" (")[0] for c in cols) + " |",
           "|---|" + "---|" * len(cols)]
    for n in names:
        a_, b_ = res["cand_obj"]["per_dataset"][n][0]["headline"], res["cand_grid"]["per_dataset"][n][0]["headline"]
        md.append("| %s | " % n + " | ".join("%s \\| %s" % (_fmt(a_[c]), _fmt(b_[c])) for c in cols) + " |")
    md.append("")
    # safety detail
    md += ["### Non-zero safety counts (held-out)", "", "| system | dataset | counts | implausible events (first) |", "|---|---|---|---|"]
    for s in SYSTEMS:
        for n in names:
            e = extras[s][n]
            if e["safety_nonzero"]:
                md.append("| %s | %s | %s | %s |" % (s, n, json.dumps(e["safety_nonzero"]),
                                                    json.dumps(e["implausible_events"][:2])[:300]))
    md.append("")
    md += ["Excused accurate above-the-drone entries (R14), total over 30 missions: " +
           ", ".join("%s %d" % (s, sum(extras[s][n]["implausible_excused"] or 0 for n in names)) for s in SYSTEMS), ""]
    # isaac
    ip = os.path.join(a.eval, "isaac.json")
    if os.path.exists(ip):
        d = json.load(open(ip))
        sysi = d["systems"]
        name = os.path.basename(d["datasets"][0])
        out["isaac"] = {s: d["results"][s]["per_dataset"][name][0] for s in sysi}
        md += ["### Recorded Isaac mission (fixed scorer)", "", "| Metric | " + " | ".join(sysi) + " |", "|---|" + "---|" * len(sysi)]
        for r, _, _ in ev.HEADLINE:
            md.append("| %s | " % r.strip() + " | ".join(_fmt(d["results"][s]["per_dataset"][name][0]["headline"][r]) for s in sysi) + " |")
        ie = {s: _extras(d["results"][s]["per_dataset"][name][0]["run_dir"]) for s in sysi if s != "existing"}
        out["isaac_extras"] = ie
        md.append("| max centre / height drift since lock m | " + " | ".join(
            "n/a" if s not in ie else "%s / %s" % (_fmt(ie[s]["max_centre_drift_since_lock_m"]), _fmt(ie[s]["max_height_drift_since_lock_m"]))
            for s in sysi) + " |")
        md.append("| target height range during its own descent m (max) | " + " | ".join(
            "n/a" if s not in ie else _fmt(ie[s]["target_height_range_in_descent_max_m"]) for s in sysi) + " |")
        md.append("| cost ms / frame mean (max) | " + " | ".join(
            "n/a" if s not in ie else "%s (%s)" % (_fmt(ie[s]["stage_ms_total_mean"]), _fmt(ie[s]["stage_ms_total_max"])) for s in sysi) + " |")
        md.append("")
    with open(a.out_md, "w") as f:
        f.write("\n".join(md) + "\n")
    for s_ in SYSTEMS:
        for n in names:
            extras[s_][n].pop("height_errors", None)
    with open(a.out_json, "w") as f:
        json.dump(out, f, indent=1, default=str)


if __name__ == "__main__":
    main()
