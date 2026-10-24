#!/usr/bin/env python3
"""Compare two map streams of the same mission (e.g. the ROS node vs the offline replay) and score both.

    compare_map_streams.py --a ros/map.jsonl --a-targets ros/targets.json --a-name ros
                           --b offline/map.jsonl --b-targets offline/targets.json --b-name offline
                           --extracted /data/work/extracted/<mission> --tools-path <repo>/tools
                           --out-md cmp.md --out-json cmp.json [--t-from 55]

* Final map: trees matched one-to-one by outline IoU (Hungarian, IoU >= 0.3); per pair: both IDs, IoU, centre
  distance, height difference, sprayed flags; unmatched trees listed.
* Scores: each stream scored by treemapper_tools.score against the ground truth and its OWN resolved hop-target
  timeline (mission.json with "targets" replaced), the same way evaluate.py does; headline table side by side.
* --t-from drops snapshots before that time from both streams (the ROS replay starts mid-bag).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    for k in ("a", "b"):
        ap.add_argument("--%s" % k, required=True)
        ap.add_argument("--%s-targets" % k, required=True)
        ap.add_argument("--%s-name" % k, default=k)
    ap.add_argument("--extracted", required=True)
    ap.add_argument("--tools-path", required=True)
    ap.add_argument("--out-md", required=True)
    ap.add_argument("--out-json", required=True)
    ap.add_argument("--t-from", type=float, default=None)
    a = ap.parse_args(argv)
    sys.path.insert(0, os.path.realpath(a.tools_path))
    import numpy as np
    from scipy.optimize import linear_sum_assignment
    from shapely.geometry import Polygon
    from treemapper_tools import score as scoring
    from treemapper_tools.contract import read_ground_truth, read_mission, read_snapshots
    from treemapper_tools.evaluate import HEADLINE, headline

    def load(path, tpath, name, workdir):
        snaps = [s for s in read_snapshots(path) if a.t_from is None or s.t >= a.t_from]
        m = json.load(open(os.path.join(a.extracted, "mission.json")))
        m["targets"] = json.load(open(tpath))
        mp = os.path.join(workdir, "mission_resolved_%s.json" % name)
        json.dump(m, open(mp, "w"), separators=(",", ":"))
        cam = os.path.join(a.extracted, "camera_info.json")
        res = scoring.score(snaps, read_ground_truth(os.path.join(a.extracted, "ground_truth.json")), read_mission(mp),
                            json.load(open(cam)) if os.path.exists(cam) else None)
        return snaps, res

    work = os.path.dirname(os.path.abspath(a.out_md))
    sa, ra = load(a.a, a.a_targets, a.a_name, work)
    sb, rb = load(a.b, a.b_targets, a.b_name, work)
    fa, fb = sa[-1], sb[-1]

    def poly(tr):
        return Polygon(tr.outline).buffer(0)

    ta, tb = list(fa.trees), list(fb.trees)
    iou = np.zeros((len(ta), len(tb)))
    for i, x in enumerate(ta):
        for j, y in enumerate(tb):
            px, py = poly(x), poly(y)
            u = px.union(py).area
            iou[i, j] = px.intersection(py).area / u if u > 0 else 0.0
    pairs, ua, ub = [], set(range(len(ta))), set(range(len(tb)))
    if len(ta) and len(tb):
        r, c = linear_sum_assignment(-iou)
        for i, j in zip(r, c):
            if iou[i, j] >= 0.3:
                x, y = ta[i], tb[j]
                pairs.append({"id_%s" % a.a_name: x.id, "id_%s" % a.b_name: y.id, "iou": round(float(iou[i, j]), 3),
                              "centre_dist_m": round(math.hypot(x.centre[0] - y.centre[0], x.centre[1] - y.centre[1]), 3),
                              "height_diff_m": round((x.height or math.nan) - (y.height or math.nan), 3),
                              "sprayed_%s" % a.a_name: x.sprayed, "sprayed_%s" % a.b_name: y.sprayed})
                ua.discard(i); ub.discard(j)
    same_ids = sum(1 for p in pairs if p["id_%s" % a.a_name] == p["id_%s" % a.b_name])
    ha, hb = headline(ra), headline(rb)
    out = {"final_t": {a.a_name: fa.t, a.b_name: fb.t}, "n_trees": {a.a_name: len(ta), a.b_name: len(tb)},
           "pairs": pairs, "same_id_pairs": same_ids,
           "unmatched_%s" % a.a_name: [ta[i].id for i in sorted(ua)], "unmatched_%s" % a.b_name: [tb[j].id for j in sorted(ub)],
           "headline": {a.a_name: ha, a.b_name: hb}, "n_snapshots": {a.a_name: len(sa), a.b_name: len(sb)}}
    json.dump(out, open(a.out_json, "w"), indent=1, default=str)

    def f(v):
        if v is None:
            return "n/a"
        return ("%.3f" % v).rstrip("0").rstrip(".") if isinstance(v, float) else str(v)
    L = ["# %s vs %s: same mission, final map and scorecard" % (a.a_name, a.b_name), "",
         "Snapshots: %s %d (final t %.2f), %s %d (final t %.2f)%s." % (
             a.a_name, len(sa), fa.t, a.b_name, len(sb), fb.t,
             "; both cut at t >= %.1f" % a.t_from if a.t_from is not None else ""), "",
         "## Final map, trees matched by outline IoU", "",
         "| id %s | id %s | IoU | centre dist m | height diff m (%s - %s) | sprayed %s / %s |" % (
             a.a_name, a.b_name, a.a_name, a.b_name, a.a_name, a.b_name), "|---|---|---|---|---|---|"]
    for p in pairs:
        L.append("| %d | %d | %.3f | %.3f | %+.3f | %s / %s |" % (
            p["id_%s" % a.a_name], p["id_%s" % a.b_name], p["iou"], p["centre_dist_m"], p["height_diff_m"],
            p["sprayed_%s" % a.a_name], p["sprayed_%s" % a.b_name]))
    L += ["", "Trees: %s %d, %s %d; matched %d, same ID %d; unmatched %s %s, %s %s." % (
        a.a_name, len(ta), a.b_name, len(tb), len(pairs), same_ids, a.a_name, out["unmatched_%s" % a.a_name],
        a.b_name, out["unmatched_%s" % a.b_name]), "", "## Scorecards (headline)", "",
          "| metric | %s | %s |" % (a.a_name, a.b_name), "|---|---|---|"]
    for label, _, _ in HEADLINE:
        L.append("| %s | %s | %s |" % (label, f(ha.get(label)), f(hb.get(label))))
    open(a.out_md, "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
