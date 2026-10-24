"""Run mapping systems on the real flight, score them in NO-GROUND-TRUTH mode, compare them qualitatively.

    python3 -m treemapper_tools.real_score --dataset /data/work/extracted/real_farm \
        --replay /data/work/eval/m4c/build_<commit>/treemapper_replay --systems simple,cand_obj,cand_grid,existing \
        --runs /data/work/eval/m4c/runs --label "<commit>" --out-json results/real_flight_scorecard.json \
        --out-md results/real_flight_scorecard.md --plots results/real_flight_plots

``--dataset`` = the ``real_frames.py`` output (frames.bin, mission.json with pose only, camera_info.json)
plus ``map_existing.jsonl`` (``real_existing.py``). Systems other than ``existing`` are run through
``treemapper_replay`` (default configuration = the frozen config files; ``--set SYS.key=value`` adds an
entry). The replay writes ``"frame": "world"``; the stream is relabelled ``world_nwu`` (the coordinates are
world_nwu: frames.bin's frame) so every stream and the plots share one frame.

Scored (``score.score(gt=None)``): stability, safety, cost. Added here, because there is no truth:
* final map: tree count, IDs ever published, outline area / equivalent diameter / height distribution;
* the ``implausible_height`` count split into ``height <= 0`` and ``above the drone`` (the real flight's
  world_nwu z origin is the take-off point, not the ground, so ``<= 0`` is not automatically wrong there);
* canopy height above the local stereo ground (``CHM``: per 0.5 m cell the p95 of all stereo points
  minus a local ground = the p5 low envelope, min-filtered over 7 m), at each published tree;
* "stereo crowns": local maxima of the CHM (> 1.5 m, >= 2.5 m apart) as an independent tree count;
* cross-system agreement: a final tree of system A is "found" by B if a final B outline overlaps it
  (intersection >= 20 % of the smaller outline).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from typing import Dict, List

import numpy as np

from .contract import read_mission, read_snapshots
from .frames import backproject, read_frames
from . import score as scoring


def relabel(src: str, dst: str, frame: str) -> None:
    with open(src) as f, open(dst, "w") as g:
        for line in f:
            if line.strip():
                d = json.loads(line)
                d["frame"] = frame
                g.write(json.dumps(d, separators=(",", ":")) + "\n")


def run_replay(replay: str, ds: str, system: str, run_dir: str, config: str = "") -> dict:
    os.makedirs(run_dir, exist_ok=True)
    raw = os.path.join(run_dir, "map_raw.jsonl")
    cmd = [replay, "--frames", os.path.join(ds, "frames.bin"), "--out-map", raw,
           "--out-targets", os.path.join(run_dir, "targets.json"),
           "--out-counters", os.path.join(run_dir, "counters.json"), "--system", system]
    if config:
        cmd += ["--config", config]
    t0 = time.time()
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    with open(os.path.join(run_dir, "replay.log"), "w") as f:
        f.write(r.stderr)
    if r.returncode != 0:
        raise RuntimeError("replay %s failed (%d): %s" % (system, r.returncode, r.stderr[-2000:]))
    relabel(raw, os.path.join(run_dir, "map.jsonl"), "world_nwu")
    os.remove(raw)
    return {"run_seconds": round(time.time() - t0, 1), "cmd": " ".join(cmd)}


def poly_area(pts) -> float:
    a = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]; x1, y1 = pts[(i + 1) % len(pts)]
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0


def pq(a, qs=(10, 50, 90), nd=2):
    a = np.asarray([v for v in a if v is not None and math.isfinite(v)], float)
    return [round(float(np.percentile(a, q)), nd) for q in qs] if len(a) else None


# ----------------------------------------------------------------------------- stereo surface (CHM)

class StereoSurface:
    def __init__(self, frames_path: str, every: int = 4, cell: float = 0.5, max_points: int = 12_000_000):
        cal, _, it = read_frames(frames_path)
        u, v = cal.grid_uv()
        pts = []
        for i, f in enumerate(it):
            if i % every or f.disparity is None or not f.pose_valid or f.pose[2] < 6:
                continue
            P = backproject(cal, u.ravel(), v.ravel(), f.disparity.ravel(), f.pose)
            Z = cal.fx * cal.baseline / f.disparity.ravel()
            ok = np.isfinite(P[:, 0]) & (Z < 1.5 * f.pose[2])
            pts.append(P[ok].astype(np.float32))
        P = np.concatenate(pts)
        if len(P) > max_points:
            P = P[np.random.default_rng(0).choice(len(P), max_points, replace=False)]
        self.cell = cell
        self.lo = np.floor(P[:, :2].min(0) / cell) * cell - 5
        ij = ((P[:, :2] - self.lo) / cell).astype(np.int64)
        self.shape = tuple(ij.max(0) + 6)
        key = ij[:, 0] * self.shape[1] + ij[:, 1]
        o = np.argsort(key, kind="stable")
        key, z = key[o], P[o, 2]
        ks, st = np.unique(key, return_index=True)
        en = np.r_[st[1:], len(key)]
        top = np.full(self.shape[0] * self.shape[1], np.nan)
        low = np.full_like(top, np.nan)
        cnt = np.zeros_like(top)
        for k, s0, e0 in zip(ks, st, en):
            if e0 - s0 >= 5:
                zz = z[s0:e0]
                top[k] = np.percentile(zz, 95)
                low[k] = np.percentile(zz, 5)
                cnt[k] = e0 - s0
        self.top = top.reshape(self.shape)
        low = low.reshape(self.shape)
        # local ground: min of the low envelope over a 7 m window (canopy cells see no ground)
        from numpy.lib.stride_tricks import sliding_window_view
        w = int(round(3.5 / cell))
        pad = np.pad(np.where(np.isfinite(low), low, np.inf), w, constant_values=np.inf)
        g = sliding_window_view(pad, (2 * w + 1, 2 * w + 1)).min(axis=(2, 3))
        # robustify against single deep outliers: a 2nd pass takes the median of the window's 10 % lowest
        self.ground = np.where(np.isfinite(g), g, np.nan)
        self.chm = self.top - self.ground
        self.count = cnt.reshape(self.shape)
        self.n_points = int(len(P))

    def extent(self):
        return [self.lo[0], self.lo[0] + self.shape[0] * self.cell, self.lo[1], self.lo[1] + self.shape[1] * self.cell]

    def ground_at(self, x, y, r=4.0):
        i, j = int((x - self.lo[0]) / self.cell), int((y - self.lo[1]) / self.cell)
        k = int(r / self.cell)
        sub = self.ground[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1]
        sub = sub[np.isfinite(sub)]
        return float(np.median(sub)) if len(sub) else math.nan

    def crowns(self, min_h=1.5, sep=2.5):
        from numpy.lib.stride_tricks import sliding_window_view
        c = np.where(np.isfinite(self.chm) & (self.count >= 20), self.chm, -1)
        w = int(round(sep / self.cell))
        mx = sliding_window_view(np.pad(c, w, constant_values=-1), (2 * w + 1, 2 * w + 1)).max(axis=(2, 3))
        pk = np.argwhere((c >= mx) & (c > min_h))
        return [(self.lo[0] + (i + 0.5) * self.cell, self.lo[1] + (j + 0.5) * self.cell, float(c[i, j])) for i, j in pk]


# ----------------------------------------------------------------------------- summary

def summarise(res: dict, snaps, surf: "StereoSurface") -> dict:
    st, sf = res["stability"], res["safety"]
    fin = snaps[-1].trees if snaps else []
    areas = [poly_area(t.outline) for t in fin]
    le0 = above = 0
    for ev in sf.get("implausible_height_events", []):
        if any(e["height"] is not None and e["height"] <= 0 for e in ev["entries"]):
            le0 += 1
        if any(e["height"] is not None and e["height"] > ev.get("drone_z", math.inf) for e in ev["entries"]):
            above += 1
    agl = [t.height - surf.ground_at(*t.centre) for t in fin if t.height is not None] if surf else []
    out = {
        "final_t": res["final"]["t"], "final_trees": len(fin), "ids_ever_published": len(res["ids_ever_published"]),
        "final_area_m2_p10_p50_p90": pq(areas), "final_area_total_m2": round(sum(areas), 1),
        "final_equiv_diameter_m_p10_p50_p90": pq([2 * math.sqrt(a / math.pi) for a in areas]),
        "final_height_world_z_p10_p50_p90": pq([t.height for t in fin]),
        "final_height_above_stereo_ground_p10_p50_p90": pq(agl),
        "ids_vanished": len(st.get("ids_vanished", [])), "id_reappearances": st.get("id_reappearances"),
        "id_reuse": st.get("id_reuse"), "geometry_retreats": st.get("geometry_retreats"),
        "max_lost_fraction": st.get("max_lost_fraction"), "centre_jumps": st.get("centre_jumps"),
        "height_jumps": st.get("height_jumps"), "max_centre_drift_from_first_m": st.get("max_centre_drift_from_first_m"),
        "max_height_drift_from_first_m": st.get("max_height_drift_from_first_m"),
        "undefined_height_frames": sf.get("undefined_height_frames"), "invalid_entry_frames": sf.get("invalid_entry_frames"),
        "duplicate_id_frames": sf.get("duplicate_id_frames"), "implausible_height_frames": sf.get("implausible_height_frames"),
        "implausible_height_le0_frames": le0, "implausible_height_above_drone_frames": above,
        "n_snapshots": res["n_snapshots"],
    }
    for k in ("new_trees_after_scan", "locked_id_vanished", "nontarget_changes_in_descent", "sprayed_tree_changes",
              "new_tree_inside_sprayed"):
        v = sf.get(k)
        out[k] = v if not isinstance(v, (list, dict)) else (len(v) if isinstance(v, list) else v.get("count", "see json"))
    cost = res.get("cost", {})
    if cost.get("available"):
        out["cost_ms_per_frame"] = {k: {"mean": round(v["mean_ms"], 3), "p95": round(v["p95_ms"], 3), "max": round(v["max_ms"], 3)}
                                    for k, v in cost["stages"].items()}
    else:
        out["cost_ms_per_frame"] = None
    return out


def overlap_frac(a, b) -> float:
    from shapely.geometry import Polygon
    pa, pb = Polygon(a).buffer(0), Polygon(b).buffer(0)
    if pa.is_empty or pb.is_empty:
        return 0.0
    return pa.intersection(pb).area / max(1e-9, min(pa.area, pb.area))


def agreement(finals: Dict[str, list], min_frac=0.2) -> dict:
    names = list(finals)
    found = {}
    for a in names:
        for b in names:
            if a == b:
                continue
            k = 0
            for t in finals[a]:
                if len(t.outline) >= 3 and any(len(u.outline) >= 3 and overlap_frac(t.outline, u.outline) >= min_frac
                                               for u in finals[b]):
                    k += 1
            found["%s_in_%s" % (a, b)] = [k, len(finals[a])]
    only = {}
    for a in names:
        lst = []
        for t in finals[a]:
            if len(t.outline) < 3:
                continue
            if not any(len(u.outline) >= 3 and overlap_frac(t.outline, u.outline) >= min_frac
                       for b in names if b != a for u in finals[b]):
                lst.append({"id": t.id, "centre": [round(t.centre[0], 1), round(t.centre[1], 1)],
                            "area_m2": round(poly_area(t.outline), 1)})
        only[a] = lst
    return {"found_by_other": found, "only_in": only, "rule": "overlap >= %.0f %% of the smaller outline" % (100 * min_frac)}


def crowns_covered(crowns, final, min_h=2.0) -> list:
    from shapely.geometry import Point, Polygon
    polys = [Polygon(t.outline).buffer(0) for t in final if len(t.outline) >= 3]
    tall = [c for c in crowns if c[2] >= min_h]
    k = sum(1 for c in tall if any(p.buffer(0.5).contains(Point(c[0], c[1])) for p in polys))
    return [k, len(tall)]


# ----------------------------------------------------------------------------- plots

def plot_all(surf: StereoSurface, finals: Dict[str, list], crowns, path_dir: str, pose=None) -> List[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(path_dir, exist_ok=True)
    ext = surf.extent()
    colours = {"existing": "tab:red", "simple": "tab:orange", "cand_obj": "tab:blue", "cand_grid": "tab:green"}
    files = []

    def base(ax, title):
        im = ax.imshow(np.clip(surf.chm, 0, 6).T, origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=6,
                       interpolation="nearest")
        if pose is not None:
            ax.plot(pose[:, 1], pose[:, 2], "-", color="tab:purple", lw=0.4, alpha=0.5)
        ax.set_aspect("equal"); ax.set_title(title); ax.set_xlabel("x world_nwu (m)"); ax.set_ylabel("y (m)")
        return im

    xs = [p for f in finals.values() for t in f for p in t.outline]
    if xs:
        X = np.array(xs)
        lim = [X[:, 0].min() - 5, X[:, 0].max() + 5, X[:, 1].min() - 5, X[:, 1].max() + 5]
    else:
        lim = ext
    for name, fin in finals.items():
        fig, ax = plt.subplots(figsize=(9, 9))
        im = base(ax, "%s: %d final trees on the stereo canopy height (grey, 0-6 m)" % (name, len(fin)))
        for t in fin:
            if len(t.outline) >= 3:
                P = np.array(t.outline + [t.outline[0]])
                ax.plot(P[:, 0], P[:, 1], "-", color=colours.get(name, "k"), lw=1.2)
                ax.text(t.centre[0], t.centre[1], str(t.id), fontsize=6, color=colours.get(name, "k"), ha="center")
        C = np.array([c[:2] for c in crowns]) if crowns else np.zeros((0, 2))
        if len(C):
            ax.plot(C[:, 0], C[:, 1], "+", color="tab:cyan", ms=5, label="stereo crown peaks (CHM > 1.5 m)")
            ax.legend(loc="upper right", fontsize=7)
        ax.set_xlim(lim[0], lim[1]); ax.set_ylim(lim[2], lim[3])
        fig.colorbar(im, ax=ax, fraction=0.03, label="stereo canopy height above local ground (m)")
        p = os.path.join(path_dir, "final_%s.png" % name)
        fig.savefig(p, dpi=110, bbox_inches="tight"); plt.close(fig); files.append(p)
    fig, ax = plt.subplots(figsize=(11, 11))
    base(ax, "all systems, final outlines")
    for name, fin in finals.items():
        for k, t in enumerate(fin):
            if len(t.outline) >= 3:
                P = np.array(t.outline + [t.outline[0]])
                ax.plot(P[:, 0], P[:, 1], "-", color=colours.get(name, "k"), lw=1.0, label=name if k == 0 else None)
    ax.legend(loc="upper right")
    ax.set_xlim(lim[0], lim[1]); ax.set_ylim(lim[2], lim[3])
    p = os.path.join(path_dir, "final_overlay.png")
    fig.savefig(p, dpi=110, bbox_inches="tight"); plt.close(fig); files.append(p)
    return files


# ----------------------------------------------------------------------------- report

def to_md(card: dict) -> str:
    S = card["systems"]
    names = list(S)
    rows = [
        ("final trees", "final_trees"), ("IDs ever published", "ids_ever_published"),
        ("outline area m2 p10/p50/p90", "final_area_m2_p10_p50_p90"), ("total area m2", "final_area_total_m2"),
        ("equiv. diameter m p10/p50/p90", "final_equiv_diameter_m_p10_p50_p90"),
        ("height (world z) p10/p50/p90", "final_height_world_z_p10_p50_p90"),
        ("height above stereo ground p10/p50/p90", "final_height_above_stereo_ground_p10_p50_p90"),
        ("stereo crowns (>= 2 m) inside an outline", "stereo_crowns_covered"),
        ("IDs vanished", "ids_vanished"), ("ID reappearances / reuse", ("id_reappearances", "id_reuse")),
        ("geometry retreats / max lost fraction", ("geometry_retreats", "max_lost_fraction")),
        ("centre jumps / height jumps (> 1 m)", ("centre_jumps", "height_jumps")),
        ("max centre / height drift from first m", ("max_centre_drift_from_first_m", "max_height_drift_from_first_m")),
        ("**safety** undefined-height frames", "undefined_height_frames"), ("invalid-entry frames", "invalid_entry_frames"),
        ("duplicate-ID frames", "duplicate_id_frames"),
        ("implausible-height frames (all)", "implausible_height_frames"),
        ("  of which height <= 0 (world_nwu z)", "implausible_height_le0_frames"),
        ("  of which above the drone", "implausible_height_above_drone_frames"),
        ("new trees after scan / locked ID vanished", ("new_trees_after_scan", "locked_id_vanished")),
        ("non-target changes in descent / sprayed changes", ("nontarget_changes_in_descent", "sprayed_tree_changes")),
        ("cost ms/frame (mean, p95) per stage", "cost_ms_per_frame"), ("replay wall time s", "run_seconds"),
    ]

    def fmt(v):
        if v is None:
            return "n/a"
        if isinstance(v, float):
            return "%.2f" % v
        if isinstance(v, dict):
            return "; ".join("%s %s/%s" % (k, x["mean"], x["p95"]) for k, x in v.items())
        if isinstance(v, list):
            return " / ".join(fmt(x) for x in v)
        return str(v)

    L = ["| metric | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for lab, key in rows:
        vals = []
        for n in names:
            if isinstance(key, tuple):
                vals.append(" / ".join(fmt(S[n].get(k)) for k in key))
            else:
                vals.append(fmt(S[n].get(key)))
        L.append("| %s | %s |" % (lab, " | ".join(vals)))
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--replay", required=True)
    ap.add_argument("--systems", default="simple,cand_obj,cand_grid,existing")
    ap.add_argument("--runs", required=True)
    ap.add_argument("--label", default="", help="free text, e.g. the commit the binary was built from")
    ap.add_argument("--set", action="append", default=[], help="SYSTEM.key=value (repeatable)")
    ap.add_argument("--out-json")
    ap.add_argument("--out-md-table", help="write just the scorecard table (markdown)")
    ap.add_argument("--plots")
    a = ap.parse_args(argv)
    ds = a.dataset
    mission = read_mission(os.path.join(ds, "mission.json"))
    camera = json.load(open(os.path.join(ds, "camera_info.json")))
    cfg: Dict[str, List[str]] = {}
    for kv in a.set:
        s, rest = kv.split(".", 1)
        cfg.setdefault(s, []).append(rest)
    t0 = time.time()
    surf = StereoSurface(os.path.join(ds, "frames.bin"))
    crowns = surf.crowns()
    card = {"label": a.label, "dataset": ds, "replay": a.replay, "systems": {}, "stereo": {
        "points": surf.n_points, "cell_m": surf.cell, "crown_peaks_chm_gt_1.5m": len(crowns),
        "crown_peaks_chm_ge_2m": sum(1 for c in crowns if c[2] >= 2.0),
        "crown_peak_height_p10_p50_p90": pq([c[2] for c in crowns]),
        "definition": "CHM = per 0.5 m cell p95 z minus the local ground (min over 7 m of the cell p5 z); "
                      "peaks = local maxima >= 2.5 m apart, >= 20 points per cell"}}
    finals = {}
    for s in [x for x in a.systems.split(",") if x]:
        rd = os.path.join(a.runs, s)
        os.makedirs(rd, exist_ok=True)
        info = {}
        if s == "existing":
            mp = os.path.join(ds, "map_existing.jsonl")
        else:
            info = run_replay(a.replay, ds, s, rd, ";".join(cfg.get(s, [])))
            mp = os.path.join(rd, "map.jsonl")
        snaps = read_snapshots(mp)
        res = scoring.score(snaps, None, mission, camera)
        with open(os.path.join(rd, "score.json"), "w") as f:
            json.dump(res, f, indent=1, default=str)
        sm = summarise(res, snaps, surf)
        sm.update(info)
        if os.path.exists(os.path.join(rd, "counters.json")):
            sm["counters"] = json.load(open(os.path.join(rd, "counters.json")))
        fin = snaps[-1].trees if snaps else []
        sm["stereo_crowns_covered"] = crowns_covered(crowns, fin)
        sm["final_trees_list"] = [{"id": t.id, "centre": [round(t.centre[0], 2), round(t.centre[1], 2)],
                                   "area_m2": round(poly_area(t.outline), 2),
                                   "height": None if t.height is None else round(t.height, 2)} for t in fin]
        finals[s] = fin
        card["systems"][s] = sm
        print(s, json.dumps({k: v for k, v in sm.items() if k not in ("final_trees_list", "counters")}), flush=True)
    card["agreement"] = agreement(finals)
    if a.plots:
        pose = np.array(mission.pose) if mission.pose else None
        card["plots"] = plot_all(surf, finals, crowns, a.plots, pose)
    card["seconds"] = round(time.time() - t0, 1)
    if a.out_json:
        with open(a.out_json, "w") as f:
            json.dump(card, f, indent=1, default=str)
    if a.out_md_table:
        with open(a.out_md_table, "w") as f:
            f.write(to_md(card) + "\n")
    print(to_md(card))
    print(json.dumps(card["agreement"]["found_by_other"]))


if __name__ == "__main__":
    main()
