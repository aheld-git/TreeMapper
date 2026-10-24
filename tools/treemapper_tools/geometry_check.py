"""Verify the camera geometry chain against ground truth.

Back-projects disparity grid points into the world with several ``T_body_cam`` hypotheses and measures
where they land relative to the ground-truth trees::

    python3 -m treemapper_tools.geometry_check --frames F.bin --gt ground_truth.json [--max-frames 500]

Per hypothesis:
* ground plane: points > 1 m outside every true canopy and below 1 m, plane fit per frame -> slope (the
  sim ground is flat, so any slope is a rotation error) and median height (should be ~0);
* canopy points (inside a detection box, z > 2 m): share inside a true canopy circle, distance outside;
* per box: the tallest 30 % of box points -> median xy -> distance to the nearest true centre;
* heights of all points inside true canopies (low hops, range < 12 m) vs the truth canopy height.
"""
from __future__ import annotations

import argparse
import json

import numpy as np

from .contract import read_ground_truth
from .frames import Calibration, backproject, read_frames


def hypotheses(cal: Calibration, meta: dict) -> dict:
    T = list(cal.T_body_cam)
    h = {"A derived: inv(world->body at spawn) * tf_static (reader default)": T,
         "B derived rotation, zero lever arm": [0.0, 0.0, 0.0] + T[3:]}
    raw = meta.get("tf_static_raw")
    if raw:
        q = raw[3:]
        h["C M1 guess: tf_static used directly as body->camera"] = list(raw)
        h["D M1 guess with inverted rotation"] = raw[:3] + [-q[0], -q[1], -q[2], q[3]]
    return h


def _r(v, n=3):
    return None if v is None else round(float(v), n)


def evaluate(cal: Calibration, frames, gt, T) -> dict:
    cal = Calibration(**{**cal.to_json(), "T_body_cam": list(T)})
    U, V = cal.grid_uv()
    cen = np.array([g.centre for g in gt]); rad = np.array([g.radius for g in gt]); hgt = np.array([g.height for g in gt])
    slopes, gz, inside, outd, box_err, low_z = [], [], [], [], [], []
    acc = [[] for _ in gt]  # canopy points (z > 2 m, in a box) assigned to the nearest true tree
    for fr in frames:
        P = backproject(cal, U, V, fr.disparity, fr.pose)
        ok = np.isfinite(P[..., 2])
        inbox = np.zeros(U.shape, bool)
        for x, y, w, h, s, _ in fr.boxes:
            m = ok & (U >= x) & (U <= x + w) & (V >= y) & (V <= y + h)
            inbox |= m
            if m.sum() >= 20:
                pts = P[m]
                top = pts[pts[:, 2] >= np.percentile(pts[:, 2], 70)]
                c = np.median(top[:, :2], axis=0)
                box_err.append(float(np.min(np.hypot(*(cen - c).T))))
        Q = P[ok]
        d = np.hypot(Q[:, None, 0] - cen[None, :, 0], Q[:, None, 1] - cen[None, :, 1])
        j = d.argmin(1); dm = d[np.arange(len(j)), j]
        g = Q[(dm - rad[j] > 1.0) & (Q[:, 2] < 1.0)]
        if len(g) > 200:
            A = np.c_[g[:, 0] - g[:, 0].mean(), g[:, 1] - g[:, 1].mean(), np.ones(len(g))]
            coef = np.linalg.lstsq(A, g[:, 2], rcond=None)[0]
            slopes.append(float(np.hypot(coef[0], coef[1]))); gz.append(float(np.median(g[:, 2])))
        cb = inbox[ok] & (Q[:, 2] > 2.0)
        for k in np.flatnonzero(cb)[::4]:
            if dm[k] <= rad[j[k]] + 1.0:
                acc[j[k]].append(Q[k, :2])
        if cb.sum():
            inside.append(float(np.mean(dm[cb] <= rad[j[cb]])))
            outd.append(float(np.mean(np.maximum(dm[cb] - rad[j[cb]], 0))))
        if fr.pose[2] < 12.0:
            low_z.append(Q[dm <= rad[j], 2] - hgt[j[dm <= rad[j]]])
    lz = np.concatenate(low_z) if low_z else np.array([np.nan])
    be = np.array(box_err)
    offs = [np.mean(a, axis=0) - cen[i] for i, a in enumerate(acc) if len(a) > 50]
    offn = [float(np.hypot(*o)) for o in offs]
    return {"ground_slope_deg_median": _r(np.degrees(np.arctan(np.median(slopes))), 2),
            "ground_slope_deg_p90": _r(np.degrees(np.arctan(np.percentile(slopes, 90))), 2),
            "ground_z_median_m": _r(np.median(gz)), "ground_z_p10_p90_m": [_r(np.percentile(gz, 10)), _r(np.percentile(gz, 90))],
            "canopy_points_inside_true_canopy": _r(np.mean(inside)),
            "canopy_points_mean_dist_outside_m": _r(np.mean(outd)),
            "per_tree_accumulated_canopy_centroid_offset_m": {"trees": len(offn), "mean": _r(np.mean(offn)),
                                                              "max": _r(np.max(offn)),
                                                              "mean_xy": [_r(v) for v in np.mean(offs, axis=0)]} if offn else None,
            "boxes": len(be), "box_top_xy_to_true_centre_median_m": _r(np.median(be)),
            "box_top_xy_to_true_centre_p90_m": _r(np.percentile(be, 90)),
            "in_canopy_z_minus_truth_height_low_range_p50_p90_p99_m": [_r(v, 2) for v in np.percentile(lz, [50, 90, 99])]
            if np.isfinite(lz).any() else None}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--frames", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--max-frames", type=int, default=500)
    ap.add_argument("--out-json")
    a = ap.parse_args(argv)
    gt = read_ground_truth(a.gt)
    cal, meta, it = read_frames(a.frames)
    fs = [f for f in it if f.disparity is not None and len(f.boxes) and f.pose_valid]
    step = max(1, len(fs) // a.max_frames)
    frames = fs[::step]
    phases = {}
    for f in frames:
        phases[f.phase] = phases.get(f.phase, 0) + 1
    out = {"frames_file": a.frames, "frames_used": len(frames), "frames_by_phase": phases,
           "altitude_range_m": [_r(min(f.pose[2] for f in frames), 2), _r(max(f.pose[2] for f in frames), 2)],
           "results": {}}
    for name, T in hypotheses(cal, meta).items():
        out["results"][name] = {"T_body_cam": [round(v, 5) for v in T], **evaluate(cal, frames, gt, T)}
    s = json.dumps(out, indent=1)
    print(s)
    if a.out_json:
        open(a.out_json, "w").write(s)


if __name__ == "__main__":
    main()
