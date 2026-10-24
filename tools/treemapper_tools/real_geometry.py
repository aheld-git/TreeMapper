"""Verify the real-flight camera chain WITHOUT ground truth (the real-flight analogue of geometry_check.py).

    python3 -m treemapper_tools.real_geometry --frames /data/work/extracted/real_farm/frames.bin \
        --out-json results/real_flight_geometry.json [--every 3]

Every test is run for several mount hypotheses (the chain in ``frames.bin`` plus deliberate alternatives),
so a number is only evidence if it separates the right chain from the wrong ones:

* ``A`` the chain in frames.bin (the reader's: lever arm from tf_static, mount pitch 82 deg, optical->ROS
  axis swap); ``T`` tf_static as recorded (75 deg about +y, 15 deg off nadir, forward);
* ``B`` exact nadir (90 deg), ``C`` 30 deg off nadir (60 deg), ``D`` 82 deg without the lever arm,
  ``E`` 75 deg with the image axes NOT swapped (sensor_down taken as an optical frame), ``F`` 82 deg on the
  RAW disparity (no principal-point offset correction); ``--extra-pitch`` adds pitch hypotheses;
* plus a pitch x roll grid search that minimises the median ground slope.

Tests:
1. **Ground plane** per frame: RANSAC plane through the points outside every (dilated) detection box.
   Slope = angle of its normal to world up (flat field -> ~0; a mount pitch/roll error tilts it by the
   error, in a direction fixed to the body, so it rotates with the heading). The tilt is reported in the
   world frame (terrain slope would be constant there) and in the heading frame (a mount error is constant
   there). Ground height vs drone altitude: a depth-scale (baseline / unit) error makes the ground follow
   the drone (d ground_z / d altitude = 1 - scale).
2. **Canopy heights**: per box fully inside the image, the p95 height above that frame's ground plane of
   the box's points.
3. **Repeatability across passes**: crown centroids (points > 1 m above ground in a box) are clustered
   into trees; per tree and pass (contiguous stretch of one altitude band and one 30-deg heading bin)
   the mean centroid; distance of every pass mean from the tree's median.
4. **Compact crowns**: accumulated crown points of each tree, distance from the tree centre (p50 / p90),
   and the box points' ground fraction.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

from .bag_frames import rot_to_quat
from .frames import Calibration, quat_to_rot, read_frames
from .real_frames import R_ROS_FROM_OPTICAL


def rot_y(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_x(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def mount(pitch_deg: float, roll_deg: float = 0.0, lever=(0.3, 0.08, -0.2), swap=True) -> List[float]:
    R = rot_y(pitch_deg) @ rot_x(roll_deg)
    if swap:
        R = R @ R_ROS_FROM_OPTICAL
    return list(lever) + rot_to_quat(R)


def hypotheses(dcx: float = 0.0, reader_T=None, dcx_calib: float = None) -> Dict[str, Tuple[List[float], float]]:
    """name -> (T_body_cam, disparity added back before triangulation). ``dcx`` = the offset the reader
    subtracted, ``dcx_calib`` = cx_l - cx_r of the calibration. F = the calibration's offset instead of the
    data-verified one; G = no offset at all (raw disparity, Z = fx*B/d)."""
    h = {"T_tf_static_75deg": (mount(75.0), 0.0), "B_nadir_90deg": (mount(90.0), 0.0),
         "C_60deg": (mount(60.0), 0.0), "D_82deg_no_lever_arm": (mount(82.0, lever=(0, 0, 0)), 0.0),
         "E_no_axis_swap": (mount(75.0, swap=False), 0.0)}
    if dcx_calib is not None:
        h["F_82deg_calib_offset"] = (mount(82.0), dcx - dcx_calib)
    h["G_82deg_raw_disparity"] = (mount(82.0), dcx)
    if reader_T is not None:
        h = dict([("A_reader_chain", (list(reader_T), 0.0))] + list(h.items()))
    return h


def cam_points(cal: Calibration, d: np.ndarray, add: float = 0.0):
    """Grid disparity (+ add) -> optical-frame points (N, 3) and their (u, v); invalid dropped."""
    u, v = cal.grid_uv()
    if add:
        d = d + add
    ok = np.isfinite(d) & (d > 0)
    Z = cal.fx * cal.baseline / d[ok]
    p = np.stack([(u[ok] - cal.cx) * Z / cal.fx, (v[ok] - cal.cy) * Z / cal.fy, Z], axis=-1)
    return p, u[ok], v[ok]


def to_world(p_opt, T_body_cam, pose):
    Rb = quat_to_rot(T_body_cam[3:]); tb = np.asarray(T_body_cam[:3])
    Rw = quat_to_rot(pose[3:7]); tw = np.asarray(pose[:3])
    return (p_opt @ Rb.T + tb) @ Rw.T + tw


def in_boxes(u, v, boxes, pad=0.0):
    m = np.zeros(len(u), bool)
    for x, y, w, h, _s, _l in boxes:
        m |= (u >= x - pad) & (u <= x + w + pad) & (v >= y - pad) & (v <= y + h + pad)
    return m


def ransac_plane(P, rng, iters=120, tol=0.25, n_sub=3000):
    """Plane n.p = c (n up-facing unit) with the most inliers; refined by least squares on them."""
    if len(P) < 50:
        return None
    Q = P[rng.choice(len(P), min(n_sub, len(P)), replace=False)]
    best, best_n = None, -1
    for _ in range(iters):
        a, b, c = Q[rng.choice(len(Q), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n /= nn
        if n[2] < 0:
            n = -n
        if n[2] < 0.5:  # > 60 deg: not a ground candidate
            continue
        k = int((np.abs(Q @ n - n @ a) < tol).sum())
        if k > best_n:
            best_n, best = k, (n, float(n @ a))
    if best is None:
        return None
    n, c = best
    inl = Q[np.abs(Q @ n - c) < tol]
    if len(inl) < 30:
        return None
    m = inl.mean(0)
    _, _, vt = np.linalg.svd(inl - m, full_matrices=False)
    n = vt[2] if vt[2][2] > 0 else -vt[2]
    return n, float(n @ m), len(inl) / len(Q)


def ground_plane(P, rng, cell=1.5, q=25, min_pts=20, tol=0.3, iters=120):
    """Low envelope -> plane: per `cell` m xy cell the q-th percentile z (the ground in canopy gaps),
    then RANSAC + least squares through those cell lows. Orchard canopy covers much of the view, so a
    plain plane fit through all non-box points often locks onto the canopy layer."""
    if len(P) < 200:
        return None
    ij = np.floor(P[:, :2] / cell).astype(np.int64)
    key = ij[:, 0] * 100003 + ij[:, 1]
    order = np.argsort(key, kind="stable")
    ks, starts = np.unique(key[order], return_index=True)
    ends = np.r_[starts[1:], len(order)]
    lows = []
    for s0, e0 in zip(starts, ends):
        if e0 - s0 >= min_pts:
            idx = order[s0:e0]
            z = P[idx, 2]
            zq = np.percentile(z, q)
            j = idx[np.argmin(np.abs(z - zq))]
            lows.append(P[j])
    if len(lows) < 8:
        return None
    L = np.array(lows)
    return ransac_plane(L, rng, iters=iters, tol=tol, n_sub=len(L))


def yaw_of(q):
    R = quat_to_rot(q)
    return math.atan2(R[1, 0], R[0, 0])


def pct(a, q):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if len(a) else None


def r3(v):
    return None if v is None else round(float(v), 3)


def find_trees(xy: np.ndarray, cell=0.25, sep=2.0, min_obs=8) -> np.ndarray:
    """Peaks of the smoothed 2D histogram of crown centroids, >= sep apart: tree seeds (k, 2)."""
    lo = xy.min(0) - 2; hi = xy.max(0) + 2
    nx, ny = (np.ceil((hi - lo) / cell)).astype(int) + 1
    H = np.zeros((nx, ny))
    ij = ((xy - lo) / cell).astype(int)
    np.add.at(H, (ij[:, 0], ij[:, 1]), 1.0)
    r = int(round(0.6 / cell))
    k1 = np.exp(-0.5 * (np.arange(-2 * r, 2 * r + 1) / max(r, 1)) ** 2)
    from numpy.lib.stride_tricks import sliding_window_view  # numpy >= 1.20
    Hs = np.apply_along_axis(lambda a: np.convolve(a, k1, "same"), 0, H)
    Hs = np.apply_along_axis(lambda a: np.convolve(a, k1, "same"), 1, Hs)
    w = int(round(sep / cell))
    pad = np.pad(Hs, w, constant_values=-1)
    mx = sliding_window_view(pad, (2 * w + 1, 2 * w + 1)).max(axis=(2, 3))
    pk = np.argwhere((Hs >= mx) & (Hs > 0))
    seeds = []
    for i, j in sorted(pk.tolist(), key=lambda p: -Hs[p[0], p[1]]):
        c = lo + (np.array([i, j]) + 0.5) * cell
        if all(np.hypot(*(c - s)) >= sep for s in seeds):
            seeds.append(c)
    seeds = np.array(seeds).reshape(-1, 2)
    if len(seeds):
        d = np.hypot(xy[:, None, 0] - seeds[None, :, 0], xy[:, None, 1] - seeds[None, :, 1])
        cnt = np.bincount(d.argmin(1)[d.min(1) < 1.5], minlength=len(seeds))
        seeds = seeds[cnt >= min_obs]
    return seeds


def analyse(frames, cal: Calibration, T: Sequence[float], rng, add: float = 0.0) -> dict:
    ground, obs = [], []
    for f in frames:
        p, u, v = cam_points(cal, f.disparity, add)
        rk = p[:, 2] < 1.5 * max(f.pose[2], 2.0)  # beyond: the far, quantisation-dominated fan (ground at <= 1.3 alt)
        p, u, v = p[rk], u[rk], v[rk]
        if len(p) < 200:
            continue
        P = to_world(p, T, f.pose)
        inb = in_boxes(u, v, f.boxes, pad=8.0)
        pl = ground_plane(P[~inb], rng)
        if pl is None:
            continue
        n, c, inl = pl
        yaw = yaw_of(f.pose[3:])
        cy_, sy_ = math.cos(-yaw), math.sin(-yaw)
        n_head = (cy_ * n[0] - sy_ * n[1], sy_ * n[0] + cy_ * n[1])  # normal xy in the heading frame
        gz = (c - n[0] * f.pose[0] - n[1] * f.pose[1]) / n[2]  # ground z below the drone
        ground.append((f.t, f.pose[2], math.degrees(math.acos(min(1.0, n[2]))), n[0], n[1],
                       n_head[0], n_head[1], gz, inl, yaw))
        hag = P @ n - c  # height above this frame's ground plane
        for b in f.boxes:
            x, y, w, h = b[:4]
            if x < 4 or y < 4 or x + w > cal.width - 4 or y + h > cal.height - 4:
                continue
            m = (u >= x) & (u <= x + w) & (v >= y) & (v <= y + h)
            if m.sum() < 15:
                continue
            hb = hag[m]
            top = pct(hb, 95)
            ztop = pct(P[m, 2], 95)
            cr = m & (hag > 1.0)
            if cr.sum() < 8 or top is None:
                obs.append((f.t, f.pose[2], yaw, top, math.nan, math.nan, math.nan, float((hb < 0.5).mean()), 0, ztop))
                continue
            Q = P[cr]
            cxy = np.median(Q[:, :2], 0)
            spread = float(np.median(np.hypot(Q[:, 0] - cxy[0], Q[:, 1] - cxy[1])))
            obs.append((f.t, f.pose[2], yaw, top, cxy[0], cxy[1], spread, float((hb < 0.5).mean()), int(cr.sum()), ztop))
    G = np.array(ground)
    O = np.array(obs, float).reshape(-1, 10)
    res = {"frames_used": len(G)}
    if len(G):
        slope = G[:, 2]
        A = np.c_[G[:, 1], np.ones(len(G))]
        k, b0 = np.linalg.lstsq(A, G[:, 7], rcond=None)[0]
        res["ground"] = {
            "slope_deg_p50_p90": [r3(pct(slope, 50)), r3(pct(slope, 90))],
            "tilt_world_mean_nxny": [r3(G[:, 3].mean()), r3(G[:, 4].mean())],
            "tilt_heading_frame_mean_fwd_left": [r3(G[:, 5].mean()), r3(G[:, 6].mean())],
            "tilt_world_mean_deg": r3(math.degrees(math.asin(min(1, math.hypot(G[:, 3].mean(), G[:, 4].mean()))))),
            "tilt_heading_frame_mean_deg": r3(math.degrees(math.asin(min(1, math.hypot(G[:, 5].mean(), G[:, 6].mean()))))),
            "ground_z_p10_p50_p90": [r3(pct(G[:, 7], 10)), r3(pct(G[:, 7], 50)), r3(pct(G[:, 7], 90))],
            "ground_z_vs_altitude_slope": r3(k), "implied_depth_scale": r3(1 - k),
            "inlier_fraction_p50": r3(pct(G[:, 8], 50)),
            "altitude_range": [r3(G[:, 1].min()), r3(G[:, 1].max())],
        }
        by_alt = {}
        for lo_, hi_ in ((5, 12.5), (12.5, 15), (15, 17.5), (17.5, 25)):
            m = (G[:, 1] >= lo_) & (G[:, 1] < hi_)
            if m.sum():
                by_alt["%g-%g m" % (lo_, hi_)] = {"n": int(m.sum()), "slope_p50": r3(pct(slope[m], 50)),
                                                  "ground_z_p50": r3(pct(G[m, 7], 50))}
        res["ground"]["by_altitude"] = by_alt
    ok = np.isfinite(O[:, 4]) if len(O) else np.zeros(0, bool)
    res["boxes_measured"] = int(len(O))
    if len(O):
        res["canopy_top_above_ground_p10_p50_p90"] = [r3(pct(O[:, 3], 10)), r3(pct(O[:, 3], 50)), r3(pct(O[:, 3], 90))]
        res["box_ground_fraction_p50"] = r3(pct(O[:, 7], 50))
        res["crown_spread_m_p50_p90"] = [r3(pct(O[ok, 6], 50)), r3(pct(O[ok, 6], 90))]
    if ok.sum() > 20:
        Ok = O[ok]
        seeds = find_trees(Ok[:, 4:6])
        res["trees_found"] = int(len(seeds))
        if len(seeds):
            d = np.hypot(Ok[:, None, 4] - seeds[None, :, 0], Ok[:, None, 5] - seeds[None, :, 1])
            tid = np.where(d.min(1) < 1.5, d.argmin(1), -1)
            res["obs_assigned_fraction"] = r3((tid >= 0).mean())
            # passes: contiguous stretches with one altitude band (1 m) and one 30-deg heading bin
            key = [(int(round(a)), int(math.floor((math.degrees(y) + 180) / 30))) for a, y in zip(Ok[:, 1], Ok[:, 2])]
            pid, cur, last_t = [], -1, None
            for i, kk in enumerate(key):
                if i == 0 or kk != key[i - 1] or Ok[i, 0] - last_t > 2.0:
                    cur += 1
                pid.append(cur); last_t = Ok[i, 0]
            pid = np.array(pid)
            dev, spreads, per_tree, alt_dev = [], [], [], []
            for t in range(len(seeds)):
                mt = tid == t
                if mt.sum() < 8:
                    continue
                med = np.median(Ok[mt, 4:6], 0)
                pm = []
                for p_ in np.unique(pid[mt]):
                    mp = mt & (pid == p_)
                    if mp.sum() >= 3:
                        pm.append((np.median(Ok[mp, 4:6], 0), np.median(Ok[mp, 1]), np.median(Ok[mp, 2])))
                if len(pm) < 2:
                    continue
                dd = [float(np.hypot(*(m_ - med))) for m_, _, _ in pm]
                dev += dd
                M = np.array([m_ for m_, _, _ in pm])
                spreads.append(float(np.max(np.hypot(M[:, None, 0] - M[None, :, 0], M[:, None, 1] - M[None, :, 1]))))
                per_tree.append(len(pm))
                alts = np.array([a_ for _, a_, _ in pm])
                if alts.max() - alts.min() > 3:
                    lo_m = M[alts < np.median(alts)].mean(0) if (alts < np.median(alts)).any() else None
                    hi_m = M[alts >= np.median(alts)].mean(0)
                    if lo_m is not None:
                        alt_dev.append(float(np.hypot(*(lo_m - hi_m))))
            res["repeatability"] = {
                "trees_seen_in_2plus_passes": len(per_tree),
                "passes_per_tree_p50": r3(pct(per_tree, 50)) if per_tree else None,
                "pass_mean_to_tree_median_m_p50_p90": [r3(pct(dev, 50)), r3(pct(dev, 90))] if dev else None,
                "max_pass_to_pass_m_p50_p90": [r3(pct(spreads, 50)), r3(pct(spreads, 90))] if spreads else None,
                "low_vs_high_altitude_mean_m_p50": r3(pct(alt_dev, 50)) if alt_dev else None,
            }
            # altitude-dependent drift: a mount pitch (roll) error d shifts an elevated point along the
            # heading (sideways) by ~tan(d) * (altitude - point z), so with tree fixed effects the slope of
            # the heading-frame residual vs altitude estimates the error directly; terrain slope cannot fake it
            rf, rl, ra, rz = [], [], [], []
            for t in range(len(seeds)):
                mt = tid == t
                if mt.sum() < 8 or np.ptp(Ok[mt, 1]) < 3.0:
                    continue
                dx = Ok[mt, 4] - np.median(Ok[mt, 4]); dy = Ok[mt, 5] - np.median(Ok[mt, 5])
                yw = Ok[mt, 2]
                rf += list(np.cos(yw) * dx + np.sin(yw) * dy); rl += list(-np.sin(yw) * dx + np.cos(yw) * dy)
                ra += list(Ok[mt, 1] - Ok[mt, 1].mean())
                rz += list(Ok[mt, 9] - np.median(Ok[mt, 9]))
            if len(ra) > 20:
                ra_, rf_, rl_ = np.array(ra), np.array(rf), np.array(rl)
                kf = float(np.polyfit(ra_, rf_, 1)[0]); kl = float(np.polyfit(ra_, rl_, 1)[0])
                kz = float(np.polyfit(ra_, np.array(rz), 1)[0])
                res["drift_vs_altitude"] = {"obs": len(ra), "fwd_m_per_m": r3(kf), "left_m_per_m": r3(kl),
                                            "crown_top_z_m_per_m": r3(kz), "implied_depth_scale_from_tops": r3(1 - kz),
                                            "implied_pitch_error_deg": r3(math.degrees(math.atan(kf))),
                                            "implied_roll_error_deg": r3(math.degrees(math.atan(kl)))}
            # compactness of accumulated crowns: obs centroids around their tree
            dist = d.min(1)[tid >= 0]
            res["obs_centroid_to_tree_m_p50_p90"] = [r3(pct(dist, 50)), r3(pct(dist, 90))]
            res["_seeds"] = seeds.tolist()
    res["_obs"] = O
    res["_ground"] = G
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--frames", required=True)
    ap.add_argument("--every", type=int, default=3)
    ap.add_argument("--min-alt", type=float, default=8.0)
    ap.add_argument("--grid-every", type=int, default=15, help="frame subsampling for the pitch/roll search")
    ap.add_argument("--extra-pitch", type=float, nargs="*", default=[], help="extra hypotheses: mount pitch (deg)")
    ap.add_argument("--skip-search", action="store_true")
    ap.add_argument("--out-json")
    ap.add_argument("--out-npz", help="keep the per-frame/per-box tables of hypothesis A (plots)")
    a = ap.parse_args(argv)
    cal, meta, it = read_frames(a.frames)
    fs = [f for f in it if f.disparity is not None and f.pose_valid and f.pose[2] > a.min_alt]
    sel = fs[::a.every]
    out = {"frames_file": a.frames, "frames_total_with_disparity_above_min_alt": len(fs),
           "frames_analysed": len(sel), "calibration": cal.to_json(),
           "reader_T_body_cam": cal.T_body_cam,
           "disparity_offset_px": meta.get("disparity_offset_px_subtracted", 0.0), "hypotheses": {}}
    hyp = hypotheses(out["disparity_offset_px"], cal.T_body_cam, meta.get("disparity_offset_calib_px"))
    for pitch in a.extra_pitch:
        hyp["P_pitch_%g" % pitch] = (mount(pitch), 0.0)
    for name, (T, add) in hyp.items():
        r = analyse(sel, cal, T, np.random.default_rng(0), add)
        if name.startswith("A_") and a.out_npz:
            np.savez_compressed(a.out_npz, obs=r["_obs"], ground=r["_ground"], seeds=np.array(r.get("_seeds", [])))
        out["hypotheses"][name] = {k: v for k, v in r.items() if not k.startswith("_")}
        print(name, json.dumps(out["hypotheses"][name]))
    if a.skip_search:
        if a.out_json:
            with open(a.out_json, "w") as f:
                json.dump(out, f, indent=1)
        return
    # pitch x roll search around A: minimise the median ground slope
    sub = fs[::a.grid_every]
    grid = []
    for pitch in np.arange(72.0, 90.01, 2.0):
        for roll in (-2.0, 0.0, 2.0):
            T = mount(float(pitch), float(roll))
            rng = np.random.default_rng(1)
            sl = []
            for f in sub:
                p, u, v = cam_points(cal, f.disparity)
                rk = p[:, 2] < 1.5 * max(f.pose[2], 2.0)
                p, u, v = p[rk], u[rk], v[rk]
                if len(p) < 200:
                    continue
                P = to_world(p, T, f.pose)
                pl = ground_plane(P[~in_boxes(u, v, f.boxes, 8.0)], rng, iters=60)
                if pl:
                    sl.append(math.degrees(math.acos(min(1.0, pl[0][2]))))
            grid.append((float(pitch), float(roll), float(np.median(sl)), float(np.percentile(sl, 90))))
    grid.sort(key=lambda g: g[2])
    out["pitch_roll_search"] = {"frames": len(sub), "best": grid[:5],
                                "all": grid,
                                "columns": "pitch_deg (75 = tf_static), roll_deg, slope_p50_deg, slope_p90_deg"}
    print("search", json.dumps(out["pitch_roll_search"]))
    if a.out_json:
        with open(a.out_json, "w") as f:
            json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
