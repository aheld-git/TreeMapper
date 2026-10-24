"""Real flight: stereo checks that need the images (not only frames.bin). Runs in tm_dev (rosbag, cv2).

    python3 -m treemapper_tools.real_stereo_check --bag /data/real/BB_treehopping_farm.bag \
        --dataset /data/work/extracted/real_farm --out-json results/real_flight_stereo_check.json

1. **Are the images rectified?** Corners tracked left -> right (LK) in the images as published and after
   rectifying both with the published (K, D, R, P): a rectified pair has equal rows (vertical offset 0).
2. **Depth scale from motion (independent of pose z / altitude).** Features tracked between left frames
   k and k+4 (~0.28 s; rectified images); their position in k+4 is predicted from the published disparity
   (Z = fx*B / (d - off)), the camera chain and the two /tf poses. A wrong depth scale mis-predicts the
   translational image motion. Fitted: (a) a depth scale s per pair at the reader's chain, per altitude
   band; (b) a joint grid over (fx*B, off) -- the two are correlated, so (b) is read at the calibration's
   fx*B. Inputs: the camera chain and offset in frames.bin's calibration (T_body_cam, reader meta).
"""
from __future__ import annotations

import argparse
import json
import math

import numpy as np

from .frames import interp_pose, quat_to_rot, read_frames

LEFT, RIGHT, DISP = "/cam/bottom/left/half/image_grey", "/cam/bottom/right/half/image_grey", "/disparity_image_down_float"


def rect_maps(c):
    import cv2
    K = np.array(c["K"]).reshape(3, 3); D = np.array(c["D"]); R = np.array(c["R"]).reshape(3, 3)
    P = np.array(c["P"]).reshape(3, 4)
    return cv2.initUndistortRectifyMap(K, D, R, P[:, :3], (c["width"], c["height"]), cv2.CV_32FC1)


def row_check(bag, cams, t0, t1, every=25):
    import cv2
    import rospy
    ml, mr = rect_maps(cams["left"]), rect_maps(cams["right"])
    L, R = {}, {}
    for tp, m, _ in bag.read_messages(topics=[LEFT, RIGHT], start_time=rospy.Time(t0), end_time=rospy.Time(t1)):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
        (L if tp == LEFT else R)[round(m.header.stamp.to_sec(), 6)] = img
    out = {}
    for name, rect in (("as_published", False), ("rectified_with_camera_info", True)):
        dy = []
        for k in sorted(set(L) & set(R))[::every]:
            l, r = L[k], R[k]
            if rect:
                l = cv2.remap(l, ml[0], ml[1], cv2.INTER_LINEAR); r = cv2.remap(r, mr[0], mr[1], cv2.INTER_LINEAR)
            p0 = cv2.goodFeaturesToTrack(l, 300, 0.01, 8)
            if p0 is None:
                continue
            g = p0.copy(); g[:, 0, 0] -= 8
            p1, st, _ = cv2.calcOpticalFlowPyrLK(l, r, p0, g.astype(np.float32), winSize=(15, 15), maxLevel=3,
                                                 flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
            ok = st[:, 0] == 1
            dy += list(p1[ok, 0, 1] - p0[ok, 0, 1])
        dy = np.array(dy)
        out[name] = {"matches": int(len(dy)), "vertical_offset_px_p10_p50_p90":
                     [round(float(np.percentile(dy, q)), 3) for q in (10, 50, 90)]}
    return out


def motion_check(bag, cams, cal, meta, tf_t, tf_v, windows, n_step=4, every=6):
    import cv2
    import rospy
    m1, m2 = rect_maps(cams["left"])
    P = np.array(cams["left"]["P"]).reshape(3, 4)
    fx, cx, cy = P[0, 0], P[0, 2], P[1, 2]
    fxB = cal.fx * cal.baseline
    off = meta["disparity_offset_px_subtracted"]
    T = cal.T_body_cam
    Rbc, tbc = quat_to_rot(T[3:]), np.asarray(T[:3])
    pairs, rows = [], []
    for w0, w1 in windows:
        L, D = {}, {}
        for tp, m, _ in bag.read_messages(topics=[LEFT, DISP], start_time=rospy.Time(w0), end_time=rospy.Time(w1)):
            st = round(m.header.stamp.to_sec(), 6)
            if tp == DISP:
                if m.step == 4 * m.width:
                    D[st] = np.frombuffer(m.data, "<f4").reshape(m.height, m.width)
            else:
                L[st] = cv2.remap(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width), m1, m2, cv2.INTER_LINEAR)
        ks = sorted(L)
        for i in range(0, len(ks) - n_step, every):
            k0, k1 = ks[i], ks[i + n_step]
            if k0 not in D:
                continue
            a, b = interp_pose(tf_t, tf_v, k0), interp_pose(tf_t, tf_v, k1)
            if a is None or b is None or a[2] < 8 or math.hypot(b[0] - a[0], b[1] - a[1]) < 0.3:
                continue
            f0 = cv2.goodFeaturesToTrack(L[k0], 400, 0.01, 10)
            if f0 is None:
                continue
            f1, s1, _ = cv2.calcOpticalFlowPyrLK(L[k0], L[k1], f0, None, winSize=(21, 21), maxLevel=3)
            fb, s2, _ = cv2.calcOpticalFlowPyrLK(L[k1], L[k0], f1, None, winSize=(21, 21), maxLevel=3)
            ok = (s1[:, 0] == 1) & (s2[:, 0] == 1) & (np.linalg.norm(fb[:, 0] - f0[:, 0], axis=1) < 0.5)
            u0, u1 = f0[ok, 0].astype(float), f1[ok, 0].astype(float)
            d = np.array([D[k0][int(y), int(x)] for x, y in u0])
            g = np.isfinite(d) & (d > off + 0.5)
            u0, u1, d = u0[g], u1[g], d[g]
            if len(u0) < 30:
                continue
            Rw0, tw0, Rw1, tw1 = quat_to_rot(a[3:]), np.asarray(a[:3]), quat_to_rot(b[3:]), np.asarray(b[:3])

            def predict(Z, u0=u0, Rw0=Rw0, tw0=tw0, Rw1=Rw1, tw1=tw1):
                pc = np.c_[(u0[:, 0] - cx) * Z / fx, (u0[:, 1] - cy) * Z / fx, Z]
                pw = (pc @ Rbc.T + tbc) @ Rw0.T + tw0
                pc1 = (((pw - tw1) @ Rw1) - tbc) @ Rbc
                return np.c_[fx * pc1[:, 0] / pc1[:, 2] + cx, fx * pc1[:, 1] / pc1[:, 2] + cy]

            Z0 = fxB / (d - off)
            err = lambda Z: float(np.median(np.linalg.norm(predict(Z) - u1, axis=1)))
            ss = np.arange(0.70, 1.40, 0.01)
            es = [err(Z0 * s) for s in ss]
            j = int(np.argmin(es))
            rows.append((a[2], float(ss[j]), es[j], err(Z0), err(Z0 * 1e4), float(np.median(Z0))))
            pairs.append((predict, u1, d, a[2]))  # d = published (raw) disparity
    A = np.array(rows)
    res = {"pairs": int(len(A)), "chain": T, "offset_px": off, "fxB": fxB,
           "depth_scale_s_p25_p50_p75": [round(float(np.percentile(A[:, 1], q)), 3) for q in (25, 50, 75)],
           "median_px_error_at_best_s / at s=1 / rotation only":
               [round(float(np.median(A[:, k])), 2) for k in (2, 3, 4)], "by_altitude": {}}
    for lo, hi in ((8, 12.5), (12.5, 15), (15, 17.5), (17.5, 25)):
        m = (A[:, 0] >= lo) & (A[:, 0] < hi)
        if m.sum():
            res["by_altitude"]["%g-%g m" % (lo, hi)] = {"n": int(m.sum()), "s_p50": round(float(np.median(A[m, 1])), 3),
                                                        "median_depth_m": round(float(np.median(A[m, 5])), 1)}
    grid = []
    for fb in np.arange(66.0, 84.1, 2.0):
        for o in np.arange(3.6, 5.31, 0.1):
            e = [float(np.median(np.linalg.norm(pr(fb / np.maximum(dd - o, 0.2)) - u1, axis=1))) for pr, u1, dd, _ in pairs]
            grid.append((round(float(np.mean(e)), 3), float(fb), round(float(o), 2)))
    grid.sort()
    near = min(grid, key=lambda g: abs(g[1] - fxB))[1]
    at = sorted([g for g in grid if g[1] == near])
    res["joint_fit_best"] = grid[:6]
    res["best_offset_at_calibration_fxB"] = {"fxB_grid": near, "best": at[0],
                                             "at_calib_offset_4.9": [g for g in at if abs(g[2] - 4.9) < 0.05]}
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bag", required=True)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--out-json")
    a = ap.parse_args(argv)
    import rosbag
    cams = json.load(open(a.dataset + "/camera_info.json"))
    meta = json.load(open(a.dataset + "/frames.bin.meta.json"))
    cal, _, _ = read_frames(a.dataset + "/frames.bin")
    tf = np.loadtxt(a.dataset + "/body_tf.csv", delimiter=",", skiprows=1)
    tf_t = tf[:, 0] - meta["clock_offset_s"]  # back to header stamps (images are looked up by header stamp)
    b = rosbag.Bag(a.bag)
    out = {"rows": row_check(b, cams, 1768476100, 1768476300)}
    print(json.dumps(out), flush=True)
    wins = [(w, w + 12) for w in (1768475960, 1768476000, 1768476060, 1768476120, 1768476180, 1768476230,
                                  1768476260, 1768476320, 1768476380, 1768476450)]
    out["motion"] = motion_check(b, cams, cal, meta, tf_t, tf[:, 1:], wins)
    print(json.dumps(out["motion"]), flush=True)
    if a.out_json:
        with open(a.out_json, "w") as f:
            json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
