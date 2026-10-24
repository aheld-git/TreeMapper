"""Measure input statistics on a recorded mission to calibrate the synthetic generator.

    python3 -m treemapper_tools.synth_calibration --dataset /data/work/extracted/<name> [--real-bag B --real-topic T]

Detector statistics against ground truth (each true tree's canopy projected into the image as the bbox of
its outline ring at canopy height): detection rate of fully visible trees, box-centre and size error
relative to the projected size, false-positive boxes per frame, boxes covering two trees (merged).
Disparity statistics: valid fraction, local roughness (std of the 2nd difference / sqrt(6), a
noise estimate insensitive to smooth surfaces), outlier fraction (|2nd diff| > 1 px).
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np

from .contract import read_ground_truth
from .frames import backproject, quat_to_rot, read_frames


def world_to_pixel(cal, pose, P):
    tb = np.asarray(cal.T_body_cam[:3]); Rb = quat_to_rot(cal.T_body_cam[3:])
    tw = np.asarray(pose[:3]); Rw = quat_to_rot(pose[3:7])
    pb = (P - tw) @ Rw          # world -> body (R^T (p - t))
    pc = (pb - tb) @ Rb         # body -> optical
    z = pc[:, 2]
    return np.c_[cal.fx * pc[:, 0] / z + cal.cx, cal.fy * pc[:, 1] / z + cal.cy], z


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    i = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def disparity_stats(disps):
    valid, rough, outl = [], [], []
    for d in disps:
        valid.append(float(np.isfinite(d).mean()))
        dd = d[:, 2:] - 2 * d[:, 1:-1] + d[:, :-2]
        dd = dd[np.isfinite(dd)]
        if len(dd) > 100:
            med = np.median(np.abs(dd))
            rough.append(float(1.4826 * med / np.sqrt(6)))
            outl.append(float(np.mean(np.abs(dd) > 1.0)))
    return {"frames": len(disps), "valid_fraction_mean": round(float(np.mean(valid)), 4),
            "noise_sigma_px_robust_median": round(float(np.median(rough)), 4) if rough else None,
            "second_diff_gt_1px_fraction": round(float(np.median(outl)), 5) if outl else None}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--canopy-z", type=float, default=4.0)
    ap.add_argument("--max-frames", type=int, default=1500)
    ap.add_argument("--real-bag", default=None)
    ap.add_argument("--real-topic", default="/disparity_image_down_float")
    ap.add_argument("--real-start", type=float, default=None)
    ap.add_argument("--out-json")
    a = ap.parse_args(argv)
    gt = read_ground_truth(os.path.join(a.dataset, "ground_truth.json"))
    cal, _, it = read_frames(os.path.join(a.dataset, "frames.bin"))
    fs = [f for f in it if f.disparity is not None and f.pose_valid and f.phase in ("SCAN", "HOPPING", "PLANNING")]
    fs = fs[:: max(1, len(fs) // a.max_frames)]
    n_vis = n_det = n_boxes = n_fp = n_merged = 0
    cerr, serr, scores_tp, scores_fp, nbox = [], [], [], [], []
    by_phase = {}
    for f in fs:
        proj = []
        for g in gt:
            ring = np.array([[x, y, a.canopy_z] for x, y in g.outline] + [[x, y, 0.5] for x, y in g.outline])
            px, z = world_to_pixel(cal, f.pose, ring)
            if (z <= 0).any():
                proj.append(None); continue
            b = [px[:, 0].min(), px[:, 1].min(), px[:, 0].max(), px[:, 1].max()]
            inside = b[0] >= 0 and b[1] >= 0 and b[2] <= cal.width and b[3] <= cal.height
            proj.append((b, inside))
        boxes = [[x, y, x + w, y + h, s] for x, y, w, h, s, _ in f.boxes]
        nbox.append(len(boxes))
        n_boxes += len(boxes)
        used = set()
        for j, p in enumerate(proj):
            if p is None or not p[1]:
                continue
            n_vis += 1
            st = by_phase.setdefault(f.phase, [0, 0]); st[0] += 1
            best = max(range(len(boxes)), key=lambda i: iou(boxes[i], p[0]), default=None)
            if best is not None and iou(boxes[best], p[0]) >= 0.3:
                n_det += 1; st[1] += 1
                used.add(best)
                b, q = boxes[best], p[0]
                sz = np.array([q[2] - q[0], q[3] - q[1]])
                cerr.append((np.array([(b[0] + b[2]) / 2 - (q[0] + q[2]) / 2, (b[1] + b[3]) / 2 - (q[1] + q[3]) / 2]) / sz).tolist())
                serr.append((np.array([b[2] - b[0], b[3] - b[1]]) / sz).tolist())
                scores_tp.append(b[4])
        for i, b in enumerate(boxes):
            cover = [j for j, p in enumerate(proj) if p is not None and iou(b, p[0]) >= 0.1
                     and (min(b[2], p[0][2]) - max(b[0], p[0][0])) * (min(b[3], p[0][3]) - max(b[1], p[0][1]))
                     >= 0.5 * (p[0][2] - p[0][0]) * (p[0][3] - p[0][1])]
            if len(cover) >= 2:
                n_merged += 1
            if all(p is None or iou(b, p[0]) < 0.1 for p in proj):
                n_fp += 1; scores_fp.append(b[4])
    cerr = np.array(cerr); serr = np.array(serr)
    out = {"frames": len(fs),
           "detector": {
               "fully_visible_tree_instances": n_vis,
               "detection_rate_iou30": round(n_det / max(1, n_vis), 4),
               "detection_rate_by_phase": {k: round(v[1] / max(1, v[0]), 4) for k, v in by_phase.items()},
               "boxes_per_frame_mean": round(float(np.mean(nbox)), 3),
               "false_positive_boxes_per_frame": round(n_fp / max(1, len(fs)), 4),
               "boxes_covering_two_trees_fraction": round(n_merged / max(1, n_boxes), 4),
               "centre_error_rel_std_xy": np.round(cerr.std(axis=0), 4).tolist() if len(cerr) else None,
               "centre_error_rel_mean_xy": np.round(cerr.mean(axis=0), 4).tolist() if len(cerr) else None,
               "size_ratio_mean_wh": np.round(serr.mean(axis=0), 4).tolist() if len(serr) else None,
               "size_ratio_std_wh": np.round(serr.std(axis=0), 4).tolist() if len(serr) else None,
               "score_tp_p10_p50_p90": np.round(np.percentile(scores_tp, [10, 50, 90]), 3).tolist() if scores_tp else None,
               "score_fp_p10_p50_p90": np.round(np.percentile(scores_fp, [10, 50, 90]), 3).tolist() if scores_fp else None,
               "note": "projected truth = bbox of the outline ring at z=%.1f and z=0.5 m" % a.canopy_z},
           "disparity_isaac": disparity_stats([f.disparity for f in fs[::5]])}
    if a.real_bag:
        import rosbag, rospy
        bag = rosbag.Bag(a.real_bag)
        kw = {"start_time": rospy.Time.from_sec(a.real_start)} if a.real_start else {}
        ds = []
        for _, msg, _ in bag.read_messages(topics=[a.real_topic], **kw):
            if msg.step != 4 * msg.width:
                continue
            d = np.frombuffer(msg.data, dtype="<f4").reshape(msg.height, msg.width)[2::4, 2::4]
            ds.append(np.where(np.isfinite(d) & (d > 0), d, np.nan))
            if len(ds) >= 60:
                break
        out["disparity_real"] = {"bag": a.real_bag, "topic": a.real_topic,
                                 "width_height": [int(msg.width), int(msg.height)], **disparity_stats(ds)}
    s = json.dumps(out, indent=1)
    print(s)
    if a.out_json:
        open(a.out_json, "w").write(s)


if __name__ == "__main__":
    main()
