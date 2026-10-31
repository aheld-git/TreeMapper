#!/usr/bin/env python3
"""Detector parity: sim_perception's YOLOX pre/post-processing vs the existing detector's recorded boxes.

    python3 detector_parity.py --bag /data/isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag \
        --detections /data/work/extracted/isaac_2026-10-23-15-34-29/detections.jsonl \
        --model /data/work/models/yolox_s_isaac/yolox_s_isaac_epoch190_416_static.onnx --out parity.json

Picks ``--n-boxes`` frames that have recorded boxes and ``--n-empty`` frames without, spread evenly over the mission,
reads the left grey image with the identical header stamp from the bag (read-only, windowed reads), runs the model
with ``sim_perception.yolox`` (letterbox, grey replicated to 3 channels or 1 channel, raw 0..255) and compares, per
score threshold, with the recorded boxes (greedy IoU matching): recall of the recorded boxes at IoU >= 0.5, extra
boxes, mean IoU and mean centre offset of the matched pairs. Also reports whether the raw output looks grid-decoded.
Needs rosbag (Noetic) and onnxruntime; runs in tm_sim_ros.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from sim_perception import yolox  # noqa: E402
from sim_perception.image_utils import msg_to_grey  # noqa: E402

LEFT = "/cam/bottom/left/half/image_grey"


def iou(a, b):
    ax2, ay2, bx2, by2 = a[0] + a[2], a[1] + a[3], b[0] + b[2], b[1] + b[3]
    iw = max(0.0, min(ax2, bx2) - max(a[0], b[0]))
    ih = max(0.0, min(ay2, by2) - max(a[1], b[1]))
    inter = iw * ih
    u = a[2] * a[3] + b[2] * b[3] - inter
    return inter / u if u > 0 else 0.0


def match(ours, ref, thr=0.5):
    """Greedy by IoU: [(i_ours, j_ref, iou)] with iou >= thr."""
    pairs = sorted(((iou(o, r), i, j) for i, o in enumerate(ours) for j, r in enumerate(ref)), reverse=True)
    used_o, used_r, out = set(), set(), []
    for v, i, j in pairs:
        if v < thr:
            break
        if i in used_o or j in used_r:
            continue
        used_o.add(i)
        used_r.add(j)
        out.append((i, j, v))
    return out


def pick(records, n_boxes, n_empty):
    with_b = [r for r in records if r["boxes"]]
    without = [r for r in records if not r["boxes"]]

    def spread(lst, n):
        if n <= 0 or not lst:
            return []
        idx = np.unique(np.linspace(0, len(lst) - 1, min(n, len(lst))).round().astype(int))
        return [lst[i] for i in idx]
    return sorted(spread(with_b, n_boxes) + spread(without, n_empty), key=lambda r: r["stamp"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--detections", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--n-boxes", type=int, default=60)
    ap.add_argument("--n-empty", type=int, default=20)
    ap.add_argument("--input-size", type=int, nargs=2, default=[416, 640], help="(h, w)")
    ap.add_argument("--channels", type=int, nargs="+", default=[3, 1], help="channel variants to try")
    ap.add_argument("--thresholds", type=float, nargs="+", default=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    ap.add_argument("--nms", type=float, default=0.45)
    ap.add_argument("--device", default="cuda")
    a = ap.parse_args()

    import onnxruntime as ort
    import rosbag
    import rospy

    recs = [json.loads(line) for line in open(a.detections)]
    chosen = pick(recs, a.n_boxes, a.n_empty)
    providers = (["CUDAExecutionProvider"] if a.device == "cuda" else []) + ["CPUExecutionProvider"]
    sess = ort.InferenceSession(a.model, providers=providers)
    inp = sess.get_inputs()[0]
    print("model input %s %s, output %s, providers %s" % (inp.name, inp.shape, sess.get_outputs()[0].shape,
                                                         sess.get_providers()), flush=True)
    variants = [c for c in a.channels if c in (1, 3) and (c == inp.shape[1] or not isinstance(inp.shape[1], int))]

    frames = []
    with rosbag.Bag(a.bag) as bag:
        for r in chosen:
            st = r["stamp"]
            img = None
            for _t, msg, _bt in bag.read_messages(topics=[LEFT], start_time=rospy.Time.from_sec(max(0.0, st - 1.0)),
                                                  end_time=rospy.Time.from_sec(st + 1.0)):
                if abs(msg.header.stamp.to_sec() - st) < 1e-4:
                    img = msg_to_grey(msg)
                    break
            if img is None:
                print("no image with stamp %.6f" % st, flush=True)
                continue
            frames.append((r, img))
    print("%d of %d chosen frames found in the bag (%d with recorded boxes)" % (
        len(frames), len(chosen), sum(1 for r, _ in frames if r["boxes"])), flush=True)

    raw_info = None
    results = {}
    for ch in variants:
        preds = []
        for r, img in frames:
            tensor, ratio = yolox.preprocess(img, tuple(a.input_size), ch)
            raw = sess.run(None, {inp.name: tensor})[0]
            if raw_info is None:
                xy = raw[0, :, 0:2]
                wh = raw[0, :, 2:4]
                raw_info = {"xy_min": float(xy.min()), "xy_max": float(xy.max()), "wh_min": float(wh.min()),
                            "wh_max": float(wh.max()), "obj_max": float(raw[0, :, 4].max()),
                            "obj_min": float(raw[0, :, 4].min())}
                # decoded output: centres span the input image (hundreds of px); raw head: offsets of order 1
                raw_info["looks_decoded"] = bool(raw_info["xy_max"] > 50.0)
            preds.append((raw, ratio))
        decoded = raw_info["looks_decoded"]
        per_thr = {}
        for thr in a.thresholds:
            n_ref = n_ours = n_match = 0
            ious, offs, dscore = [], [], []
            empty_fp = 0
            per_frame = []
            for (r, _img), (raw, ratio) in zip(frames, preds):
                pred = raw if decoded else yolox.decode(raw, tuple(a.input_size))
                ours = yolox.postprocess(pred, ratio, thr, a.nms, [0])
                ref = [b for b in r["boxes"]]
                m = match([o[:4] for o in ours], [b[:4] for b in ref])
                per_frame.append({"stamp": r["stamp"], "recorded": len(ref), "ours": len(ours), "matched": len(m),
                                  "our_scores": [round(o[4], 4) for o in ours]})
                n_ref += len(ref)
                n_ours += len(ours)
                n_match += len(m)
                if not ref:
                    empty_fp += len(ours)
                for i, j, v in m:
                    o, b = ours[i], ref[j]
                    ious.append(v)
                    offs.append(math.hypot(o[0] + o[2] / 2 - (b[0] + b[2] / 2), o[1] + o[3] / 2 - (b[1] + b[3] / 2)))
                    dscore.append(o[4] - b[5])
            per_thr["%.2f" % thr] = {
                "recorded_boxes": n_ref, "our_boxes": n_ours, "matched": n_match,
                "match_rate": n_match / n_ref if n_ref else None,
                "precision_vs_recorded": n_match / n_ours if n_ours else None,
                "boxes_on_empty_frames": empty_fp,
                "mean_iou": float(np.mean(ious)) if ious else None,
                "mean_centre_offset_px": float(np.mean(offs)) if offs else None,
                "max_centre_offset_px": float(np.max(offs)) if offs else None,
                "mean_score_diff": float(np.mean(dscore)) if dscore else None,
                "frames": per_frame}
            print("channels %d thr %.2f: %s" % (ch, thr, json.dumps({k: v for k, v in per_thr["%.2f" % thr].items()
                                                                  if k != "frames"})), flush=True)
        results["channels_%d" % ch] = per_thr
    ref_scores = [b[5] for r, _ in frames for b in r["boxes"]]
    doc = {"bag": a.bag, "detections": a.detections, "model": a.model, "input_size": a.input_size,
           "frames": len(frames), "frames_with_boxes": sum(1 for r, _ in frames if r["boxes"]),
           "recorded_score_min": min(ref_scores) if ref_scores else None,
           "raw_output": raw_info, "variants": results}
    with open(a.out, "w") as f:
        json.dump(doc, f, indent=1)
    print("written %s" % a.out)


if __name__ == "__main__":
    main()
