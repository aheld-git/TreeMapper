"""Extract compact scoring/replay inputs from a recorded Isaac sim mission bag.

Runs inside the tm_dev container (needs the ROS 1 ``rosbag`` Python API)::

    python3 -m treemapper_tools.extract_bag \
        --bag /data/isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag \
        --out /data/work/extracted/isaac_2026-10-23-15-34-29

Writes (all in the sim ``world`` frame; ``world_nwu`` input is converted with
(x, y) -> (-y, x)):

* ``ground_truth.json``       ground-truth trees (contract.py format)
* ``map_existing.jsonl``      the existing system's /validation/fusion_snapshot stream (contract format)
* ``mission.json``            /mission_state, /spray_target, drone position
* ``active_target.json``      /object_fusion/active_target (what the existing system froze on)
* ``detections.jsonl``        /object_detection/detections (pixel boxes, half-res left image)
* ``body_tf.csv``             world->body transform from /tf (t, x, y, z, qx, qy, qz, qw)
* ``gt_tree_tf.json``         world->Tree_XXXX transforms (cross-check of ground truth)
* ``camera_info.json``        first left/right CameraInfo
* ``meta.json``               source, counts, frames seen, consistency checks

Images and disparity are NOT extracted (large); read them from the bag directly.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import sys
import time

from .contract import (GroundTruthTree, Mission, Snapshot, Tree, nwu_to_world, polygon_centroid,
                       write_ground_truth, write_mission, write_snapshots)

TOPICS = {
    "gt": "/validation/ground_truth",
    "snap": "/validation/fusion_snapshot",
    "phase": "/mission_state",
    "target": "/spray_target",
    "active": "/object_fusion/active_target",
    "pos": "/height_estimator/drone_position",
    "det": "/object_detection/detections",
    "tf": "/tf",
    "caminfo_l": "/cam/bottom/left/half/camera_info",
    "caminfo_r": "/cam/bottom/right/half/camera_info",
}


def to_world(frame: str, x: float, y: float):
    if frame == "world_nwu":
        return nwu_to_world(x, y)
    if frame in ("world", ""):
        return (x, y)
    raise ValueError("unknown map frame %r" % frame)


def polygon_msg_to_tree(p, frame: str) -> Tree:
    fr = p.header.frame_id or frame
    outline = [to_world(fr, v.x, v.y) for v in p.vertices]
    cx, cy = p.centroid.x, p.centroid.y
    if cx == 0.0 and cy == 0.0:  # "Zero = unset" per the message definition
        centre = polygon_centroid(outline)
    else:
        centre = to_world(fr, cx, cy)
    # max_height is the canopy top the existing system hands navigation as descent reference.
    return Tree(int(p.fusion_id), outline, centre, p.max_height, bool(p.sprayed))


def gt_signature(msg):
    return tuple((p.fusion_id, tuple((round(v.x, 3), round(v.y, 3)) for v in p.vertices),
                  round(p.max_height, 3)) for p in msg.polygons)


def caminfo_dict(m):
    return {"frame_id": m.header.frame_id, "width": m.width, "height": m.height,
            "distortion_model": m.distortion_model, "D": list(m.D), "K": list(m.K),
            "R": list(m.R), "P": list(m.P)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    import rosbag  # imported late so the rest of the package works without ROS

    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    bag = rosbag.Bag(a.bag)
    counts = collections.Counter()
    snap_frames = collections.Counter()
    gt_msg = None
    gt_sigs = set()
    snaps, phases, targets, active, pose, dets, body_tf = [], [], [], [], [], [], []
    tree_tf = {}
    caminfo = {}
    for topic, msg, t in bag.read_messages(topics=list(TOPICS.values())):
        ts = t.to_sec()
        counts[topic] += 1
        if topic == TOPICS["gt"]:
            if msg.polygons:
                gt_sigs.add(gt_signature(msg))
                if gt_msg is None:
                    gt_msg = msg
        elif topic == TOPICS["snap"]:
            fr = msg.header.frame_id
            snap_frames[(fr, bool(msg.polygons))] += 1
            snaps.append(Snapshot(ts, [polygon_msg_to_tree(p, fr) for p in msg.polygons], "world"))
        elif topic == TOPICS["phase"]:
            if not phases or phases[-1][1] != msg.data:
                phases.append((ts, msg.data))
        elif topic == TOPICS["target"]:
            if not targets or targets[-1][1] != msg.data:
                targets.append((ts, int(msg.data)))
        elif topic == TOPICS["active"]:
            if not active or active[-1][1] != msg.data:
                active.append((ts, int(msg.data)))
        elif topic == TOPICS["pos"]:
            x, y = to_world(msg.header.frame_id, msg.x, msg.y)
            pose.append((ts, x, y, float(msg.z)))
        elif topic == TOPICS["det"]:
            dets.append({"t": round(ts, 6), "stamp": round(msg.header.stamp.to_sec(), 6),
                         "boxes": [[b.x, b.y, b.width, b.height, b.label, round(b.score, 4)]
                                   for b in msg.bounding_boxes]})
        elif topic == TOPICS["tf"]:
            for tr in msg.transforms:
                c = tr.child_frame_id
                if tr.header.frame_id == "world" and c == "body":
                    q, p = tr.transform.rotation, tr.transform.translation
                    body_tf.append((tr.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                elif tr.header.frame_id == "world" and c.startswith("Tree_") and c not in tree_tf:
                    p = tr.transform.translation
                    tree_tf[c] = [p.x, p.y, p.z]
        elif topic in (TOPICS["caminfo_l"], TOPICS["caminfo_r"]):
            side = "left" if "left" in topic else "right"
            if side not in caminfo:
                caminfo[side] = caminfo_dict(msg)
    bag.close()
    if gt_msg is None:
        sys.exit("no non-empty ground truth in bag")

    gt = []
    fr = gt_msg.header.frame_id
    for p in gt_msg.polygons:
        outline = [to_world(p.header.frame_id or fr, v.x, v.y) for v in p.vertices]
        cx = sum(q[0] for q in outline) / len(outline)
        cy = sum(q[1] for q in outline) / len(outline)
        r = sum(math.hypot(q[0] - cx, q[1] - cy) for q in outline) / len(outline)
        gt.append(GroundTruthTree(int(p.fusion_id), outline, (cx, cy), r, float(p.max_height)))
    gt.sort(key=lambda g: g.id)

    write_ground_truth(os.path.join(a.out, "ground_truth.json"), gt)
    n_snap = write_snapshots(os.path.join(a.out, "map_existing.jsonl"), snaps)
    write_mission(os.path.join(a.out, "mission.json"), Mission(phases, targets, pose))
    with open(os.path.join(a.out, "active_target.json"), "w") as f:
        json.dump(active, f)
    with open(os.path.join(a.out, "detections.jsonl"), "w") as f:
        for d in dets:
            f.write(json.dumps(d, separators=(",", ":")) + "\n")
    with open(os.path.join(a.out, "body_tf.csv"), "w") as f:
        f.write("t,x,y,z,qx,qy,qz,qw\n")
        for row in body_tf:
            f.write(",".join("%.6f" % v for v in row) + "\n")
    with open(os.path.join(a.out, "gt_tree_tf.json"), "w") as f:
        json.dump(tree_tf, f, indent=1, sort_keys=True)
    with open(os.path.join(a.out, "camera_info.json"), "w") as f:
        json.dump(caminfo, f, indent=1)
    meta = {"bag": os.path.abspath(a.bag), "extract_seconds": round(time.time() - t0, 1),
            "counts": dict(counts), "snapshot_frames": {"%s/%s" % k: v for k, v in snap_frames.items()},
            "ground_truth_distinct_versions": len(gt_sigs), "n_snapshots": n_snap,
            "n_gt_trees": len(gt), "height_field_used": "max_height"}
    with open(os.path.join(a.out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print(json.dumps(meta, indent=1))


if __name__ == "__main__":
    main()
