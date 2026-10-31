#!/usr/bin/env python3
"""Moving-flight projection check: does the image content match the pose at the IMAGE STAMP?

Run in tm_sim_ros (ROS 1 only) against orchard_sim started with ``--v-max-xy 4`` on the check spec:

    docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/isaac/check_moving.py --tree 5 --out /data/work/sim_runs/<run>/moving

The drone is flown at a constant ``--speed`` (a moving carrot of setpoints) along +x straight across the tree at
``--z`` (15 m). Every published left image is kept; afterwards, for each frame in which the tree is well inside the image,

  * the pose of the left optical frame is a tf2 lookup ``world -> cam_left`` AT THE IMAGE STAMP (ROS ``/tf`` + ``/tf_static``),
  * the tree's truth centre (xy hull centroid, z at 62.5 % of the tree height) is projected with the left P,
  * the crown centroid is measured in the image (green blob, as ``check_projection.py``),
  * the offset (centroid - projection) is split into its component along the image direction of the flight and across it.

A whole-frame lag between image content and stamp (6 physics steps = 0.1 s = 0.4 m at 4 m/s, about 13 px at 15 m) would
show up as a positive along-track offset of that size. Exit 1 if |mean along-track offset| > ``--max-along`` px
(default 5) or fewer than ``--min-frames`` frames qualify.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Image

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)
from isaac import check_projection, frames  # noqa: E402
from sim_check import Link, tree_from_truth  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tree", type=int, default=5)
    ap.add_argument("--z", type=float, default=15.0)
    ap.add_argument("--speed", type=float, default=4.0)
    ap.add_argument("--lead-in", type=float, default=24.0, help="m before the tree where the carrot starts")
    ap.add_argument("--camera", default=os.path.join(os.path.dirname(_HERE), "specs", "camera.json"))
    ap.add_argument("--out", default="", help="directory for the in-view frames (PNG) and result.json")
    ap.add_argument("--max-along", type=float, default=5.0)
    ap.add_argument("--min-frames", type=int, default=8)
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("tm_check_moving", anonymous=True)
    while not rospy.is_shutdown() and rospy.get_time() == 0.0:
        rospy.sleep(0.1)

    rig = frames.load_rig(args.camera)
    buf = tf2_ros.Buffer(cache_time=rospy.Duration(300.0))
    tf2_ros.TransformListener(buf)
    tree = tree_from_truth(args.tree)
    tx, ty = tree["centre"]
    z_c = tree["ground_z"] + 0.625 * (tree["top_z"] - tree["ground_z"])
    p_world = np.array([tx, ty, z_c])

    link = Link()
    link.wait_ready()
    x0, x1 = tx - args.lead_in, tx + 16.0
    link.goto(x0, ty, args.z, 0.0)
    link.hold(3.0)

    kept = []
    flying = {"on": False}

    def on_image(msg):
        if flying["on"] and msg.encoding == "rgb8":
            kept.append((msg.header.stamp, np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()))

    rospy.Subscriber("/cam/bottom/left/half/image_raw", Image, on_image, queue_size=200, buff_size=2 ** 26)
    link.trace, link.record = [], True
    flying["on"] = True
    t0 = rospy.get_time()
    length = x1 - x0
    while not rospy.is_shutdown():
        s = (rospy.get_time() - t0) * args.speed
        if s >= length:
            break
        link.target = (x0 + s, ty, args.z, 0.0)
        rospy.sleep(0.05)
    link.target = (x1, ty, args.z, 0.0)
    link.hold(2.0)
    flying["on"] = False
    link.record = False
    rospy.sleep(1.0)   # the last frames and /tf_static

    rows = []
    for stamp, img in kept:
        try:
            T = buf.lookup_transform("world", "cam_left", stamp, rospy.Duration(0.5))
        except Exception as e:   # outside the buffer (before the first /tf, or lost): skip, reported below
            continue
        tr, q = T.transform.translation, T.transform.rotation
        R = frames.quat_to_rot([q.x, q.y, q.z, q.w])
        t = np.array([tr.x, tr.y, tr.z])

        def proj(p):
            return rig.project(R.T @ (p - t))

        try:
            u, v = proj(p_world)
        except ValueError:
            continue
        depth = float((R.T @ (p_world - t))[2])
        r_px = tree["radius"] * rig.fx / depth
        if not (r_px * 1.3 < u < rig.width - r_px * 1.3 and r_px * 1.3 < v < rig.height - r_px * 1.3):
            continue   # crown not fully inside the image
        cen, area = check_projection.crown_centroid(check_projection.green_mask(img[..., ::-1].copy()), (u, v), r_px)
        if cen is None or cen[2]:
            continue
        # image direction of +x (the direction of flight) at the tree
        u2, v2 = proj(p_world + np.array([1.0, 0.0, 0.0]))
        d = np.array([u2 - u, v2 - v])
        d /= np.linalg.norm(d)
        off = np.array([cen[0] - u, cen[1] - v])
        along = float(off @ d)
        cross = float(off @ np.array([-d[1], d[0]]))
        # speed from the TF trace around the stamp
        ts = stamp.to_sec()
        near = [p for p in link.trace if abs(p[0] - ts) < 0.3]
        speed = (near[-1][1] - near[0][1]) / (near[-1][0] - near[0][0]) if len(near) > 5 else float("nan")
        rows.append({"stamp": ts, "body_x": float(T.transform.translation.x), "speed_mps": speed,
                     "projected_uv": [u, v], "crown_uv": [cen[0], cen[1]], "offset_px": off.tolist(),
                     "along_track_px": along, "cross_track_px": cross, "_img": img})

    summary = {"tree": args.tree, "z": args.z, "speed_cmd": args.speed, "frames_received": len(kept),
               "frames_used": len(rows), "image_dir_of_flight": None}
    if rows:
        al = np.array([r["along_track_px"] for r in rows])
        cr = np.array([r["cross_track_px"] for r in rows])
        sp = np.array([r["speed_mps"] for r in rows if not math.isnan(r["speed_mps"])])
        summary.update({
            "mean_speed_mps": float(sp.mean()) if len(sp) else None,
            "along_track_px": {"mean": float(al.mean()), "std": float(al.std()), "min": float(al.min()), "max": float(al.max())},
            "cross_track_px": {"mean": float(cr.mean()), "std": float(cr.std()), "min": float(cr.min()), "max": float(cr.max())},
            "one_frame_lag_would_be_px": float(0.1 * (sp.mean() if len(sp) else args.speed) * rig.fx / (args.z - z_c)),
        })
    summary["max_along_px_limit"] = args.max_along
    summary["ok"] = bool(len(rows) >= args.min_frames and abs(summary["along_track_px"]["mean"]) <= args.max_along) if rows else False
    out = dict(summary)
    out["frames"] = [{k: v for k, v in r.items() if k != "_img"} for r in rows]
    print(json.dumps(summary, indent=1))
    if args.out:
        import cv2
        os.makedirs(args.out, exist_ok=True)
        for i, r in enumerate(rows):
            if i % max(1, len(rows) // 6) == 0:   # a handful of frames, not all
                cv2.imwrite(os.path.join(args.out, "moving_%02d.png" % i), r["_img"][..., ::-1])
        with open(os.path.join(args.out, "result.json"), "w") as f:
            json.dump(out, f, indent=1)
    if not summary["ok"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
