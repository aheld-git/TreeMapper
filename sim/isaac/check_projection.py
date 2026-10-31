#!/usr/bin/env python3
"""Task 12 projection check: truth centre projected with the left P and body pose o mount vs the crown in the image.

    python3 sim/isaac/check_projection.py --run <run dir> --frame frame_hover1 --tree 5 [--max-px 10]

Reads ``<run>/truth.json``, ``<frame>.json`` (pose the sim saved with the image) and ``<frame>.png``.
The crown is found as the green blob (foliage; the check scene has a tan ground) nearest to the
projected centre; its centroid is compared with the projection of the tree's truth centre
(xy = hull centroid, z = 62.5 % of the tree height, the middle of the canopy volume).
With ``--enforce`` exit 1 if the offset exceeds ``--max-px`` or the crown touches the image border.
Runs under Python 3.8 (tm_sim_ros) or Isaac's 3.11; needs numpy and opencv.
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

_SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _SIM)
from isaac import frames  # noqa: E402


def green_mask(bgr):
    b, g, r = (bgr[..., i].astype(int) for i in range(3))
    return ((g > r + 8) & (g > b + 12)).astype(np.uint8)


def crown_centroid(mask, expect_uv, radius_px):
    """Centroid (u, v) of the closed green blob nearest to ``expect_uv``, plus its area in pixels."""
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(closed, connectivity=8)
    best, best_d = None, 1e18
    for i in range(1, n):
        ys, xs = np.nonzero(lab == i)
        d = np.min(np.hypot(xs - expect_uv[0], ys - expect_uv[1]))
        if stats[i, cv2.CC_STAT_AREA] > 0.2 * np.pi * radius_px ** 2 and d < best_d:
            best, best_d = i, d
    if best is None or best_d > radius_px:
        return None, 0
    ys, xs = np.nonzero(lab == best)
    touches = xs.min() == 0 or ys.min() == 0 or xs.max() == mask.shape[1] - 1 or ys.max() == mask.shape[0] - 1
    return (float(xs.mean()), float(ys.mean()), touches), int(len(xs))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--frame", required=True, help="frame name without extension, e.g. frame_hover1")
    ap.add_argument("--tree", type=int, required=True)
    ap.add_argument("--camera", default=os.path.join(_SIM, "specs", "camera.json"))
    ap.add_argument("--max-px", type=float, default=10.0)
    ap.add_argument("--enforce", action="store_true", help="exit 1 if the offset exceeds --max-px")
    ap.add_argument("--save-overlay", default="")
    args = ap.parse_args()

    rig = frames.load_rig(args.camera)
    truth = json.load(open(os.path.join(args.run, "truth.json")))
    tree = next(t for t in truth["trees"] if t["id"] == args.tree)
    fj = json.load(open(os.path.join(args.run, args.frame + ".json")))
    img = cv2.imread(os.path.join(args.run, args.frame + ".png"))
    R = frames.body_rotation(fj["roll"], fj["pitch"], fj["yaw"])
    z_c = tree["ground_z"] + 0.625 * (tree["top_z"] - tree["ground_z"])
    p_world = [tree["centre"][0], tree["centre"][1], z_c]
    u, v = frames.project_world_point(rig, [fj["x"], fj["y"], fj["z"]], R, p_world)
    # depth along the optical axis for the expected crown radius in pixels
    t_opt, R_opt = rig.optical_pose_in_body("left")
    cam_p = (R @ R_opt).T @ (np.array(p_world) - (R @ t_opt + np.array([fj["x"], fj["y"], fj["z"]])))
    r_px = tree["radius"] * rig.fx / cam_p[2]
    cen, area = crown_centroid(green_mask(img), (u, v), r_px)
    res = {"frame": args.frame, "tree": args.tree, "pose": [fj["x"], fj["y"], fj["z"], fj["yaw"]],
           "projected_uv": [u, v], "expected_radius_px": r_px, "depth_m": float(cam_p[2])}
    if cen is None:
        res["crown_found"] = False
        res["ok"] = False
    else:
        du, dv = cen[0] - u, cen[1] - v
        res.update({"crown_found": True, "crown_centroid_uv": [cen[0], cen[1]], "crown_area_px": area,
                    "crown_touches_border": bool(cen[2]), "offset_px": [du, dv],
                    "offset_norm_px": float(np.hypot(du, dv)),
                    "equivalent_radius_px": float(np.sqrt(area / np.pi))})
        res["ok"] = bool(np.hypot(du, dv) <= args.max_px and not cen[2])
    print(json.dumps(res, indent=1))
    if args.save_overlay and cen is not None:
        o = img.copy()
        cv2.circle(o, (int(round(u)), int(round(v))), 6, (0, 0, 255), 2)       # projected truth centre: red
        cv2.circle(o, (int(round(cen[0])), int(round(cen[1]))), 6, (255, 0, 0), 2)   # crown centroid: blue
        cv2.imwrite(args.save_overlay, o)
    if args.enforce and not res["ok"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
