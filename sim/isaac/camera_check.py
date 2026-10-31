#!/usr/bin/env python
"""Isaac check of the camera model: emissive spheres at known world points, a tilted and yawed drone,
both cameras rendered from ``cameras.define_cameras``; measured sphere centroids vs the projection with
the LEFT P intrinsics (``frames.project_world_point``). Also the stereo disparity fx * B / Z.

    /isaac-sim/python.sh /ws/sim/isaac/camera_check.py --out /data/work/sim_runs/camera_check.json

Exit 1 if any centroid is more than ``--max-px`` (default 1.0) from its prediction. No ROS involved.
"""
import argparse
import json
import math
import os
import sys

_SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _SIM)
sys.path.insert(0, os.path.join(os.path.dirname(_SIM), "tools"))

ap = argparse.ArgumentParser()
ap.add_argument("--camera", default=os.path.join(_SIM, "specs", "camera.json"))
ap.add_argument("--out", default="")
ap.add_argument("--max-px", type=float, default=1.0)
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "width": 640, "height": 400})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: E402
from scipy import ndimage  # noqa: E402

from isaac import cameras, drone as drone_mod, frames  # noqa: E402
from isaac.controller import State  # noqa: E402

rig = frames.load_rig(args.camera)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")

mat = UsdShade.Material.Define(stage, "/World/mat")
sh = UsdShade.Shader.Define(stage, "/World/mat/sh")
sh.CreateIdAttr("UsdPreviewSurface")
sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1, 1, 1))
sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0, 0, 0))
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")

pose = State(2.0, -1.0, 10.0, 0, 0, 0, 0.05, -0.03, 0.4)   # x y z vx vy vz roll pitch yaw
R = frames.body_rotation(pose.roll, pose.pitch, pose.yaw)
UsdGeom.Xform.Define(stage, "/World/drone").AddTransformOp().Set(drone_mod._pose_matrix(pose))
cam_paths = cameras.define_cameras(stage, rig)

# world points on a grid below the drone (z 0 and 1.5 m)
pts = []
for dx in (-4.0, 0.0, 4.0):
    for dy in (-3.0, 3.0):
        pts.append((pose.x + dx, pose.y + dy, 1.5 if len(pts) % 2 else 0.0))
for i, p in enumerate(pts):
    s = UsdGeom.Sphere.Define(stage, "/World/s%d" % i)
    s.GetRadiusAttr().Set(0.12)
    UsdGeom.XformCommonAPI(s).SetTranslate(Gf.Vec3d(*p))
    UsdShade.MaterialBindingAPI(s.GetPrim()).Bind(mat)

anns = {}
for side, path in cam_paths.items():
    rp = rep.create.render_product(path, (rig.width, rig.height))
    a = rep.AnnotatorRegistry.get_annotator("rgb")
    a.attach([rp])
    anns[side] = a
for _ in range(20):
    app.update()
for _ in range(10):
    rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=False)

result = {"pose": [pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw], "points": {}}
worst = 0.0
meas = {}
for side, a in anns.items():
    img = np.asarray(a.get_data())[..., :3].astype(float).sum(2)
    lab, n = ndimage.label(img > 300)
    cents = [(c[1], c[0]) for c in ndimage.center_of_mass(img, lab, range(1, n + 1))]   # (u, v) pixel indices
    meas[side] = cents
    for i, p in enumerate(pts):
        u, v = frames.project_world_point(rig, [pose.x, pose.y, pose.z], R, p, side)
        if not (0 <= u < rig.width and 0 <= v < rig.height):
            continue
        if not cents:
            worst = 1e9
            continue
        c = min(cents, key=lambda q: (q[0] - u) ** 2 + (q[1] - v) ** 2)
        err = math.hypot(c[0] - u, c[1] - v)
        worst = max(worst, err)
        result["points"].setdefault(str(i), {})[side] = {"expected": [u, v], "measured": list(c), "err_px": err}
disp = []
for i, d in result["points"].items():
    if "left" in d and "right" in d:
        measured = d["left"]["measured"][0] - d["right"]["measured"][0]
        _, R_opt = rig.optical_pose_in_body("left")
        t_opt, _ = rig.optical_pose_in_body("left")
        pc = (R @ R_opt).T @ (np.array(pts[int(i)]) - (R @ t_opt + np.array([pose.x, pose.y, pose.z])))
        expected = rig.fx * rig.baseline_m / pc[2]
        disp.append({"point": int(i), "disparity_measured_px": measured, "disparity_expected_px": float(expected)})
        worst = max(worst, abs(measured - expected))
result["disparity"] = disp
result["max_err_px"] = worst
result["max_px_limit"] = args.max_px
result["ok"] = bool(worst <= args.max_px)
print("CAMERA_CHECK " + json.dumps(result), flush=True)
if args.out:
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=1)
app.close()
sys.exit(0 if result["ok"] else 1)
