"""Stream the per-frame input contract (``frames.py``) out of the REAL orchard flight and its companion bag.

Runs inside tm_dev (ROS 1 ``rosbag`` API)::

    python3 -m treemapper_tools.real_frames --bag /data/real/BB_treehopping_farm.bag \
        --companion /data/real/BB_treehopping_farm_companion_2026-10-23-18-01-20.bag \
        --out /data/work/extracted/real_farm [--start S --end E]   (bag seconds)

Writes into ``--out``: ``frames.bin`` (+ ``.meta.json``), ``mission.json`` (pose only), ``camera_info.json``,
``body_tf.csv`` (world_nwu -> body, bag clock) and ``drone_position.csv`` (companion, for cross-checks).

What differs from the Isaac reader (``bag_frames.py``), and why:

* **Two bags, two clocks.** Header stamps are the original flight time (Oct 2025); bag time is the replay
  time (Jan 2026), ~6.55e6 s later. The companion bag was recorded while the flight bag was replayed, so its
  bag time is the flight bag's bag time (plus the detector's latency). Frame ``t`` = the companion bag time
  of the detection message (receive time, as in Isaac). Frame ``stamp`` and the pose clock = header stamp
  + ``clock_offset`` (constant: the minimum ``bag time - header stamp`` over ``/tf``), so every stamp the
  contract carries lives on ONE clock, the one the map stream and ``mission.json`` use.
* **Pairing**: detections (companion ``/object_detection/detections``) with disparity (flight bag
  ``/disparity_image_down_float``) by exact header stamp, else the nearest valid disparity within
  ``--nearest-tol`` (20 ms), else the frame is written without disparity (counted).
* **Pose**: ``world_nwu -> body`` from ``/tf``, interpolated at the header stamp (max gap 0.25 s). Missing
  -> ``pose_valid = False`` (the mapper holds), counted in ``pose_invalid_frames``; never a crash.
* **Camera chain**: ``/tf_static`` ``body -> sensor_down`` = lever arm (0.30, 0.08, -0.20) m, rotation
  75.0 deg about +y (15 deg off nadir, forward). The disparity/image frame is optical (x right, y down,
  z forward); ``sensor_down`` is a ROS-style frame (x along the optical axis) -- the existing pipeline applies
  the same OpenCV->ROS swap (``sensor_optical_frame: false``). So
  ``T_body_cam = [lever, Ry(pitch)] * R_ros_from_optical``. **The pitch is NOT the tf_static 75 deg**: the
  no-ground-truth checks (results/real_flight_geometry.md) put it at **82 deg** (8 deg off nadir, +-1.5):
  with 75 deg the ground tilts 6.7 deg (world) / 7.4 deg (heading frame) and tree positions drift along
  the heading by 0.14 m per metre of altitude; at 82 deg both vanish. ``--mount-pitch-deg`` (default 82)
  sets it; ``--mount-pitch-deg tf`` uses the tf_static rotation as recorded; ``--t-body-cam`` overrides all.
* **Calibration**: rectified left ``P`` (fx = fy = 477.15, cx = 322.72, cy = 200.06) and the baseline
  from the right ``P[3] = -fx*B``. The real driver stores it in **millimetres** like Isaac's
  (-76633.363 / 477.152 = 160.6 -> 0.1606 m); ``baseline_from_P`` deduces the unit (plausible 0.01-1 m).
* **Disparity offset**: the rectified principal points differ (left P cx = 322.72, right P cx = 317.80), so
  the published disparity carries a constant offset: Z = fx*B / (d - off) (the Q-matrix term the existing
  localizer applies with off = cx_l - cx_r = 4.918 px). The contract's Z = fx*B/d needs the corrected value,
  so the reader writes ``d - off`` (``correct_disparity``; <= 0 -> invalid, counted). Isaac has off = 0.
  Without it the depth scale is wrong by ~2.8x. **The data say off = 4.30 px, not 4.918**: a pose-z-free
  motion test (features tracked between frames, predicted from depth + /tf poses) fits off = 4.2-4.3 at the
  calibration's fx*B, and the ground stops sinking with altitude at off ~4.3-4.6; with 4.918 the depth is
  7 % (at 9 m) to 16 % (at 18 m) too long. The images are NOT rectified (left/right rows differ by 8.8 px
  raw, 1.0 px after rectifying with the published calibration), so the disparity node likely used a
  slightly different rectification than camera_info. ``--disparity-offset`` (default 4.30; ``calib`` =
  cx_l - cx_r). See results/real_flight_geometry.md.
* **No mission phase.** The real flight has no ``/mission_state`` and no hop target: every frame carries
  the constant phase ``SCAN`` and target -1, ``mission.json`` has no phases or targets. The locked-phase
  and descent rules (no new trees after the scan, hop-target freeze, sprayed trees) are therefore NOT
  exercised by this data; the scorer reports them ``n/a``.
* Heartbeat frames (no boxes, no disparity) every ``--heartbeat`` s where the detection stream has a gap.
"""
from __future__ import annotations

import argparse
import bisect
import collections
import json
import math
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .bag_frames import rot_to_quat
from .contract import Mission, write_mission
from .frames import Calibration, Frame, FrameWriter, interp_pose, quat_to_rot

DET = "/object_detection/detections"
POS = "/height_estimator/drone_position"
DISP = "/disparity_image_down_float"
TF, TFS = "/tf", "/tf_static"
CAM_L, CAM_R = "/cam/bottom/left/half/camera_info", "/cam/bottom/right/half/camera_info"
WORLD, BODY, MOUNT = "world_nwu", "body", "sensor_down"
PHASE = "SCAN"  # constant: the real flight has no mission phase (module docstring)
DISPARITY_OFFSET_PX = 4.30  # verified from the data; the calibration's cx_l - cx_r is 4.918
MOUNT_PITCH_DEG = 82.0  # verified from the data, not tf_static's 75.0 (results/real_flight_geometry.md)

# optical (x right, y down, z forward) -> ROS camera-body convention (x forward, y left, z up)
R_ROS_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def baseline_from_P(P_right: Sequence[float]) -> Tuple[float, str]:
    """Stereo baseline (m, positive) from the right rectified P (P[3] = -fx*B). The unit is deduced:
    metres if |B| lies in 0.01..1, millimetres if |B|/1000 does. Anything else raises."""
    fx = float(P_right[0])
    if not (fx > 0 and math.isfinite(P_right[3])):
        raise ValueError("bad right P: fx=%r P[3]=%r" % (fx, P_right[3]))
    b = abs(float(P_right[3]) / fx)
    if 0.01 <= b <= 1.0:
        return b, "m"
    if 0.01 <= b / 1000.0 <= 1.0:
        return b / 1000.0, "mm"
    raise ValueError("implausible stereo baseline %.6g (neither m nor mm in 0.01..1 m)" % b)


def mount_to_body_cam(t_body_mount: Sequence[float], q_body_mount: Sequence[float],
                      mount_is_optical: bool = False) -> List[float]:
    """body -> optical camera as [x y z qx qy qz qw]. ``mount_is_optical`` false = the mount frame is ROS
    convention (x along the optical axis) and the optical axes are swapped in."""
    R = quat_to_rot(q_body_mount)
    if not mount_is_optical:
        R = R @ R_ROS_FROM_OPTICAL
    return [float(v) for v in t_body_mount] + rot_to_quat(R)


def mount_pitch_to_body_cam(t_body_mount: Sequence[float], pitch_deg: float) -> List[float]:
    """body -> optical camera for a mount rotated `pitch_deg` about body +y (90 = nadir), ROS-style mount."""
    a = math.radians(pitch_deg)
    q = [0.0, math.sin(a / 2), 0.0, math.cos(a / 2)]
    return mount_to_body_cam(t_body_mount, q)


def optical_axis_in_body(T_body_cam: Sequence[float]) -> np.ndarray:
    return quat_to_rot(T_body_cam[3:]) @ np.array([0.0, 0.0, 1.0])


def off_nadir_deg(T_body_cam: Sequence[float]) -> float:
    a = optical_axis_in_body(T_body_cam)
    return math.degrees(math.acos(max(-1.0, min(1.0, -a[2]))))


def nearest_within(sorted_keys: Sequence[float], s: float, tol: float) -> Optional[float]:
    """The key closest to s if within tol (ties -> the earlier key), else None."""
    if not len(sorted_keys):
        return None
    k = bisect.bisect_left(sorted_keys, s)
    best = None
    for j in (k - 1, k):
        if 0 <= j < len(sorted_keys):
            d = abs(sorted_keys[j] - s)
            if d <= tol and (best is None or d < abs(best - s)):
                best = sorted_keys[j]
    return best


def heartbeat_times(det_t: Sequence[float], t_lo: float, t_hi: float, period: float) -> List[float]:
    """A heartbeat every `period` s at which the last detection frame is >= `period` old."""
    det_t = sorted(det_t)
    out, tt = [], t_lo
    while tt <= t_hi:
        k = bisect.bisect_right(det_t, tt)
        if k == 0 or tt - det_t[k - 1] >= period:
            out.append(tt)
        tt += period
    return out


def disparity_offset(P_left: Sequence[float], P_right: Sequence[float]) -> float:
    """cx_left - cx_right of the rectified projections: the constant the raw disparity carries."""
    return float(P_left[2]) - float(P_right[2])


def correct_disparity(d: np.ndarray, offset: float) -> np.ndarray:
    """Raw grid disparity -> d - offset (contract: Z = fx*B/d); <= 0 or non-finite -> NaN."""
    c = np.asarray(d, np.float32) - np.float32(offset)
    return np.where(np.isfinite(c) & (c > 0), c, np.nan).astype(np.float32)


def caminfo_dict(m) -> dict:
    return {"frame_id": m.header.frame_id, "width": m.width, "height": m.height,
            "distortion_model": m.distortion_model, "D": list(m.D), "K": list(m.K),
            "R": list(m.R), "P": list(m.P)}


def sample_grid(msg, stride: int):
    """32FC1 disparity message -> stride grid (NaN invalid), or None if malformed."""
    if msg.encoding != "32FC1" or msg.step != 4 * msg.width or len(msg.data) != msg.step * msg.height:
        return None
    o = stride // 2
    d = np.frombuffer(msg.data, dtype="<f4").reshape(msg.height, msg.width)[o::stride, o::stride]
    return np.where(np.isfinite(d) & (d > 0), d, np.nan).astype(np.float32)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bag", required=True)
    ap.add_argument("--companion", required=True)
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--start", type=float, default=None, help="bag seconds")
    ap.add_argument("--end", type=float, default=None, help="bag seconds")
    ap.add_argument("--heartbeat", type=float, default=0.2)
    ap.add_argument("--nearest-tol", type=float, default=0.02)
    ap.add_argument("--pose-max-gap", type=float, default=0.25)
    ap.add_argument("--disparity-offset", default=str(DISPARITY_OFFSET_PX),
                    help="px subtracted from the published disparity, or 'calib' for cx_left - cx_right")
    ap.add_argument("--mount-pitch-deg", default=str(MOUNT_PITCH_DEG),
                    help="mount rotation about body +y in deg (90 = nadir), or 'tf' for tf_static's rotation")
    ap.add_argument("--t-body-cam", type=float, nargs=7, default=None, help="override body->optical camera")
    ap.add_argument("--mission-pose-rate", type=float, default=10.0)
    a = ap.parse_args(argv)
    import rosbag
    import rospy

    t0 = time.time()
    os.makedirs(a.out, exist_ok=True)
    cnt = collections.Counter()
    kw = {}
    if a.start is not None:
        kw["start_time"] = rospy.Time.from_sec(a.start)
    if a.end is not None:
        kw["end_time"] = rospy.Time.from_sec(a.end)

    # ---- companion bag: detections + height-estimator position
    dets = []  # (bag t, header stamp, boxes)
    dpos = []  # (bag t, header stamp, x, y, z)
    with rosbag.Bag(a.companion) as cb:
        for topic, msg, bt in cb.read_messages(topics=[DET, POS], **kw):
            st = msg.header.stamp.to_sec()
            if topic == DET:
                cnt["detection_msgs"] += 1
                boxes = np.array([[b.x, b.y, b.width, b.height, b.score, b.label] for b in msg.bounding_boxes],
                                 dtype=np.float32).reshape(-1, 6)
                dets.append((bt.to_sec(), round(st, 6), boxes))
            else:
                dpos.append((bt.to_sec(), st, msg.x, msg.y, msg.z))

    # ---- flight bag: static mount, camera info, /tf body pose, disparity grids
    T_static = None
    cams: Dict[str, dict] = {}
    cam_changes = collections.Counter()
    tf_rows, tf_lat = [], []
    disp: Dict[float, np.ndarray] = {}
    with rosbag.Bag(a.bag) as b:
        for _, msg, _ in b.read_messages(topics=[TFS]):
            for tr in msg.transforms:
                if tr.header.frame_id == BODY and tr.child_frame_id == MOUNT:
                    p, q = tr.transform.translation, tr.transform.rotation
                    T_static = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        for topic, msg, bt in b.read_messages(topics=[TF, CAM_L, CAM_R, DISP], **kw):
            if topic == TF:
                for tr in msg.transforms:
                    if tr.header.frame_id == WORLD and tr.child_frame_id == BODY:
                        st = tr.header.stamp.to_sec()
                        p, q = tr.transform.translation, tr.transform.rotation
                        tf_rows.append((st, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                        tf_lat.append(bt.to_sec() - st)
            elif topic in (CAM_L, CAM_R):
                side = "left" if topic == CAM_L else "right"
                d = caminfo_dict(msg)
                if side not in cams:
                    cams[side] = d
                elif d["P"] != cams[side]["P"] or d["K"] != cams[side]["K"]:
                    cam_changes[side] += 1
            else:
                cnt["disparity_msgs"] += 1
                st = round(msg.header.stamp.to_sec(), 6)
                g = sample_grid(msg, a.stride)
                if g is None:
                    cnt["disparity_rejected_malformed"] += 1
                elif st in disp:
                    cnt["disparity_duplicate"] += 1
                else:
                    disp[st] = g
    if T_static is None and a.t_body_cam is None:
        raise SystemExit("no body->%s in /tf_static; pass --t-body-cam" % MOUNT)
    if "left" not in cams or "right" not in cams:
        raise SystemExit("camera_info missing")
    if not tf_rows:
        raise SystemExit("no %s->%s in /tf" % (WORLD, BODY))

    clock_offset = float(np.min(tf_lat))
    tf = np.array(sorted(tf_rows), dtype=np.float64)
    keep = np.concatenate([[True], np.diff(tf[:, 0]) > 1e-9])
    tf = tf[keep]
    tf_t, tf_v = tf[:, 0], tf[:, 1:]

    L, R = cams["left"], cams["right"]
    baseline, unit = baseline_from_P(R["P"])
    dcx_calib = disparity_offset(L["P"], R["P"])
    dcx = dcx_calib if a.disparity_offset == "calib" else float(a.disparity_offset)
    for k in list(disp):
        g = correct_disparity(disp[k], dcx)
        cnt["disparity_cells_invalid_after_offset"] += int((np.isfinite(disp[k]) & ~np.isfinite(g)).sum())
        disp[k] = g
    if a.t_body_cam:
        T, chain = list(a.t_body_cam), "--t-body-cam"
    elif a.mount_pitch_deg == "tf":
        T, chain = mount_to_body_cam(T_static[:3], T_static[3:]), "tf_static rotation"
    else:
        T, chain = mount_pitch_to_body_cam(T_static[:3], float(a.mount_pitch_deg)), \
            "tf_static lever arm, pitch %s deg (data-verified)" % a.mount_pitch_deg
    cal = Calibration(width=L["width"], height=L["height"], fx=L["P"][0], fy=L["P"][5], cx=L["P"][2],
                      cy=L["P"][6], baseline=baseline, stride=a.stride, T_body_cam=T, frame=WORLD,
                      note="real flight: body->optical = " + chain + "; tf_static body->%s %s; R_ros_from_optical; "
                           "baseline from right P[3] in %s; disparity corrected by -(cx_l - cx_r) = %.4f px; "
                           "constant phase %s" % (MOUNT, T_static, unit, -dcx, PHASE))
    gh, gw = cal.grid_shape

    # ---- frames
    disp_keys = sorted(disp)
    frames = []
    for bt, st, boxes in dets:
        if st in disp:
            ds, d = st, disp[st]
            cnt["frames_paired"] += 1
        else:
            ds = nearest_within(disp_keys, st, a.nearest_tol)
            if ds is not None:
                d = disp[ds]
                cnt["frames_paired_nearest"] += 1
            else:
                d = None
                ds = st
                cnt["frames_without_disparity"] += 1
        if d is not None and d.shape != (gh, gw):
            cnt["frames_disparity_resolution_mismatch"] += 1
            d = None
        pose = interp_pose(tf_t, tf_v, ds, a.pose_max_gap)  # pose at the disparity stamp: depth + pose agree
        if pose is None:
            cnt["frames_pose_missing"] += 1
        frames.append(Frame(bt, ds + clock_offset, pose if pose else (0, 0, 0, 0, 0, 0, 1), pose is not None,
                            PHASE, -1, (math.nan, math.nan), boxes, d))
    if not frames:
        raise SystemExit("no detection frames in the window")
    t_lo = frames[0].t if a.start is None else a.start
    t_hi = max(f.t for f in frames)
    empty = np.zeros((0, 6), np.float32)
    hb = heartbeat_times([f.t for f in frames], t_lo, t_hi, a.heartbeat)
    for t in hb:
        pose = interp_pose(tf_t, tf_v, t - clock_offset, a.pose_max_gap)
        if pose is None:
            cnt["heartbeat_pose_missing"] += 1
        frames.append(Frame(t, t, pose if pose else (0, 0, 0, 0, 0, 0, 1), pose is not None, PHASE, -1,
                            (math.nan, math.nan), empty, None))
    cnt["heartbeat_frames"] = len(hb)
    frames.sort(key=lambda f: (f.t, f.disparity is None))

    meta = {"source": os.path.abspath(a.bag), "companion": os.path.abspath(a.companion),
            "tf_static_body_sensor_down": T_static, "clock_offset_s": clock_offset,
            "tf_latency_s_p50_p99": [float(np.percentile(tf_lat, 50)), float(np.percentile(tf_lat, 99))],
            "baseline_m": baseline, "baseline_unit_in_P": unit, "disparity_offset_px_subtracted": dcx, "disparity_offset_calib_px": dcx_calib, "camera_info_changes": dict(cam_changes),
            "off_nadir_deg": off_nadir_deg(T), "camera_chain": chain, "T_body_cam": T, "phase": PHASE, "t_start": t_lo, "t_end": t_hi,
            "counts": dict(cnt), "pose_invalid_frames": sum(1 for f in frames if not f.pose_valid)}
    w = FrameWriter(os.path.join(a.out, "frames.bin"), cal, meta)
    for f in frames:
        w.write(f)
    w.close()
    meta["frames_written"] = w.n

    # ---- side files
    step = max(1, int(round(len(tf_t) / max(1.0, (tf_t[-1] - tf_t[0]) * a.mission_pose_rate))))
    pose = [(float(tf_t[i] + clock_offset), float(tf_v[i, 0]), float(tf_v[i, 1]), float(tf_v[i, 2]))
            for i in range(0, len(tf_t), step)]
    write_mission(os.path.join(a.out, "mission.json"), Mission([], [], pose))
    with open(os.path.join(a.out, "camera_info.json"), "w") as f:
        json.dump(cams, f, indent=1)
    with open(os.path.join(a.out, "body_tf.csv"), "w") as f:
        f.write("t,x,y,z,qx,qy,qz,qw\n")
        for row in tf:
            f.write(",".join("%.6f" % v for v in [row[0] + clock_offset] + list(row[1:])) + "\n")
    with open(os.path.join(a.out, "drone_position.csv"), "w") as f:
        f.write("t_bag,stamp,x,y,z\n")
        for row in dpos:
            f.write(",".join("%.6f" % v for v in row) + "\n")
    meta["seconds"] = round(time.time() - t0, 1)
    with open(os.path.join(a.out, "frames.bin.meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print(json.dumps(meta, indent=1))


if __name__ == "__main__":
    main()
