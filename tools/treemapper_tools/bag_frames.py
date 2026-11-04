"""Stream the per-frame input contract (``frames.py``) straight out of a recorded Isaac mission bag.

Runs inside tm_dev (ROS 1 ``rosbag`` API)::

    python3 -m treemapper_tools.bag_frames --bag /data/isaac/<bag>.bag \
        --extracted /data/work/extracted/<name> --out /data/work/frames/<name>.bin [--start 60 --end 90]

* frames = ``/object_detection/detections`` messages, paired with ``/disparity_image_down_float`` by
  **exact** header stamp. Every disparity stamp is published twice in this bag: one real 32FC1 frame
  (step = 4 * width) and one malformed copy (step = width, 256000 bytes, garbage floats). Malformed
  messages are rejected by size and counted (``disparity_rejected_malformed``); a second valid copy of
  a stamp is dropped as a duplicate (``disparity_duplicate``).
* drone pose = world -> body from ``/tf`` (``body_tf.csv`` of the M1 extraction), interpolated at the
  image stamp; frames outside the pose range get ``pose_valid = False``.
* phase / target = ``mission.json`` step lookup at the receive time. ``target_xy`` = drone xy at the start
  of the next SPRAYING phase after the target was set (the recorded hover point: where navigation
  actually descended for that target).
* heartbeat frames (no boxes, no disparity) every ``--heartbeat`` s while the detector sleeps and at every
  phase change, so a system can be driven purely by this stream.
* camera extrinsic: ``/tf_static`` parents the cameras to ``world``: it is the camera pose at spawn, not a
  mount. body -> optical camera is derived as inv(world->body at spawn) * tf_static (``static_to_body_cam``),
  verified against ground truth in results/geometry_check.md. Override with ``--t-body-cam x y z qx qy qz qw``.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import time

import numpy as np

from .contract import read_mission, step_lookup
from .frames import Calibration, Frame, FrameWriter, interp_pose

DET = "/object_detection/detections"
DISP = "/disparity_image_down_float"
TFS = "/tf_static"
LEFT_CAM = "bottom_left_camera"


def load_body_tf(path: str):
    a = np.loadtxt(path, delimiter=",", skiprows=1)
    a = a[np.argsort(a[:, 0], kind="stable")]
    keep = np.concatenate([[True], np.diff(a[:, 0]) > 1e-9])
    return a[keep, 0], a[keep, 1:]


def static_to_body_cam(T_world_cam, body0) -> list:
    """Isaac's /tf_static world->camera is the camera pose in the world AT SPAWN (body resting with a
    -3.4 deg pitch), not a body->camera mount. body->camera = inv(world->body at spawn) * world->camera.
    On the recorded mission this gives an exact nadir mount, R = [[0,-1,0],[-1,0,0],[0,0,-1]], lever arm
    (0.15, 0.08, 0.00) m; see results/geometry_check.md for the ground-truth verification."""
    from .frames import quat_to_rot
    Twb = np.eye(4); Twb[:3, :3] = quat_to_rot(body0[3:7]); Twb[:3, 3] = body0[:3]
    Twc = np.eye(4); Twc[:3, :3] = quat_to_rot(T_world_cam[3:]); Twc[:3, 3] = T_world_cam[:3]
    Tbc = np.linalg.inv(Twb) @ Twc
    return Tbc[:3, 3].tolist() + rot_to_quat(Tbc[:3, :3])


def rot_to_quat(R) -> list:
    """Rotation matrix -> quaternion [x, y, z, w] (Shepperd's method)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2; w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; x = 0.25 * s
        w = (R[2, 1] - R[1, 2]) / s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; y = 0.25 * s
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; z = 0.25 * s
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s
    return [float(x), float(y), float(z), float(w)]


def target_points(mission) -> list:
    """[(t_set, id, (x, y))]: hover xy at the first SPRAYING start after each target change."""
    pose_t = np.array([p[0] for p in mission.pose])
    out = []
    for t, tid in mission.targets:
        xy = (math.nan, math.nan)
        if tid >= 0:
            sp = next((ts for ts, ph in mission.phases if ts > t and ph == "SPRAYING"), None)
            # the target must still be the same one when that SPRAYING starts
            if sp is not None and step_lookup(mission.targets, sp, -1) == tid and len(pose_t):
                q = min(int(np.searchsorted(pose_t, sp)), len(pose_t) - 1)
                xy = (mission.pose[q][1], mission.pose[q][2])
        out.append((t, tid, xy))
    return out


class FrameMaker:
    """Builds ``Frame`` records: pose interpolated at the image stamp, phase / hop target looked up at the receive time."""

    def __init__(self, mission, tf_t, tf_rows):
        self.mission = mission
        self.tf_t, self.tf_rows = tf_t, tf_rows
        tpts = target_points(mission)
        self._tid = [(x[0], x[1]) for x in tpts]
        self._txy = [(x[0], x[2]) for x in tpts]

    def __call__(self, t, stamp, boxes, disp):
        pose = interp_pose(self.tf_t, self.tf_rows, stamp)
        ph = step_lookup(self.mission.phases, t, "") or ""
        tid = step_lookup(self._tid, t, -1)
        txy = step_lookup(self._txy, t, (math.nan, math.nan))
        return Frame(t, stamp, pose if pose else (0, 0, 0, 0, 0, 0, 1), pose is not None, ph,
                     -1 if tid is None else tid, txy, boxes, disp)


def boxes_from_bounding_boxes(msg) -> np.ndarray:
    """(N, 6) x, y, w, h, score, label of a spot_spraying_msgs/DetectionResult (the old Isaac bags)."""
    return np.array([[b.x, b.y, b.width, b.height, b.score, b.label] for b in msg.bounding_boxes],
                    dtype=np.float32).reshape(-1, 6)


def clean_disparity(grid) -> np.ndarray:
    """Sampled disparity values -> float32 grid, invalid (non-finite or <= 0) = NaN (``sampleDisparity``)."""
    d = np.asarray(grid, dtype=np.float32)
    return np.where(np.isfinite(d) & (d > 0), d, np.nan).astype(np.float32)


def pair_frames(messages, make, stride=4, nearest_tol=0.02, boxes_of=boxes_from_bounding_boxes, expect_wh=None,
                disp_topic=DISP, disparity_of=None):
    """Pair detections with disparity images by exact header stamp (then the nearest stamp within ``nearest_tol``).

    ``messages`` yields ``(topic, msg, t)`` in receive order, ``t`` in seconds; every topic but ``disp_topic`` is a
    detection stream (``boxes_of(msg)`` -> (N, 6) x, y, w, h, score, label). Returns ``(frames, counts)`` where
    ``frames`` are the paired ``Frame`` records in completion order and ``counts`` a ``collections.Counter``.
    ``expect_wh = (width, height)``: a disparity image of another size is rejected (``disparity_rejected_resolution``),
    as treemapper_node does; ``None`` keeps the old Isaac bags' behaviour (no size check).
    ``disparity_of(msg) -> (status, grid)``: for a disparity topic that does not carry a ``sensor_msgs/Image`` (the recorded
    decimated grid); ``status`` is ``"ok"`` (``grid`` = the sampled raw values), ``"malformed"`` or ``"resolution"``
    (rejected and counted as ``disparity_rejected_<status>``). ``None`` = the message is a full image, sampled here."""
    o = stride // 2
    cnt = collections.Counter()
    disp_by_stamp = collections.OrderedDict()
    pending = {}  # stamp -> (t, boxes)
    frames = []

    def resolve(s):
        tt, boxes = pending.pop(s)
        if disp_by_stamp:
            ks = np.fromiter(disp_by_stamp.keys(), dtype=np.float64)
            j = int(np.argmin(np.abs(ks - s)))
            if abs(ks[j] - s) <= nearest_tol:
                f = make(tt, float(ks[j]), boxes, disp_by_stamp[float(ks[j])])
                frames.append(f)
                cnt["frames_paired_nearest"] += 1
                return
        frames.append(make(tt, s, boxes, None))
        cnt["frames_without_disparity"] += 1

    for topic, msg, t in messages:
        st = round(msg.header.stamp.to_sec(), 6)
        if topic == disp_topic:
            cnt["disparity_msgs"] += 1
            if disparity_of is None:
                if msg.encoding != "32FC1" or msg.step != 4 * msg.width or len(msg.data) != msg.step * msg.height:
                    cnt["disparity_rejected_malformed"] += 1
                    continue
                if expect_wh is not None and (msg.width, msg.height) != tuple(expect_wh):
                    cnt["disparity_rejected_resolution"] += 1
                    continue
                grid = None
            else:
                status, grid = disparity_of(msg)
                if status != "ok":
                    cnt["disparity_rejected_" + status] += 1
                    continue
            if st in disp_by_stamp:
                cnt["disparity_duplicate"] += 1
                continue
            if grid is None:
                grid = np.frombuffer(msg.data, dtype="<f4").reshape(msg.height, msg.width)[o::stride, o::stride]
            d = clean_disparity(grid)
            disp_by_stamp[st] = d
            while len(disp_by_stamp) > 400:
                disp_by_stamp.popitem(last=False)
            if st in pending:
                tt, boxes = pending.pop(st)
                frames.append(make(tt, st, boxes, d))
                cnt["frames_paired_late"] += 1
        else:
            cnt["detection_msgs"] += 1
            boxes = boxes_of(msg)
            if st in disp_by_stamp:
                frames.append(make(t, st, boxes, disp_by_stamp[st]))
                cnt["frames_paired"] += 1
            else:
                pending[st] = (t, boxes)
        # detections that waited > 0.5 s for an exact-stamp disparity: nearest valid one within
        # `nearest_tol` (pose taken at the disparity stamp, so depth and pose stay consistent), else none
        for s in [s for s, (tt, _) in pending.items() if t - tt > 0.5]:
            resolve(s)
    for s in list(pending):
        resolve(s)
    return frames, cnt


def time_range(start, end, mission, frames):
    """(t_lo, t_hi) of the heartbeat grid: the requested window, else first phase .. last frame."""
    t_lo = start if start is not None else (mission.phases[0][0] if mission.phases else frames[0].t)
    t_hi = end if end is not None else max(f.t for f in frames)
    return t_lo, t_hi


def add_heartbeats(frames, make, mission, t_lo, t_hi, heartbeat, cnt) -> None:
    """Heartbeat frames: phase changes + every ``heartbeat`` s without a detection frame. Appends to ``frames``,
    sorts them by (t, has-no-disparity) in place and sets ``cnt['heartbeat_frames']``."""
    det_t = np.array(sorted(f.t for f in frames))
    hb = [t for t, _ in mission.phases if t_lo <= t <= t_hi]
    tt = t_lo
    while tt <= t_hi:
        k = int(np.searchsorted(det_t, tt, side="right"))
        if k == 0 or tt - det_t[k - 1] >= heartbeat:
            hb.append(tt)
        tt += heartbeat
    empty = np.zeros((0, 6), np.float32)
    for t in hb:
        frames.append(make(t, t, empty, None))
    cnt["heartbeat_frames"] = len(hb)
    frames.sort(key=lambda f: (f.t, f.disparity is None))


def write_frames(path, cal, frames, meta) -> int:
    """Write ``frames`` as a TMF1 file (``meta`` goes into its header). Returns the number of records."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    w = FrameWriter(path, cal, meta)
    for f in frames:
        w.write(f)
    w.close()
    return w.n


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bag", required=True)
    ap.add_argument("--extracted", required=True, help="M1 extraction dir (body_tf.csv, mission.json, camera_info.json)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--start", type=float, default=None)
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--heartbeat", type=float, default=0.2)
    ap.add_argument("--nearest-tol", type=float, default=0.02,
                    help="fallback pairing to the nearest disparity stamp within this many seconds")
    ap.add_argument("--t-body-cam", type=float, nargs=7, default=None)
    a = ap.parse_args(argv)
    import rosbag
    import rospy

    t0 = time.time()
    cam = json.load(open(os.path.join(a.extracted, "camera_info.json")))
    mission = read_mission(os.path.join(a.extracted, "mission.json"))
    tf_t, tf_rows = load_body_tf(os.path.join(a.extracted, "body_tf.csv"))
    bag = rosbag.Bag(a.bag)

    T = a.t_body_cam
    T_static = None
    if T is None:
        for _, msg, _ in bag.read_messages(topics=[TFS]):
            for tr in msg.transforms:
                if tr.child_frame_id == LEFT_CAM:
                    p, q = tr.transform.translation, tr.transform.rotation
                    T_static = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if T_static:
                break
        T = static_to_body_cam(T_static, tf_rows[0])
    L, R = cam["left"], cam["right"]
    fx = L["P"][0]
    baseline = -R["P"][3] / R["P"][0]
    if abs(baseline) > 1.0:  # the field driver's -fx*B in millimetres (CLAUDE.md of the existing system)
        baseline /= 1000.0
    cal = Calibration(width=L["width"], height=L["height"], fx=fx, fy=L["P"][5], cx=L["P"][2], cy=L["P"][6],
                      baseline=baseline, stride=a.stride, T_body_cam=list(T), frame="world",
                      note="Isaac: body->camera derived from /tf_static world->%s (spawn pose) and the spawn "
                           "body pose; tf_static raw = %s" % (LEFT_CAM, T_static))
    make = FrameMaker(mission, tf_t, tf_rows)
    kw = {}
    if a.start is not None:
        kw["start_time"] = rospy.Time.from_sec(a.start)
    if a.end is not None:
        kw["end_time"] = rospy.Time.from_sec(a.end)
    msgs = ((topic, msg, bt.to_sec()) for topic, msg, bt in bag.read_messages(topics=[DET, DISP], **kw))
    frames, cnt = pair_frames(msgs, make, a.stride, a.nearest_tol)
    bag.close()

    t_lo, t_hi = time_range(a.start, a.end, mission, frames)
    add_heartbeats(frames, make, mission, t_lo, t_hi, a.heartbeat, cnt)

    meta = {"source": os.path.abspath(a.bag), "tf_static_raw": T_static, "t_start": t_lo, "t_end": t_hi, "counts": dict(cnt),
            "pose_invalid_frames": sum(1 for f in frames if not f.pose_valid)}
    meta["frames_written"] = write_frames(a.out, cal, frames, dict(meta))
    meta["seconds"] = round(time.time() - t0, 1)
    with open(a.out + ".meta.json", "w") as f:
        json.dump(meta, f, indent=1)
    print(json.dumps(meta, indent=1))


if __name__ == "__main__":
    main()
