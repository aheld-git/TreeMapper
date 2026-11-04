"""Turn an Isaac closed-loop run directory into a TMF1 frame file: what ``treemapper_node`` consumed in that run.

Runs inside the Noetic image (``rosbag``), after ``mission_score`` has scored the run::

    python3 -m treemapper_tools.sim_frames --run <run dir> [--out <run dir>/frames.bin] [--filter-bag]

The run was recorded with ``scripts/sim_mission.sh --record-frames``: ``<run>/mission.bag`` holds, besides the normal
allow-list, ``/rec/disparity_grid`` (``treemapper_msgs/DisparityGrid``: only the pixels the node samples, ~64 kB instead of
the ~1 MB image, made by ``sim_perception/disparity_grid_node.py``) and the left / right ``camera_info``. A bag that holds the
full ``/disparity_image_down_float`` instead (older recordings, tests) is read as well; the frames are identical. Output: ``frames.bin`` (TMF1, replayed by
``treemapper_replay``) and ``frames.bin.meta.json`` with ``frames``, ``frames_with_disparity``, ``pose_invalid``,
``disparity_rejected``, ``t_start``, ``t_end`` (+ counts and provenance). Exit status 0 = written and ``check()`` passed,
1 = written but ``check()`` failed (the reasons go to stderr), 2 = could not convert. ``check()`` fails on: no frames, no
detection paired with a disparity, fewer than 80 % of the detection frames paired, any rejected (malformed / wrong-size)
disparity, more than 1 % of the frames without a pose.

``--filter-bag``: only when ``check()`` passed, ``mission.bag`` is replaced by a copy without the disparity (grid or
image) and camera_info topics (a normal run bag again: ``mission_score --run`` works on it unchanged). Any failure keeps the bag.

How the live node's inputs are mirrored (``ros/treemapper_ros/src/treemapper_node.cpp``, ``input_sync.cpp``):

* calibration = the first left / right ``camera_info`` (width, height, fx, fy, cx, cy from left ``P``; baseline from
  right ``P[3] / P[0]`` with the node's unit rules). Messages received before both had arrived are dropped, as the
  node has no mapper before that (``frames_dropped_no_calibration``): the window starts at that instant.
* ``T_body_cam`` = ``sim/specs/camera.json`` ``mount`` (optical -> body): the node runs with ``camera_mount_source: param``
  and that mount, not with ``/tf_static`` (which is only compared with it, as a warning).
* frames = ``/object_detection/detections`` (``treemapper_msgs/Detection2DArray``), paired with
  ``/disparity_image_down_float`` by exact header stamp, else the nearest stamp within 20 ms, by ``bag_frames.pair_frames``
  (the same rules as ``FrameSync``); a disparity of another size or a malformed one is rejected and counted, as the node does.
* the disparity is sampled on the stride-4 grid at ``(r*4 + 2, c*4 + 2)`` like ``sampleDisparity``; no offset is subtracted
  (``disparity_offset: none`` in the sim launch). From a ``DisparityGrid`` the same values come from the recorded samples; its
  ``src_*`` fields (the image's own size, encoding, step, endianness) give the node's malformed / wrong-size verdict, so
  ``disparity_rejected`` is the same as from the full images.
* pose = ``/tf`` world -> body interpolated at the image stamp; phase and hop target = the ``/mission_state`` and
  ``/spray_target`` values in force when the detection was received (``mission.json`` of the run dir if present, else
  derived from the bag the same way ``mission_score`` does).
* heartbeat frames at phase changes and every 0.2 s without a detection frame (``--heartbeat 0`` = none).

What TMF1 cannot carry (replay is close to the live map, not byte-identical to it): the disparity is stored as
``u16 / 256`` (the live sim node runs with ``disparity_quantum_px: 0``, full float precision); heartbeat frames are
re-synthesised on a 0.2 s grid instead of the node's timer; the pose is interpolated by ``interp_pose`` (linear + nlerp)
instead of tf2.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional

import numpy as np

from . import contract
from .bag_frames import FrameMaker, add_heartbeats, pair_frames, time_range, write_frames
from .frames import Calibration

DET = "/object_detection/detections"
DISP = "/disparity_image_down_float"
GRID = "/rec/disparity_grid"
LEFT_INFO = "/cam/bottom/left/half/camera_info"
RIGHT_INFO = "/cam/bottom/right/half/camera_info"
TF = "/tf"
TF_STATIC = "/tf_static"
STATE = "/mission_state"
TARGET = "/spray_target"
DROP_TOPICS = (GRID, DISP, LEFT_INFO, RIGHT_INFO)  # recorded only by --record-frames; not in the normal allow-list
CAMERA_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "sim", "specs", "camera.json")
MAX_POSE_INVALID_FRACTION = 0.01
MIN_PAIRED_FRACTION = 0.8
BODY = "body"
WORLD = "world"


# ------------------------------------------------------------------------------------------------------ check

def check(meta: dict) -> List[str]:
    """The failed checks of a conversion (empty = the frame file may replace the bag)."""
    out = []
    frames = int(meta.get("frames", 0) or 0)
    with_disp = int(meta.get("frames_with_disparity", 0) or 0)
    pose_invalid = int(meta.get("pose_invalid", 0) or 0)
    if frames <= 0:
        out.append("no frames: the bag holds no detections after the calibration latch")
    if with_disp == 0:
        out.append("frames_with_disparity == 0: no detection was paired with a disparity image "
                   "(was the run recorded with --record-frames?)")
    detections = int(meta.get("detection_frames", 0) or 0)
    if with_disp > 0 and detections > 0 and with_disp < MIN_PAIRED_FRACTION * detections:
        out.append("only %d of %d detection frames (%.0f %% < %.0f %%) were paired with a disparity image"
                   % (with_disp, detections, 100.0 * with_disp / detections, 100.0 * MIN_PAIRED_FRACTION))
    if int(meta.get("disparity_rejected", 0) or 0) > 0:
        out.append("disparity_rejected %d: malformed or wrong-size disparity images (there should be none)"
                   % int(meta["disparity_rejected"]))
    if frames > 0 and pose_invalid > MAX_POSE_INVALID_FRACTION * frames:
        out.append("pose_invalid %d of %d frames (%.1f %% > %.0f %%): /tf world -> body does not cover the frames"
                   % (pose_invalid, frames, 100.0 * pose_invalid / frames, 100.0 * MAX_POSE_INVALID_FRACTION))
    return out


# ------------------------------------------------------------------------------------------------------ bag scan

def deduce_baseline(right_p3: float, right_fx: float) -> float:
    """Stereo baseline in metres from the right camera_info, with treemapper_node's unit rules (input_sync.cpp
    ``deduceBaseline``): -P[3]/fx in metres (REP 104), else in millimetres (either sign)."""
    if not (math.isfinite(right_p3) and math.isfinite(right_fx) and right_fx > 0):
        raise ValueError("right camera P[0] (fx) must be finite and > 0")
    b = -right_p3 / right_fx
    for cand in (b, b / 1000.0, -b / 1000.0):
        if 0.01 <= cand <= 1.0:
            return cand
    raise ValueError("right camera P[3] = %r gives no baseline in [0.01, 1] m (-fx*B m, -fx*B mm, +fx*B mm)" % right_p3)


class BagScan:
    """What the frames need from the bag besides detections and disparity (one cheap pass)."""

    def __init__(self):
        self.tf: List[List[float]] = []                 # stamp, x, y, z, qx, qy, qz, qw (world -> body)
        self.phases: List = []
        self.targets: List = []
        self.left = self.right = None                   # (CameraInfo, receive time) of the first message
        self.static: Dict[str, tuple] = {}              # child frame -> (parent, [x y z qx qy qz qw])


def scan_bag(bag_path: str) -> BagScan:
    import rosbag
    s = BagScan()
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, bt in bag.read_messages(topics=[TF, TF_STATIC, STATE, TARGET, LEFT_INFO, RIGHT_INFO]):
            if topic == TF:
                for tr in msg.transforms:
                    if tr.header.frame_id.lstrip("/") == WORLD and tr.child_frame_id.lstrip("/") == BODY:
                        p, q = tr.transform.translation, tr.transform.rotation
                        s.tf.append([tr.header.stamp.to_sec() or bt.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            elif topic == STATE:
                if not s.phases or s.phases[-1][1] != msg.data:
                    s.phases.append((bt.to_sec(), msg.data))
            elif topic == TARGET:
                if not s.targets or s.targets[-1][1] != int(msg.data):
                    s.targets.append((bt.to_sec(), int(msg.data)))
            elif topic == TF_STATIC:
                for tr in msg.transforms:
                    p, q = tr.transform.translation, tr.transform.rotation
                    s.static[tr.child_frame_id.lstrip("/")] = (tr.header.frame_id.lstrip("/"),
                                                               [p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            elif topic == LEFT_INFO and s.left is None:
                s.left = (msg, bt.to_sec())
            elif topic == RIGHT_INFO and s.right is None:
                s.right = (msg, bt.to_sec())
    return s


def body_tf_arrays(rows: List[List[float]]):
    """Sorted, strictly increasing in time (bag_frames.load_body_tf on the same data)."""
    a = np.array(rows, dtype=np.float64).reshape(-1, 8)
    a = a[np.argsort(a[:, 0], kind="stable")]
    if len(a):
        a = a[np.concatenate([[True], np.diff(a[:, 0]) > 1e-9])]
    return a[:, 0], a[:, 1:]


def load_mission(run_dir: str, scan: BagScan) -> contract.Mission:
    """``mission.json`` of the run dir (written by mission_score) if present, else the same series from the bag."""
    path = os.path.join(run_dir, "mission.json")
    if os.path.exists(path):
        return contract.read_mission(path)
    from .mission_score import _downsample_poses
    poses = sorted(((r[0], r[1], r[2], r[3]) for r in scan.tf), key=lambda r: r[0])
    return contract.Mission(list(scan.phases), list(scan.targets), _downsample_poses(poses))


def grid_disparity_of(stride: int, expect_wh):
    """``pair_frames`` hook for ``/rec/disparity_grid`` messages: the node's verdict from the source image's own fields
    (``sampleDisparity``: malformed, then wrong size), then the recorded samples."""
    def of(msg):
        w, h = int(msg.src_width), int(msg.src_height)
        if (msg.src_encoding != "32FC1" or msg.src_is_bigendian or w == 0 or h == 0 or msg.src_step != 4 * w
                or msg.src_data_size != msg.src_step * h):
            return "malformed", None
        if expect_wh is not None and (w, h) != tuple(expect_wh):
            return "resolution", None
        if int(msg.stride) != stride:
            raise ValueError("DisparityGrid stride %d, expected %d" % (msg.stride, stride))
        gh, gw = len(range(stride // 2, h, stride)), len(range(stride // 2, w, stride))
        if (msg.grid_height, msg.grid_width) != (gh, gw) or len(msg.data) != gh * gw:
            raise ValueError("DisparityGrid %dx%d with %d values does not fit a %dx%d image at stride %d"
                             % (msg.grid_width, msg.grid_height, len(msg.data), w, h, stride))
        return "ok", np.asarray(msg.data, dtype=np.float32).reshape(gh, gw)
    return of


def mount_mismatch(scan: BagScan, frame_id: str, mount: List[float]) -> Optional[float]:
    """Largest component difference between the mount and /tf_static body -> camera, None if /tf_static has none."""
    got = scan.static.get(frame_id.lstrip("/"))
    if not got or got[0] != BODY:
        return None
    q, m = np.array(got[1][3:]), np.array(mount[3:])
    if float(np.dot(q, m)) < 0:
        q = -q
    return float(max(np.max(np.abs(np.array(got[1][:3]) - np.array(mount[:3]))), np.max(np.abs(q - m))))


# ------------------------------------------------------------------------------------------------------ convert

def convert(run_dir: str, out: Optional[str] = None, bag: Optional[str] = None, camera_json: str = CAMERA_JSON,
            stride: int = 4, nearest_tol: float = 0.02, heartbeat: float = 0.2) -> dict:
    """Write ``out`` (default ``<run>/frames.bin``) and ``out + '.meta.json'``; returns the meta dict."""
    import rosbag
    t_wall = time.time()
    bag_path = bag or os.path.join(run_dir, "mission.bag")
    out = out or os.path.join(run_dir, "frames.bin")
    camera = json.load(open(camera_json))
    mount = [float(v) for v in camera["mount"]]
    if len(mount) != 7:
        raise ValueError("%s: mount must be [x, y, z, qx, qy, qz, qw]" % camera_json)

    scan = scan_bag(bag_path)
    if scan.left is None or scan.right is None:
        raise ValueError("%s: no %s / %s message (was the run recorded with --record-frames?)"
                         % (bag_path, LEFT_INFO, RIGHT_INFO))
    (li, t_left), (ri, t_right) = scan.left, scan.right
    t_cal = max(t_left, t_right)
    cal = Calibration(width=int(li.width), height=int(li.height), fx=float(li.P[0]), fy=float(li.P[5]),
                      cx=float(li.P[2]), cy=float(li.P[6]), baseline=deduce_baseline(ri.P[3], ri.P[0]),
                      stride=stride, T_body_cam=mount, frame=WORLD,
                      note="Isaac closed loop: camera_info of the run, body->camera = sim/specs/camera.json mount")
    if not (cal.fx > 0 and cal.fy > 0 and cal.width > 0 and cal.height > 0):
        raise ValueError("left camera_info has no usable calibration: %r" % ((cal.width, cal.height, cal.fx, cal.fy),))
    tf_t, tf_rows = body_tf_arrays(scan.tf)
    mission = load_mission(run_dir, scan)
    make = FrameMaker(mission, tf_t, tf_rows)

    with rosbag.Bag(bag_path) as b:
        topics = b.get_type_and_topic_info().topics
        disp_topic = GRID if GRID in topics else DISP
        expect_wh = (cal.width, cal.height)
        hook = grid_disparity_of(stride, expect_wh) if disp_topic == GRID else None
        msgs = ((topic, msg, bt.to_sec()) for topic, msg, bt in
                b.read_messages(topics=[DET, disp_topic], start_time=_rostime(t_cal)))
        frames, cnt = pair_frames(msgs, make, stride, nearest_tol, boxes_of=boxes_from_detection_array,
                                  expect_wh=expect_wh, disp_topic=disp_topic, disparity_of=hook)
    if frames:
        t_lo, t_hi = time_range(t_cal, None, mission, frames)
        if heartbeat > 0:
            add_heartbeats(frames, make, mission, t_lo, t_hi, heartbeat, cnt)
        else:
            frames.sort(key=lambda f: (f.t, f.disparity is None))
    else:
        t_lo = t_hi = t_cal

    run = _load_json(os.path.join(run_dir, "run.json")) or {}
    mismatch = mount_mismatch(scan, li.header.frame_id, mount)
    meta = {"frames": len(frames),
            "frames_with_disparity": sum(1 for f in frames if f.disparity is not None),
            "pose_invalid": sum(1 for f in frames if not f.pose_valid),
            "disparity_rejected": int(cnt["disparity_rejected_malformed"] + cnt["disparity_rejected_resolution"]),
            "t_start": t_lo, "t_end": t_hi,
            "detection_frames": int(cnt["detection_msgs"]), "heartbeat_frames": int(cnt["heartbeat_frames"]),
            "disparity_topic": disp_topic, "counts": dict(cnt), "t_calibration": t_cal, "source": os.path.abspath(bag_path),
            "calibration": cal.to_json(), "mount_vs_tf_static_max_diff": mismatch,
            "run": {"spec_file": run.get("spec_file"), "strategy": run.get("strategy"), "seed": run.get("seed"),
                    "git_commit": run.get("git_commit")}}
    meta["frames_written"] = write_frames(out, cal, frames, dict(meta))
    meta["seconds"] = round(time.time() - t_wall, 1)
    with open(out + ".meta.json", "w") as f:
        json.dump(meta, f, indent=1)
    return meta


def boxes_from_detection_array(msg) -> np.ndarray:
    """(N, 6) x, y, w, h, score, label of a treemapper_msgs/Detection2DArray (as the node's ``fromMsg``, scale 1)."""
    return np.array([[d.x, d.y, d.width, d.height, d.score, d.label] for d in msg.detections],
                    dtype=np.float32).reshape(-1, 6)


def _rostime(sec: float):
    import rospy
    return rospy.Time.from_sec(sec)


def _load_json(path: str):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


# ------------------------------------------------------------------------------------------------------ bag filter

def filter_bag(src: str, dst: str, drop=DROP_TOPICS) -> dict:
    """Copy ``src`` to ``dst`` (lz4) without the messages on ``drop`` topics.

    A raw python copy, not ``rosbag filter``: ``rosbag filter`` deserialises and re-serialises every message (megabyte
    images included) to evaluate its expression; this copies the serialized bytes, one pass, bounded memory. The copy
    is verified: every kept topic must hold exactly as many messages as in ``src``."""
    import rosbag
    with rosbag.Bag(src) as ib:
        info = ib.get_type_and_topic_info().topics
        keep = [t for t in info if t not in drop]
        want = {t: info[t].message_count for t in keep}
        dropped = sum(i.message_count for t, i in info.items() if t in drop)
        with rosbag.Bag(dst, "w", compression=rosbag.Compression.LZ4) as ob:
            for topic, raw, t in ib.read_messages(topics=keep, raw=True):
                ob.write(topic, raw, t, raw=True)
    with rosbag.Bag(dst) as ob:
        got = {t: i.message_count for t, i in ob.get_type_and_topic_info().topics.items()}
    if got != {t: n for t, n in want.items() if n}:
        raise RuntimeError("filtered bag differs from %s: wanted %r, got %r" % (src, want, got))
    return {"kept": sum(got.values()), "dropped": dropped}


# ------------------------------------------------------------------------------------------------------ CLI

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run", required=True, help="run directory (mission.bag, optional mission.json / run.json)")
    ap.add_argument("--out", default=None, help="frame file (default <run>/frames.bin)")
    ap.add_argument("--bag", default=None, help="bag (default <run>/mission.bag)")
    ap.add_argument("--camera-json", default=CAMERA_JSON)
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--nearest-tol", type=float, default=0.02)
    ap.add_argument("--heartbeat", type=float, default=0.2, help="s; 0 = no heartbeat frames")
    ap.add_argument("--filter-bag", action="store_true",
                    help="after a passing check, replace the bag by a copy without the disparity grid / image and "
                         "camera_info topics")
    a = ap.parse_args(argv)
    bag = a.bag or os.path.join(a.run, "mission.bag")
    try:
        meta = convert(a.run, a.out, a.bag, a.camera_json, a.stride, a.nearest_tol, a.heartbeat)
    except Exception as e:  # any failure: loud, the bag is untouched
        print("sim_frames: cannot convert %s: %s: %s" % (bag, type(e).__name__, e), file=sys.stderr)
        return 2
    keys = ("frames", "frames_with_disparity", "pose_invalid", "disparity_rejected", "t_start", "t_end")
    print("sim_frames: " + ", ".join("%s %s" % (k, meta[k]) for k in keys))
    if meta["disparity_rejected"]:
        print("sim_frames: %d disparity messages rejected (malformed or wrong size)" % meta["disparity_rejected"],
              file=sys.stderr)
    if meta["mount_vs_tf_static_max_diff"] is not None and meta["mount_vs_tf_static_max_diff"] > 1e-4:
        print("sim_frames: WARNING /tf_static body -> camera differs from camera.json mount by %.3g"
              % meta["mount_vs_tf_static_max_diff"], file=sys.stderr)
    failed = check(meta)
    if failed:
        for f in failed:
            print("sim_frames: CHECK FAILED: " + f, file=sys.stderr)
        return 1
    if a.filter_bag:
        tmp = bag + ".filtered"
        try:
            n = filter_bag(bag, tmp)
            os.replace(tmp, bag)
        except Exception as e:
            print("sim_frames: cannot filter %s (kept as it is): %s: %s" % (bag, type(e).__name__, e), file=sys.stderr)
            if os.path.exists(tmp):
                os.remove(tmp)
            return 2
        print("sim_frames: %s filtered: %d messages kept, %d disparity / camera_info dropped"
              % (bag, n["kept"], n["dropped"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
