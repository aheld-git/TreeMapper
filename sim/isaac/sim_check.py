#!/usr/bin/env python3
"""Noetic-side driver and measurements for the Isaac sim checks (run inside tm_sim_ros, ROS 1 only).

    docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/isaac/sim_check.py <mode> [options]

Modes
  topics      rates, stamp lag, camera_info/image stamp match, TF chain, /sim/truth, /tf_static replay
  square      take off to --z, fly a square with a moving carrot (setpoints at 20 Hz), report the cross-track
              deviation of /tf world->body from the square
  flight      projection-check flight over one tree: A overhead, B offset, C offset + yaw 90 deg
              (start orchard_sim with --save-hover 3: it saves the left image at each of the three hovers)
  hover       fly to --x --y --z (--yaw-deg) and hold --hold sim seconds
  spray       publish /corridor_nav/spray (Bool) at --rate Hz for --seconds sim seconds

Setpoints are stamped with ROS time (= the sim /clock, /use_sim_time is set by the container entrypoint).
"""
import argparse
import json
import math
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Link:
    """Setpoint publisher (20 Hz of the latest target) and /tf world->body recorder."""

    def __init__(self, rate_hz=20.0):
        self.pub = rospy.Publisher("/corridor_nav/setpoint", PoseStamped, queue_size=5)
        self.spray = rospy.Publisher("/corridor_nav/spray", Bool, queue_size=5)
        self.target = None
        self.pose = None            # (t, x, y, z, yaw)
        self.trace = []             # (t, x, y, z, yaw)
        self.record = False
        self._lock = threading.Lock()
        rospy.Subscriber("/tf", TFMessage, self._on_tf, queue_size=500)
        self._timer = rospy.Timer(rospy.Duration(1.0 / rate_hz), self._tick)

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "world" and t.child_frame_id == "body":
                tr, q = t.transform.translation, t.transform.rotation
                s = (t.header.stamp.to_sec(), tr.x, tr.y, tr.z, quat_yaw(q))
                with self._lock:
                    self.pose = s
                    if self.record:
                        self.trace.append(s)

    def _tick(self, _evt):
        tgt = self.target
        if tgt is None:
            return
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = tgt[0], tgt[1], tgt[2]
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = yaw_quat(tgt[3])
        self.pub.publish(m)

    def wait_ready(self, timeout=120.0):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and self.pose is None:
            if rospy.get_time() - t0 > timeout:
                raise SystemExit("no /tf world->body within %.0f sim s" % timeout)
            rospy.sleep(0.1)

    def goto(self, x, y, z, yaw=0.0, tol=0.15, yaw_tol=math.radians(2.0), timeout=90.0):
        """Publish the target until the pose is within tolerance; returns the sim time it took."""
        self.target = (x, y, z, yaw)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            p = self.pose
            if p is not None:
                d = math.sqrt((p[1] - x) ** 2 + (p[2] - y) ** 2 + (p[3] - z) ** 2)
                dy = abs((p[4] - yaw + math.pi) % (2 * math.pi) - math.pi)
                if d < tol and dy < yaw_tol:
                    return rospy.get_time() - t0
            if rospy.get_time() - t0 > timeout:
                raise SystemExit("goto (%.1f, %.1f, %.1f) timed out at %s" % (x, y, z, p))
            rospy.sleep(0.05)

    def hold(self, seconds):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < seconds:
            rospy.sleep(0.05)


def seg_dist(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    den = sum(c * c for c in ab)
    s = 0.0 if den == 0 else max(0.0, min(1.0, sum(ap[i] * ab[i] for i in range(3)) / den))
    q = [a[i] + s * ab[i] for i in range(3)]
    return math.sqrt(sum((p[i] - q[i]) ** 2 for i in range(3)))


def mode_square(args):
    link = Link()
    link.wait_ready()
    z, h = args.z, args.half
    cx, cy = args.cx, args.cy
    corners = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h), (cx - h, cy - h)]
    t_takeoff = link.goto(corners[0][0], corners[0][1], z)
    print("takeoff + approach to square start: %.1f sim s" % t_takeoff, flush=True)
    link.hold(1.0)
    link.trace, link.record = [], True
    t_start = rospy.get_time()
    # moving carrot along the square at --speed, as the navigator does
    pts3 = [(x, y, z) for x, y in corners]
    for a, b in zip(pts3[:-1], pts3[1:]):
        length = math.dist(a, b)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            s = (rospy.get_time() - t0) * args.speed
            if s >= length:
                break
            f = s / length
            link.target = (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2], 0.0)
            rospy.sleep(0.05)
    link.target = (pts3[-1][0], pts3[-1][1], pts3[-1][2], 0.0)
    link.hold(3.0)
    link.record = False
    dev = [min(seg_dist(p[1:4], a, b) for a, b in zip(pts3[:-1], pts3[1:])) for p in link.trace]
    end = link.trace[-1]
    mid = [d for d, p in zip(dev, link.trace)
           if min(math.hypot(p[1] - c[0], p[2] - c[1]) for c in corners) > 2.0]
    res = {"mode": "square", "z": z, "half": h, "speed": args.speed, "samples": len(link.trace),
           "duration_sim_s": rospy.get_time() - t_start,
           "max_deviation_m": max(dev), "mean_deviation_m": sum(dev) / len(dev),
           "p95_deviation_m": sorted(dev)[int(0.95 * (len(dev) - 1))],
           "max_deviation_more_than_2m_from_a_corner_m": max(mid) if mid else None,
           "end_error_m": math.dist(end[1:4], pts3[-1]),
           "x_range": [min(p[1] for p in link.trace), max(p[1] for p in link.trace)],
           "y_range": [min(p[2] for p in link.trace), max(p[2] for p in link.trace)]}
    print(json.dumps(res, indent=1))
    if args.out:
        with open(args.out, "w") as f:
            json.dump(res, f, indent=1)


def tree_from_truth(tree_id, timeout=60.0):
    msg = rospy.wait_for_message("/sim/truth", String, timeout=timeout)
    doc = json.loads(msg.data)
    for t in doc["trees"]:
        if t["id"] == tree_id:
            return t
    raise SystemExit("tree %d not in /sim/truth" % tree_id)


def mode_flight(args):
    link = Link()
    link.wait_ready()
    tree = tree_from_truth(args.tree)
    tx, ty = tree["centre"]
    print("tree %d centre (%.3f, %.3f) top_z %.3f" % (tree["id"], tx, ty, tree["top_z"]), flush=True)
    log = {}
    t = link.goto(tx, ty, args.z_a, 0.0)
    link.hold(args.settle)
    log["A_overhead"] = {"pose": link.pose, "arrival_sim_s": rospy.get_time()}
    print("A overhead at sim %.1f (arrival took %.1f s) pose %s" % (rospy.get_time(), t, link.pose), flush=True)
    link.hold(args.hold)
    bx, by = tx - args.offset, ty
    link.goto(bx, by, args.z_b, 0.0)
    link.hold(args.settle)
    log["B_offset"] = {"pose": link.pose, "sim_s": rospy.get_time()}
    print("B offset at sim %.1f pose %s" % (rospy.get_time(), link.pose), flush=True)
    link.hold(args.hold)
    link.goto(bx, by, args.z_b, math.pi / 2)
    link.hold(args.settle)
    log["C_yaw90"] = {"pose": link.pose, "sim_s": rospy.get_time()}
    print("C yaw 90 at sim %.1f pose %s" % (rospy.get_time(), link.pose), flush=True)
    link.hold(args.hold)
    print(json.dumps(log))


def mode_hover(args):
    link = Link()
    link.wait_ready()
    t = link.goto(args.x, args.y, args.z, math.radians(args.yaw_deg))
    print("arrived after %.1f sim s, pose %s" % (t, link.pose), flush=True)
    link.hold(args.hold)
    print("held; pose %s sim %.1f" % (link.pose, rospy.get_time()), flush=True)


def mode_spray(args):
    pub = rospy.Publisher("/corridor_nav/spray", Bool, queue_size=5)
    rospy.sleep(1.0)
    t0 = rospy.get_time()
    r = rospy.Rate(args.rate)
    n = 0
    while not rospy.is_shutdown() and rospy.get_time() - t0 < args.seconds:
        pub.publish(Bool(True))
        n += 1
        r.sleep()
    pub.publish(Bool(False))
    print("spray: %d messages in %.2f sim s" % (n, rospy.get_time() - t0), flush=True)


class Meter:
    def __init__(self):
        self.t = {}

    def add(self, name, stamp, arrival):
        self.t.setdefault(name, []).append((stamp, arrival))


def mode_topics(args):
    m = Meter()
    clock = {"t": 0.0}
    infos = {"left": [], "right": []}
    imgs = {"left": [], "right": []}
    statics = []
    truth = []
    depo = []

    def on_clock(msg):
        clock["t"] = msg.clock.to_sec()
        m.add("clock", clock["t"], clock["t"])

    rospy.Subscriber("/clock", Clock, on_clock, queue_size=100)
    for side in ("left", "right"):
        rospy.Subscriber("/cam/bottom/%s/half/image_raw" % side, Image,
                         lambda msg, s=side: (imgs[s].append((msg.header.stamp.to_sec(), clock["t"], msg.header.frame_id,
                                                              msg.width, msg.height, msg.encoding))), queue_size=20)
        rospy.Subscriber("/cam/bottom/%s/half/camera_info" % side, CameraInfo,
                         lambda msg, s=side: (infos[s].append((msg.header.stamp.to_sec(), clock["t"], msg)),), queue_size=20)
    tf_stamps = set()

    def on_tf(msg):
        st = msg.transforms[0].header.stamp.to_sec()
        tf_stamps.add(round(st, 6))
        m.add("tf", st, clock["t"])

    rospy.Subscriber("/tf", TFMessage, on_tf, queue_size=500)
    rospy.Subscriber("/tf_static", TFMessage, lambda msg: statics.append((clock["t"], [(t.header.frame_id, t.child_frame_id) for t in msg.transforms])), queue_size=20)
    rospy.Subscriber("/sim/truth", String, lambda msg: truth.append((clock["t"], len(json.loads(msg.data)["trees"]))), queue_size=5)
    rospy.Subscriber("/sim/deposition", String, lambda msg: depo.append((clock["t"], json.loads(msg.data))), queue_size=10)
    rospy.sleep(2.0)
    c0 = clock["t"]
    rospy.sleep(args.duration)
    c1 = clock["t"]
    span = c1 - c0
    out = {"sim_window_s": span}

    def rate(lst, key=lambda e: e[0]):
        return len([e for e in lst if key(e) > c0]) / span

    for side in ("left", "right"):
        i, f = imgs[side], infos[side]
        st_i = {round(e[0], 6) for e in i}
        st_f = {round(e[0], 6) for e in f}
        both = st_i & st_f
        out[side] = {
            "image_hz": rate(i), "camera_info_hz": rate(f),
            "image_stamps_with_matching_info": "%d/%d" % (len(both), len(st_i)),
            "image_size": [i[-1][3], i[-1][4], i[-1][5]] if i else None,
            "image_frame": i[-1][2] if i else None,
            "info_K": list(f[-1][2].K) if f else None, "info_P": list(f[-1][2].P) if f else None,
            "info_frame": f[-1][2].header.frame_id if f else None,
            "info_dist": [f[-1][2].distortion_model, list(f[-1][2].D)] if f else None,
        }
    li, ri = {round(e[0], 6) for e in imgs["left"]}, {round(e[0], 6) for e in imgs["right"]}
    out["left_right_image_stamps_equal"] = "%d of %d left stamps also on the right (exact match)" % (len(li & ri), len(li))
    import bisect
    grid = sorted(tf_stamps)

    def on_grid(x, tol=1e-4):
        i = bisect.bisect_left(grid, x)
        return any(0 <= j < len(grid) and abs(grid[j] - x) <= tol for j in (i - 1, i))

    inside = [x for x in li if grid and grid[0] <= x <= grid[-1]]   # images queued before the first /tf sample do not count
    out["image_stamps_on_the_tf_grid"] = "%d of %d left image stamps (inside the /tf window) within 0.1 ms of a /tf stamp" % (
        len([x for x in inside if on_grid(x)]), len(inside))
    out["tf_hz"] = rate(m.t.get("tf", []))
    out["clock_hz"] = rate(m.t.get("clock", []))
    out["tf_static_msgs"] = len([s for s in statics if s[0] > c0])
    out["tf_static_frames"] = sorted({tuple(p) for s in statics for p in s[1]})
    out["truth_msgs"] = len([s for s in truth if s[0] > c0])
    out["truth_trees"] = truth[-1][1] if truth else None
    out["deposition_msgs"] = len([s for s in depo if s[0] > c0])
    print(json.dumps(out, indent=1))
    if args.out:
        with open(args.out, "w") as f:
            json.dump(out, f, indent=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    p = sub.add_parser("topics"); p.add_argument("--duration", type=float, default=10.0); p.add_argument("--out", default="")
    p = sub.add_parser("square")
    p.add_argument("--z", type=float, default=5.0); p.add_argument("--half", type=float, default=4.0)
    p.add_argument("--cx", type=float, default=0.0); p.add_argument("--cy", type=float, default=0.0)
    p.add_argument("--speed", type=float, default=3.0); p.add_argument("--out", default="")
    p = sub.add_parser("flight")
    p.add_argument("--tree", type=int, default=5); p.add_argument("--z-a", type=float, default=15.0)
    p.add_argument("--z-b", type=float, default=12.0); p.add_argument("--offset", type=float, default=2.5)
    p.add_argument("--settle", type=float, default=4.0); p.add_argument("--hold", type=float, default=3.0)
    p = sub.add_parser("hover")
    p.add_argument("--x", type=float, required=True); p.add_argument("--y", type=float, required=True)
    p.add_argument("--z", type=float, required=True); p.add_argument("--yaw-deg", type=float, default=0.0)
    p.add_argument("--hold", type=float, default=2.0)
    p = sub.add_parser("spray")
    p.add_argument("--seconds", type=float, default=4.0); p.add_argument("--rate", type=float, default=10.0)
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("tm_sim_check", anonymous=True)
    while not rospy.is_shutdown() and rospy.get_time() == 0.0:
        rospy.sleep(0.1)   # wait for /clock
    {"topics": mode_topics, "square": mode_square, "flight": mode_flight,
     "hover": mode_hover, "spray": mode_spray}[args.mode](args)


if __name__ == "__main__":
    main()
