#!/usr/bin/env python3
"""M0 bridge spike (Noetic side): measure what arrives from Isaac and check the loop.

Runs in tm_sim_ros with ROS 1 only sourced and /use_sim_time true. For --duration sim
seconds it measures, per topic, the rate (per sim second and per wall second), the
stamp lag (arrival /clock time minus header stamp) and the max gap; verifies /clock
is monotonic, camera_info is present, tf2 lookups world->body / world->cam_* work,
then runs setpoint round trips (publish a setpoint, watch the world->body TF move)
and a spray on/off check against /sim/deposition. Exit 0 iff every check passes.

    docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/bridge/spike_check.py \
        --duration 60 --out /data/work/logs/sim_bridge_spike_check.json
"""
import argparse
import json
import math
import sys
import threading
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

ap = argparse.ArgumentParser()
ap.add_argument("--duration", type=float, default=60.0, help="sim seconds of passive measurement")
ap.add_argument("--wait", type=float, default=120.0, help="wall seconds to wait for the first /clock")
ap.add_argument("--out", default="")
ap.add_argument("--min-image-hz", type=float, default=9.0)
ap.add_argument("--max-roundtrip", type=float, default=1.0)
args, _ = ap.parse_known_args()


class Stat:
    def __init__(self):
        self.lock = threading.Lock()
        self.n = 0
        self.arr_sim = []     # sim time at arrival
        self.arr_wall = []
        self.lag = []         # sim seconds, arrival - header stamp
        self.first_msg = None

    def add(self, stamp):
        now_sim = rospy.Time.now().to_sec()
        with self.lock:
            self.n += 1
            self.arr_sim.append(now_sim)
            self.arr_wall.append(time.time())
            if stamp is not None:
                self.lag.append(now_sim - stamp)

    def summary(self, t0_sim, t1_sim):
        with self.lock:
            sim = [t for t in self.arr_sim if t0_sim <= t <= t1_sim]
            wall = [w for t, w in zip(self.arr_sim, self.arr_wall) if t0_sim <= t <= t1_sim]
            lag = [l for t, l in zip(self.arr_sim, self.lag) if t0_sim <= t <= t1_sim] if self.lag else []
        out = {"count": len(sim), "total_count": self.n}
        if len(sim) >= 2:
            ds = max(sim[-1] - sim[0], 1e-9)
            dw = max(wall[-1] - wall[0], 1e-9)
            gaps = [b - a for a, b in zip(sim, sim[1:])]
            out.update(rate_per_sim_s=(len(sim) - 1) / ds, rate_per_wall_s=(len(sim) - 1) / dw,
                       max_gap_sim_s=max(gaps))
        if lag:
            srt = sorted(lag)
            out.update(lag_mean_s=sum(lag) / len(lag), lag_p95_s=srt[int(0.95 * (len(srt) - 1))],
                       lag_max_s=srt[-1], lag_min_s=srt[0])
        return out


stats = {k: Stat() for k in ("left", "right", "info_left", "info_right", "tf", "tf_static",
                             "clock", "truth", "deposition")}
clock_log = {"last": None, "backwards": 0, "n": 0}
info_first = {}
img_first = {}
tf_samples = []          # (stamp, x, y, z) of world->body from /tf
tf_lock = threading.Lock()
truth_log = []           # (sim_arrival, dict)
depo_log = []
tf_static_frames = set()


def on_clock(msg):
    t = msg.clock.to_sec()
    if clock_log["last"] is not None and t < clock_log["last"]:
        clock_log["backwards"] += 1
    clock_log["last"] = t
    clock_log["n"] += 1
    stats["clock"].add(None)


def img_cb(key):
    def cb(msg):
        stats[key].add(msg.header.stamp.to_sec())
        img_first.setdefault(key, {"w": msg.width, "h": msg.height, "enc": msg.encoding,
                                   "frame": msg.header.frame_id,
                                   "bytes": len(msg.data)})
    return cb


def info_cb(key):
    def cb(msg):
        stats[key].add(msg.header.stamp.to_sec())
        info_first.setdefault(key, {"w": msg.width, "h": msg.height, "fx": msg.K[0], "fy": msg.K[4],
                                    "cx": msg.K[2], "cy": msg.K[5], "frame": msg.header.frame_id,
                                    "distortion_model": msg.distortion_model})
    return cb


def on_tf(msg):
    for t in msg.transforms:
        if t.header.frame_id == "world" and t.child_frame_id == "body":
            stats["tf"].add(t.header.stamp.to_sec())
            tr = t.transform.translation
            with tf_lock:
                tf_samples.append((t.header.stamp.to_sec(), tr.x, tr.y, tr.z))


def on_tf_static(msg):
    stats["tf_static"].add(None)
    for t in msg.transforms:
        tf_static_frames.add((t.header.frame_id, t.child_frame_id))


def on_truth(msg):
    stats["truth"].add(None)
    try:
        d = json.loads(msg.data)
        truth_log.append((rospy.Time.now().to_sec(), d))
    except ValueError:
        pass


def on_depo(msg):
    stats["deposition"].add(None)
    try:
        depo_log.append((rospy.Time.now().to_sec(), json.loads(msg.data)))
    except ValueError:
        pass


def wall_sleep(s):
    time.sleep(s)


rospy.init_node("tm_spike_check", anonymous=True)
node_start_wall = time.time()
if not rospy.get_param("/use_sim_time", False):
    print("FAIL: /use_sim_time is not true on the ROS 1 master (start tm_sim_ros via the bridge entrypoint)")
    sys.exit(2)

tfbuf = tf2_ros.Buffer(rospy.Duration(30.0))
tflis = tf2_ros.TransformListener(tfbuf)
pub_sp = rospy.Publisher("/corridor_nav/setpoint", PoseStamped, queue_size=10)
pub_spray = rospy.Publisher("/corridor_nav/spray", Bool, queue_size=10)
rospy.Subscriber("/cam/bottom/left/half/image_raw", Image, img_cb("left"), queue_size=5)
rospy.Subscriber("/cam/bottom/right/half/image_raw", Image, img_cb("right"), queue_size=5)
rospy.Subscriber("/cam/bottom/left/half/camera_info", CameraInfo, info_cb("info_left"), queue_size=5)
rospy.Subscriber("/cam/bottom/right/half/camera_info", CameraInfo, info_cb("info_right"), queue_size=5)
rospy.Subscriber("/tf", TFMessage, on_tf, queue_size=200)
rospy.Subscriber("/tf_static", TFMessage, on_tf_static, queue_size=10)
rospy.Subscriber("/sim/truth", String, on_truth, queue_size=50)
rospy.Subscriber("/sim/deposition", String, on_depo, queue_size=50)
# /clock is consumed by rospy itself for sim time; a second subscriber lets us check monotonicity.
rospy.Subscriber("/clock", Clock, on_clock, queue_size=500)

print("waiting for /clock ...", flush=True)
w0 = time.time()
while clock_log["last"] is None and time.time() - w0 < args.wait and not rospy.is_shutdown():
    wall_sleep(0.2)
if clock_log["last"] is None:
    print("FAIL: no /clock within %.0f s wall" % args.wait)
    sys.exit(3)
print("clock up at sim t=%.3f; waiting for both images ..." % clock_log["last"], flush=True)
while (stats["left"].n < 3 or stats["right"].n < 3) and time.time() - w0 < args.wait:
    wall_sleep(0.2)

# Static transforms reach a late joiner only through Isaac's 1 Hz /tf_static replay (the bridge
# does not latch); wait for the first one and report how long a fresh Noetic node waited.
while not tf_static_frames and time.time() - w0 < args.wait:
    wall_sleep(0.1)
static_first_wait_s = time.time() - node_start_wall if tf_static_frames else None

# ---------------------------------------------------------------- passive window
t0 = rospy.Time.now().to_sec()
wall_t0 = time.time()
print("measuring for %.0f sim s from t=%.2f" % (args.duration, t0), flush=True)
lookups = {"ok": 0, "fail": 0, "last_error": ""}
lookup_pairs = [("world", "body"), ("world", "cam_left"), ("world", "cam_right"), ("cam_left", "cam_right")]
lookup_ok = {"%s->%s" % p: 0 for p in lookup_pairs}
lookup_fail = {"%s->%s" % p: 0 for p in lookup_pairs}
while rospy.Time.now().to_sec() - t0 < args.duration and not rospy.is_shutdown():
    for p in lookup_pairs:
        try:
            tfbuf.lookup_transform(p[0], p[1], rospy.Time(0), rospy.Duration(0.0))
            lookup_ok["%s->%s" % p] += 1
        except Exception as e:   # noqa: BLE001 - tf2 raises several exception types
            lookup_fail["%s->%s" % p] += 1
            lookups["last_error"] = str(e)[:200]
    wall_sleep(0.5)
t1 = rospy.Time.now().to_sec()
wall_t1 = time.time()
passive = {k: s.summary(t0, t1) for k, s in stats.items()}
passive_window = {"sim_s": t1 - t0, "wall_s": wall_t1 - wall_t0, "rtf": (t1 - t0) / max(wall_t1 - wall_t0, 1e-9)}
print("passive window done: sim %.1f s in %.1f s wall (RTF %.2f)" % (
    t1 - t0, wall_t1 - wall_t0, passive_window["rtf"]), flush=True)

# ---------------------------------------------------------------- setpoint round trips
def current_pos():
    with tf_lock:
        return tf_samples[-1][1:] if tf_samples else None


def roundtrip(target, label, timeout_sim=10.0):
    start = current_pos()
    # warm-up: Isaac's subscriber must be matched through the bridge before the timed message.
    # A first untimed publish of the current position is echoed back via /sim/truth sp_seq.
    seq0 = truth_log[-1][1].get("sp_seq", 0) if truth_log else 0
    matched = False
    for _ in range(50):
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = start
        m.pose.orientation.w = 1.0
        pub_sp.publish(m)
        wall_sleep(0.2)
        if truth_log and truth_log[-1][1].get("sp_seq", 0) > seq0:
            matched = True
            break
    if not matched:
        return {"label": label, "error": "setpoint never reached Isaac (sp_seq did not increase)"}
    seq1 = truth_log[-1][1]["sp_seq"]
    # let Isaac settle at the held position
    wall_sleep(1.0)
    start = current_pos()
    t_pub = rospy.Time.now().to_sec()
    m = PoseStamped()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = "world"
    m.pose.position.x, m.pose.position.y, m.pose.position.z = target
    m.pose.orientation.w = 1.0
    pub_sp.publish(m)
    echo = onset = arrive = None
    dist0 = math.dist(start, target)
    while rospy.Time.now().to_sec() - t_pub < timeout_sim and not rospy.is_shutdown():
        if echo is None and truth_log and truth_log[-1][1].get("sp_seq", 0) > seq1:
            # first truth message that carries the new setpoint
            for arr, d in truth_log:
                if d.get("sp_seq", 0) > seq1 and arr >= t_pub:
                    echo = d["t"] - t_pub     # Isaac sim time of the truth sample - our publish time
                    break
        with tf_lock:
            samples = [s for s in tf_samples if s[0] >= t_pub]
        for st, x, y, z in samples:
            d_start = math.dist((x, y, z), start)
            d_goal = math.dist((x, y, z), target)
            if onset is None and d_start >= 0.05:
                onset = st - t_pub
            if arrive is None and d_goal <= 0.1:
                arrive = st - t_pub
        if onset is not None and arrive is not None:
            break
        wall_sleep(0.05)
    return {"label": label, "start": list(start), "target": list(target), "distance_m": dist0,
            "setpoint_echo_sim_s": echo, "tf_onset_sim_s": onset, "tf_arrive_sim_s": arrive}


rt_results = []
base = current_pos() or (0.0, 0.0, 5.0)
for i, delta in enumerate([(3.0, 0.0, 0.0), (0.0, -3.0, 0.0), (-3.0, 3.0, 0.0)]):
    tgt = (base[0] + delta[0], base[1] + delta[1], base[2] + delta[2])
    r = roundtrip(tgt, "rt%d" % i)
    rt_results.append(r)
    print("roundtrip", json.dumps(r), flush=True)
    base = tgt

# ---------------------------------------------------------------- spray -> deposition
spray = {"ok": False}
if depo_log:
    v_before = depo_log[-1][1].get("volume_l", 0.0)
    t_spray = rospy.Time.now().to_sec()
    for _ in range(20):
        pub_spray.publish(Bool(data=True))
        wall_sleep(0.1)
    sim_t_end = rospy.Time.now().to_sec() + 3.0
    while rospy.Time.now().to_sec() < sim_t_end and not rospy.is_shutdown():
        pub_spray.publish(Bool(data=True))
        wall_sleep(0.1)
    for _ in range(20):
        pub_spray.publish(Bool(data=False))
        wall_sleep(0.1)
    wall_sleep(1.5)
    v_after = depo_log[-1][1].get("volume_l", 0.0)
    spray = {"volume_before_l": v_before, "volume_after_l": v_after, "ok": v_after > v_before + 0.01}

# ---------------------------------------------------------------- verdict
def get(k, f):
    return passive.get(k, {}).get(f)


checks = {}
for k in ("left", "right"):
    r = get(k, "rate_per_sim_s")
    checks["image_%s_rate>=%.0fHz" % (k, args.min_image_hz)] = bool(r and r >= args.min_image_hz)
for k in ("info_left", "info_right"):
    checks["%s_present" % k] = passive[k]["count"] > 0
checks["clock_monotonic"] = clock_log["backwards"] == 0 and clock_log["n"] > 10
checks["tf_world_body_lookups"] = lookup_fail["world->body"] == 0 and lookup_ok["world->body"] > 0
checks["tf_static_cam_lookups"] = all(lookup_fail[k] == 0 and lookup_ok[k] > 0
                                      for k in ("world->cam_left", "world->cam_right", "cam_left->cam_right"))
checks["truth_and_deposition_rx"] = passive["truth"]["count"] > 0 and passive["deposition"]["count"] > 0
rt_ok = [r.get("tf_onset_sim_s") is not None and 0 <= r["tf_onset_sim_s"] < args.max_roundtrip
         and r.get("tf_arrive_sim_s") is not None for r in rt_results]
checks["setpoint_roundtrip<%.1fs" % args.max_roundtrip] = bool(rt_ok) and all(rt_ok)
checks["spray_deposition"] = spray["ok"]

report = {"passive_window": passive_window, "topics": passive, "clock": clock_log,
          "first_image": img_first, "first_camera_info": info_first,
          "tf_lookups_ok": lookup_ok, "tf_lookups_fail": lookup_fail,
          "tf_lookup_last_error": lookups["last_error"],
          "tf_static_frames": sorted(list(x) for x in tf_static_frames),
          "tf_static_first_arrival_wall_s_after_node_start": static_first_wait_s,
          "roundtrips": rt_results, "spray": spray, "checks": checks,
          "pass": all(checks.values())}
text = json.dumps(report, indent=2, default=str)
print(text)
if args.out:
    with open(args.out, "w") as f:
        f.write(text)
sys.exit(0 if report["pass"] else 1)
