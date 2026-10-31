#!/usr/bin/env python3
"""ROS 1 side of scripts/sim_mission.sh: wait for the sim, start the corridor navigator, wait for the end.

Runs inside tm_sim_ros (Noetic, /use_sim_time true; the catkin workspace's devel/setup.bash sourced for NavStatus):

    sim_mission_driver.py wait-clock --budget 400
    sim_mission_driver.py run --run-dir /data/work/sim_runs/<name>/run_1 --timeout 1800

wait-clock  /clock is advancing (two different values) within ``--budget`` wall s (Isaac startup takes ~140 s).
run         1. every input of the loop is live, each within ``--startup-budget`` wall s: both camera_infos, the drone
               pose (/tf world -> body), the disparity, the detections, /treemapper/map and the ~start service;
            2. ~start (repeated while the navigator answers "not ready");
            3. waits for NavStatus state DONE on /corridor_nav/status. Stops early, calling ~abort and waiting
               ``--abort-wait`` s of sim time for DONE, when ``--timeout`` sim s have passed since the start, when the
               wall time exceeds timeout / ``--min-rtf``, or when the run directory's drive has less than
               ``--min-free-gb`` free. Fails at once when /clock stalls for ``--stall`` wall s (Isaac is gone) or
               the navigator's status stops for ``--stall`` wall s while the clock runs.
            Writes <run dir>/driver.json (times, end state, reason).
Exit codes: 0 DONE; 3 timeout (sim or wall budget) -> aborted; 4 startup (an input never came, ~start refused);
5 disk below the limit -> aborted; 6 /clock stalled; 7 the navigator status stopped.
"""
import argparse
import json
import os
import shutil
import sys
import threading
import time

import rospy
from rosgraph_msgs.msg import Clock

EXIT_DONE, EXIT_TIMEOUT, EXIT_STARTUP, EXIT_DISK, EXIT_STALL, EXIT_NAV_GONE = 0, 3, 4, 5, 6, 7


def log(msg):
    print("[sim_mission_driver %s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


class Watch:
    """Arrival bookkeeping: the wall time of the last message per topic, and the latest sim clock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.last_wall = {}
        self.clock = None
        self.clock_wall = None
        self.clock_values = set()
        self.status = None
        self.status_wall = None

    def seen(self, key):
        with self.lock:
            self.last_wall[key] = time.time()

    def on_clock(self, msg):
        t = msg.clock.to_sec()
        with self.lock:
            if self.clock is None or t != self.clock:
                self.clock_wall = time.time()
            self.clock = t
            if len(self.clock_values) < 3:
                self.clock_values.add(t)

    def on_status(self, msg):
        with self.lock:
            self.status = msg
            self.status_wall = time.time()


def wait_until(pred, budget, what, poll=0.5):
    t0 = time.time()
    while time.time() - t0 < budget:
        if rospy.is_shutdown():
            return False
        if pred():
            log("%s after %.1f s wall" % (what, time.time() - t0))
            return True
        time.sleep(poll)
    log("FAIL: %s not within %.0f s wall" % (what, budget))
    return False


def cmd_wait_clock(a):
    rospy.init_node("sim_mission_wait_clock", anonymous=True, disable_signals=True)
    w = Watch()
    rospy.Subscriber("/clock", Clock, w.on_clock, queue_size=10)
    ok = wait_until(lambda: len(w.clock_values) >= 2, a.budget, "/clock advancing")
    if ok:
        log("sim clock at %.2f s" % w.clock)
    return 0 if ok else EXIT_STARTUP


def free_gb(path):
    return shutil.disk_usage(path).free / 1e9


def cmd_run(a):
    from corridor_nav_msgs.msg import NavStatus
    from sensor_msgs.msg import CameraInfo, Image
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from treemapper_msgs.msg import Detection2DArray, TreeMap

    rospy.init_node("sim_mission_driver", anonymous=True, disable_signals=True)
    w = Watch()
    summary = {"run_dir": a.run_dir, "timeout_sim_s": a.timeout, "wall_start": time.time(), "exit": None,
               "reason": None, "end_state": None}

    def on_tf(msg):
        if any(t.header.frame_id.lstrip("/") == "world" and t.child_frame_id.lstrip("/") == "body"
               for t in msg.transforms):
            w.seen("pose")

    rospy.Subscriber("/clock", Clock, w.on_clock, queue_size=10)
    rospy.Subscriber("/cam/bottom/left/half/camera_info", CameraInfo, lambda m: w.seen("left_info"), queue_size=1)
    rospy.Subscriber("/cam/bottom/right/half/camera_info", CameraInfo, lambda m: w.seen("right_info"), queue_size=1)
    rospy.Subscriber("/tf", TFMessage, on_tf, queue_size=50)
    rospy.Subscriber("/disparity_image_down_float", Image, lambda m: w.seen("disparity"), queue_size=1,
                     buff_size=2 ** 24)
    rospy.Subscriber("/object_detection/detections", Detection2DArray, lambda m: w.seen("detections"), queue_size=1)
    rospy.Subscriber("/treemapper/map", TreeMap, lambda m: w.seen("map"), queue_size=1)
    rospy.Subscriber("/corridor_nav/status", NavStatus, w.on_status, queue_size=5)

    def finish(code, reason):
        summary.update(exit=code, reason=reason, wall_end=time.time(), sim_end=w.clock,
                       end_state=w.status.state if w.status is not None else None)
        if w.status is not None:
            summary["trees_sprayed"] = int(w.status.trees_sprayed)
            summary["trees_skipped"] = int(w.status.trees_skipped)
            summary["trees_remaining"] = int(w.status.trees_remaining)
            summary["counters"] = {str(r): int(c) for r, c in zip(w.status.reasons, w.status.counts)}
        summary["wall_s"] = summary["wall_end"] - summary["wall_start"]
        try:
            with open(os.path.join(a.run_dir, "driver.json"), "w") as f:
                json.dump(summary, f, indent=1)
        except OSError as e:
            log("cannot write driver.json: %s" % e)
        log("exit %d: %s" % (code, reason))
        return code

    # 1. inputs
    for key, what in (("left_info", "left camera_info"), ("right_info", "right camera_info"),
                      ("pose", "/tf world -> body"), ("disparity", "disparity"), ("detections", "detections"),
                      ("map", "/treemapper/map")):
        if not wait_until(lambda k=key: k in w.last_wall, a.startup_budget, what):
            return finish(EXIT_STARTUP, "input never arrived: %s" % what)
    try:
        rospy.wait_for_service("/corridor_nav/start", timeout=a.startup_budget)
    except rospy.ROSException:
        return finish(EXIT_STARTUP, "/corridor_nav/start service not available")
    start = rospy.ServiceProxy("/corridor_nav/start", Trigger)
    abort = rospy.ServiceProxy("/corridor_nav/abort", Trigger)

    # 2. start
    t0 = time.time()
    while True:
        r = start()
        log("~start: success %s, %s" % (r.success, r.message))
        if r.success:
            break
        if "not ready" not in r.message or time.time() - t0 > a.startup_budget:
            return finish(EXIT_STARTUP, "~start refused: %s" % r.message)
        time.sleep(1.0)
    sim_start, wall_start = w.clock, time.time()
    summary.update(sim_start=sim_start, wall_mission_start=wall_start)
    log("mission started at sim %.2f s" % sim_start)

    # 3. wait for DONE
    def abort_and_wait(code, reason):
        log("%s: calling ~abort" % reason)
        try:
            log("~abort: %s" % abort().message)
        except rospy.ServiceException as e:
            log("~abort failed: %s" % e)
        s0, wall0 = w.clock, time.time()
        while time.time() - wall0 < a.abort_wait / a.min_rtf:
            if w.status is not None and w.status.state == "DONE":
                break
            if w.clock is not None and s0 is not None and w.clock - s0 >= a.abort_wait:
                break
            time.sleep(0.5)
        log("after ~abort: state %s" % (w.status.state if w.status is not None else "none"))
        return finish(code, reason)

    last_state, last_log = None, 0.0
    while True:
        now = time.time()
        st = w.status
        if st is not None and st.state != last_state:
            log("state %s (corridor %d/%d, target %d, sprayed %d, skipped %d, remaining %d) sim %.2f s" % (
                st.state, st.corridor, st.corridors_total, st.target_id, st.trees_sprayed, st.trees_skipped,
                st.trees_remaining, w.clock))
            last_state = st.state
        if st is not None and st.state == "DONE":
            return finish(EXIT_DONE, "DONE")
        if w.clock_wall is not None and now - w.clock_wall > a.stall:
            return finish(EXIT_STALL, "/clock stalled for %.0f s wall (Isaac gone?)" % (now - w.clock_wall))
        if w.status_wall is None or now - w.status_wall > a.stall:
            if now - wall_start > a.stall:
                return finish(EXIT_NAV_GONE, "no /corridor_nav/status for %.0f s wall" % a.stall)
        if w.clock - sim_start > a.timeout:
            return abort_and_wait(EXIT_TIMEOUT, "timeout: %.0f sim s" % a.timeout)
        if now - wall_start > a.timeout / a.min_rtf:
            return abort_and_wait(EXIT_TIMEOUT, "wall budget: %.0f s (timeout / min_rtf)" % (a.timeout / a.min_rtf))
        fg = free_gb(a.run_dir)
        if fg < a.min_free_gb:
            return abort_and_wait(EXIT_DISK, "disk: %.1f GB free < %.1f GB" % (fg, a.min_free_gb))
        if now - last_log >= 30.0:
            last_log = now
            elapsed = w.clock - sim_start
            log("sim +%.1f s, wall +%.1f s (RTF %.2f), state %s, %.1f GB free" % (
                elapsed, now - wall_start, elapsed / max(now - wall_start, 1e-9),
                st.state if st is not None else "none", fg))
        time.sleep(0.5)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    w = sub.add_parser("wait-clock")
    w.add_argument("--budget", type=float, default=400.0, help="wall s")
    r = sub.add_parser("run")
    r.add_argument("--run-dir", required=True)
    r.add_argument("--timeout", type=float, default=1800.0, help="sim s from ~start to DONE")
    r.add_argument("--abort-wait", type=float, default=60.0, help="sim s to wait for DONE after ~abort")
    r.add_argument("--min-rtf", type=float, default=0.25, help="wall budget = sim budget / min_rtf")
    r.add_argument("--startup-budget", type=float, default=180.0, help="wall s per input / for ~start")
    r.add_argument("--stall", type=float, default=60.0, help="wall s without /clock or NavStatus -> fail")
    r.add_argument("--min-free-gb", type=float, default=8.0)
    a = ap.parse_args(argv)
    return cmd_wait_clock(a) if a.cmd == "wait-clock" else cmd_run(a)


if __name__ == "__main__":
    sys.exit(main())
