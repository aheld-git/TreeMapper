#!/usr/bin/env python3
"""rostest checker for test/nav_sim_lite.test: flies the whole mission of corridor_nav_node against nav_sim_lite.py.

Calls ~start (retried until the node accepts: it needs camera_info and a fresh pose first), waits for
/mission_state == RETURN and then for the landing (NavStatus DONE), all inside `sim_limit_s` of SIM time, then asserts

  * 2 corridors planned (NavStatus.corridors_total), 8 trees sprayed, none skipped, final NavStatus state DONE;
  * every tree marked through set_tree_sprayed exactly once, no refused or unknown-id call, every tree sprayed=true in
    the last map;
  * the nozzle was never true outside the SPRAYING phase (+-0.15 s for the order of two topics), each tree got a dose
    of nozzle time with the drone over it, and no nozzle time at all away from a tree (the world's own account);
  * while the nozzle was open the drone's TRUE clearance over the canopy under it was >= the cfg min_clearance - 0.2 m;
  * the setpoint stream never went lower than max(PUBLISHED height of any tree within 1.3 m xy) + 1.5 m (the
    planner's promise on what it was told);
  * the clearance guard (stereo from the mm-form right camera_info, disparity from the truth) caught the tree the map
    publishes 1.2 m too low: guard_triggers >= 1, spray_point_guard_raised >= 1, that tree still sprayed once, and
    the drone's TRUE clearance over it stayed > 0 (over every other tree >= 1.4 m); no "clearance unavailable"
    warning;
  * the drone landed back at home.
The world's account (truth trees, service calls, nozzle time, drone) is read from /nav_sim_lite/stats."""
import json
import math
import os
import sys
import threading
import time
import unittest

import rospkg
import rospy
import rostest
from corridor_nav_msgs.msg import NavStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from treemapper_msgs.msg import TreeMap

CLEARANCE_RADIUS = 1.3   # m xy: a setpoint this close to a tree centre is "over" the tree
CLEARANCE_MIN = 1.5      # m above the canopy top
NOZZLE_CLEARANCE_TOL = 0.2   # m: the true clearance with the nozzle open may sit this far below the cfg min_clearance
PHASE_SLACK_S = 0.15     # s: spray and /mission_state are separate topics, one control tick apart at most
HOME = (0.0, -2.0)       # field_lite.yaml


def cfg_value(key):
    """`key` of the cfg file corridor_nav_node runs with (its ~config_file, else the node's default path)."""
    path = rospy.get_param("/corridor_nav/config_file", "config/core/corridor_nav.cfg")
    if not path.startswith("/"):
        path = os.path.join(rospkg.RosPack().get_path("corridor_nav_ros"), path)
    with open(path) as f:
        for line in f:
            k, sep, v = line.split("#", 1)[0].partition("=")
            if sep and k.strip() == key:
                return float(v)
    raise KeyError("%s not in %s" % (key, path))


class MissionCheck(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.status = None
        self.statuses = 0
        self.phases = []      # (sim t, phase) on every message
        self.sprays = []      # (sim t, bool)
        self.setpoints = []   # (x, y, z)
        self.last_map = None
        rospy.Subscriber("/corridor_nav/status", NavStatus, self.on_status, queue_size=10)
        rospy.Subscriber("/mission_state", String, self.on_phase, queue_size=50)
        rospy.Subscriber("/corridor_nav/spray", Bool, self.on_spray, queue_size=500)
        rospy.Subscriber("/corridor_nav/setpoint", PoseStamped, self.on_setpoint, queue_size=500)
        rospy.Subscriber("/treemapper/map", TreeMap, self.on_map, queue_size=2)

    # --- callbacks -------------------------------------------------------------------------------------------
    def on_status(self, m):
        with self.lock:
            self.status = m
            self.statuses += 1

    def on_phase(self, m):
        with self.lock:
            if not self.phases or self.phases[-1][1] != m.data:
                self.phases.append((rospy.get_time(), m.data))

    def on_spray(self, m):
        with self.lock:
            self.sprays.append((rospy.get_time(), bool(m.data)))

    def on_setpoint(self, m):
        with self.lock:
            self.setpoints.append((m.pose.position.x, m.pose.position.y, m.pose.position.z))

    def on_map(self, m):
        with self.lock:
            self.last_map = m

    # --- helpers ---------------------------------------------------------------------------------------------
    def wait(self, cond, wall_timeout):
        t0 = time.time()
        while not cond() and time.time() - t0 < wall_timeout and not rospy.is_shutdown():
            time.sleep(0.05)
        return cond()

    def state(self):
        with self.lock:
            return self.status.state if self.status else None

    def phase_seen(self, name):
        with self.lock:
            return any(p == name for _, p in self.phases)

    def phase_at(self, t):
        cur = None
        for tp, p in self.phases:
            if tp <= t:
                cur = p
        return cur

    def stats(self):
        res = rospy.ServiceProxy("/nav_sim_lite/stats", Trigger)()
        self.assertTrue(res.success)
        return json.loads(res.message)

    # --- the test --------------------------------------------------------------------------------------------
    def test_mission(self):
        sim_limit = rospy.get_param("~sim_limit_s", 600.0)
        self.assertTrue(self.wait(lambda: rospy.get_time() > 0.0, 60.0), "no /clock from nav_sim_lite")
        rospy.wait_for_service("/nav_sim_lite/stats", 30.0)
        rospy.wait_for_service("/corridor_nav/start", 30.0)
        self.assertTrue(self.wait(lambda: self.statuses > 0, 30.0), "no NavStatus on /corridor_nav/status")

        # ~start is refused until the navigator exists (camera_info) and the pose is fresh: retry
        start = rospy.ServiceProxy("/corridor_nav/start", Trigger)
        t_start = None
        wall0 = time.time()
        reply = None
        while t_start is None and time.time() - wall0 < 60.0:
            reply = start()
            if reply.success:
                t_start = rospy.get_time()
            else:
                time.sleep(0.5)
        self.assertIsNotNone(t_start, "~start never accepted: %s" % (reply.message if reply else "no reply"))
        rospy.loginfo("mission started at sim t=%.1f", t_start)

        # RETURN, then the landing
        def over(): return rospy.get_time() - t_start > sim_limit

        def failed(): return self.phase_seen("ERROR") or self.state() in ("ABORT",)
        self.wait(lambda: self.phase_seen("RETURN") or over() or failed(), 600.0)
        self.assertFalse(failed(), "mission ended in ERROR / ABORT: %s" % self.describe())
        self.assertTrue(self.phase_seen("RETURN"), "never reached RETURN in %.0f s sim: %s" % (sim_limit, self.describe()))
        t_return = rospy.get_time()
        self.wait(lambda: self.state() == "DONE" or over() or failed(), 600.0)
        self.assertEqual(self.state(), "DONE", "no landing within %.0f s sim: %s" % (sim_limit, self.describe()))
        t_done = rospy.get_time()
        rospy.loginfo("RETURN at sim t=%.1f (+%.1f s), DONE at +%.1f s", t_return, t_return - t_start, t_done - t_start)
        time.sleep(1.0)   # a last NavStatus / map / stats after the landing

        with self.lock:
            status = self.status
            sprays = list(self.sprays)
            phases = list(self.phases)
            setpoints = list(self.setpoints)
            last_map = self.last_map
        world = self.stats()
        counters = dict(zip(status.reasons, status.counts))
        rospy.loginfo("phases: %s", [p for _, p in phases])
        rospy.loginfo("NavStatus: state=%s corridors=%d sprayed=%d skipped=%d remaining=%d; counters %s; warnings %s",
                      status.state, status.corridors_total, status.trees_sprayed, status.trees_skipped,
                      status.trees_remaining, counters, list(status.warnings))
        rospy.loginfo("world: %s", json.dumps({k: v for k, v in world.items() if k != "trees"}, sort_keys=True))

        # --- the mission as the node reports it
        self.assertEqual(status.corridors_total, 2)
        self.assertEqual(status.trees_sprayed, 8)
        self.assertEqual(status.trees_skipped, 0)
        self.assertEqual(status.trees_remaining, 0)
        self.assertNotIn("ERROR", [p for _, p in phases])
        self.assertEqual([p for _, p in phases][:1] + [p for _, p in phases if p != "IDLE"][-1:], ["IDLE", "RETURN"])

        # --- the world's account: every tree marked once, and sprayed in the map
        trees = world["trees"]
        self.assertEqual(len(trees), 8)
        for t in trees:
            self.assertTrue(t["published"], "tree %d never appeared in the map" % t["id"])
            self.assertEqual(t["marked_calls"], 1, "tree %d: set_tree_sprayed(true) called %d times" % (t["id"], t["marked_calls"]))
            self.assertTrue(t["sprayed"], "tree %d not sprayed" % t["id"])
        self.assertEqual(world["refused_calls"], 0, "set_tree_sprayed calls refused or for an unknown id")
        self.assertIsNotNone(last_map)
        self.assertEqual(sorted(e.id for e in last_map.trees), sorted(t["id"] for t in trees))
        self.assertTrue(all(e.sprayed for e in last_map.trees), "last map has a tree that is not sprayed")

        # --- nozzle: only while SPRAYING (+- slack), only over a tree, a dose per tree
        self.assertTrue(any(v for _, v in sprays), "the nozzle never opened")
        for ts, v in sprays:
            if not v:
                continue
            near = {self.phase_at(ts + d) for d in (-PHASE_SLACK_S, 0.0, PHASE_SLACK_S)}
            self.assertIn("SPRAYING", near, "nozzle true at sim t=%.2f outside SPRAYING (phases %s)" % (ts, sorted(map(str, near))))
        self.assertLessEqual(world["nozzle_elsewhere_s"], 0.1, "nozzle open %.2f s away from any tree" % world["nozzle_elsewhere_s"])
        for t in trees:
            self.assertGreaterEqual(t["nozzle_s"], 1.6, "tree %d got only %.2f s of nozzle with the drone over it" % (t["id"], t["nozzle_s"]))

        # --- nozzle altitude: with the nozzle open the TRUE clearance over the canopy under the drone stays at the cfg
        # min_clearance (less a tolerance), not just above 0 - the guard's floor holds while spraying too
        min_clearance = cfg_value("min_clearance")
        for t in trees:
            c = t["nozzle_min_clearance_m"]
            self.assertIsNotNone(c, "tree %d: no nozzle clearance recorded" % t["id"])
            rospy.loginfo("tree %d: lowest true clearance with the nozzle open %.2f m (floor %.2f - %.1f)",
                          t["id"], c, min_clearance, NOZZLE_CLEARANCE_TOL)
            self.assertGreaterEqual(c, min_clearance - NOZZLE_CLEARANCE_TOL,
                                    "tree %d sprayed from %.2f m above its canopy (min_clearance %.2f - %.1f)" % (
                                        t["id"], c, min_clearance, NOZZLE_CLEARANCE_TOL))

        # --- clearance: setpoints never lower than the PUBLISHED canopy + 1.5 m within 1.3 m xy of a tree
        self.assertGreater(len(setpoints), 100)
        worst = None
        for (x, y, z) in setpoints:
            tops = [t["map_height"] for t in trees if math.hypot(x - t["x"], y - t["y"]) <= CLEARANCE_RADIUS]
            if tops and (worst is None or z - max(tops) < worst):
                worst = z - max(tops)
        self.assertIsNotNone(worst, "no setpoint came near a tree")
        rospy.loginfo("lowest setpoint over a tree: %.2f m above the published canopy", worst)
        self.assertGreaterEqual(worst, CLEARANCE_MIN - 1e-6, "setpoint %.2f m above a canopy (< %.1f)" % (worst, CLEARANCE_MIN))

        # --- the clearance guard end to end: the tree published too low is caught, raised and still sprayed
        biased = [t for t in trees if t["map_height"] < t["height"] - 1.0]
        self.assertEqual(len(biased), 1, "nav_sim_lite should publish exactly one tree too low")
        b = biased[0]
        rospy.loginfo("guard: triggers %d, spray points raised %d; true clearance over tree %d (published %.1f m low): "
                      "%.2f m; over the others: %s", counters.get("guard_triggers", 0),
                      counters.get("spray_point_guard_raised", 0), b["id"], b["height"] - b["map_height"],
                      b["min_clearance_m"], {t["id"]: t["min_clearance_m"] for t in trees if t is not b})
        self.assertGreaterEqual(counters.get("guard_triggers", 0), 1, "the clearance guard never climbed")
        self.assertGreaterEqual(counters.get("spray_point_guard_raised", 0), 1, "no spray point raised by the guard")
        self.assertEqual(b["marked_calls"], 1)
        self.assertGreaterEqual(b["nozzle_s"], 1.6)
        self.assertIsNotNone(b["min_clearance_m"])
        self.assertGreater(b["min_clearance_m"], 0.0, "the drone hit tree %d (true clearance %.2f m)" % (b["id"], b["min_clearance_m"]))
        self.assertGreater(world["min_drone_clearance_m"], 0.0, "true clearance %.2f m" % world["min_drone_clearance_m"])
        for t in trees:
            if t is not b:
                self.assertGreaterEqual(t["min_clearance_m"], CLEARANCE_MIN - 0.1,
                                        "the drone got to %.2f m above tree %d" % (t["min_clearance_m"], t["id"]))
        self.assertFalse([w for w in status.warnings if w.startswith("clearance unavailable")],
                         "clearance reported unavailable: %s" % list(status.warnings))

        # --- landed at home
        x, y, z = world["drone"]
        self.assertLessEqual(z, 0.35, "not on the ground: z=%.2f" % z)
        self.assertLessEqual(math.hypot(x - HOME[0], y - HOME[1]), 1.0, "landed at (%.2f, %.2f), home %s" % (x, y, HOME))

        # --- the sim kept its schedule (a starved machine would make the above flaky: say so)
        rospy.loginfo("sim rate x%.2f wall (asked %.2f); worst loop lag %.0f ms", world["rate_achieved"], world["rate_asked"], 1e3 * world["wall_lag_max_s"])

    def describe(self):
        with self.lock:
            s = self.status
            phases = [p for _, p in self.phases]
        if s is None:
            return "no status"
        return "state=%s sprayed=%d phases=%s counters=%s warnings=%s" % (
            s.state, s.trees_sprayed, phases, dict(zip(s.reasons, s.counts)), list(s.warnings))


if __name__ == "__main__":
    rospy.init_node("check_mission", anonymous=True)
    rostest.rosrun("corridor_nav_ros", "nav_sim_lite", MissionCheck, sys.argv)
