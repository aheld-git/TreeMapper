#!/usr/bin/env python3
"""rostest checker for a live treemapper_node (test/replay_smoke.test): heartbeat present, a non-empty map
within ~wait_map s, get_trees (all / by id / unknown id) and set_tree_sprayed (unknown id, un-spray refused)
answer, drop counters published, and the node still alive (heartbeat) at the end."""
import sys
import time
import unittest

import rosgraph
import rospy
import rostest
from treemapper_msgs.msg import DropCounters, Health, TreeMap
from treemapper_msgs.srv import GetTrees, SetTreeSprayed


class LiveCheck(unittest.TestCase):
    def setUp(self):
        self.health, self.maps, self.counters = [], [], []
        rospy.Subscriber("/treemapper/health", Health, self.health.append)
        rospy.Subscriber("/treemapper/map", TreeMap, self.maps.append)
        rospy.Subscriber("/treemapper/counters", DropCounters, self.counters.append)

    def wait(self, cond, timeout):
        t0 = time.time()
        while not cond() and time.time() - t0 < timeout and not rospy.is_shutdown():
            time.sleep(0.2)
        return cond()

    def test_live(self):
        self.assertTrue(self.wait(lambda: len(self.health) > 0, 30.0), "no heartbeat on /treemapper/health")
        wait_map = rospy.get_param("~wait_map", 90.0)
        self.assertTrue(self.wait(lambda: any(len(m.trees) > 0 for m in self.maps), wait_map),
                        "no non-empty map within %.0f s (%d map messages)" % (wait_map, len(self.maps)))
        states = sorted(set(h.state for h in self.health))
        rospy.loginfo("health states seen: %s, last reason: %s", states, self.health[-1].reason)
        rospy.wait_for_service("/treemapper/get_trees", 10.0)
        get = rospy.ServiceProxy("/treemapper/get_trees", GetTrees)
        allt = get(-1)
        self.assertTrue(allt.found)
        self.assertGreater(len(allt.trees), 0)
        for e in allt.trees:
            self.assertGreaterEqual(len(e.outline), 3)
            self.assertTrue(e.height == e.height and abs(e.height) < 1e3, "non-finite height %r" % e.height)
        one = get(allt.trees[0].id)
        self.assertTrue(one.found)
        self.assertEqual([e.id for e in one.trees], [allt.trees[0].id])
        self.assertFalse(get(987654).found)
        spray = rospy.ServiceProxy("/treemapper/set_tree_sprayed", SetTreeSprayed)
        self.assertFalse(spray(987654, True).success)
        self.assertFalse(spray(allt.trees[0].id, False).success, "un-spraying must be refused")
        self.assertTrue(self.wait(lambda: any(c.stage == "input" for c in self.counters)
                                  and any(c.stage == "mapper" for c in self.counters), 5.0), "no drop counters")
        n = len(self.health)
        self.assertTrue(self.wait(lambda: len(self.health) > n, 5.0), "heartbeat stopped: node dead?")
        master = rosgraph.Master("/check_live")
        pubs = dict((t, ns) for t, ns in master.getSystemState()[0])
        self.assertIn("/treemapper", pubs.get("/treemapper/health", []), "treemapper node gone")


if __name__ == "__main__":
    rospy.init_node("check_live", anonymous=True)
    rostest.rosrun("treemapper_ros", "replay_smoke", LiveCheck, sys.argv)
