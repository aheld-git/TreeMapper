#!/usr/bin/env python3
"""nav_emulator: plays navigation for a REPLAYED mission (replay tooling, not for flight).

The recorded hop target (/spray_target in the bag, remapped to ~recorded_target_topic) belongs to the RECORDED
system's ID space. Like core/apps/replay_main.cpp does offline, this node resolves each recorded target to THIS
mapper's ID through its descent point (the drone xy at the start of the next SPRAYING for that target, from the
M1 extraction's mission.json; see treemapper_tools.bag_frames.target_points): the published tree whose outline
contains the point, else the nearest centre within 3 m. It publishes the resolved ID (std_msgs/Int32, latched)
on ~out_topic and calls set_tree_sprayed when a SPRAYING phase ends, as navigation would.

Parameters: ~mission_json (required), ~tools_path (repo tools/ dir, for treemapper_tools), ~out_topic
(/treemapper/hop_target), ~mission_state_topic (/mission_state), ~get_trees (/treemapper/get_trees),
~set_tree_sprayed (/treemapper/set_tree_sprayed), ~rate (20 Hz), ~targets_out ("" = none: JSON [[t, id], ...]).
"""
from __future__ import annotations

import json
import math
import os
import sys

import rospy
from std_msgs.msg import Int32, String
from treemapper_msgs.srv import GetTrees, SetTreeSprayed


def point_in_polygon(poly, x, y):
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def resolve_target(trees, x, y):
    """Same rule as resolveTarget() in core/apps/replay_main.cpp."""
    if not (math.isfinite(x) and math.isfinite(y)):
        return -1
    best, bd = -1, 3.0
    for e in trees:
        if point_in_polygon([(p.x, p.y) for p in e.outline], x, y):
            return e.id
        d = math.hypot(e.centre.x - x, e.centre.y - y)
        if d < bd:
            bd, best = d, e.id
    return best


class NavEmulator:
    def __init__(self):
        sys.path.insert(0, os.path.realpath(rospy.get_param("~tools_path")))
        from treemapper_tools.bag_frames import target_points
        from treemapper_tools.contract import read_mission, step_lookup
        self.step_lookup = step_lookup
        mission = read_mission(rospy.get_param("~mission_json"))
        tp = target_points(mission)
        self.t_ids = [(t, tid) for t, tid, _ in tp]
        self.t_xy = [(t, xy) for t, _, xy in tp]
        self.pub = rospy.Publisher(rospy.get_param("~out_topic", "/treemapper/hop_target"), Int32, queue_size=2, latch=True)
        self.get_name = rospy.get_param("~get_trees", "/treemapper/get_trees")
        self.spray_name = rospy.get_param("~set_tree_sprayed", "/treemapper/set_tree_sprayed")
        self.targets_out = rospy.get_param("~targets_out", "")
        self.nav_id, self.tx, self.ty, self.resolved = -1, math.nan, math.nan, -1
        self.prev_phase = ""
        self.timeline = []
        rospy.wait_for_service(self.get_name)
        self.get = rospy.ServiceProxy(self.get_name, GetTrees, persistent=True)
        self.spray = rospy.ServiceProxy(self.spray_name, SetTreeSprayed)
        self.pub.publish(Int32(-1))
        rospy.Subscriber(rospy.get_param("~mission_state_topic", "/mission_state"), String, self.on_phase, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param("~rate", 20.0)), self.on_timer)
        rospy.loginfo("nav_emulator: %d recorded target changes", len(self.t_ids))

    def trees(self):
        try:
            return self.get(-1).trees
        except Exception as e:  # node restarting / shutting down
            rospy.logwarn_throttle(5.0, "nav_emulator: get_trees failed: %s" % e)
            self.get = rospy.ServiceProxy(self.get_name, GetTrees, persistent=True)
            return []

    def set_resolved(self, r, t):
        if r != self.resolved:
            self.resolved = r
            self.timeline.append([t, r])
            self.pub.publish(Int32(r))
            rospy.loginfo("nav_emulator: recorded target %d -> mapper id %d", self.nav_id, r)

    def on_timer(self, _ev):
        t = rospy.Time.now().to_sec()
        if t <= 0:
            return
        if not self.timeline:
            self.timeline.append([t, -1])
        tid = self.step_lookup(self.t_ids, t, -1)
        tid = -1 if tid is None else tid
        x, y = self.step_lookup(self.t_xy, t, (math.nan, math.nan))
        changed = (tid != self.nav_id or math.isfinite(x) != math.isfinite(self.tx) or
                   (math.isfinite(x) and (x != self.tx or y != self.ty)))
        if changed:
            self.nav_id, self.tx, self.ty = tid, x, y
            self.set_resolved(-1 if tid < 0 else resolve_target(self.trees(), x, y), t)
        elif self.nav_id >= 0 and self.resolved == -1:
            r = resolve_target(self.trees(), self.tx, self.ty)
            if r != -1:
                self.set_resolved(r, t)

    def on_phase(self, msg):
        if self.prev_phase == "SPRAYING" and msg.data != "SPRAYING" and self.resolved != -1:
            try:
                res = self.spray(self.resolved, True)
                rospy.loginfo("nav_emulator: set_tree_sprayed(%d): %s" % (self.resolved, res.message))
            except Exception as e:
                rospy.logerr("nav_emulator: set_tree_sprayed(%d) failed: %s" % (self.resolved, e))
        self.prev_phase = msg.data

    def write(self):
        if self.targets_out:
            with open(self.targets_out, "w") as f:
                json.dump(self.timeline, f)


def main():
    rospy.init_node("nav_emulator")
    n = NavEmulator()
    rospy.on_shutdown(n.write)
    rospy.spin()


if __name__ == "__main__":
    main()
