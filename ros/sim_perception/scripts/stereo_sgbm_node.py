#!/usr/bin/env python3
"""stereo_sgbm: exact-stamp synced left/right image_raw -> SGBM -> 32FC1 disparity (px, invalid = NaN).

Output header = the LEFT image's header, unchanged. Frames that cannot be processed are counted by reason on
``~counters`` (DropCounters, stage "stereo"):
  unsynced       a stamp arrived on one camera and never got its partner (TimeSynchronizer is exact-stamp)
  size_mismatch  left and right resolutions differ
  bad_image      unsupported encoding / malformed buffer
Per-frame cost on ``~timing`` (StageTiming, stage "sgbm").
"""
import collections
import time

import message_filters
import rospy
from sensor_msgs.msg import Image
from treemapper_msgs.msg import DropCounters, StageTiming

from sim_perception import sgbm
from sim_perception.image_utils import msg_to_grey

REASONS = ("unsynced", "size_mismatch", "bad_image")
SYNC_QUEUE = 10


class StereoSgbmNode:
    def __init__(self):
        left_topic = rospy.get_param("~left_topic", "/cam/bottom/left/half/image_raw")
        right_topic = rospy.get_param("~right_topic", "/cam/bottom/right/half/image_raw")
        out_topic = rospy.get_param("~disparity_topic", "/disparity_image_down_float")
        try:
            self.matcher = sgbm.make_matcher(dict(rospy.get_param("~sgbm", {})))
        except ValueError as e:
            rospy.logfatal("stereo_sgbm: bad ~sgbm parameters: %s", e)
            raise SystemExit(1)

        self.counts = collections.OrderedDict((r, 0) for r in REASONS)
        self.frames = 0
        # stamp -> [arrived on left, arrived on right, delivered by the synchronizer]; oldest evicted past SYNC_QUEUE
        self.pending = collections.OrderedDict()

        self.pub = rospy.Publisher(out_topic, Image, queue_size=2)
        self.pub_counters = rospy.Publisher("~counters", DropCounters, queue_size=2)
        self.pub_timing = rospy.Publisher("~timing", StageTiming, queue_size=2)

        sub_l = message_filters.Subscriber(left_topic, Image, queue_size=SYNC_QUEUE)
        sub_r = message_filters.Subscriber(right_topic, Image, queue_size=SYNC_QUEUE)
        self.sync = message_filters.TimeSynchronizer([sub_l, sub_r], SYNC_QUEUE)
        self.sync.registerCallback(self.on_pair)
        # bookkeeping only, to count stamps the synchronizer silently drops
        sub_l.registerCallback(lambda m: self.on_arrival(m, 0))
        sub_r.registerCallback(lambda m: self.on_arrival(m, 1))
        rospy.Timer(rospy.Duration(1.0), lambda _e: self.publish_counters())
        rospy.loginfo("stereo_sgbm: %s + %s -> %s", left_topic, right_topic, out_topic)

    # --- drop accounting ------------------------------------------------------------------------------------------
    def _entry(self, stamp):
        e = self.pending.get(stamp)
        if e is None:
            e = self.pending[stamp] = [False, False, False]
            while len(self.pending) > SYNC_QUEUE:
                _s, old = self.pending.popitem(last=False)
                if not old[2]:
                    self.counts["unsynced"] += 1
        return e

    def on_arrival(self, msg, side):
        self._entry(msg.header.stamp)[side] = True

    def drop(self, reason):
        self.counts[reason] += 1
        self.publish_counters()

    def publish_counters(self):
        m = DropCounters()
        m.header.stamp = rospy.Time.now()
        m.stage = "stereo"
        m.reasons = list(self.counts.keys())
        m.counts = list(self.counts.values())
        self.pub_counters.publish(m)

    # --- frame ----------------------------------------------------------------------------------------------------
    def on_pair(self, left, right):
        self._entry(left.header.stamp)[2] = True
        if (left.width, left.height) != (right.width, right.height):
            self.drop("size_mismatch")
            return
        t0 = time.perf_counter()
        try:
            lg, rg = msg_to_grey(left), msg_to_grey(right)
        except ValueError as e:
            rospy.logwarn_throttle(5.0, "stereo_sgbm: %s", e)
            self.drop("bad_image")
            return
        disp = sgbm.disparity(lg, rg, self.matcher)
        out = Image()
        out.header = left.header
        out.height, out.width = disp.shape
        out.encoding = "32FC1"
        out.is_bigendian = 0
        out.step = 4 * out.width
        out.data = disp.tobytes()
        self.pub.publish(out)
        self.frames += 1
        t = StageTiming()
        t.header = left.header
        t.stages = ["sgbm"]
        t.ms = [(time.perf_counter() - t0) * 1000.0]
        self.pub_timing.publish(t)


if __name__ == "__main__":
    rospy.init_node("stereo_sgbm")
    StereoSgbmNode()
    rospy.spin()
