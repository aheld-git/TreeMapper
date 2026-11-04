#!/usr/bin/env python3
"""disparity_grid: /disparity_image_down_float -> /rec/disparity_grid (treemapper_msgs/DisparityGrid), RECORDING ONLY.

treemapper_node samples the disparity image on a stride-4 grid and uses nothing else of it. A bag with the full 640x400
float image costs ~1 MB per message; this node republishes only the sampled pixels (~64 kB), same header, so
``sim_mission.sh --record-frames`` can record what TreeMapper consumed within the disk budget. The live TreeMapper node keeps
consuming the full image; nothing else changes for it. The source image's own size / encoding / step are carried in the
message (see DisparityGrid.msg) so the reader applies the node's malformed / wrong-size rules exactly; a malformed image is
published as an empty grid, never dropped.

Nothing is dropped silently. ``~counters`` (DropCounters, stage "disparity_grid", 1 Hz, cumulative) holds:
  received    images received
  published   grids published
  malformed   images published as an empty grid (treemapper_node would reject them)
  stamp_gap   an image whose stamp is > 1.5x the previous interval after the previous one: images were lost before
              this node saw them
and each gap / malformed image is logged (throttled to 1 per second).

Where images can be lost, and who counts it: ``stereo_sgbm_node`` publishes the disparity with ``queue_size=2`` (a
per-connection outgoing queue that drops the oldest message when this node falls ~0.2 s behind; it also feeds the live
TreeMapper node, so it is not changed). Those drops happen before this node and are not visible to it except as
``stamp_gap``; ``tools/treemapper_tools/sim_frames.py`` counts them exactly as the number of images the stereo node published
(``/stereo_sgbm/timing``, one message per image) minus the grids in the bag, and fails the conversion above 1 %. The
subscriber's ``buff_size`` (64 MB) is rospy's receive chunk size, not a socket buffer; the callback takes about a millisecond,
and the outgoing queue of this node's own publisher is 100 deep.
"""
import collections

import rospy
from sensor_msgs.msg import Image
from treemapper_msgs.msg import DisparityGrid, DropCounters

from sim_perception.disparity_grid import grid_fields

REASONS = ("received", "published", "malformed", "stamp_gap")


class DisparityGridNode:
    def __init__(self):
        in_topic = rospy.get_param("~in_topic", "/disparity_image_down_float")
        out_topic = rospy.get_param("~out_topic", "/rec/disparity_grid")
        self.stride = int(rospy.get_param("~stride", 4))
        self.counts = collections.OrderedDict((r, 0) for r in REASONS)
        self.last_stamp = None
        self.last_dt = None
        self.pub = rospy.Publisher(out_topic, DisparityGrid, queue_size=100)
        self.pub_counters = rospy.Publisher("~counters", DropCounters, queue_size=2)
        rospy.Subscriber(in_topic, Image, self.on_image, queue_size=100, buff_size=2 ** 26, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(1.0), lambda _e: self.publish_counters())
        rospy.loginfo("disparity_grid: %s -> %s (stride %d)", in_topic, out_topic, self.stride)

    def publish_counters(self):
        m = DropCounters()
        m.header.stamp = rospy.Time.now()
        m.stage = "disparity_grid"
        m.reasons = list(self.counts.keys())
        m.counts = list(self.counts.values())
        self.pub_counters.publish(m)

    def on_image(self, msg):
        self.counts["received"] += 1
        stamp = msg.header.stamp.to_sec()
        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if self.last_dt and dt > 1.5 * self.last_dt:
                self.counts["stamp_gap"] += 1
                rospy.logwarn_throttle(1.0, "disparity_grid: %.3f s between images (previous %.3f s): images were lost",
                                       dt, self.last_dt)
            if dt > 0:
                self.last_dt = dt
        self.last_stamp = stamp
        f = grid_fields(msg.width, msg.height, msg.step, msg.encoding, msg.is_bigendian, msg.data, self.stride)
        out = DisparityGrid()
        out.header = msg.header
        for k, v in f.items():
            setattr(out, k, v)
        if f["grid_width"] == 0:
            self.counts["malformed"] += 1
            rospy.logwarn_throttle(1.0, "disparity_grid: malformed disparity image (%s %dx%d step %d, %d bytes)",
                                   msg.encoding, msg.width, msg.height, msg.step, len(msg.data))
        self.pub.publish(out)
        self.counts["published"] += 1


if __name__ == "__main__":
    rospy.init_node("disparity_grid")
    DisparityGridNode()
    rospy.spin()
