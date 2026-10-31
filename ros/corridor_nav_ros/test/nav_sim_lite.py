#!/usr/bin/env python3
"""nav_sim_lite: the whole world of corridor_nav_node for test/nav_sim_lite.test, no Isaac, no TreeMapper.

  clock      owns /clock: sim time advances 0.02 s per loop iteration, paced at `~rate` x wall time (default 5).
  drone      a point that flies at the latest /corridor_nav/setpoint at 4 m/s (xy) and 1.5 m/s (z); the pose is a
             DYNAMIC TF world -> body at 50 Hz sim, stamped with the sim time (a static TF reads as a stale pose).
  map        TreeMapper's role: 8 scripted trees (2 corridors of 4), latched /treemapper/map at 5 Hz sim, stamped with
             the sim time. A tree appears once the drone has been within 6 m xy of it while /mission_state is SCAN,
             and stays. Service /treemapper/set_tree_sprayed marks a published tree sprayed in later maps.
  disparity  /disparity_image_down_float (32FC1, constant range Z = drone z - 4 m: ground at z = 0, canopies 4 m tall;
             invalid, NaN, while Z < 0.2 m) at 4 Hz sim, with the half camera_infos of the Isaac / farm setup:
             fx = fy = 400, baseline 0.12 m. The camera is 640 x 640 px (77.3 deg) so that the default config plans
             exactly 2 corridors in field_lite.yaml (the brief's 640 x 400 would give a 53 deg vertical field of view,
             a 2 m maximum corridor width and "coverage impossible").
  stats      /nav_sim_lite/stats (std_srvs/Trigger, JSON in `message`): the world's own account for check_mission.py -
             truth trees, set_tree_sprayed calls, nozzle seconds over a tree / elsewhere, drone pose, lowest clearance.
Real-time factor: if the machine cannot keep up the loop just runs late (sim time stays consistent); the achieved
rate and the worst lag are in the stats."""
import json
import math
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from tf2_msgs.msg import TFMessage
from treemapper_msgs.msg import TreeEntry, TreeMap
from treemapper_msgs.srv import SetTreeSprayed, SetTreeSprayedResponse

DT = 0.02                    # s of sim time per loop iteration (TF at 50 Hz sim)
T0_NS = 100 * 10 ** 9        # the sim clock starts at 100 s (a node ignores the tick while ROS time is 0)
MAP_EVERY = 10               # iterations: 5 Hz
DISPARITY_EVERY = 12         # iterations: 4.2 Hz (the guard wants a stamp at most guard_max_age_s = 0.5 s old)
INFO_EVERY = 100             # iterations: camera_info also every 2 s, besides the latch
SEE_RADIUS = 6.0             # m xy
V_XY, V_Z = 4.0, 1.5         # m/s
TREE_RADIUS = 1.5            # m, circular crowns (16-gon outline)
CANOPY_Z = 4.0               # m, what the fake disparity "sees" below the drone
CLEAR_RADIUS = 1.3           # m xy: tree "under" the drone for the clearance statistic
NOZZLE_RADIUS = 0.5          # m xy: nozzle time within this of a tree centre counts as dosing that tree
HOME = (0.0, -2.0, 0.0)      # field_lite.yaml home; the drone starts on the ground there
FX = FY = 400.0
BASELINE = 0.12
WIDTH = HEIGHT = 640

# id, x, y, canopy top (world z). Corridor 0 is the strip y in [0, 6], corridor 1 the strip y in [6, 12].
TREES = [
    (1, 6.0, 3.5, 4.0), (2, 12.0, 4.6, 4.2), (3, 18.0, 3.0, 3.8), (4, 24.0, 3.8, 4.4),
    (5, 7.0, 9.4, 4.1), (6, 13.0, 8.5, 3.9), (7, 19.0, 9.0, 4.3), (8, 25.0, 9.8, 4.0),
]


def stamp(ns):
    return rospy.Time(ns // 10 ** 9, ns % 10 ** 9)


class Tree:
    def __init__(self, tid, x, y, h):
        self.id, self.x, self.y, self.h = tid, x, y, h
        self.published = False
        self.sprayed = False
        self.marked_calls = 0       # set_tree_sprayed(true) calls that succeeded
        self.nozzle_s = 0.0         # nozzle seconds with the drone within NOZZLE_RADIUS xy of the centre
        self.first_ns = 0

    def entry(self, now):
        e = TreeEntry()
        e.id = self.id
        for k in range(16):
            a = 2.0 * math.pi * k / 16
            e.outline.append(Point(self.x + TREE_RADIUS * math.cos(a), self.y + TREE_RADIUS * math.sin(a), 0.0))
        e.centre = Point(self.x, self.y, 0.0)
        e.height = self.h
        e.sprayed = self.sprayed
        e.observations = 10
        e.first_published = stamp(self.first_ns)
        e.last_updated = now
        return e


class World:
    def __init__(self):
        self.rate = rospy.get_param("~rate", 5.0)
        self.lock = threading.Lock()
        self.trees = [Tree(*t) for t in TREES]
        self.drone = list(HOME)
        self.yaw = 0.0
        self.setpoint = None          # (x, y, z, yaw)
        self.phase = "IDLE"
        self.nozzle = False
        self.nozzle_edges = 0
        self.nozzle_elsewhere_s = 0.0
        self.min_clearance = float("inf")   # drone z - canopy top, over trees within CLEAR_RADIUS xy
        self.refused_calls = 0
        self.map_dirty = True
        self.wall_lag_max = 0.0
        self.iterations = 0
        self.wall_elapsed = 0.0

        self.pub_clock = rospy.Publisher("/clock", Clock, queue_size=10)
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=100)
        self.pub_map = rospy.Publisher("/treemapper/map", TreeMap, queue_size=2, latch=True)
        self.pub_disp = rospy.Publisher("/disparity_image_down_float", Image, queue_size=1)
        self.pub_left = rospy.Publisher("/cam/bottom/left/half/camera_info", CameraInfo, queue_size=1, latch=True)
        self.pub_right = rospy.Publisher("/cam/bottom/right/half/camera_info", CameraInfo, queue_size=1, latch=True)
        rospy.Subscriber("/corridor_nav/setpoint", PoseStamped, self.on_setpoint, queue_size=1)
        rospy.Subscriber("/corridor_nav/spray", Bool, self.on_spray, queue_size=1)
        rospy.Subscriber("/mission_state", String, self.on_phase, queue_size=5)
        rospy.Service("/treemapper/set_tree_sprayed", SetTreeSprayed, self.on_set_sprayed)
        rospy.Service("/nav_sim_lite/stats", Trigger, self.on_stats)

    # --- inputs (rospy threads) -------------------------------------------------------------------------------
    def on_setpoint(self, m):
        q = m.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        with self.lock:
            self.setpoint = (m.pose.position.x, m.pose.position.y, m.pose.position.z, yaw)

    def on_spray(self, m):
        with self.lock:
            self.nozzle = bool(m.data)

    def on_phase(self, m):
        with self.lock:
            self.phase = m.data

    def on_set_sprayed(self, req):
        with self.lock:
            tree = next((t for t in self.trees if t.id == req.id and t.published), None)
            if tree is None:
                self.refused_calls += 1
                return SetTreeSprayedResponse(False, "unknown tree id %d" % req.id)
            if not req.sprayed:      # TreeMapper refuses to un-spray
                self.refused_calls += 1
                return SetTreeSprayedResponse(False, "un-spraying is refused")
            tree.marked_calls += 1
            tree.sprayed = True
            self.map_dirty = True
            return SetTreeSprayedResponse(True, "")

    def on_stats(self, _req):
        with self.lock:
            out = {
                "drone": list(self.drone),
                "trees": [{"id": t.id, "x": t.x, "y": t.y, "height": t.h, "published": t.published,
                           "sprayed": t.sprayed, "marked_calls": t.marked_calls, "nozzle_s": t.nozzle_s}
                          for t in self.trees],
                "refused_calls": self.refused_calls,
                "nozzle_edges": self.nozzle_edges,
                "nozzle_elsewhere_s": self.nozzle_elsewhere_s,
                "min_drone_clearance_m": self.min_clearance,
                "wall_lag_max_s": self.wall_lag_max,
                "rate_asked": self.rate,
                "rate_achieved": self.iterations * DT / self.wall_elapsed if self.wall_elapsed > 0 else 0.0,
            }
        return TriggerResponse(True, json.dumps(out))

    # --- one iteration of sim time ----------------------------------------------------------------------------
    def integrate(self):
        """The drone flies at the setpoint; the world updates what depends on the drone (under the lock)."""
        with self.lock:
            d = self.drone
            if self.setpoint is not None:
                sx, sy, sz, yaw = self.setpoint
                dx, dy = sx - d[0], sy - d[1]
                dxy = math.hypot(dx, dy)
                step = min(dxy, V_XY * DT)
                if dxy > 0.0:
                    d[0] += dx / dxy * step
                    d[1] += dy / dxy * step
                d[2] += max(-V_Z * DT, min(V_Z * DT, sz - d[2]))
                d[2] = max(0.0, d[2])
                self.yaw = yaw
            for t in self.trees:
                dist = math.hypot(d[0] - t.x, d[1] - t.y)
                if not t.published and self.phase == "SCAN" and dist <= SEE_RADIUS:
                    t.published = True
                    t.first_ns = self.now_ns
                    self.map_dirty = True
                if dist <= CLEAR_RADIUS:
                    self.min_clearance = min(self.min_clearance, d[2] - t.h)
            if self.nozzle:
                if not self.was_nozzle:
                    self.nozzle_edges += 1
                over = [t for t in self.trees if math.hypot(d[0] - t.x, d[1] - t.y) <= NOZZLE_RADIUS]
                if over:
                    over[0].nozzle_s += DT
                else:
                    self.nozzle_elsewhere_s += DT
            self.was_nozzle = self.nozzle
            return tuple(d), self.yaw

    was_nozzle = False
    now_ns = T0_NS

    def publish_tf(self, now, pos, yaw):
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "world"
        tf.child_frame_id = "body"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pos
        tf.transform.rotation.z = math.sin(0.5 * yaw)
        tf.transform.rotation.w = math.cos(0.5 * yaw)
        self.pub_tf.publish(TFMessage([tf]))

    def publish_map(self, now):
        m = TreeMap()
        m.header.stamp = now
        m.header.frame_id = "world"
        with self.lock:
            m.trees = [t.entry(now) for t in self.trees if t.published]
            self.map_dirty = False
        self.pub_map.publish(m)

    def publish_disparity(self, now, z):
        clear = z - CANOPY_Z
        d = FX * BASELINE / clear if clear >= 0.2 else float("nan")
        img = Image()
        img.header.stamp = now
        img.header.frame_id = "cam_bottom_left"
        img.height, img.width = HEIGHT, WIDTH
        img.encoding = "32FC1"
        img.is_bigendian = 0
        img.step = WIDTH * 4
        img.data = np.full((HEIGHT, WIDTH), d, dtype="<f4").tobytes()
        self.pub_disp.publish(img)

    def publish_info(self, now):
        cx, cy = WIDTH / 2.0, HEIGHT / 2.0
        for pub, tx in ((self.pub_left, 0.0), (self.pub_right, -FX * BASELINE)):
            m = CameraInfo()
            m.header.stamp = now
            m.header.frame_id = "cam_bottom_left"
            m.width, m.height = WIDTH, HEIGHT
            m.distortion_model = "plumb_bob"
            m.D = [0.0] * 5
            m.K = [FX, 0.0, cx, 0.0, FY, cy, 0.0, 0.0, 1.0]
            m.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            m.P = [FX, 0.0, cx, tx, 0.0, FY, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            pub.publish(m)

    def run(self):
        wall0 = time.monotonic()
        k = 0
        step_wall = DT / self.rate
        while not rospy.is_shutdown():
            k += 1
            self.now_ns = T0_NS + k * int(round(DT * 1e9))
            now = stamp(self.now_ns)
            pos, yaw = self.integrate()
            # TF first, then the clock: a node that has seen the new time has the pose of that time on its way
            self.publish_tf(now, pos, yaw)
            self.pub_clock.publish(Clock(clock=now))
            if k == 1 or k % INFO_EVERY == 0:
                self.publish_info(now)
            if k % DISPARITY_EVERY == 0:
                self.publish_disparity(now, pos[2])
            if k % MAP_EVERY == 0 or self.map_dirty:
                self.publish_map(now)
            wall_now = time.monotonic() - wall0
            lag = wall_now - k * step_wall
            if lag < 0.0:
                time.sleep(-lag)
            with self.lock:
                self.iterations = k
                self.wall_elapsed = time.monotonic() - wall0
                self.wall_lag_max = max(self.wall_lag_max, lag)


if __name__ == "__main__":
    # No rospy time anywhere in here: with /use_sim_time set (the test sets it for everyone) rospy.Time.now() would
    # follow the /clock this node publishes. Everything is stamped from the loop's own sim counter.
    rospy.init_node("nav_sim_world")
    World().run()
