"""Isaac-only: the stereo rig on the drone, the /clock + image OmniGraph, camera_info and static TF.

Both cameras are ideal rectified pinholes with the LEFT ``P`` intrinsics of ``sim/specs/camera.json``
(fx = fy = 454.53, cx = 346.71, cy = 209.68, 640 x 400). The right camera is the left one translated
+B (0.160 m) along the left optical x axis. Mount (optical -> body) comes from the same file.

Topics (rgb8, 10 Hz by default):
    /cam/bottom/{left,right}/half/image_raw        OmniGraph ROS2CameraHelper
    /cam/bottom/{left,right}/half/camera_info      ``CameraInfoRelay``: K / P exactly as in camera.json, stamped like
                                                   the image it accompanies
    /clock                                         rclpy, every physics step (orchard_sim.py)
    /tf_static  world-independent body -> cam_left / cam_right (optical frames), replayed at 1 Hz by the caller
"""
from typing import Dict, List, Sequence

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom

from isaac import frames
from isaac.drone import BODY_FRAME, DRONE_PATH

LEFT_FRAME = "cam_left"
RIGHT_FRAME = "cam_right"
TOPICS = {
    "left": {"image": "/cam/bottom/left/half/image_raw", "info": "/cam/bottom/left/half/camera_info"},
    "right": {"image": "/cam/bottom/right/half/image_raw", "info": "/cam/bottom/right/half/camera_info"},
}
FRAME_IDS = {"left": LEFT_FRAME, "right": RIGHT_FRAME}


def camera_path(side: str, parent: str = DRONE_PATH) -> str:
    return "%s/cam_%s" % (parent, side)


def _row_matrix(t: np.ndarray, R: np.ndarray) -> Gf.Matrix4d:
    return Gf.Matrix4d(
        float(R[0, 0]), float(R[1, 0]), float(R[2, 0]), 0.0,
        float(R[0, 1]), float(R[1, 1]), float(R[2, 1]), 0.0,
        float(R[0, 2]), float(R[1, 2]), float(R[2, 2]), 0.0,
        float(t[0]), float(t[1]), float(t[2]), 1.0)


def define_cameras(stage: Usd.Stage, rig: frames.CameraRig, parent: str = DRONE_PATH,
                   far: float = 200.0) -> Dict[str, str]:
    """Create ``<parent>/cam_left`` and ``cam_right``; returns {side: prim path}."""
    par = rig.usd_camera_params()
    out = {}
    for side in ("left", "right"):
        path = camera_path(side, parent)
        cam = UsdGeom.Camera.Define(stage, path)
        cam.CreateProjectionAttr(UsdGeom.Tokens.perspective)
        cam.CreateHorizontalApertureAttr(par["h_aperture_mm"])
        cam.CreateVerticalApertureAttr(par["v_aperture_mm"])
        cam.CreateFocalLengthAttr(par["focal_mm"])
        cam.CreateHorizontalApertureOffsetAttr(par["h_off_mm"])
        cam.CreateVerticalApertureOffsetAttr(par["v_off_mm"])
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, far))
        t, R = rig.usd_pose_in_body(side)
        UsdGeom.Xformable(cam).AddTransformOp().Set(_row_matrix(t, R))
        out[side] = path
    return out


def build_ros_graph(cam_paths: Dict[str, str], rig: frames.CameraRig, domain_id: int) -> Dict[str, str]:
    """OmniGraph with the two image publishers (one frame per app update; /clock and TF come from rclpy)."""
    import omni.graph.core as og
    keys = og.Controller.Keys
    path = "/ActionGraph"
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
                # The image stamps come from this graph's simulation time. A probe publisher keeps the node
                # evaluated so orchard_sim can read the very time base the images are stamped with.
                ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("probe", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("rp_left", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("rp_right", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("img_left", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("img_right", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("ctx.inputs:useDomainIDEnvVar", False),
                ("ctx.inputs:domain_id", int(domain_id)),
                ("probe.inputs:topicName", "/tm_sim_time_probe"),
                ("rp_left.inputs:cameraPrim", [Sdf.Path(cam_paths["left"])]),
                ("rp_right.inputs:cameraPrim", [Sdf.Path(cam_paths["right"])]),
                ("rp_left.inputs:width", rig.width), ("rp_left.inputs:height", rig.height),
                ("rp_right.inputs:width", rig.width), ("rp_right.inputs:height", rig.height),
                ("img_left.inputs:type", "rgb"), ("img_right.inputs:type", "rgb"),
                ("img_left.inputs:topicName", TOPICS["left"]["image"]),
                ("img_right.inputs:topicName", TOPICS["right"]["image"]),
                ("img_left.inputs:frameId", LEFT_FRAME), ("img_right.inputs:frameId", RIGHT_FRAME),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "probe.inputs:execIn"),
                ("ctx.outputs:context", "probe.inputs:context"),
                ("simtime.outputs:simulationTime", "probe.inputs:timeStamp"),
                ("tick.outputs:tick", "rp_left.inputs:execIn"),
                ("tick.outputs:tick", "rp_right.inputs:execIn"),
                ("ctx.outputs:context", "img_left.inputs:context"),
                ("ctx.outputs:context", "img_right.inputs:context"),
                ("rp_left.outputs:execOut", "img_left.inputs:execIn"),
                ("rp_right.outputs:execOut", "img_right.inputs:execIn"),
                ("rp_left.outputs:renderProductPath", "img_left.inputs:renderProductPath"),
                ("rp_right.outputs:renderProductPath", "img_right.inputs:renderProductPath"),
            ],
        },
    )
    return {"graph": path, "rp_left": path + "/rp_left"}


def graph_sim_time() -> float:
    """Simulation time of the OmniGraph in the current app update (the time base of the image stamps)."""
    import omni.graph.core as og
    return float(og.Controller.attribute("/ActionGraph/simtime.outputs:simulationTime").get())


# ----------------------------------------------------------------------------- ROS messages

def static_tf_message(rig: frames.CameraRig, stamp_sec: int = 0, stamp_nanosec: int = 0):
    """tf2_msgs/TFMessage with body -> cam_left and body -> cam_right (optical frames, from the mount)."""
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    msg = TFMessage()
    for side in ("left", "right"):
        t, R = rig.optical_pose_in_body(side)
        qx, qy, qz, qw = frames.rot_to_quat(R)
        ts = TransformStamped()
        ts.header.stamp.sec = int(stamp_sec)
        ts.header.stamp.nanosec = int(stamp_nanosec)
        ts.header.frame_id = BODY_FRAME
        ts.child_frame_id = FRAME_IDS[side]
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = (float(c) for c in t)
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = qx, qy, qz, qw
        msg.transforms.append(ts)
    return msg


def camera_info_message(rig: frames.CameraRig, side: str, stamp):
    """sensor_msgs/CameraInfo exactly as in camera.json (K, P, size; D zeros, plumb_bob, R identity)."""
    from sensor_msgs.msg import CameraInfo
    m = CameraInfo()
    m.header.stamp = stamp
    m.header.frame_id = FRAME_IDS[side]
    m.width, m.height = rig.width, rig.height
    m.distortion_model = "plumb_bob"
    m.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    m.k = list(rig.left_K if side == "left" else rig.right_K)
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = list(rig.left_P if side == "left" else rig.right_P)
    return m


class CameraInfoRelay:
    """Publishes camera_info with the stamp of every image the OmniGraph helper publishes.

    The helper's frame gate has an unknown phase, so camera_info is triggered by the image itself
    (a local subscription; only the header is used) and carries the identical stamp.
    """

    def __init__(self, node, rig: frames.CameraRig):
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, Image
        self.rig = rig
        self.count = {"left": 0, "right": 0}
        qos_img = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pubs = {}
        self._subs = []
        for side in ("left", "right"):
            self._pubs[side] = node.create_publisher(CameraInfo, TOPICS[side]["info"], 5)
            self._subs.append(node.create_subscription(
                Image, TOPICS[side]["image"], (lambda msg, s=side: self._on_image(s, msg)), qos_img))

    def _on_image(self, side: str, msg) -> None:
        self.count[side] += 1
        self._pubs[side].publish(camera_info_message(self.rig, side, msg.header.stamp))
