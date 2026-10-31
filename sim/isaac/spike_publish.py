#!/usr/bin/env python
"""M0 bridge spike (Isaac side): prove the Isaac Sim 5.1 <-> ros1_bridge <-> Noetic transport.

Empty stage, one cube ("body") at 5 m, two downward cameras (640x400, 10 Hz), TF
(world -> body dynamic, body -> cam_* static) and /clock, all published through the
Isaac ROS 2 bridge extension (OmniGraph, internal Humble libs). The cube is moved
toward /corridor_nav/setpoint (geometry_msgs/PoseStamped) at a capped speed;
/corridor_nav/spray (std_msgs/Bool) gates a fake deposition counter. /sim/truth and
/sim/deposition (std_msgs/String, JSON) report the cube state.

Run inside the Isaac image (full docker commands: results/sim_bridge_spike.md):

    export ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77
    export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
    /isaac-sim/python.sh /ws/sim/isaac/spike_publish.py --max-wall 300

Time is the sim clock: every message stamp and /clock come from the same simulation
time, so ROS 1 nodes on use_sim_time see consistent stamps.
"""
import argparse
import json
import os
import signal
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["headless", "livestream"], default="headless")
ap.add_argument("--renderer", default="RaytracedLighting")
ap.add_argument("--experience", default="",
                help="Kit experience file (default: SimulationApp's headless default)")
ap.add_argument("--cam-hz", type=float, default=10.0)
ap.add_argument("--physics-hz", type=float, default=60.0)
ap.add_argument("--width", type=int, default=640)
ap.add_argument("--height", type=int, default=400)
ap.add_argument("--vmax", type=float, default=3.0, help="cube speed cap toward the setpoint, m/s")
ap.add_argument("--max-wall", type=float, default=0.0, help="stop after this many wall s (0 = run until SIGTERM)")
ap.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "77")))
ap.add_argument("--livestream-ip", default=os.environ.get("LIVESTREAM_IP", "127.0.0.1"))
args, _unknown = ap.parse_known_args()


def log(msg):
    print(f"[spike_publish] {msg}", flush=True)


# ------------------------------------------------------------------ app bootstrap
from isaacsim import SimulationApp  # noqa: E402

cfg = {"headless": True, "renderer": args.renderer,
       "width": args.width, "height": args.height}
if args.mode == "livestream":
    sys.argv += [f"--/app/livestream/publicEndpointAddress={args.livestream_ip}",
                 "--/app/livestream/port=49100"]
    exp = args.experience or "/isaac-sim/apps/isaacsim.exp.full.streaming.kit"
else:
    exp = args.experience
simulation_app = SimulationApp(cfg, experience=exp) if exp else SimulationApp(cfg)
log(f"SimulationApp up (mode={args.mode} renderer={args.renderer} experience={exp or 'default'})")

import carb  # noqa: E402,F401
import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy  # noqa: E402  (provided by the ros2 bridge extension's internal Humble)
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, Vt  # noqa: E402

# ------------------------------------------------------------------ stage
PHYS_DT = 1.0 / args.physics_hz
world = World(stage_units_in_meters=1.0, physics_dt=PHYS_DT, rendering_dt=PHYS_DT)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# Frame names come from prim names: world, body, cam_left, cam_right.
UsdGeom.Xform.Define(stage, "/world")


def colored_cube(path, size, pos, rgb):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetScale(Gf.Vec3f(*size))
    xf.SetTranslate(Gf.Vec3d(*pos))
    cube.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*rgb)]))
    return cube


colored_cube("/world/ground", (60.0, 60.0, 0.2), (0, 0, -0.1), (0.35, 0.55, 0.25))
for i, (x, y, rgb) in enumerate([(4, 3, (0.8, 0.2, 0.2)), (-5, 2, (0.2, 0.2, 0.8)),
                                  (1, -6, (0.9, 0.8, 0.1)), (-3, -4, (0.6, 0.3, 0.7)),
                                  (8, -1, (0.1, 0.7, 0.7))]):
    colored_cube(f"/world/mark{i}", (1.5, 1.5, 2.0 + i * 0.5), (x, y, 1.0 + i * 0.25), rgb)
light = UsdLux.DistantLight.Define(stage, "/world/sun")
light.GetIntensityAttr().Set(500.0)
UsdLux.DomeLight.Define(stage, "/world/dome").GetIntensityAttr().Set(150.0)

START = (0.0, 0.0, 5.0)
body = UsdGeom.Xform.Define(stage, "/world/body")
body_xf = UsdGeom.XformCommonAPI(body)
body_xf.SetTranslate(Gf.Vec3d(*START))
# The visible vehicle cube sits above the cameras so it never blocks them.
colored_cube("/world/body/vis", (0.4, 0.4, 0.2), (0.0, 0.0, 0.3), (1.0, 1.0, 1.0))

FX = 454.0  # px, matches the real rig; focal length derived from a 20.955 mm aperture
H_AP = 20.955
V_AP = H_AP * args.height / args.width
for name, y in (("cam_left", 0.08), ("cam_right", -0.08)):
    cam = UsdGeom.Camera.Define(stage, f"/world/body/{name}")
    cam.GetHorizontalApertureAttr().Set(H_AP)
    cam.GetVerticalApertureAttr().Set(V_AP)
    cam.GetFocalLengthAttr().Set(FX * H_AP / args.width)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 200.0))
    # identity orientation: a USD camera looks along its local -Z = straight down here
    UsdGeom.XformCommonAPI(cam).SetTranslate(Gf.Vec3d(0.0, y, 0.0))

# ------------------------------------------------------------------ ROS 2 graph
keys = og.Controller.Keys
skip = max(int(round(args.physics_hz / args.cam_hz)) - 1, 0)
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("tf_static", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("rp_left", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("rp_right", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("img_left", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("img_right", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("info_left", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("info_right", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ],
        keys.SET_VALUES: [
            ("ctx.inputs:useDomainIDEnvVar", False),
            ("ctx.inputs:domain_id", args.domain_id),
            ("clock.inputs:topicName", "/clock"),
            ("tf.inputs:topicName", "/tf"),
            ("tf_static.inputs:topicName", "/tf_static"),
            ("tf_static.inputs:staticPublisher", True),
            ("rp_left.inputs:cameraPrim", [Sdf.Path("/world/body/cam_left")]),
            ("rp_right.inputs:cameraPrim", [Sdf.Path("/world/body/cam_right")]),
            ("rp_left.inputs:width", args.width), ("rp_left.inputs:height", args.height),
            ("rp_right.inputs:width", args.width), ("rp_right.inputs:height", args.height),
            ("img_left.inputs:type", "rgb"), ("img_right.inputs:type", "rgb"),
            ("img_left.inputs:topicName", "/cam/bottom/left/half/image_raw"),
            ("img_right.inputs:topicName", "/cam/bottom/right/half/image_raw"),
            ("img_left.inputs:frameId", "cam_left"), ("img_right.inputs:frameId", "cam_right"),
            ("img_left.inputs:frameSkipCount", skip), ("img_right.inputs:frameSkipCount", skip),
            ("info_left.inputs:topicName", "/cam/bottom/left/half/camera_info"),
            ("info_right.inputs:topicName", "/cam/bottom/right/half/camera_info"),
            ("info_left.inputs:frameId", "cam_left"), ("info_right.inputs:frameId", "cam_right"),
            ("info_left.inputs:frameSkipCount", skip), ("info_right.inputs:frameSkipCount", skip),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "clock.inputs:execIn"),
            ("tick.outputs:tick", "tf.inputs:execIn"),
            ("tick.outputs:tick", "tf_static.inputs:execIn"),
            ("tick.outputs:tick", "rp_left.inputs:execIn"),
            ("tick.outputs:tick", "rp_right.inputs:execIn"),
            ("ctx.outputs:context", "clock.inputs:context"),
            ("ctx.outputs:context", "tf.inputs:context"),
            ("ctx.outputs:context", "tf_static.inputs:context"),
            ("ctx.outputs:context", "img_left.inputs:context"),
            ("ctx.outputs:context", "img_right.inputs:context"),
            ("ctx.outputs:context", "info_left.inputs:context"),
            ("ctx.outputs:context", "info_right.inputs:context"),
            ("simtime.outputs:simulationTime", "clock.inputs:timeStamp"),
            ("simtime.outputs:simulationTime", "tf.inputs:timeStamp"),
            ("simtime.outputs:simulationTime", "tf_static.inputs:timeStamp"),
            ("rp_left.outputs:execOut", "img_left.inputs:execIn"),
            ("rp_left.outputs:execOut", "info_left.inputs:execIn"),
            ("rp_right.outputs:execOut", "img_right.inputs:execIn"),
            ("rp_right.outputs:execOut", "info_right.inputs:execIn"),
            ("rp_left.outputs:renderProductPath", "img_left.inputs:renderProductPath"),
            ("rp_left.outputs:renderProductPath", "info_left.inputs:renderProductPath"),
            ("rp_right.outputs:renderProductPath", "img_right.inputs:renderProductPath"),
            ("rp_right.outputs:renderProductPath", "info_right.inputs:renderProductPath"),
        ],
    },
)
# Relationship (target) inputs are set on the USD prim, not through SET_VALUES.
stage.GetPrimAtPath("/ActionGraph/tf").GetRelationship("inputs:parentPrim").SetTargets([Sdf.Path("/world")])
stage.GetPrimAtPath("/ActionGraph/tf").GetRelationship("inputs:targetPrims").SetTargets([Sdf.Path("/world/body")])
stage.GetPrimAtPath("/ActionGraph/tf_static").GetRelationship("inputs:parentPrim").SetTargets([Sdf.Path("/world/body")])
stage.GetPrimAtPath("/ActionGraph/tf_static").GetRelationship("inputs:targetPrims").SetTargets(
    [Sdf.Path("/world/body/cam_left"), Sdf.Path("/world/body/cam_right")])
log(f"OmniGraph built (domain {args.domain_id}, cameras every {skip + 1} ticks of {args.physics_hz:g} Hz)")

# ------------------------------------------------------------------ rclpy side (setpoints, JSON strings)
rclpy.init()
node = rclpy.create_node("tm_spike_isaac")
state = {"sp": list(START), "sp_seq": 0, "spray": False, "volume_l": 0.0, "pos": list(START)}
sp_rx = {"n": 0}


def on_setpoint(msg):
    p = msg.pose.position
    state["sp"] = [float(p.x), float(p.y), float(p.z)]
    state["sp_seq"] += 1
    sp_rx["n"] += 1


def on_spray(msg):
    state["spray"] = bool(msg.data)


node.create_subscription(PoseStamped, "/corridor_nav/setpoint", on_setpoint, 10)
node.create_subscription(Bool, "/corridor_nav/spray", on_spray, 10)
# /tf_static replay. ros1_bridge's ROS 2 -> ROS 1 subscriber is fixed best-effort + volatile and
# its ROS 1 publisher is not latched, so a static transform published once (what the OmniGraph
# static publisher does) never reaches Noetic. Capture the graph's /tf_static (transient-local,
# so it is delivered on subscribe) and republish it at 1 Hz; tf2 tolerates repeated statics.
STATIC_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
static_tf = {}


def on_static(msg):
    for t in msg.transforms:
        static_tf[(t.header.frame_id, t.child_frame_id)] = t


node.create_subscription(TFMessage, "/tf_static", on_static, STATIC_QOS)
pub_static = node.create_publisher(TFMessage, "/tf_static", STATIC_QOS)
pub_truth = node.create_publisher(String, "/sim/truth", 10)
pub_depo = node.create_publisher(String, "/sim/deposition", 10)

# ------------------------------------------------------------------ run
stop = {"flag": False}
signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))

world.reset()
world.play()
SPRAY_LPS = 0.075
TRUTH_EVERY = max(int(round(args.physics_hz / 10.0)), 1)   # 10 Hz
DEPO_EVERY = max(int(round(args.physics_hz / 2.0)), 1)     # 2 Hz
STATIC_EVERY = max(int(round(args.physics_hz)), 1)         # 1 Hz
wall0 = time.time()
last_print = wall0
sim_t = 0.0
step = 0
sim0 = None
while simulation_app.is_running() and not stop["flag"]:
    world.step(render=True)
    step += 1
    sim_t = float(world.current_time)
    if sim0 is None:
        sim0 = sim_t
    rclpy.spin_once(node, timeout_sec=0.0)

    # kinematic motion toward the setpoint, capped speed
    pos, sp = state["pos"], state["sp"]
    d = [sp[i] - pos[i] for i in range(3)]
    dist = sum(c * c for c in d) ** 0.5
    if dist > 1e-6:
        k = min(1.0, args.vmax * PHYS_DT / dist)
        pos = [pos[i] + d[i] * k for i in range(3)]
        state["pos"] = pos
        body_xf.SetTranslate(Gf.Vec3d(*pos))
    if state["spray"]:
        state["volume_l"] += SPRAY_LPS * PHYS_DT

    if step % TRUTH_EVERY == 0:
        m = String()
        m.data = json.dumps({"t": round(sim_t, 4), "pos": [round(c, 4) for c in pos],
                             "setpoint": state["sp"], "sp_seq": state["sp_seq"],
                             "spray": state["spray"]})
        pub_truth.publish(m)
    if step % DEPO_EVERY == 0:
        m = String()
        m.data = json.dumps({"t": round(sim_t, 4), "volume_l": round(state["volume_l"], 5),
                             "pos": [round(c, 4) for c in pos]})
        pub_depo.publish(m)

    if step % STATIC_EVERY == 0 and static_tf:
        m = TFMessage()
        m.transforms = list(static_tf.values())
        pub_static.publish(m)

    now = time.time()
    if now - last_print >= 5.0:
        wall = now - wall0
        log(f"wall={wall:6.1f}s sim={sim_t - sim0:7.2f}s RTF={(sim_t - sim0) / wall:5.2f} "
            f"pos={[round(c, 2) for c in pos]} setpoints_rx={sp_rx['n']} spray={state['spray']}")
        last_print = now
    if args.max_wall and now - wall0 >= args.max_wall:
        break

wall = time.time() - wall0
log(f"done: steps={step} wall={wall:.1f}s sim={sim_t - (sim0 or 0):.2f}s "
    f"RTF={(sim_t - (sim0 or 0)) / max(wall, 1e-9):.3f} setpoints_rx={sp_rx['n']}")
node.destroy_node()
rclpy.shutdown()
simulation_app.close()
