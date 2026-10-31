#!/usr/bin/env python
"""Isaac Sim orchard: seeded scene, kinematic drone, stereo cameras, nozzle deposition, ROS 2 topics.

Run inside the Isaac image (docker commands: results/sim_isaac_checks.md):

    export ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=77
    export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
    /isaac-sim/python.sh /ws/sim/isaac/orchard_sim.py --spec /ws/sim/specs/3x3_smoke.json --out /data/work/sim_runs/x

Publishes  /clock /tf (world->body, 60 Hz) /tf_static (body->cam_left, cam_right; replayed 1 Hz)
           /cam/bottom/{left,right}/half/{image_raw,camera_info} /sim/truth (latched + 1 Hz)
           /sim/deposition (2 Hz)
Subscribes /corridor_nav/setpoint (PoseStamped; header stamp = sim time) /corridor_nav/spray (Bool)
On SIGTERM / SIGINT writes <out>/deposition.json and <out>/rtf.json and closes the app. Any exception while
building the scene is printed and the process exits non-zero.
"""
import argparse
import json
import math
import os
import signal
import sys
import threading
import time
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIM = os.path.dirname(_HERE)
_TOOLS = os.path.join(os.path.dirname(_SIM), "tools")
for _p in (_SIM, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from isaac import frames, layout  # noqa: E402  (pure modules; no Isaac needed)


KIT_PASSTHROUGH_PREFIXES = ("--/",)


def log(msg):
    print("[orchard_sim] %s" % msg, flush=True)


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
                                 allow_abbrev=False)
    ap.add_argument("--spec", required=True, help="scene spec JSON (sim/specs/*.json)")
    ap.add_argument("--out", required=True, help="run directory (truth.json, deposition.json, rtf.json, frames)")
    ap.add_argument("--camera", default=os.path.join(_SIM, "specs", "camera.json"))
    ap.add_argument("--assets-dir", default="", help="tree asset folder (default: spec assets_dir)")
    ap.add_argument("--only-scene", action="store_true", help="build the scene, write truth.json, exit")
    ap.add_argument("--physics-hz", type=float, default=60.0)
    ap.add_argument("--render-hz", type=float, default=10.0, help="camera publish rate")
    ap.add_argument("--v-max-xy", type=float, default=6.0)
    ap.add_argument("--v-max-z", type=float, default=2.0)
    ap.add_argument("--save-frame", type=float, action="append", default=[],
                    help="sim second at which to save the left image (repeatable)")
    ap.add_argument("--save-hover", type=float, default=0.0,
                    help="also save the left image after the drone hovered (z > 1 m) this many sim s at a new pose")
    ap.add_argument("--save-hover-max", type=int, default=3)
    ap.add_argument("--max-rtf", type=float, default=0.0,
                    help="cap the real-time factor by sleeping (0 = run free, as fast as the GPU allows)")
    ap.add_argument("--max-wall", type=float, default=0.0, help="stop after this many wall s (0 = until SIGTERM)")
    ap.add_argument("--max-sim", type=float, default=0.0, help="stop after this many sim s (0 = until SIGTERM)")
    ap.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "77")))
    ap.add_argument("--renderer", default="RaytracedLighting")
    args, unknown = ap.parse_known_args()
    # Only carb settings (--/section/key=value) may pass through to Kit, which reads sys.argv itself; none is
    # needed by the checks, the prefix is the explicit allow-list. Anything else is a typo: fail loud.
    bad = [a for a in unknown if not a.startswith(KIT_PASSTHROUGH_PREFIXES)]
    if bad:
        ap.error("unrecognised arguments: %s" % " ".join(bad))
    return args


def die(code, msg=None):
    if msg:
        print(msg, file=sys.stderr, flush=True)
    sys.stderr.flush()
    sys.stdout.flush()
    os._exit(code)


def main():
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)
    spec = layout.load_spec(args.spec)        # fails loud before the app starts
    rig = frames.load_rig(args.camera)
    assets_dir = args.assets_dir or spec.get("assets_dir", "")
    if not assets_dir:
        raise SystemExit("no tree assets: pass --assets-dir or set assets_dir in the spec")
    dt = 1.0 / args.physics_hz
    clutter = spec.get("clutter") or {}
    if clutter.get("grass") or clutter.get("rocks"):
        log("WARNING: spec clutter %s is requested but not implemented: the ground is a flat-colour mesh, no grass or rocks"
            % json.dumps(clutter))

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": args.renderer, "width": rig.width, "height": rig.height})
    log("SimulationApp up")
    try:
        code = run(app, args, spec, rig, assets_dir, dt)
    except BaseException:
        traceback.print_exc()
        # Fail loud: leave at once with a non-zero status. SimulationApp.close() exits the process with status 0
        # (and can hang on a half-built stage), so it is not used on this path.
        die(1, "orchard_sim: FAILED (see traceback above)")
    threading.Timer(60.0, lambda: die(code)).start()   # watchdog for a hanging shutdown
    app.close()
    die(code)


def run(app, args, spec, rig, assets_dir, dt):
    import omni.timeline
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    from isaac import scene, truth
    from isaac.controller import KinematicController

    n_sub = int(round(args.physics_hz / args.render_hz))
    if n_sub < 1 or abs(n_sub * args.render_hz - args.physics_hz) > 1e-6:
        raise ValueError("--physics-hz (%g) must be an integer multiple of --render-hz (%g)"
                         % (args.physics_hz, args.render_hz))
    # One app update = one rendered frame = n_sub physics steps; the controller, TF, /clock and the nozzle run
    # in a physics callback at the physics rate, the cameras render (and publish) once per update.
    world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=n_sub * dt)
    stage = omni.usd.get_context().get_stage()

    trees = scene.build_scene(stage, spec, assets_dir)
    truth_path = os.path.join(args.out, "truth.json")
    truth.write_truth(truth_path, trees, frame="world")
    log("scene built: %d trees, top_z %.2f..%.2f m; truth -> %s"
        % (len(trees), min(t["top_z"] for t in trees), max(t["top_z"] for t in trees), truth_path))
    if args.only_scene:
        return 0

    from isaac import cameras, drone as drone_mod, nozzle as nozzle_mod

    enable_extension("isaacsim.ros2.bridge")
    app.update()
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage

    start = spec["drone"]["start"]
    ctrl = KinematicController(v_max_xy=args.v_max_xy, v_max_z=args.v_max_z)
    drone = drone_mod.Drone(stage, ctrl, start, 0.0)
    cam_paths = cameras.define_cameras(stage, rig)
    cameras.build_ros_graph(cam_paths, rig, args.domain_id)   # image publishers start disabled (no pre-clock frames)

    saver = None
    if args.save_frame or args.save_hover > 0.0:
        saver = FrameSaver(args.out, cam_paths["left"], rig, sorted(args.save_frame), args.save_hover,
                           args.save_hover_max)

    rclpy.init()
    node = rclpy.create_node("tm_orchard_sim")
    latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_clock = node.create_publisher(Clock, "/clock", 100)
    pub_tf = node.create_publisher(TFMessage, "/tf", 100)
    pub_static = node.create_publisher(TFMessage, "/tf_static", latched)
    pub_truth = node.create_publisher(String, "/sim/truth", latched)
    pub_depo = node.create_publisher(String, "/sim/deposition", 10)
    relay = cameras.CameraInfoRelay(node, rig)

    inbox = {"sp": None, "setpoints": 0, "spray_cmds": 0}
    sim = {"t": 0.0, "step": 0, "err": None, "warm": True, "offset": None}
    nozzle_box = {}

    def on_setpoint(msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        inbox["sp"] = (float(p.x), float(p.y), float(p.z), yaw, stamp)   # non-finite values: controller ignores
        inbox["setpoints"] += 1

    def on_spray(msg):
        inbox["spray_cmds"] += 1
        if "n" in nozzle_box:
            nozzle_box["n"].on_command(bool(msg.data), sim["t"])

    node.create_subscription(PoseStamped, "/corridor_nav/setpoint", on_setpoint, 10)
    node.create_subscription(Bool, "/corridor_nav/spray", on_spray, 10)

    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))

    world.reset()
    # PhysX has the colliders after reset; the caster needs the scene-query interface.
    nozzle = nozzle_mod.NozzleCaster(spec["nozzle"])
    nozzle_box["n"] = nozzle

    truth_msg = String()
    with open(truth_path) as f:
        truth_msg.data = json.dumps(json.load(f))
    static_msg = cameras.static_tf_message(rig)
    every_1hz = max(int(round(args.physics_hz)), 1)
    every_depo = max(int(round(args.physics_hz / 2.0)), 1)

    timeline = omni.timeline.get_timeline_interface()

    def on_physics_step(step_size):
        """Runs once per physics step: ROS inbox, controller, pose, /clock, /tf, nozzle, 1 Hz and 2 Hz topics."""
        if sim["err"] is not None:
            return
        try:
            tl = timeline.get_current_time()
            if sim["offset"] is None:
                return                           # warm-up updates: nothing is published, the clock has not started
            if sim["warm"]:
                # first step of the first clocked update; the update's last step is at (timeline + offset)
                start = tl + sim["offset"] - (n_sub - 1) * dt
                if start < -1e-9:
                    raise RuntimeError("sim clock would start negative (%.5f s, timeline %.5f, offset %.5f)"
                                       % (start, tl, sim["offset"]))
                sim["t"] = start - dt
                sim["warm"] = False
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0.0)
            sp, inbox["sp"] = inbox["sp"], None
            sim["t"] += dt                       # time at the end of this step; the camera frame of this update ends here too
            sim["step"] += 1

            t = sim["t"]
            state = drone.step(dt, t, sp)
            sec, nsec = frames.split_stamp(t)
            clk = Clock()
            clk.clock.sec, clk.clock.nanosec = sec, nsec
            pub_clock.publish(clk)
            tf = TFMessage()
            tf.transforms.append(drone_mod.world_to_body_transform(state, sec, nsec))
            pub_tf.publish(tf)
            nozzle.step(t, dt, (state.x, state.y, state.z), drone.rotation())
            if sim["step"] % every_1hz == 1 or every_1hz == 1:
                for ts in static_msg.transforms:
                    ts.header.stamp.sec, ts.header.stamp.nanosec = sec, nsec
                pub_static.publish(static_msg)
                pub_truth.publish(truth_msg)
            if sim["step"] % every_depo == 0:
                d = String()
                d.data = json.dumps(nozzle.deposition.to_json())
                pub_depo.publish(d)
        except BaseException as e:   # a callback exception would only be logged by Kit: stop and raise it in the loop
            sim["err"] = e
            traceback.print_exc()

    world.add_physics_callback("tm_orchard_step", on_physics_step)
    world.play()

    wall0 = time.time()
    last_log = wall0
    log("running: physics %.0f Hz, cameras %.1f Hz (render every %d steps), v_max %.1f/%.1f m/s, start %s"
        % (args.physics_hz, args.render_hz, n_sub, args.v_max_xy, args.v_max_z, start))
    max_skew = 0.0
    n_updates = 0
    while app.is_running() and not stop["flag"]:
        world.step(render=True)
        if sim["err"] is not None:
            raise sim["err"]
        n_updates += 1
        if sim["offset"] is None:
            # Warm-up: the first update runs 5 of 6 steps. Measure how the graph time (image stamps) relates to the
            # timeline on the second one; from the third update on /clock, /tf and the nozzle run on graph time.
            if n_updates == 2:
                sim["offset"] = cameras.graph_sim_time() - timeline.get_current_time()
                log("clock starts on graph time: timeline %.5f s, graph %.5f s, offset %+.5f s"
                    % (timeline.get_current_time(), cameras.graph_sim_time(), sim["offset"]))
                # The clock starts in the next update: only now may images go out. A frame rendered during warm-up
                # would carry a stamp before the first /clock and /tf.
                cameras.set_image_publishers_enabled(True)
            continue
        t = sim["t"]
        g = cameras.graph_sim_time()
        skew = g - t
        max_skew = max(max_skew, abs(skew)) if math.isfinite(skew) else float("nan")
        if not frames.clock_skew_ok(g, t, dt):
            # the image stamps (graph time) and our /clock, /tf must stay on one time base: stop rather than drift
            raise RuntimeError("sim clock left the image time base: graph %.5f s, ours %.5f s, skew %.5f s "
                               "(limit %.5f s)" % (g, t, skew, 0.5 * dt))
        if saver is not None:
            saver.poll(t, drone.state)
        now = time.time()
        if now - last_log >= 10.0:
            wall = now - wall0
            st = drone.state
            log("wall=%6.1fs sim=%7.2fs RTF=%5.2f pos=(%.2f,%.2f,%.2f) yaw=%.0fdeg setpoints_rx=%d spray_rx=%d "
                "open_t=%.1fs info=%d/%d graph_skew=%.4fs"
                % (wall, t, t / wall, st.x, st.y, st.z, math.degrees(st.yaw), inbox["setpoints"],
                   inbox["spray_cmds"], nozzle.deposition.open_time, relay.count["left"], relay.count["right"],
                   max_skew))
            last_log = now
        if args.max_rtf > 0.0:
            ahead = t / args.max_rtf - (time.time() - wall0)
            if ahead > 0.0:
                time.sleep(ahead)
        if args.max_wall and now - wall0 >= args.max_wall:
            break
        if args.max_sim and t >= args.max_sim:
            break

    wall = time.time() - wall0
    sim_s = sim["t"]
    with open(os.path.join(args.out, "deposition.json"), "w") as f:
        json.dump(nozzle.deposition.to_json(), f, indent=1)
    with open(os.path.join(args.out, "rtf.json"), "w") as f:
        json.dump({"sim_s": sim_s, "wall_s": wall, "rtf": sim_s / wall if wall > 0 else 0.0}, f)
    log("done: steps=%d wall=%.1fs sim=%.2fs RTF=%.3f graph_skew_max=%.4fs deposition=%s unknown_hits=%d"
        % (sim["step"], wall, sim_s, sim_s / max(wall, 1e-9), max_skew, json.dumps(nozzle.deposition.to_json()),
           nozzle.unknown_hits))
    node.destroy_node()
    rclpy.shutdown()
    return 0


class FrameSaver:
    """Saves the left camera image (PNG) and the pose it was taken at.

    Triggers: sim time passing each of ``times`` (``--save-frame``), and/or the drone having hovered above
    1 m for ``hover_s`` seconds at a pose not yet saved (``--save-hover``, at most ``hover_max`` frames), so a
    scripted flight does not need to know the sim clock in advance.
    """

    def __init__(self, out_dir, cam_path, rig, times, hover_s=0.0, hover_max=3):
        import omni.replicator.core as rep
        self.out_dir = out_dir
        self.times = list(times)
        self.rig = rig
        self.done = 0
        self.hover_s = hover_s
        self.hover_max = hover_max
        self.hover_saved = []      # poses of saved hover frames
        self._last = None
        self._still_since = None
        # a dedicated render product for the saved frames (it renders every frame once it exists; flag-only cost)
        self.rp = rep.create.render_product(cam_path, (rig.width, rig.height))
        self.annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotator.attach([self.rp])

    def _grab(self):
        import numpy as np
        data = np.asarray(self.annotator.get_data())
        return data if data.ndim == 3 else None   # the first frames of a render product are empty

    def _save(self, tag, want, t, state, data):
        import numpy as np
        from PIL import Image
        png = os.path.join(self.out_dir, "frame_%s.png" % tag)
        Image.fromarray(data[..., :3].astype(np.uint8)).save(png)
        with open(os.path.join(self.out_dir, "frame_%s.json" % tag), "w") as f:
            json.dump({"tag": tag, "requested": want, "sim_s": t, "x": state.x, "y": state.y, "z": state.z,
                       "roll": state.roll, "pitch": state.pitch, "yaw": state.yaw,
                       "width": self.rig.width, "height": self.rig.height}, f, indent=1)
        log("saved %s at sim %.3f s pose (%.3f, %.3f, %.3f) yaw %.1f deg"
            % (png, t, state.x, state.y, state.z, math.degrees(state.yaw)))

    def poll(self, t, state):
        if self.done < len(self.times) and t >= self.times[self.done]:
            data = self._grab()
            if data is not None:
                want = self.times[self.done]
                self.done += 1
                self._save(("t%g" % want).replace(".", "p"), want, t, state, data)
        if self.hover_s > 0.0 and len(self.hover_saved) < self.hover_max:
            pose = (state.x, state.y, state.z, state.yaw, state.roll, state.pitch)
            moved = (self._last is None or max(abs(a - b) for a, b in zip(pose[:4], self._last[:4])) > 1e-4
                     or max(abs(pose[4]), abs(pose[5])) > 2e-3)
            self._last = pose
            if moved:
                self._still_since = None
                return
            if self._still_since is None:
                self._still_since = t
            fresh = all(math.dist(pose[:3], q[:3]) > 0.5 or abs(frames_wrap(pose[3] - q[3])) > 0.2
                        for q in self.hover_saved)
            if state.z > 1.0 and t - self._still_since >= self.hover_s and fresh:
                data = self._grab()
                if data is not None:
                    self.hover_saved.append(pose)
                    self._save("hover%d" % len(self.hover_saved), "hover", t, state, data)


def frames_wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


if __name__ == "__main__":
    main()
