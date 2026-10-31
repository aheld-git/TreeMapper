# M0 bridge spike: Isaac Sim 5.1 <-> ros1_bridge <-> Noetic

Decision: **GO.** The Isaac-only closed loop transport works with the stock Humble `ros1_bridge`
`parameter_bridge`; every listed type crosses in both directions, no relay fallback needed. Isaac
runs plain `headless=True` in the default Kit experience (no crash), at RTF 0.95-0.98 with two
640x400 cameras on a shared RTX 3070. Two bridge behaviours need handling by the rest of the
plan: `/tf_static` must be re-published (see Findings) and images are best-effort (1-4 % lost).

Raw logs: `/data/work/logs/sim_bridge_*` (`sim_bridge_spike_check_final.json` is the run below; `/data/work` is `/media/alex/waspsim/cloud_build_bundle/work` on the host; the table is the fix-round-1 re-run, the first run had the same pass/fail).
Images grabbed on the Noetic side: `sim_bridge_spike_{left,right}.png` (real renders, mean 149, std 24).

## Setup that was measured

| Item | Value |
|---|---|
| Base image of `tm_sim_ros` | `treehoppingnavigation-bridge:latest`, id `sha256:4d87252ea3a4fc8cd225fccc026b1026ba7e02bd9c49416a5de15a5943ed97f4` (Ubuntu 20.04, Noetic, ROS 2 **Humble** built from source at `/ros2_humble/install`, `ros1_bridge` built) |
| `tm_sim_ros:latest` | id `sha256:67388516f070d77a502c937ceb5f912c09daf84003129aeaad1c54c0cda540dd` (rebuilt in fix round 1; the e2e re-run used the previous build, which differs only in YAML comments), 32.2 GB (base 27.7 GB + ~4.5 GB of onnxruntime/CUDA 12/cuDNN 9 wheels); build log `/data/work/logs/sim_bridge_build.log` |
| Isaac image | `treehoppingnavigation-treehoppingsim:latest` (Isaac Sim 5.1.0), id `sha256:b51d0a47110da609390e1483e5970af6a63904bd7a526d3bf8f084f83db1832b` |
| Isaac ROS 2 | `isaacsim.ros2.bridge` extension, internal **Humble** libs, `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=77` |
| Noetic side | `rmw_fastrtps_cpp` for the bridge process (same RMW as Isaac), `ROS_DOMAIN_ID=77`, `/use_sim_time true` set before any ROS 1 node starts |
| Network | user bridge network `tm_sim_net` |
| Scene | empty stage, one cube ("body", moved at <= 3 m/s toward the setpoint), 2 cameras 640x400 at 10 Hz (physics/render 60 Hz, camera helpers `frameSkipCount=5`), TF, `/clock` |

No Foxy anywhere (the brief's Foxy image was replaced by the Humble base per the controller decision).

### Launch mode that works

`SimulationApp({"headless": True, "renderer": "RaytracedLighting", "width": 640, "height": 400})`
with the default Kit experience (`isaacsim.exp.base.python.kit`) via `/isaac-sim/python.sh`,
`ROS_DISTRO=humble`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
`LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib`. Cameras render and publish;
app ready after ~12-17 s, graph publishing ~25 s after container start (warm and cold shader cache
alike). The earlier headless crash noted in `TreeHoppingNavigation/scripts/auto_sim.py` involved
`enable_cameras` plus the **jazzy** lib path with `isaacsim.exp.full.kit`; with the Humble libs and
the default experience it did not occur. Livestream mode was not needed and was not tried.

### DDS

Default FastDDS multicast discovery works across the two containers on `tm_sim_net`; no unicast peers,
no XML profile, no `FASTRTPS_DEFAULT_PROFILES_FILE` were needed. (Both containers must keep the same
`ROS_DOMAIN_ID` and RMW; set both explicitly.) If discovery ever fails, the fallback is a FastDDS
profile with `initialPeersList` of `tm_isaac` / `tm_sim_ros` and `useBuiltinTransports=false` UDPv4,
like `TreeHoppingNavigation/IsaacSim-ros_workspaces/humble_ws/fastdds.xml`.

## Measurements (`spike_check.py`, 60.2 sim s window, all checks pass)

| Topic | Rate / sim s | Rate / wall s | Stamp lag (arrival `/clock` - header stamp), mean / max | Max gap |
|---|---|---|---|---|
| `/cam/bottom/left/half/image_raw` | 9.77 | 9.32 | 0.067 / 0.067 s | 0.22 s |
| `/cam/bottom/right/half/image_raw` | 9.73 | 9.29 | 0.067 / 0.067 s | 0.20 s |
| `.../left/half/camera_info` | 10.00 | 9.54 | 0.050 / 0.050 s | 0.10 s |
| `.../right/half/camera_info` | 10.00 | 9.54 | 0.050 / 0.050 s | 0.10 s |
| `/tf` (world->body) | 60.0 | 57.2 | -0.017 s (TF stamp is one tick ahead of the `/clock` seen on arrival; harmless) | 0.033 s |
| `/tf_static` (body->cam_left, body->cam_right) | 1.0 (replay) | 0.95 | - | 1.0 s |
| `/clock` | 60.0 | 57.2 | - | 0.017 s |
| `/sim/truth` | 10.0 | 9.54 | - | 0.10 s |
| `/sim/deposition` | 2.0 | 1.91 | - | 0.50 s |

`/clock`: 3,610 messages fall in the 60 s window; the monotonicity check covered all 4,545 received over the node's life, with 0 backward steps.

Across four full runs the image rates were 9.62-9.88 (left) and
9.73-9.93 (right) per sim s: 1-4 % of frames are lost, never more than 2-3 consecutive.
Camera info is 640x400, fx=fy=454.0, cx=320, cy=200, `plumb_bob`; images are `rgb8`, 768,000 bytes.
Lag is two to four 60 Hz ticks, i.e. Isaac's own render-to-publish latency; the bridge adds < 1 tick.

- **RTF** (sim s per wall s): 0.954 during the check (0.95-0.98 across runs; 0.97 averaged over a 210 s run). GPU 2.5 GB used, 100 % utilised while other Isaac containers were idle. `parameter_bridge` used ~7 % of one core.
- **tf2 lookups** (Noetic `tf2_ros.Buffer`): `world->body`, `world->cam_left`, `world->cam_right`, `cam_left->cam_right` 126/126 ok, 0 failures. A fresh node waited 0.7-0.9 s for its first `/tf_static`.
- **Setpoint round trip** (publish on `/corridor_nav/setpoint` at `/clock` time T, watch `/tf` world->body). The transport criterion (check `setpoint_onset<1.0s`) is the **onset**: T to the first TF sample >= 5 cm from the start. The **arrival** time (<= 10 cm of goal) is info only: it is distance / the spike cube's 3 m/s speed cap, not a property of the bridge.

  | Setpoint | echo in `/sim/truth` | TF onset (checked, < 1 s) | TF arrival (info only) |
  |---|---|---|---|
  | 3.0 m in +x | 0.00 s | 0.017 s | 0.97 s |
  | 3.0 m in -y | 0.083 s | 0.033 s | 0.98 s |
  | 4.2 m back to start | 0.067 s | 0.017 s | 1.38 s |

  Transport latency is 0-5 sim ticks (< 0.09 s). The ROS 1 side's notion of "now" is the last `/clock` it received, so all of these are accurate to about one tick (hence the 0.00 echo).
- **Spray**: `/corridor_nav/spray` true for ~5 sim s then false raised `/sim/deposition` `volume_l` by 0.366 L (0.075 L/s, as modelled).

Check verdict (`sim_bridge_spike_check_final.json`): both images >= 9 Hz, camera_info present, `/clock` monotonic, tf2 lookups, `setpoint_onset<1.0s`, truth/deposition received, spray deposition: all true. `spike_check.py` also exits non-zero (with a message) if `/clock` stalls for 5 s wall (30 s while waiting for the first images and `/tf_static`), or if the images or `/tf_static` do not arrive, or a measurement loop overruns its wall deadline.

## Bridge type coverage (Humble <-> Humble, no relay)

`bridge_topics.yaml` loads onto the ROS 1 parameter server and `ros2 run ros1_bridge parameter_bridge`
creates all 11 bridges: `sensor_msgs/Image` x2, `sensor_msgs/CameraInfo` x2, `tf2_msgs/TFMessage` x2,
`rosgraph_msgs/Clock`, `std_msgs/String` x2 (Isaac -> Noetic), `geometry_msgs/PoseStamped`,
`std_msgs/Bool` (Noetic -> Isaac). `parameter_bridge` bridges each topic in both directions; each
topic has exactly one real publisher so nothing loops (`/clock` stays at 60.0 Hz). No Humble/Foxy
interop problem exists (no Foxy), so `sim/bridge/relay.py` was **not** written.

## Findings the rest of the plan must respect

1. **`/tf_static` is lost unless re-published.** This `ros1_bridge` build subscribes ROS 2 -> ROS 1 with
   a fixed best-effort + volatile QoS and creates its ROS 1 publisher unlatched; the `qos:` block in
   `bridge_topics.yaml` only shapes the ROS 2 *publisher* (Noetic -> Isaac direction) and has no effect
   ROS 2 -> ROS 1. A static transform published once by Isaac (OmniGraph `staticPublisher`) never reaches Noetic
   (first run: 0 frames, tf2 lookups failed). `spike_publish.py` therefore captures the graph's
   transient-local `/tf_static` and re-publishes it at 1 Hz; with that, late joiners get it within ~1 s.
   Task 16's Isaac scene script must keep this replay (or publish the static frames on `/tf`), and
   Noetic consumers must not assume `/tf_static` is latched.
2. **Images are best-effort** (the bridge uses SensorDataQoS): 1-4 % of 10 Hz frames dropped, gaps
   up to 0.3 s. Perception must tolerate that (it is stamp-driven, so it should).
3. **`rosgraph_msgs/Clock` and everything else Noetic-side needs `/use_sim_time true` before node
   start**: the entrypoint sets it right after roscore comes up.
4. Isaac runs as uid 1234 and cannot write to `/data/work` (host `/media/alex/waspsim/cloud_build_bundle/work`, owned by uid 1000), so Isaac logs are captured with
   `docker logs -f` on the host instead of a redirect in the container (used for every Isaac log here).
5. **Never source ROS 2 in the shell that runs Noetic nodes**: `rospy` then imports Humble's
   `rosgraph_msgs` and fails. The entrypoint sources Noetic only for `exec`, and Noetic then Humble only for `parameter_bridge`.

## Detector model and onnxruntime check (in `tm_sim_ros`, RTX 3070, driver 580.178.04)

- `onnxruntime-gpu==1.19.2` (last release with a cp38 wheel; CUDA 12.x + cuDNN 9) plus pip
  `nvidia-*-cu12` wheels (cuda-runtime/nvrtc 12.4.127, cublas 12.4.5.8, cufft 11.2.1.3,
  curand 10.3.5.147, nvjitlink 12.4.127, cudnn 9.1.0.70) made visible with `ldconfig`
  (`/etc/ld.so.conf.d/tm-cuda-pip.conf`). The base ships no CUDA toolkit. `docker run --gpus all` with
  `NVIDIA_DRIVER_CAPABILITIES=compute,utility`.
- Available providers: `TensorrtExecutionProvider` (listed, TensorRT libs not installed, unused),
  `CUDAExecutionProvider`, `CPUExecutionProvider`. Session created with
  `["CUDAExecutionProvider", "CPUExecutionProvider"]` -> **providers actually used:
  `['CUDAExecutionProvider', 'CPUExecutionProvider']`**, no fallback warnings.
- Model `/data/work/models/yolox_s_isaac/yolox_s_isaac_epoch190_416_static.onnx`:
  input `images` `[1, 3, 416, 640]` float32; output `output` `[1, 5460, 6]` float32
  (5460 = 52x80 + 26x40 + 13x20 anchors). Mean 5.6 ms per inference (batch 1, 50 runs after warm-up; 5.0 ms in the fix-round re-run). The check is committed as `sim/bridge/onnx_smoke.py <model>`: it prints the I/O and providers in use, times 50 runs and exits non-zero if CUDA is not among the providers actually used (log: `/data/work/logs/sim_bridge_onnx_smoke.log`).
- Already in the base and not re-installed: python3-opencv 4.2.0, numpy 1.24.4, PyYAML 5.3.1,
  libyaml-cpp-dev 0.6.2, Noetic cv_bridge 1.16.2, tf2_ros 0.7.10, tf2_geometry_msgs, image_transport, rosbag. The Dockerfile only installs them if `dpkg -s` finds one missing.

## Exact commands

```bash
docker network create tm_sim_net                       # once
docker build -t tm_sim_ros:latest sim/bridge           # ~95 s with warm layers; log to /data/work/logs

# Noetic side: roscore + parameter_bridge (the entrypoint arg "bridge")
docker run -d --rm --name tm_sim_ros --network tm_sim_net --gpus all \
  -e ROS_DOMAIN_ID=77 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /home/alex/SUIND/TreeMapper:/ws:ro \
  -v /media/alex/waspsim/cloud_build_bundle/work:/data/work \
  tm_sim_ros:latest bridge

# Isaac side (cache mounts keep shader/kit caches off the shared project caches)
mkdir -p ~/.cache/tm_isaac/{main,compute,ovdata}; chmod -R 777 ~/.cache/tm_isaac
docker run -d --rm --name tm_isaac --gpus all --network tm_sim_net --entrypoint bash \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e ROS_DOMAIN_ID=77 -e ROS_DISTRO=humble \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib \
  -v /home/alex/SUIND/TreeMapper:/ws:ro \
  -v /media/alex/waspsim/cloud_build_bundle/work:/data/work \
  -v ~/.cache/tm_isaac/main:/isaac-sim/.cache \
  -v ~/.cache/tm_isaac/compute:/isaac-sim/.nv/ComputeCache \
  -v ~/.cache/tm_isaac/ovdata:/isaac-sim/.local/share/ov/data \
  treehoppingnavigation-treehoppingsim:latest \
  -lc 'cd /isaac-sim && ./python.sh /ws/sim/isaac/spike_publish.py --max-wall 600'
docker logs -f tm_isaac > /media/alex/waspsim/cloud_build_bundle/work/logs/sim_bridge_isaac.log 2>&1 &

# Where files go: /data/work = /media/alex/waspsim/cloud_build_bundle/work on the host. The Isaac
# container runs as uid 1234 and cannot write there, so its log is captured on the host with
# `docker logs -f` (above), not by a redirect inside the container.

# Check (ROS 1 only is sourced by the entrypoint for any command)
docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/bridge/spike_check.py \
  --duration 60 --out /data/work/logs/sim_bridge_spike_check.json   # tm_sim_ros can write there (root)

# Model smoke test
docker run --rm --gpus all -v /media/alex/waspsim/cloud_build_bundle/work:/data/work:ro -v "$PWD":/ws:ro \
  tm_sim_ros:latest python3 /ws/sim/bridge/onnx_smoke.py \
  /data/work/models/yolox_s_isaac/yolox_s_isaac_epoch190_416_static.onnx

docker stop tm_isaac tm_sim_ros                        # --rm removes them
```
