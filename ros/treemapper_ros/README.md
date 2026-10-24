# treemapper_ros

ROS 1 (Noetic) wrapper of the tree mapper. One node, `treemapper_node`, feeds a mapping system from the core
library (`core/`, ROS-free C++17) and serves its map to navigation. The node only talks to the abstract
`treemapper::Mapper` interface and the registry (`core/include/treemapper/mapper.hpp`, `mapper_registry.hpp`):
the system is chosen by name (`~mapper`, default `cand_obj`), so a different system is a parameter change.

```
ros/treemapper_ros/
  CMakeLists.txt, package.xml      catkin package; builds the core in-tree (add_subdirectory, see "Build")
  config/treemapper.yaml           node parameters (topics, frames, cadence) + POINTER to the mapper config
  config/core -> ../../../config   symlink to the repository's config/ (config/<mapper>.cfg = single source)
  include/treemapper_ros/          input_sync.hpp (ROS-free: disparity sampling, calibration, pairing,
                                   DetectionResult decoding), node_support.hpp (cfg reader, phase classes,
                                   per-tree diagnostics), conversions.hpp (msgs <-> core types, markers)
  src/treemapper_node.cpp          the node (thin: subscribe, convert, Mapper::process, publish)
  launch/treemapper_sim.launch     Isaac conventions (world/body, camera mount from params, sim clock)
  launch/treemapper_real.launch    real flight / drone (world_nwu/body, mount from TF; all names are args)
  launch/replay_isaac.launch       plays the recorded Isaac bag with --clock into the node (+ nav emulator, + record)
  scripts/nav_emulator.py          replay-only stand-in for navigation (hop target in our IDs, marks sprayed)
  scripts/treemapper_bag_to_jsonl.py  recorded ~map topic -> the scorer's map-stream JSONL (+ targets.json)
  scripts/compare_map_streams.py   two map streams of one mission: final trees/IDs matched + both scorecards
  scripts/ros_replay_check.sh      end-to-end: roscore + replay + live probes + convert + offline replay + compare
  test/test_*.cpp                  gtest (28 cases): pairing, disparity, calibration, decoding, conversions, config
  test/replay_smoke.test           rostest: 60 s of the Isaac bag at rate 1.0, test/check_live.py asserts
  rviz/treemapper.rviz             top-down view of ~markers
```

## Topics and services

Node name `treemapper`; outputs are in its private namespace (`/treemapper/...`).

| | name (parameter) | type | notes |
|---|---|---|---|
| in | `/disparity_image_down_float` (`disparity_topic`) | sensor_msgs/Image 32FC1 | disparity in px of the calibrated (left) image. Malformed images (encoding, step != 4*width, size) are dropped and counted; a second valid copy of a stamp is a duplicate (counted); resolution must equal the calibration's. |
| in | `/cam/bottom/{left,right}/half/camera_info` (`left/right_camera_info_topic`) | sensor_msgs/CameraInfo | latched once, validated first (below) |
| in | `/object_detection/detections` (`detections_topic`) | treemapper_msgs/Detection2DArray **or** spot_spraying_msgs/DetectionResult | subscribed as `topic_tools/ShapeShifter`; the existing detector's message is decoded from its bytes (md5 checked), so spot_spraying_msgs is not a dependency. Other types: dropped, counted, WARN. |
| in | TF `world_frame <- body_frame` (`pose_source: tf`) or `/treemapper/drone_pose` (`pose_source: pose_topic`) | tf2 / geometry_msgs/PoseStamped | pose at the frame stamp (interpolated by tf2); z = world z |
| in | `/mission_state` (`mission_state_topic`) | std_msgs/String | IDLE, SCAN_APPROACH, SCAN, PLANNING, HOPPING, SPRAYING, RETURN, ERROR |
| in | `/spray_target` (`hop_target_topic`) | std_msgs/Int32 | hop target **in this mapper's ID space**, -1 = none |
| out | `~map` | treemapper_msgs/TreeMap (latched) | every `map_rate` s, at each phase change (stamped with the last frame of the old phase) and after every sprayed change. `header.stamp` = snapshot time (ROS clock), `frame_id` = `world_frame` |
| out | `~markers` | visualization_msgs/MarkerArray | outline (at canopy height), centre, label "id height"; blue = scan, orange = locked, red = hop target, green = sprayed; starts with DELETEALL |
| out | `~health` | treemapper_msgs/Health | `health_rate` Hz, always (also while asleep): AWAKE / SLEEPING ("phase X: sleeping, not dead") / HOLDING (waiting for calibration, no pose) / FAULT (calibration rejected); phase + target in force, trees published, input age, active warnings |
| out | `~counters` | treemapper_msgs/DropCounters | 1 Hz, two messages: stage `input` (node + pairing counters) and stage `mapper` (`Mapper::counters()`), cumulative |
| out | `~stage_timing` | treemapper_msgs/StageTiming | mean ms per frame since the previous map snapshot: the mapper's stages + `node_frame_total`, `node_disparity_sample` |
| srv | `~get_trees` | treemapper_msgs/GetTrees | `id = -1`: all trees; else that tree (`found = false` if unknown) |
| srv | `~set_tree_sprayed` | treemapper_msgs/SetTreeSprayed | `sprayed = true` marks the tree (false if unknown). `sprayed = false` is refused: sprayed trees are protected (rule 5) |

Input counters (stage `input`): `disparity_msgs`, `disparity_rejected_malformed`, `disparity_rejected_resolution`,
`disparity_dropped_no_calibration`, `frames_dropped_no_calibration`, `detections_rejected_type`,
`detections_rejected_malformed`, `camera_info_rejected`, `pose_unavailable`, `heartbeat_frames`, and the pairing's
`sync_detection_msgs`, `sync_disparity_msgs_valid`, `sync_disparity_duplicate`, `sync_frames_paired`,
`sync_frames_paired_late`, `sync_frames_paired_nearest`, `sync_frames_without_disparity`.

`TreeEntry` diagnostics: `locked` (the tree was published while the list was fixed: PLANNING / HOPPING / SPRAYING),
`first_published`, `last_updated` (outline, centre, height or sprayed changed) are kept by the node;
`confidence` and `observations` are not part of the Mapper interface and are published as 0 (= not provided).

## Behaviour on missing information (brief section 7.6)

| situation | choice | what you see |
|---|---|---|
| no camera_info yet | fail closed: no mapper, every frame dropped | health HOLDING "waiting for calibration (...)", `frames_dropped_no_calibration` |
| left P all zero / non-positive focal length / no plausible baseline | fail closed, retried on the next pair | ERROR log, health FAULT, `camera_info_rejected` |
| baseline convention | right `P[3]` accepted as -fx*B metres (REP-104), -fx*B millimetres (field driver, Isaac recording) or +fx*B millimetres, only within 0.01..1 m | the chosen form is logged at latch |
| camera mount TF missing (`camera_mount_source: tf`) | fail closed, retried | WARN, health HOLDING |
| pose not available at the frame stamp | fail closed: the frame reaches the mapper with `pose.valid = false`, which holds processing | `pose_unavailable`, WARN (throttled), health HOLDING |
| no `/mission_state` at all (log replay without navigation) | fail open: phase UNKNOWN = scanning behaviour | health warning "no mission_state received: permissive" |
| unknown phase string | fail open: UNKNOWN, WARN once per string | health warning |
| no hop target | -1 (none); what the mapper freezes then is the mapper's rule | |
| bad mapper config (unknown key, bad value, unknown mapper) | fail at startup (the node exits) | FATAL with the key |
| detection without its disparity | after `sync_wait` the nearest disparity within `sync_nearest_tol`, else a frame without depth | `sync_frames_paired_nearest`, `sync_frames_without_disparity` |
| SPRAYING / IDLE / SCAN_APPROACH / RETURN | frames still go to the mapper (it applies its phase policy, the detector is dark anyway) | health SLEEPING "sleeping, not dead" |

## Parameters

All in `config/treemapper.yaml` (commented there); the launch files override the mode-specific ones.

- **Mapper**: `mapper` (registered name), `mapper_config_file` (relative paths resolve against this package;
  default `config/core/cand_obj.cfg` = `<repo>/config/cand_obj.cfg`), `mapper_overrides` (dict, default empty, each
  override logged as WARN). The mapper's own tunables live only in the `.cfg` (see its header and the design doc);
  the node never repeats them. `test_config_agreement` fails if the yaml stops pointing at `config/<mapper>.cfg`, if
  overrides are set, if a launch file sets mapper keys, or if the mapper rejects the file.
- **Frames / pose**: `world_frame`, `body_frame`, `pose_source` (tf | pose_topic), `pose_topic`, `pose_wait` (s a
  frame may wait for its pose; after 3 failures in a row the node stops waiting until a pose succeeds),
  `pose_max_age` (latest pose used for heartbeat frames).
- **Camera mount**: `camera_mount_source` (tf | param), `camera_frame` ("" = left camera_info frame_id),
  `camera_frame_is_optical` (false = ROS camera frame, x along the optical axis: the fixed rotation
  q = (-0.5, 0.5, -0.5, 0.5) to the optical frame is appended), `camera_mount` ([x y z qx qy qz qw], optical -> body,
  with `param`).
- **Stereo / detections**: `disparity_stride` (4: the mapper's grid), `disparity_offset` (none | auto = cx_l - cx_r |
  px, subtracted from the raw disparity; real launch 4.30), `disparity_quantum_px` (0; 1/256 reproduces the
  offline frames.bin cache for A/B runs), `detection_scale` (detector px -> calibrated px).
- **Pairing**: `sync_wait` 0.5 s, `sync_nearest_tol` 0.02 s, `disparity_cache` 400, `sync_early_resolve` true.
  Frames are released in detection order, so a detection waiting for a disparity that never comes would stall all
  later frames for `sync_wait`; with early resolve it is paired (nearest / none) as soon as a disparity newer than
  its stamp + tolerance arrived, which needs in-order disparity stamps (one publisher). Same pairing result as the
  offline reader; it removed a +0.35 s scan-latency penalty on the Isaac replay.
- **Cadence**: `heartbeat_frame_period` 0.2 s (empty frame to the mapper when the detector is silent, so phase
  changes and timeouts still run), `map_rate` 5 Hz, `health_rate` 2 Hz, `input_stale_warn` 2 s.

## Launch

The real launch defaults `mapper_config_file` to `config/core/cand_obj_real.cfg` (M4c's real-flight profile: on the
farm log the shipped cand_obj profile over-fragments); the sim launch and `treemapper.yaml` use `cand_obj.cfg`.
`test_config_agreement` checks that every launch default resolves inside `<repo>/config` and is accepted by the mapper.

```bash
# drone / real flight log (all frames and topics are args)
roslaunch treemapper_ros treemapper_real.launch                      # live: wall clock
roslaunch treemapper_ros treemapper_real.launch use_sim_time:=true   # + rosbag play --clock <log>.bag
# Isaac simulator (live or recorded)
roslaunch treemapper_ros treemapper_sim.launch
# the recorded Isaac mission, end to end (bag is read-only; nav_emulator plays navigation)
roslaunch treemapper_ros replay_isaac.launch [start:=45] [duration:=60] [rate:=1.0] [record:=/tmp/rec.bag]
rviz -d $(rospack find treemapper_ros)/rviz/treemapper.rviz
```

Sim vs real is configuration only: frames (`world`/`body` vs `world_nwu`/`body`), the camera mount (Isaac's
`/tf_static` parents the cameras to `world` at their spawn pose, so the sim launch passes the verified nadir mount
from `results/geometry_check.md` as a parameter; the real launch reads `body <- sensor_down` from TF), and the clock.

**Hop target in a replay.** A recorded `/spray_target` is in the recorded system's ID space. `replay_isaac.launch`
remaps it away and runs `nav_emulator.py`, which resolves every recorded target to this mapper's ID by its descent
point and marks trees sprayed when SPRAYING ends, the same rule as the offline runner (`core/apps/replay_main.cpp`).
Do not feed a foreign `/spray_target` straight into the node: it would freeze the wrong tree. On the drone navigation
must publish IDs it got from `~get_trees`.

## Build

The package builds the core in-tree (`add_subdirectory(<repo>/core)`), statically linked into the node. That is
deliberate: the registry reaches the candidates through weak symbols, so every executable must take the registry's
object files directly (`tm_link_registry()`, core/CMakeLists.txt); an installed static core would silently lose the
candidates. The core is found through the real path of this package (`../../core`), or `-DTM_CORE_DIR=<path>`.

In tm_dev (Ubuntu 20.04, Noetic, g++ 9.4; ROS master inside the container only):

```bash
mkdir -p /data/work/catkin_m4b/src && cd /data/work/catkin_m4b/src
ln -s /ws/ros/treemapper_msgs && ln -s /ws/ros/treemapper_ros
cd .. && catkin init && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build -j2 -p1 && catkin build treemapper_ros -j2 --make-args tests
./devel/lib/treemapper_ros/test_treemapper_ros                    # gtest
source devel/setup.bash && rostest treemapper_ros replay_smoke.test   # ~75 s, needs /data/isaac + the M1 extraction
ros_replay_check.sh /data/work/m4b/full 45 0 0.00390625               # full mission ROS vs offline (~5 min)
```

**On the Jetson Orin Nano** (arm64, Ubuntu 20.04 + Noetic, same as tm_dev):

1. Clone the repository anywhere and symlink (or copy) `ros/treemapper_msgs` and `ros/treemapper_ros` into the
   catkin workspace's `src/` — do not put the whole repository in `src/`: catkin would also find `core/package.xml`
   and build the core a second time as a plain CMake package (harmless, but slower). If the package is copied
   rather than symlinked, pass `-DTM_CORE_DIR=<repo>/core` and point `mapper_config_file` at `<repo>/config/<mapper>.cfg`
   (the `config/core` symlink must resolve).
2. Dependencies are all stock Noetic: roscpp, roslib, std/geometry/sensor/visualization_msgs, tf2, tf2_ros,
   topic_tools, rosbag, rostest, yaml-cpp (tests only). The core is standard-library C++17 (no Eigen, CGAL, OpenCV).
   `rosdep install --from-paths src --ignore-src -y` covers them.
3. `catkin build treemapper_ros -j2 -DCMAKE_BUILD_TYPE=Release` (or RelWithDebInfo). The scripts/nav emulator and
   converters need python3-numpy, scipy and shapely only for the offline comparison, not for flight.
4. In the drone's launch, include `treemapper_real.launch` and set the frame / topic args; navigation subscribes to
   `/treemapper/map` or calls `/treemapper/get_trees`, and calls `/treemapper/set_tree_sprayed`.

## Verified (in tm_dev, x86_64, core = main 960ccb7 / cand_obj)

- **Build**: `catkin build` of treemapper_msgs + treemapper_ros (core built in-tree) with `-Wall -Wextra` on the
  package targets (the core keeps its own `-Wall -Wextra -Wpedantic`): 0 warnings.
- **gtest** `test_treemapper_ros`: 28 cases pass (pairing incl. late / nearest / early-resolve / ordering / cache,
  disparity sampling + malformed + resolution + offset + quantum, baseline conventions, calibration rejects,
  DetectionResult decoding, conversions, markers, per-tree diagnostics, config agreement, cfg reader).
- **rostest** `replay_smoke.test` (60 s of the Isaac bag at rate 1.0): SUCCESS — heartbeat present, non-empty map,
  get_trees all / by id / unknown id, set_tree_sprayed unknown id and un-spray refused, counters, node alive.
- **ROS path = offline replay** on the full recorded Isaac mission (`scripts/ros_replay_check.sh`, bag from
  t = 55 s to the end at rate 1.0, `disparity_quantum_px` 1/256 to feed the same quantised depth as frames.bin):
  same 9 trees with the **same 9 IDs**, outline IoU 0.98-1.00 per tree, centres within 0.023 m, heights within
  0.033 m, same sprayed flags, same hop-target timeline (20-45 ms later than offline: the emulator polls at 20 Hz),
  and identical scorecards except per-frame recall 0.738/0.738, mean IoU 0.865/0.867, scan latency 2.35/2.32 s.
  Details: `results/ros_replay_isaac.md`.
- **Real-flight launch smoke** (45 s of the farm log + companion bag, `treemapper_real.launch use_sim_time:=true`):
  calibration latched (baseline 0.1606 m, "neg_mm"), 4.30 px offset applied, pose from TF throughout, 615 detections
  all paired, 25 trees published, health AWAKE with the "no mission_state" warning. Not scored here (M4c owns the
  real-flight evaluation).

## Unverified — test first on hardware

1. **Build and timing on the Orin** (arm64): never compiled there. On x86 the node takes ~1 ms per frame
   (`~stage_timing`: `node_frame_total`); check it on the Orin next to the detector, at the real frame rate.
2. **Real camera chain**: the real launch uses M4c's data-derived chain, not the recorded `/tf_static`:
   `body <- sensor_down` = lever (0.30, 0.08, -0.20) m, **82 deg** about +y (tf_static says 75), sensor_down a
   ROS-convention camera frame, and a **4.30 px** disparity offset (camera_info's cx_l - cx_r says 4.92; the raw
   images are not rectified, so the disparity node's rectification differs from camera_info). Both come from one
   flight without ground truth (docs/REAL_FLIGHT.md, results/real_flight_geometry.md). On the drone: confirm the
   mount (or fix the TF and use `camera_mount_source:=tf`) and re-measure the offset after any recalibration.
   The camera_info frame_id in the log is `sensor`, hence `camera_frame: sensor_down` explicitly.
3. **Real detector**: the existing detector's DetectionResult decoding is tested on synthetic bytes and the recorded
   Isaac bag; the real-flight companion bag carries the same type/md5. Detection pixel scale (`detection_scale`) is
   1.0 because both recordings detect on the half-resolution image.
4. **Navigation integration**: navigation must publish hop targets in this mapper's IDs (`/spray_target` by default)
   and `/mission_state` latched. With no `/mission_state` the node is permissive (scanning). Only `nav_emulator.py`
   has driven the services so far.
5. **Pose**: real pose comes from TF (`world_nwu <- body`, published by the flight stack in the log); `pose_topic`
   mode (PoseStamped) is implemented but was not exercised on data. `spot_spraying_msgs/DronePosition` (no
   orientation) is not supported as a pose source on purpose: the mapper needs the attitude.
6. **Two-bag real replays** have two clocks (bag time vs flight-time header stamps); the node pairs and looks up
   poses by header stamp and keeps heartbeat stamps on that clock, so they work, but `last_input_age_s` and
   receive times are bag time. On the drone there is one clock.
7. **Load**: rosbag play + node at rate 1.0 kept up in tm_dev (2 cores); at higher rates disparity messages can be
   dropped by the subscriber queue (20) — the counters show it as missing pairs (`sync_frames_paired_nearest` /
   `sync_frames_without_disparity`) rather than silently.
