# TreeMapper: on-board tree mapping for a spot-spraying drone

TreeMapper turns the drone's downward stereo and 2D tree detections into the tree map that navigation flies on: one entry
per tree with a stable integer ID, a ground outline, a centre, a canopy height (the descent reference) and a sprayed flag.
It runs as a ROS 1 Noetic node around a ROS-free C++17 core library, and ships with the evaluation harness that decided
the design: a synthetic orchard generator, a scorer that judges any system emitting the map contract, and one-command
evaluations.

**Shipped architecture: `cand_obj`, an object-centric crown mapper.** Every frame the disparity is binned into a canopy
height model on a fixed 0.4 m world lattice; a persistence watershed splits it into crowns at the saddles between tree
tops (so touching trees separate in height, not in xy); a crown counts only if its top is inside a detection box; crowns
are matched to tree instances by world geometry (no 2D tracks); each instance accumulates per-cell evidence and a cell
latches permanently after 5 consistent frames; the published outline is the hull of latched cells (grow-only), the height
the 0.99 quantile of the fused latched surface, both rate-limited. It was chosen over a map-centric DSM alternative
(`cand_grid`) on a held-out synthetic suite and the recorded Isaac mission. Read next: `docs/DESIGN.md` (design, brief
section 10 answers, rule-by-rule guarantees and tests), `results/REPORT.md` (all results), `docs/DECISIONS.md`.

## Repository layout

```
core/                         ROS-free C++17 library treemapper_core (plain CMake; standard library only)
  include/treemapper/         contract types, Frame / frames.bin I/O, Mapper interface + registry, simple baseline
  include|src/treemapper/candidates/cand_obj/   the shipped mapper
  include|src/treemapper/candidates/cand_grid/  the alternative candidate (kept for comparison)
  apps/replay_main.cpp        treemapper_replay: offline runner (frames.bin -> map stream JSONL) + navigation emulator
  tests/                      gtest suite test_core (rule tests per system, registry, config)
config/                       ALL mapper tunables: cand_obj.cfg (sim profile, shipped default), cand_obj_real.cfg
                              (real-flight profile), cand_grid.cfg. Built-in defaults == these files (tests).
ros/treemapper_msgs/          interface package: TreeEntry, TreeMap, Health, DropCounters, StageTiming, Detection2D(Array),
                              DronePose, MissionPhase; services GetTrees, SetTreeSprayed (every field documented)
ros/treemapper_ros/           the node, launch files (sim / real / Isaac replay), yaml, gtests, rostest, RViz config
tools/treemapper_tools/       Python 3.8: bag extraction, frame readers (Isaac, real flight), synthetic generator,
                              scorer, evaluation, real-flight scoring and geometry checks
tools/tests/                  pytest (scorer, generator, frame readers, evaluation)
scripts/                      run_tests.sh (all tests), evaluate.sh (one-command evaluation), score_baseline.sh
docs/                         PROJECT_BRIEF, LOCAL_RUN (environment rules), DATA_README, DESIGN, DECISIONS, SCORER,
                              REAL_FLIGHT, candidates/{cand_obj,cand_grid}.md
results/                      committed scorecards (md + json); results/REPORT.md is the index and the final report
PROGRESS.md                   milestone log and open items
```

Large data never enters the repo: builds go to `$TM_BUILD` (default `/data/work/build`), extracted frames to
`/data/work/extracted`, synthetic suites to `/data/work/synthetic`, evaluation outputs to `/data/work/eval`.

## Build and test (dev container, one command)

All commands run in the `tm_dev` container (Ubuntu 20.04 + ROS Noetic, the Orin's software stack; `docs/LOCAL_RUN.md`):

```bash
docker exec tm_dev bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws && TM_JOBS=2 ./scripts/run_tests.sh'
```

It configures and builds the core with every candidate (`TM_CANDIDATES=cand_obj` builds one), fails on a build error,
runs ctest (gtest `test_core`, 63 cases) and pytest (102 tests), and prints `ALL TESTS PASSED`. Use a fresh
`TM_BUILD=/data/work/build_<name>` to build from scratch.

ROS package (catkin, scratch workspace outside the repo; details in `ros/treemapper_ros/README.md`):

```bash
docker exec tm_dev bash -lc 'source /opt/ros/noetic/setup.bash && mkdir -p /data/work/catkin_ws/src && cd /data/work/catkin_ws/src \
  && ln -sfn /ws/ros/treemapper_msgs && ln -sfn /ws/ros/treemapper_ros && cd .. && catkin init \
  && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  && catkin build -j2 -p1 && catkin build treemapper_ros -j2 -p1 --make-args tests \
  && ./devel/lib/treemapper_ros/test_treemapper_ros'                      # 28 gtest cases
# end-to-end smoke on 60 s of the Isaac bag (needs /data/isaac and the extraction):
#   source devel/setup.bash && rostest treemapper_ros replay_smoke.test
```

The node links the core in-tree (`add_subdirectory`), because the registry reaches the mappers through weak symbols
(D41); do not install the core as a separate static library.

## Build on the Jetson Orin Nano (UNVERIFIED: never built on the device)

Target: arm64, Ubuntu 20.04, ROS Noetic, the same stack as `tm_dev`.

1. Clone the repository anywhere (not into the catkin `src/`), then symlink the two ROS packages into the workspace:
   `ln -s <repo>/ros/treemapper_msgs <ws>/src/ && ln -s <repo>/ros/treemapper_ros <ws>/src/`. The node finds the core via
   the package's real path (`../../core`). If you copy instead of symlinking, pass `-DTM_CORE_DIR=<repo>/core` and point
   `mapper_config_file` at `<repo>/config/<mapper>.cfg` (the `config/core` symlink must resolve).
2. Dependencies, all stock Noetic: roscpp, roslib, std_msgs, geometry_msgs, sensor_msgs, visualization_msgs, tf2, tf2_ros,
   topic_tools, rosbag, rostest; yaml-cpp for the tests. `rosdep install --from-paths src --ignore-src -y`. The core uses
   only the C++17 standard library (no Eigen, CGAL, OpenCV, CUDA), so there is nothing arm64-specific to port.
3. `catkin build treemapper_ros -j2 -DCMAKE_BUILD_TYPE=Release` (keep `-j2`: the detector shares the CPU). g++ 9.4 on 20.04
   is enough. Python tools are only for offline evaluation (numpy, scipy, shapely; `tools/requirements.txt`).
4. Include `treemapper_real.launch` in the drone launch and set the frame / topic arguments. Navigation subscribes to
   `/treemapper/map` (latched) or calls `/treemapper/get_trees`, publishes `/mission_state` (latched String) and the hop
   target on `/spray_target` **in TreeMapper's IDs**, and calls `/treemapper/set_tree_sprayed`.
5. First checks on the device: the checklist at the end of this file.

## Run on a recorded flight log

### Offline (no ROS graph): `treemapper_replay`

The replay reads a `frames.bin` (the per-frame input contract: pose, phase, target, boxes, stride-4 disparity), runs a
registered mapper with an emulated navigation (resolves hop targets by descent point, marks trees sprayed when SPRAYING
ends) and writes the map stream JSONL the scorer reads.

```bash
D='docker exec tm_dev bash -lc'
# Isaac simulator mission (with ground truth): extract once (~1 min), build frames.bin, replay
$D 'source /opt/ros/noetic/setup.bash && cd /ws/tools && E=/data/work/extracted/isaac_2026-10-23-15-34-29 \
  && python3 -m treemapper_tools.extract_bag --bag /data/isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag --out $E \
  && python3 -m treemapper_tools.bag_frames --bag /data/isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag --extracted $E --out $E/frames.bin'
$D 'cd /ws && E=/data/work/extracted/isaac_2026-10-23-15-34-29 && /data/work/build/core/treemapper_replay --frames $E/frames.bin \
  --system cand_obj --out-map /data/work/runs/isaac_map.jsonl --out-targets /data/work/runs/isaac_targets.json \
  --out-counters /data/work/runs/isaac_counters.json'
# real farm flight (no ground truth, no phases): flight bag + companion bag with the detections (~30 s)
$D 'source /opt/ros/noetic/setup.bash && cd /ws/tools && python3 -m treemapper_tools.real_frames \
  --bag /data/real/BB_treehopping_farm.bag --companion /data/real/BB_treehopping_farm_companion_2026-10-23-18-01-20.bag \
  --out /data/work/extracted/real_farm'
#   then treemapper_replay --frames /data/work/extracted/real_farm/frames.bin --system cand_obj ... with the real
#   profile (see "Real-flight profile" below), or score it in one go with real_score (docs/REAL_FLIGHT.md).
```

`--config "key=value;key=value"` overrides mapper keys on the command line (unknown keys are rejected).
`treemapper_replay --list-systems` prints the registered systems (`cand_grid cand_obj simple`).

### Through ROS: launch files

```bash
# Isaac simulator, live or recorded (world/body frames, verified nadir mount, /clock)
roslaunch treemapper_ros treemapper_sim.launch
# the recorded Isaac mission end to end: rosbag play --clock + node + nav_emulator.py (+ optional record)
roslaunch treemapper_ros replay_isaac.launch [start:=45] [duration:=60] [rate:=1.0] [record:=/tmp/rec.bag]
# real flight log (world_nwu/body, mount 82 deg + disparity offset 4.30 px + real mapper profile as defaults; every name is an arg)
roslaunch treemapper_ros treemapper_real.launch use_sim_time:=true      # mapper profile: config/cand_obj_real.cfg (default)
rosbag play --clock /data/real/BB_treehopping_farm.bag /data/real/BB_treehopping_farm_companion_2026-10-23-18-01-20.bag
# drone: treemapper_real.launch with use_sim_time:=false
rviz -d $(rospack find treemapper_ros)/rviz/treemapper.rviz
```

Outputs (`/treemapper/...`): `map` (TreeMap, latched, 5 Hz + on phase and sprayed changes), `markers`, `health`
(AWAKE / SLEEPING "sleeping, not dead" / HOLDING / FAULT, 2 Hz, also while asleep), `counters` (every drop by reason),
`stage_timing`; services `get_trees` (id -1 = all) and `set_tree_sprayed`. Topic and parameter reference:
`ros/treemapper_ros/README.md`. A recorded `/spray_target` belongs to the recorded system's ID space: never feed it to
the node directly (the replay launch remaps it to `nav_emulator.py`). `scripts/ros_replay_check.sh` (in the package)
runs the full mission through ROS and compares it with the offline replay (`results/ros_replay_isaac.md`).

## Score a system, and compare with the existing one

### One command for our systems

```bash
docker exec tm_dev bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws && TM_OUT=/data/work/eval/mine ./scripts/evaluate.sh cand_obj'
docker exec tm_dev bash -lc 'cd /ws && ./scripts/evaluate.sh simple,cand_obj,cand_grid suite_v2'   # paired by seed, ref = first
```

It builds, extracts or generates what is missing, runs each system on the recorded Isaac mission **side by side with the
existing system's recorded map** and on a synthetic suite, and prints scorecards (`$TM_OUT/isaac.md`, `synthetic.md`,
`.json`). Options in the script header: `TM_SET="cand_obj.key=value ..."`, `TM_REF`, `TM_REPEAT=2` (run-to-run floor),
`TM_SKIP_ISAAC` / `TM_SKIP_SYNTH`, `TM_CANDIDATES`, `TM_BUILD`. Suites: `suite_v1`, `suite_v2` (tuning, fixed hover) under
`/data/work/synthetic/`; the held-out suite is `/data/work/holdout/suite_holdout` (generate more with
`python3 -m treemapper_tools.synth --out <dir> --levels easy,medium,hard --seeds 300-309`).

### Any other system that emits the contract

The scorer only reads files. A system has to write the **map stream JSONL**: one line per snapshot (about 5 Hz,
including empty snapshots, on the clock of `mission.json`):
`{"t": 12.3, "frame": "world", "trees": [{"id": 4, "outline": [[x, y], ...], "centre": [x, y], "height": 5.4, "sprayed": false}], "stage_ms": {...}}`
(`height: null` = undefined; full definition in `docs/SCORER.md`). Then:

```bash
docker exec tm_dev bash -lc 'cd /ws/tools && E=/data/work/extracted/isaac_2026-10-23-15-34-29 && python3 -m treemapper_tools.score \
  --map /path/other_map.jsonl --gt $E/ground_truth.json --mission $E/mission.json --camera $E/camera_info.json \
  --title "other system" --out-md /data/work/eval/other.md --out-json /data/work/eval/other.json'
# no ground truth (real flight): omit --gt; stability + safety + cost only
```

If the other system navigates by its own IDs, pass the `mission.json` whose targets are in **its** ID space (the
recording's own `mission.json` is in the existing system's IDs). For a paired comparison in one table, make a dataset
directory with the same files (symlinks to `frames.bin`, `ground_truth.json`, `mission.json`, `camera_info.json`) and the
other system's stream as `map_existing.jsonl`, then `python3 -m treemapper_tools.evaluate --system cand_obj,existing
--dataset <dir> --reference cand_obj`.

**The existing system's map extractors** (these produce `map_existing.jsonl`):
- Isaac recording: `python3 -m treemapper_tools.extract_bag --bag <isaac bag> --out <dir>` converts
  `/validation/fusion_snapshot` (frame `world_nwu` -> `world` where needed; height = `max_height`, D4-D7);
  `scripts/score_baseline.sh` extracts once and writes the baseline scorecard (`results/baseline_existing_system.md`).
- Real flight: `python3 -m treemapper_tools.real_existing --companion <companion bag> --out <dir>/map_existing.jsonl`
  converts `/object_fusion/polygon_marker` (outlines, ID = marker ID) + `/object_fusion/fused_bboxes` (height = its median
  feature height; not comparable in absolute terms with canopy-top heights). Scored with
  `python3 -m treemapper_tools.real_score --systems simple,cand_obj,cand_grid,existing ...` (`docs/REAL_FLIGHT.md`).
- Our own node's recorded `~map`: `ros/treemapper_ros/scripts/treemapper_bag_to_jsonl.py`.

## Tunable parameters

**Every mapper tunable lives in one file per profile** (`config/cand_obj.cfg` for sim / synthetic, which equals the
built-in defaults, enforced by `CandObjConfig.ShippedFileEqualsBuiltInDefaults`; `config/cand_obj_real.cfg` for real
stereo). Values are validated and clamped in one place (`Config::sanitized()` in `core/src/candidates/cand_obj/cand_obj.cpp`;
every clamp is logged); unknown keys are rejected at start. The node's own parameters (topics, frames, mount, pairing,
cadence) are in `ros/treemapper_ros/config/treemapper.yaml` and documented in `ros/treemapper_ros/README.md`; the yaml
only points at the mapper file and must not override it (`ConfigAgreement.*`). Real-profile values that differ from the sim
profile are in bold. (Generated from the two files by `python3 -m treemapper_tools.param_table`, run in `tools/`.)

| group | key | sim profile (`cand_obj.cfg`) | real profile (`cand_obj_real.cfg`) | meaning | trade-off |
|---|---|---|---|---|---|
| points -> cells | `min_range` | 0.5 | 0.5 | m, optical depth gate (near) | lower keeps very close points (low hover); very near stereo is unreliable |
| points -> cells | `max_range` | 30.0 | 30.0 | m, optical depth gate (far) | higher sees more of the swath edge but admits noisy far depth (sigma_Z ~ Z^2) |
| points -> cells | `above_drone_margin` | 0.5 | 0.5 | m, points above drone z - margin are dropped (impossible) | larger also drops real tops when the drone hovers low over a tall neighbour |
| points -> cells | `max_frame_noise` | 2.0 | 2.0 | m, a frame is dropped when 1.4826 x median per-cell MAD of z exceeds it (noise burst, turns) | lower rejects more blurred turn frames (fewer artefacts, slower evidence); higher admits bursts |
| points -> cells | `height_sample_clearance` | 2.0 | 2.0 | m, cells within this of the drone altitude give no height (the above-drone cap may have cut the top) | larger is safer against shoulder heights under a low hover, but low hovers then add no height |
| points -> cells | `cell` | 0.4 | 0.4 | m, world lattice cell (CHM and every evidence grid) | smaller = finer outlines but fewer points per cell (noisier medians, more fragments); larger merges gaps |
| points -> cells | `min_cell_points` | 2 | 2 | points for a cell to be known | higher = more robust medians, more unknown cells at range |
| points -> cells | `max_grid` | 160 | 160 | cells per side cap of the per-frame CHM window | memory/CPU bound; too small truncates the footprint at high altitude |
| point and frame weighting (M4) | `z_ref` | 15.0 | 15.0 | m, range of unit point weight, weight ~ (z_ref/Z)^4 | sets how fast far points lose influence on heights and evidence |
| point and frame weighting (M4) | `lateral_k` | 10.0 | 10.0 | off-axis weight, weight /= 1 + k rho^2 | higher trusts only near-nadir points (tilted real mount sees less weight) |
| point and frame weighting (M4) | `rot_rate_ref` | 1.0 | 1.0 | rad/s, frame quality = 1 / (1 + (rate/ref)^2) | lower discounts turns more (fewer turn-blur fragments, +0.1-0.2 s latency at 0.35) |
| point and frame weighting (M4) | `quality_recover_s` | 1.5 | 1.5 | s, frame quality recovers linearly after a fast rotation | longer = more caution after turns, later latching at row ends |
| ground plane | `ground_low_fraction` | 0.4 | 0.4 | fraction of lowest cells the ground plane is fitted to | higher is stabler on open ground but lifts the plane under dense canopy |
| ground plane | `ground_inlier_band` | 0.6 | 0.6 | m, refit band around the first plane | wider tolerates rough ground, narrower rejects low bushes |
| ground plane | `ground_max_slope` | 0.3 | 0.3 | max |slope| of an accepted plane (else flat fallback, counted) | higher allows steep terrain but accepts fits tilted by canopy |
| ground plane | `min_above_ground` | 1.0 | 1.0 | m, smoothed cell this far above the plane is canopy | higher rejects bushes/ground noise, loses low crown skirts |
| crowns | `min_prominence` | 0.6 | **1.0** | m, a basin peak this little above its saddle merges into the neighbour | the touching-tree knob: lower splits touching crowns (and over-fragments rough real crowns); higher merges neighbours |
| crowns | `min_basin_cells` | 4 | **8** | cells, smaller crowns ignored | higher suppresses noise blobs, delays small/edge crowns |
| crowns | `box_dilate` | 0.15 | 0.15 | fraction, boxes grown before the crown-top test | higher tolerates box jitter / pixel misalignment, admits more non-tree tops |
| crowns | `peak_border_px` | 24.0 | 24.0 | px, a crown top this close to the image edge is flagged truncated (its top may be outside the image) | only matters with seed_truncated = false (then truncated crowns never seed); with the shipped true it only feeds a counter (instances_seeded_from_truncated_crown) |
| instances | `seed_truncated` | true | **false** | a crown whose top is flagged truncated may still seed an instance | true = earlier evidence (latency), slightly more neighbour spill at crown entry |
| instances | `window` | 28 | 28 | cells per side of an instance evidence window | must cover the largest crown (28 x 0.4 = 11.2 m); memory ~14 kB per instance |
| instances | `hull_cell_corners` | false | false | outline from latched cell corners (true) or centres (false) | corners = larger outlines (+spill), centres = tighter |
| instances | `match_radius` | 2.0 | **3.0** | m, crown top this close to an instance centre is that instance | higher re-associates more (fewer duplicates), risks crediting a neighbour |
| instances | `match_overlap` | 0.5 | 0.5 | fraction of an instance latched cells a crown must cover to be it, wherever its top is | lower = fewer transient duplicates, more neighbour takeover risk |
| instances | `match_reach` | 3.0 | **4.5** | m, ... provided the top is within this of the centre | larger helps big crowns, credits neighbours at tight spacing (hard recall 0.78 at 4 m) |
| instances | `seed_overlap` | 0.25 | **0.1** | fraction of a would-be seed on a published tree latched cells that credits it to that tree (seed guard) | lower prevents duplicates, can swallow a neighbour first slice |
| instances | `max_cell_radius` | 4.0 | 4.0 | m, cells farther from the instance centre are not credited (spill guard) | must exceed the largest crown radius; higher allows spill |
| instances | `latch_rel_h` | 0.0 | 0.0 | skirt trim: cell claimed only if its height >= this x crown peak (0 = off) | cuts spill, costs hard recall (0.82 -> 0.55 at 0.3, suite_v1) |
| instances | `latch_frames` | 5 | 5 | quality-weighted positive frames before a cell latches | rule 2 knob: higher = more robust to bad frames, slower outlines / latency |
| instances | `latch_ratio` | 0.75 | 0.75 | fraction of judged frames a cell must be positive in | higher = tighter outlines, less spill; too high loses recall under heavy noise |
| instances | `confirm_frames` | 3 | 3 | frames with a matched crown before publication | higher = fewer false trees, later publication |
| instances | `min_height_cells` | 3 | 3 | latched cells with a fused height before publication (rule 3) | higher = more robust first height, later publication |
| instances | `min_publish_area` | 3.0 | **8.0** | m^2 of latched cells before publication | higher = fewer fragments / centre drift, later publication, misses small trees |
| instances | `min_tree_height` | 1.5 | 1.5 | m above ground, lower crowns never seed | higher rejects bushes/clutter, misses young trees |
| instances | `tentative_timeout` | 1.5 | 1.5 | s, unconfirmed instances unseen this long are deleted | longer keeps slow-building trees, keeps noise seeds alive |
| height model (M4) | `cell_weight_cap` | 30.0 | 30.0 | saturation of a cell fused-height weight | lower adapts faster; higher averages more; the cap is what stops hover inflation |
| height model (M4) | `outlier_gate_m` | 2.0 | 2.0 | m, frame height this far from an established cell is rejected | lower rejects more noise, slower to fix a wrong early value (3 rejections re-initialise) |
| height model (M4) | `gate_min_weight` | 3.0 | 3.0 | weight after which the outlier gate applies | lower gates earlier |
| height model (M4) | `height_quantile` | 0.99 | 0.99 | published height = this quantile of the latched fused surface | descent reference: higher = closer to the true top (safe direction), more upward noise |
| height model (M4) | `max_height_rate` | 0.5 | 0.5 | m/s a published height may move | lower = no jumps, slower to converge after a merge |
| height model (M4) | `max_centre_rate` | 0.8 | 0.8 | m/s a published centre may move (outline not delayed) | lower = smoother centre, lags the outline |
| merges | `merge_overlap` | 0.5 | **0.3** | merge when latched overlap >= this x the smaller latched area | lower merges fragments readily, risks merging neighbours |
| merges | `merge_centre_dist` | 1.5 | **2.5** | m, or merge when centres are closer | higher merges real-crown fragments, risks merging tight neighbours |
| phases | `post_spray_hold_radius` | 2.0 | 2.0 | m, after SPRAYING non-targets stay frozen until the drone is this far from the hover point | larger = longer protection on the climb-out |

45 keys; 9 keys differ in the real profile

Node parameters that matter most on the drone (all in `treemapper.yaml` / the launch args): `camera_mount` +
`camera_mount_source` (wrong mount = tilted ground and shifted crowns), `disparity_offset` (wrong offset = depth scale
error, 7-16 % with 4.918 instead of 4.30 px on the farm flight), `pose_wait` (pose latency vs holding frames),
`sync_wait` / `sync_nearest_tol` (pairing latency vs frames without depth), `heartbeat_frame_period`, `map_rate`,
`health_rate`, `input_stale_warn`.

### Real-flight profile: which file goes with which mode

| Mode | Mapper profile | How it is selected |
|---|---|---|
| real drone, real flight logs (`treemapper_real.launch`) | `config/cand_obj_real.cfg` | default of the launch's `mapper_config_file` arg (`config/core/cand_obj_real.cfg`) |
| Isaac sim / recorded Isaac mission (`treemapper_sim.launch`, `replay_isaac.launch`), `treemapper.yaml`'s pointer, `scripts/evaluate.sh`, `treemapper_replay` without `--config` | `config/cand_obj.cfg` = built-in defaults | default |

The real profile changes 9 keys (bold above). With the sim profile the real farm flight over-fragments: 102 trees where
the stereo shows about 63 crowns, 39 outlines without a crown peak. With the real profile: 63 trees, 6 outlines without a
peak, 2 with two, 5 peaks missed, 0 centre / height jumps (`results/real_flight_profile.md`). The decisive key is
`merge_overlap` 0.5 -> 0.3: real crowns are rough and split into two or three instances that overlap by 30-50 %.
**Do not use the real profile in simulation, and do not use the sim profile on real data**: on the synthetic held-out suite
the real profile drops recall from 0.93 to 0.85, raises merged entries from 1.2 to 2.6 per mission and has 20 safety frames
on one hard mission (a height 0.65 m above the truth top, above a 7.35 m hover); on Isaac it adds ~2 s latency (4.23 vs
2.32 s). The two cases differ in tree spacing relative to crown roughness (about 8 m on the farm, 4.5-5 m on synthetic
medium / hard); a spacing-aware merge rule in code could unify them later. The real profile is tuned on **one** flight
without ground truth.

Offline replay with the real profile (`treemapper_replay` takes key/values, not a file):

```bash
docker exec tm_dev bash -lc 'cd /ws && CFG=$(grep -v "^\s*#" config/cand_obj_real.cfg | sed "s/#.*//" | tr -d " " | grep . | paste -sd";") \
  && /data/work/build/core/treemapper_replay --frames /data/work/extracted/real_farm/frames.bin --system cand_obj --config "$CFG" \
     --out-map /data/work/runs/real_map.jsonl --out-targets /data/work/runs/real_targets.json'
```

## What is unverified, and what to test first on real hardware

Nothing here ran on the Orin, on the real camera live, or with real navigation. In order:

1. **Build and timing on the Orin** next to the running detector: `catkin build` (arm64), then
   `roslaunch treemapper_ros treemapper_real.launch use_sim_time:=true` with the farm bag; read `~stage_timing`
   (`node_frame_total` is about 0.85 ms mean, 1.5 ms p99 per frame on x86; `results/ros_replay_isaac.md`) and check the node
   keeps up at the real frame rate (`~counters`: `sync_frames_without_disparity`, queue drops show up as missing pairs).
2. **Camera chain on the drone**: confirm the mount pitch (the data say 82 deg, `/tf_static` says 75; `results/real_flight_geometry.md`)
   and re-measure the disparity offset (4.30 px from a motion test, camera_info says 4.918) after any recalibration. A
   wrong value here is a depth-scale / tilt error that no mapper parameter can fix. Fix the TF and use
   `camera_mount_source:=tf` once it is right. The 7 deg could also be a constant attitude bias (the body reads +3.2 deg
   nose-down in hover). Also unverified from the data alone: the mount **roll** (only bounded to +-2 deg) and the **lever arm**
   (0.30, 0.08, -0.20) m from tf_static (below the noise). The left/right raw images are not rectified, so the disparity
   node's rectification is what the 4.30 px offset describes.
3. **Detector pixel frame**: boxes come from the raw (unrectified) left image, the disparity lives in the rectified frame;
   up to ~13 px apart at the image edges (`docs/REAL_FLIGHT.md`). Check with an overlay on the device and set
   `detection_scale` / an offset if needed.
4. **Real-flight profile** (`config/cand_obj_real.cfg`): tuned on one flight, one orchard, without ground truth, against
   stereo crown peaks (a proxy; the 5 missed peaks were not checked by eye). Fly a corridor over a counted block of trees
   and check the tree count, one outline per crown, and that neighbours are not merged. Known open residual: the crown-top
   world z falls by 0.30 m per metre of altitude on the farm flight, not explained by the depth scale (box clipping /
   visibility suspected; `results/real_flight_geometry.md`).
5. **Navigation integration**: `/mission_state` latched, hop targets in TreeMapper's IDs, `set_tree_sprayed` calls. Only
   `nav_emulator.py` has driven these so far. Watch `~health` warnings ("no mission_state received: permissive").
6. **Phase behaviour on the real stack**: the real flight has no phases, so the lock, descent freeze and sprayed
   protection were exercised only in simulation (synthetic suites and the Isaac recording). Run one hop/spray mission
   with `record` on and score it with `real_score` / `score.py` (no ground truth).
7. **Heights**: the published height is world z of the canopy top (0.99 quantile of the fused surface). Pose z comes from
   the flight controller (GNSS / baro) and its drift is unknown; check against a measured tree before trusting descents.
8. **Load and clocks**: at higher replay rates the subscriber queue drops disparity frames (counted, not silent); on the
   drone there is one clock, in two-bag replays receive times are bag time.
9. **Pose source**: real pose comes from TF (`world_nwu <- body`); the `pose_topic` (PoseStamped) mode is implemented but was
   never exercised on data.

Comparisons with the existing system on the real flight are against its companion replay, whose commit, configuration
and camera chain are unknown; its heights there are median feature heights, not canopy tops.
