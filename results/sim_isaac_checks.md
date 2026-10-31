# Isaac orchard sim: the three Isaac checks (Tasks 11, 12, 13)

Status: all three checks pass with the numbers below. Isaac Sim 5.1 (`treehoppingnavigation-treehoppingsim:latest`),
headless `SimulationApp({"headless": True, "renderer": "RaytracedLighting", ...})`, ROS 2 Humble through the
`tm_sim_ros:latest` bridge (`tm_sim_net`, `ROS_DOMAIN_ID=77`, `rmw_fastrtps_cpp`), RTX 3070 shared.
Raw outputs: `/data/work/sim_runs/chk/final3/` (truth, 3 frames + overlays, square traces, topics) and the Noetic-side
logs in `/data/work/logs/final3/` (`/data/work` = `/media/alex/waspsim/cloud_build_bundle/work`).
Check spec: `sim/specs/3x3_check.json` = `3x3_smoke.json` with one extra key, `terrain.color = [0.55, 0.42, 0.28]` (tan soil,
so the foliage is the only green in the check images); the trees, seed and layout are those of `3x3_smoke.json`.
Fix round 1 re-ran the topics, hover projection, moving-flight and spray checks (`/data/work/sim_runs/chk/fix2/`, logs
`/data/work/logs/fix2/`) after the clock change described below; the Task 11 table, the camera model, the square and the
spray numbers of the first run (`final3`) are unchanged by it (spray re-checked: 0.2998 L on tree 5).

## Task 11: scene (`orchard_sim.py --only-scene`, spec `3x3_smoke.json`)

`truth.json` has 9 trees, 9 distinct `top_z`; `ground_truth.json` loads with the scorer contract and has
`height_reliable: true`. (`/data/work/sim_runs/chk/scene_only/`)

| id | species | centre xy (m) | radius (m) | top_z (m) | ground_z |
|---|---|---|---|---|---|
| 1 | orange | (0.05, 0.45) | 1.51 | 2.774 | 0.00 |
| 2 | lemon | (6.34, -0.01) | 1.99 | 2.983 | 0.00 |
| 3 | lemon | (11.88, 0.33) | 1.54 | 2.312 | 0.00 |
| 4 | lemon | (-0.33, 5.70) | 1.79 | 2.693 | 0.00 |
| 5 | lemon | (6.43, 6.17) | 1.56 | 2.341 | 0.00 |
| 6 | orange | (11.98, 5.62) | 1.71 | 3.136 | 0.00 |
| 7 | lemon | (-0.54, 12.01) | 1.91 | 2.862 | 0.00 |
| 8 | orange | (6.08, 11.79) | 1.64 | 3.013 | 0.00 |
| 9 | orange | (11.63, 12.34) | 1.43 | 2.630 | 0.00 |

`rows_8x12.json` also builds: 86 trees (8 % dropped), `top_z` 2.50 .. 5.22 m (sloped terrain), 2.5 GB of GPU memory.

## Task 12: drone, cameras, entry point

**Camera model (`camera_check.py`, no ROS).** Emissive spheres at known world points, drone at (2, -1, 10) with
roll 0.05, pitch -0.03, yaw 0.4 rad, both cameras from `cameras.define_cameras`: measured sphere centroid vs the projection
with the LEFT `P` (fx = fy = 454.53, cx = 346.71, cy = 209.68) and the body pose o mount: **max error 0.13 px** (left and
right, 4 points in view each), stereo disparity measured vs `fx * B / Z` within 0.05 px (B = 0.160 m).
The principal point is rendered with the RTX renderer's aperture offsets (sign and a 0.5 px edge-vs-centre
convention measured, see `frames.CameraRig.usd_camera_params`); without the offset the error would be 26.7 px.

**Topics (`sim_check.py topics`, 10 sim s window).**

| Item | Result |
|---|---|
| `/clock`, `/tf` | 60.0 Hz, 59.9 Hz (0.2 % lost in the bridge) |
| left / right `image_raw` | 9.7 Hz each per sim s (best-effort loss), 640x400 `rgb8`, frame ids `cam_left`, `cam_right` |
| `camera_info` | 9.7 Hz, 121/121 image stamps have a camera_info with the identical stamp; K, P, size exactly as `camera.json`, D = 5 zeros, `plumb_bob`, R = I |
| left vs right image stamps | 121 of 121 identical (exact-stamp `TimeSynchronizer` of `stereo_sgbm_node` works) |
| image stamps vs `/tf` | 118 of 118 image stamps lie within 0.1 ms of a `/tf` stamp (grid only; the moving-flight check below tests the content) |
| `/tf_static` | body -> cam_left, body -> cam_right, replayed 1 Hz (10 in 10 s) |
| `/sim/truth` | 9 trees, 1 Hz (latched on the ROS 2 side) |
| `/sim/deposition` | 2 Hz |

**Moving-flight check (`check_moving.py`, the image time base in motion).** orchard_sim with `--v-max-xy 4`, flown at a
constant 4 m/s (measured 3.96 m/s) along +x straight across tree 5 at 15 m (`sim/specs/3x3_check.json`). All 119 published
left images were kept in `tm_sim_ros`; for the 21 frames with the whole crown inside the image the pose is a tf2 lookup
`world -> cam_left` AT THE IMAGE STAMP (`/tf` + `/tf_static` through the bridge), the truth centre is projected with the left P and
compared with the measured crown centroid. Along-track = component of the offset along the image direction of the flight.

| | along-track (px) | cross-track (px) |
|---|---|---|
| mean | **-2.6** (limit: \|mean\| <= 5, enforced in code, exit 1) | -3.9 |
| std / min / max | 2.6 / -7.0 / +2.6 | 1.9 / -7.5 / -0.9 |
| hover value (frame A, same sign convention) | -2.2 | -3.1 |
| a one-frame (0.1 s) lag would be | +-13.3 | |

Frames: `/data/work/sim_runs/chk/fix2/moving/`; per-frame table in `result.json`.

This check found a real lag in the first version of the clock. The ROS2 camera helper stamps images with the graph's
simulation time, which is **0.18333 s behind the timeline** (graph 0.21667 s when the timeline reads 0.4 s), while `/clock` and `/tf`
were on the timeline: the image content was 0.18 s newer than its stamp, which is -24 px along-track at 4 m/s
(mean -23.5 px, 19 frames; the hover and grid tests could not see it, a whole number of steps is on the 60 Hz grid).
Fix: the sim clock now runs on the graph time. During the first two app updates nothing is published (the first update runs
5 of 6 steps); the offset graph - timeline is measured on the second, and from the third update on `/clock`, `/tf`, the
controller and the nozzle use `t = timeline + offset`, seeded before the first publish so `/clock` starts without a jump and
never moves back (start would be negative -> exit 1). Every update afterwards `orchard_sim.py` reads the graph time (a probe
`ROS2PublishClock` on `/tm_sim_time_probe` keeps `IsaacReadSimulationTime` evaluated) and exits non-zero if
|graph time - our t| > dt/2: logged `graph_skew_max = 0.0000 s` over 210 sim s.

**Square (`sim_check.py square`, moving carrot at 20 Hz, 8 x 8 m at z = 5 m).** Cross-track deviation of `/tf` from the
square (3-D distance to the nearest edge):

| carrot speed | max deviation | mean | p95 | max, > 2 m from a corner | end error |
|---|---|---|---|---|---|
| 1.5 m/s | 0.169 m | 0.023 m | 0.130 m | 0.042 m | < 1e-9 m |
| 3.0 m/s | 1.33 m | 0.337 m | 0.860 m | 1.12 m | < 1e-9 m |

At 3 m/s the corners are cut and overshot (a_max = 2 m/s^2 cannot turn 3 m/s on the spot: x range -4.87 .. 4.60 m for a
-4 .. 4 m square). That is the controller's physics, not a transport effect. Task 16 should keep the carrot slow
around corners or accept ~1 m of corner cutting at 3-4 m/s.

**Projection check (`check_projection.py`, left P, body pose o mount vs the crown in the saved left image).** Tree 5
(centre (6.428, 6.169), top_z 2.341). The crown is the green blob nearest the projection, centroid of its closed mask.
Frames: `/data/work/sim_runs/chk/fix2/frame_hover{1,2,3}.png` (+ `.overlay.png`: red = projected truth centre, blue = crown
centroid). Saved at hover (`--save-hover 3`).

| frame | pose (x, y, z, yaw) | projected (u, v) | crown centroid (u, v) | offset | limit |
|---|---|---|---|---|---|
| A overhead (the check) | (6.43, 6.17, 15.0, 0) | (349.4, 214.7) | (346.3, 216.9) | **3.8 px** (-3.1, +2.2) | <= 10 px |
| B 2.5 m west, 12 m up | (3.93, 6.17, 12.0, 0) | (350.2, 108.3) | (348.2, 114.0) | 6.0 px | info |
| C as B, yaw +90 deg | (3.93, 6.17, 12.0, 1.571) | (458.0, 216.2) | (452.4, 214.3) | 6.0 px | info |

Yaw: with yaw +90 deg the tree 2.5 m ahead-in-x moves from the top of the image (frame B, v = 114) to the right of it
(frame C, u = 452), i.e. image-up = body forward and image-right = body right (-y), as the mount requires.
B and C use the z of the middle of the canopy volume for the truth point, so 6 px there includes the
visible-top-surface perspective bias; only frame A is a limit.

## Task 13: nozzle (`sim_check.py hover` + `spray`: a rospy publisher of `/corridor_nav/spray` Bool at 10 Hz for 4 s, the same as `rostopic pub -r 10`)

Body hovering 3.0 m above the canopy top of tree 5 (z = 5.341, nozzle 2.7 m above the canopy), nozzle open 4.0 sim s
(0.075 L/s, 0.3 L). Final `deposition.json` of the run (two sprays):

| spray | tree 5 | ground | emitted |
|---|---|---|---|
| 1: over tree 5 | **0.2998 L = 99.9 % of 0.3 L** (>= 80 % required) | 0.0002 L | 0.3000 L |
| 2: at (3.2, 3.0), between trees | 0 | 0.3002 L (all off-target) | 0.6000 L total |

`open_time` 8.0 s, `unknown_hits` 0 (no ray hit a collider that is not a canopy hull). Live `/sim/deposition` after
spray 1 on the Noetic side matched.

## Speed

3x3 scene: RTF 3.5 on the ground, 2.2 .. 3.3 in flight (3x3, 2 cameras, 60 Hz physics, cameras 10 Hz, Noetic
checkers running). `rows_8x12` (86 trees) hovering at 20 m over the field: RTF 2.4 (2.29 over a 120 sim s run), 2.5 GB GPU.
Physics, controller, TF, `/clock` and the nozzle run per physics step in a callback; one app update renders the two cameras
every 6th physics step (rendering every physics step as in the Task 10 spike, with a third render product for saved frames, gave RTF 0.45 on the same 3x3 scene).
`--max-rtf` caps the factor by sleeping. Startup (`SimulationApp` ready) takes ~140 s on this machine today (CPU-bound
RTX pipeline start, not changed by the cache mounts; Task 10 measured 12-17 s).

## Commands

```bash
docker network create tm_sim_net                       # once; image tm_sim_ros:latest from Task 10
mkdir -p -m 777 /media/alex/waspsim/cloud_build_bundle/work/sim_runs   # the Isaac image runs as uid 1234
mkdir -p ~/.cache/tm_isaac/{main,compute,ovdata}; chmod -R 777 ~/.cache/tm_isaac

# Noetic side: roscore + parameter_bridge
docker run -d --rm --name tm_sim_ros --network tm_sim_net --gpus all \
  -e ROS_DOMAIN_ID=77 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /home/alex/SUIND/TreeMapper:/ws:ro -v /media/alex/waspsim/cloud_build_bundle/work:/data/work \
  tm_sim_ros:latest bridge

# Isaac side (orchard_sim.py; the wrapper sets the Humble env and forwards SIGTERM, so `docker stop` writes the outputs)
docker run -d --rm --name tm_isaac --gpus all --network tm_sim_net --entrypoint bash \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e ROS_DOMAIN_ID=77 \
  -v /home/alex/SUIND/TreeMapper:/ws:ro \
  -v /home/alex/SUIND/TreeHoppingNavigation/Nvidia-Isaac-Sim-Tree-Generator/models/Tree:/assets/trees:ro \
  -v /media/alex/waspsim/cloud_build_bundle/work/sim_runs:/data/work/sim_runs \
  -v ~/.cache/tm_isaac/main:/isaac-sim/.cache -v ~/.cache/tm_isaac/compute:/isaac-sim/.nv/ComputeCache \
  -v ~/.cache/tm_isaac/ovdata:/isaac-sim/.local/share/ov/data \
  treehoppingnavigation-treehoppingsim:latest \
  /ws/sim/isaac/run_orchard_sim.sh --spec /ws/sim/specs/3x3_smoke.json --out /data/work/sim_runs/<name> \
  [--v-max-xy 6.0 --v-max-z 2.0 --render-hz 10 --physics-hz 60 --max-rtf 0 --save-frame <sim s> --save-hover <s>]
docker logs -f tm_isaac > /media/alex/waspsim/cloud_build_bundle/work/logs/<name>.log 2>&1 &   # uid 1234 cannot write /data/work/logs

# scene only (Task 11): same command with --only-scene (exits when truth.json is written)

# checks (ROS 1 only; /data/work in tm_sim_ros is writable)
docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/isaac/sim_check.py topics --duration 10
docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/isaac/sim_check.py square --z 5 --half 4 --speed 1.5
# the checks use --spec /ws/sim/specs/3x3_check.json --v-max-xy 4 --save-hover 3
docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/isaac/sim_check.py flight --tree 5      # frames via --save-hover 3
docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/isaac/check_moving.py --tree 5 --z 15 --speed 4 --out /data/work/sim_runs/<name>/moving
docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/isaac/check_projection.py --run /data/work/sim_runs/<name> --frame frame_hover1 --tree 5
docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/isaac/sim_check.py hover --x 6.428 --y 6.169 --z 5.341
docker exec tm_sim_ros /tm_entrypoint.sh python3 /ws/sim/isaac/sim_check.py spray --seconds 4 --rate 10
docker stop tm_isaac                                  # writes deposition.json, rtf.json (~3 s)
# camera model, no ROS:  python.sh /ws/sim/isaac/camera_check.py --out /data/work/sim_runs/camera_check.json
docker stop tm_sim_ros
```

Exit behaviour: SIGTERM/SIGINT write `deposition.json` and `rtf.json` (`{"sim_s","wall_s","rtf"}`) and close the app
(2.9 s). Unknown command-line arguments are an error (only `--/...` carb settings pass through to Kit). An exception during scene build prints the traceback and exits 1 (tested with `--assets-dir /nonexistent`:
`FileNotFoundError`, exit code 1).

## Concerns and limits

- `clutter.rocks` of the spec is not instantiated. `clutter.grass` is rendered (since commit 0b9f740, Task 16B) as a
  periodic band-limited brightness texture on the terrain colour (`sim/isaac/ground_texture.py`), not as grass
  geometry; before that the ground was a flat-colour mesh, on which SGBM could not match the ground. **The checks on
  this page were taken on the flat ground.** They are analytically unaffected by the texture: the camera model,
  topics, clock, square and nozzle checks do not look at the ground's appearance, and the projection / moving-flight
  checks find the crown as the green blob (g > r + 8 and g > b + 12) on the tan soil of `3x3_check.json` (0.55, 0.42,
  0.28): the texture multiplies all three channels by the same factor, so the soil keeps r > g everywhere and never
  becomes "green". Perception that was trained on the Isaac tropical scene may still see a different background
  (texture, not grass).
- The canopy collider is a convex hull (a hull vertex limit of 255): it overestimates the foliage volume, so a ray that
  passes through a gap in the crown still counts as a hit. Off-target deposition is therefore a lower bound.
- Setpoints must carry a header stamp from `/clock`; an unstamped one (stamp 0) is stale for the controller and is ignored.
- At RTF > 1 a Noetic node that is slow in wall time lags in sim time (a 100 ms processing delay is 0.25 sim s at RTF 2.5).
  Use `--max-rtf` to hold the sim at what perception can follow.
- `/tf_static`, `/sim/truth` are replayed at 1 Hz because the bridge does not latch (Task 10); Noetic consumers wait up to 1 s.
- Images are best-effort through the bridge: 2-4 % of frames are lost (9.7 Hz received of 10).
- `/clock` starts at ~0.32 s (not 0): the first two app updates are warm-up and publish nothing; the clock is the graph time
  (see the moving-flight section). `/tm_sim_time_probe` (10 Hz Clock, not bridged) exists only to read that time.
- camera_info comes from a local subscription to the image topic (`CameraInfoRelay`): 119/119 images had an identical-stamp
  camera_info and RTF was unaffected, so it was not moved into the main loop.
