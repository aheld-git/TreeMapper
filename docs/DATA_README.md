# Recorded data for the tree-mapping build

All files are ROS 1 bags. Every bag embeds the full definition of each message type it contains, so it can be read without the original packages, for example with the Python `rosbags` library. Times are bag time: the simulator's clock in sim bags, the original flight time in real bags.

## 1. Simulator mission with ground truth (`isaac/`)

### `cloud_bundle_isaac_2026-10-23-15-34-29.bag` — 18.9 GB, LZ4-compressed, 278 s of sim time

This is one complete autonomous mission: take-off, approach, scan, planning, hopping between trees, spraying 7 trees, return and landing. The scene is a 3 × 3 grid of orange and lemon trees spaced 6 m apart with up to 1 m of jitter. The drone travels at 15 m and scans at 4 m/s.

| Topic | Type | What it is |
|---|---|---|
| `/cam/bottom/{left,right}/half/image_grey` | sensor_msgs/Image | downward stereo, half resolution, grey |
| `/cam/bottom/{left,right}/half/camera_info` | sensor_msgs/CameraInfo | stereo calibration |
| `/disparity_image_down_float` | sensor_msgs/Image (32FC1) | disparity, aligned to the left image |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | full transform tree, including the camera mount |
| `/clock` | rosgraph_msgs/Clock | sim clock |
| `/object_detection/detections` | spot_spraying_msgs/DetectionResult | 2D tree boxes from the existing detector (pixel x, y, width, height, label, score) |
| `/height_estimator/drone_position` | spot_spraying_msgs/DronePosition | drone position plus height above the world origin |
| `/drone_pose` | tree_hopping_msgs/TreeHopping | navigator-side drone state |
| `/mission_state` | std_msgs/String | IDLE, SCAN_APPROACH, SCAN, PLANNING, HOPPING, SPRAYING, RETURN |
| `/spray_target` | std_msgs/Int32 | current hop target tree ID (-1 = none) |
| `/validation/ground_truth` | spot_spraying_msgs/PolygonArray | **ground truth**: one entry per tree |
| `/validation/fusion_snapshot` | spot_spraying_msgs/PolygonArray | **the existing system's published map**: the baseline to beat |
| `/object_fusion/polygon_marker` | visualization_msgs/MarkerArray | the existing system's RViz view of the same map |
| `/object_fusion/active_target` | std_msgs/Int32 | which target the existing system was actually freezing on |

How to read the two PolygonArray topics:
- Each entry has an integer ID, a 2D outline (vertices), min, max and average height, a sprayed flag and a centroid.
- Ground truth outlines are regular polygons: one circle per tree from its simulated canopy radius. They are approximations. Do not tune your system to produce circles.
- Ground-truth heights come from each simulated tree's canopy top.
- **Frames:** ground truth is in the sim `world` frame (ENU). The existing system's map can be in `world_nwu`. Convert `world_nwu` to `world` with (x, y) → (−y, x).

### `sweep_000-e9d737ac_2026-10-23-15-34-29.bag` — 39 MB

The standard validation recording of the same mission. It repeats the ground truth and map snapshots and adds `/tree_hopping_markers`: the field, corridors and planned path as the navigator saw them. It is useful for plotting.

### Baseline: how the existing system scored on this mission

| Metric | Final map | Per-frame average over the mission |
|---|---|---|
| Trees in truth / in map / matched | 9 / 7 / 7 | — |
| Recall / precision | 0.78 / 1.00 | 0.50 / 1.00 |
| Mean outline IoU | 0.61 | 0.59 |
| Centre RMSE (xy) | 0.97 m | 0.93 m |
| ID switches | 0 | 0 |
| Merged neighbours (over-merge) | 4 | — |
| Fragmented trees | 3 | — |
| Outline spill outside true canopies | 0.13 | — |
| Height bias / RMSE | 1.41 m / 1.46 m | — |
| Trees sprayed | 7 of 9 (no revisits, no orphans) | — |

The two missed trees were absorbed into neighbours. The existing system ran at commit `5f608dc` of its repo.

## 2. Real flight, no ground truth (`real/`)

### `BB_treehopping_farm.bag` — 10.3 GB, 511 s

This is a recorded orchard flight. It contains real stereo (left/right grey at half resolution, calibration and disparity) and the transform tree. Flight-controller topics are also present; ignore them, or they may have been removed before upload. There is no mission phase, no hop target and no ground truth.

### `BB_treehopping_farm_companion_*.bag`

Recorded by replaying the real flight through the existing pipeline, so the stamps match the flight bag. It contains:
- `/object_detection/detections`: 2D detections from the existing detector;
- `/height_estimator/drone_position`;
- `/object_fusion/polygon_marker` and `/object_fusion/fused_bboxes`: the existing system's map, for qualitative comparison only.

**How to use it:** there is no ground truth, so score stability, safety and cost on it. That means ID stability, published geometry never shrinking or jumping, no undefined heights, and processing time. Also use it to calibrate your synthetic stereo noise against real stereo.
