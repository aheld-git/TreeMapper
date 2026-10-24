# Decision log

Each entry: decision, alternatives, evidence, assumptions. Newest at the bottom.

## M1 (2026-10-24, 15:00–)

**D1. Execution environment: everything in `tm_dev`.** At the start of M1 I created a host venv under
`work/venv` (rosbags, numpy, shapely, pytest via pip). The coordinator then made `tm_dev` mandatory; the
venv was deleted (`rm -rf work/venv`) before anything else ran from it, and nothing else was installed on
the host. Nothing was pip-installed in the container either: it already has rosbag, numpy 1.24, shapely 2.0,
scipy 1.10, pytest 8.3, gtest sources, g++ 9.4, CMake 3.16 (the Orin's Noetic/20.04 stack). The real
`rosbag` API is used for extraction. Requirements listed in `tools/requirements.txt`.

**D2. Repo layout.** `core/` plain-CMake C++17 library (`treemapper_core`, also has a `package.xml` with
`build_type cmake` so catkin can build it in M4); `ros/treemapper_msgs/` catkin interface package;
`tools/treemapper_tools/` Python 3.8 package (extraction, contract I/O, scorer, later the synthetic world);
`scripts/` one-command entry points; `results/` committed scorecards. Build trees go to `/data/work/build`,
extracted data to `/data/work/extracted`, never into the repo. Alternative (single catkin workspace at the
repo root) rejected: it forces catkin onto the ROS-free core and tests.

**D3. Contract file format = JSON Lines map stream** (one snapshot per line: id, outline, centre, height,
sprayed, optional `stage_ms`). Undefined height is written as `null` (JSON has no NaN) and read back as
undefined; NaN/inf are converted on write. Any system (ours, the existing one, a replay) only has to write
this. Alternative: rosbag of TreeMap messages — rejected for the scorer because it would tie scoring to ROS.

**D4. Existing system's published height = `max_height`.** It is the canopy top that the existing system hands
navigation as descent reference; it reproduces DATA_README's height bias/RMSE exactly (1.41 / 1.46 m).

**D5. Ground truth = the recorded 12-gon outlines**, centre = vertex mean (the message's `centroid` is unset,
zero), radius = mean vertex distance, height = `max_height` (4.0 m for every tree). The truth never changes over
the bag (1 distinct version in 557 messages) and matches the `/tf` `world->Tree_XXXX` positions. A true-circle
truth changes IoU by < 0.01 and spill by 0.02; the recorded polygons are used as given.

**D6. Frames.** Every non-empty `/validation/fusion_snapshot` is already in `world`; only the empty
pre-mission ones say `world_nwu`. The extractor converts per message ((x, y) -> (-y, x)) regardless, and raises
on an unknown frame (fail loudly).

**D7. Time base = bag receive time** of each message (same recorder for map, phases, targets and pose). The
sweep bag repeats identical snapshot content with receive times up to 81 ms different — the order of the
noise in any time-based metric.

**D8. Matching and detection definitions** (thresholds in `ScoreConfig`): Hungarian one-to-one on IoU >= 0.10;
cover >= 30 % of the truth area; own >= 20 % of the entry area. Alternatives: centre-distance gating
(fails on merged entries whose centre sits in the gap), greedy IoU (order dependent). Recall/precision match
DATA_README exactly with these.

**D9. Per-frame window = every snapshot of the stream** (empty ones score recall 0), with a second variant from
the first publication. Evidence: only the "every snapshot" window reproduces DATA_README's per-frame recall 0.50
(first publication on: 0.63; SCAN..RETURN: 0.58).

**D10. Locked phases = PLANNING, HOPPING, SPRAYING** for "new trees after scanning". The mission scans twice
(SCAN -> hop/spray -> SCAN_APPROACH -> SCAN again), so "after scanning ends" cannot be "after the first scan";
SCAN_APPROACH and RETURN are not locked. Assumption: creating trees during RETURN is harmless (navigation is
done); revisit if navigation ever re-plans during RETURN.

**D11. Descent window.** `/mission_state` has no DESCENT value and hop transit flies at ~10.5 m, below the 15 m
scan altitude, so altitude alone cannot separate transit from descent. Window = SPRAYING frames + HOPPING frames
within 1.5 m (xy) of where the next SPRAYING starts, only while a target >= 0 is set. Transit is not judged
(the brief lets existing trees grow while hopping). Without pose every HOPPING frame is judged (stricter).
On the recording: 370 judged snapshots.

**D12. Spray outcome via entry centre inside the true canopy** (navigation flies to the centre). Coverage-based
counting called all 9 trees sprayed because the two merged entries cover two trees each — physically wrong;
the centre rule gives DATA_README's 7 of 9.

**D13. ID switches reported total and after the list locks.** The single switch in the recording happens in
the first scan when fragment ID 2 merged into ID 1 (the brief's rule: the better-established entry keeps its
identity); before planning nobody holds IDs. Both numbers are kept; "after lock" is the navigation-relevant one.

**D14. A sprayed tree absorbing the hop target counts as a safety violation** (rule 5: sprayed trees stop
changing) even though the existing system does it on purpose. It also makes the target ID vanish while
navigation holds it; the scorer reports that separately (`ids_vanished_while_held_by_navigation`).

**D15. Timeliness footprint = image half-diagonal per metre of range above the canopy** (yaw-free; the pose has
no camera yaw). The inscribed-circle alternative produced negative latencies (trees published before
"in view"). Latencies are therefore upper bounds of the true delay. Reported from first view in any phase and
from first view while SCAN (the existing detector sleeps during SCAN_APPROACH).

**D16. Extracted now:** truth, map, phases, targets, active target, drone position, 2D detections, world->body TF,
tree TFs, camera info (1 min for the 19 GB bag, ~4.4 MB output). Images/disparity not extracted (large; read
from the bag when needed).

**D17. Core C++ dependencies: standard library only for now.** gtest from `/usr/src/googletest` (sources only on
20.04). Eigen 3.3 and CGAL 5.5 exist in the Noetic image and are acceptable later; anything else must be checked
for arm64 availability first.

**D18. Interface package defines its own input types** (`Detection2DArray`, `DronePose`) and documents that the
node must adapt the existing `spot_spraying_msgs/DetectionResult` and `std_msgs/String` `/mission_state`,
`std_msgs/Int32` `/spray_target` used in the bags. `treemapper_msgs` was test-built in a scratch catkin
workspace (`/data/work/catkin_m1`), not yet integrated.
