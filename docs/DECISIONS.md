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

## Scorer review fixes (2026-10-24, ~15:30-; series R, kept separate from the milestone D numbers)

An independent review (`/data/work/review_scorer/adv*.py`) found defects that let a system look better than it is.
Each fix has a regression test in `tools/tests/test_score_review.py`. 40 of its 41 tests fail on the pre-fix
scorer; the 41st is a duplicate-ID guard that the old code passed by luck. `tools/tests/test_score_baseline.py`
pins the recomputed existing-system scorecard.

**R1. Stability is publish-rate independent.** Retreat is measured against each ID's union of every outline it
has ever published (not the previous snapshot). Drift is reported per ID from its first value and from its value
at lock. Jumps are > 1 m within a 1 s window (plus the last value). Retreats and jumps are counted once per onset.
Supersedes the consecutive-snapshot part of D8/D11-era definitions. Alternative: per-update deltas (rejected: a
1.5 %/snapshot shrink lost 78 % of a tree and scored 0 retreats).

**R2. Sprayed trees are judged against a record taken at first spray.** A sprayed ID vanishing, losing the flag,
or differing (outline, centre or height) is a violation. New-inside-sprayed tests against the union of every
outline ever sprayed, by centre or >= 30 % of the new entry's area. Spray outcome is computed over the whole
stream (centre at spray time, distinct IDs per truth tree), replacing D12's final-map-only count. The D12
centre rule itself stands.

**R3. Descent window includes the climb out.** After a SPRAYING ends, every snapshot until the drone first
leaves `descent_radius_m` (1.5 m) of that hover point is judged, whatever the phase. I chose distance, not
altitude, for symmetry with D11's approach rule: over the tree the drone is low whatever `/mission_state` says.
The alternative, "below transit altitude near any spray point", needs a per-hop transit altitude
(10.8 m / 10.4 m here) and would let the hover at transit altitude go unjudged. Consequence: the existing
system's 145.98 s event (sprayed tree 7 absorbs target 8) is inside the climb (z = 8.4 m, 0.1 m from the hover
point), not in hop transit as the M1 notes said. With the target gone, the freeze failed open, and tree 1
changed twice before the drone left. 3 non-target descent changes.

**R4. Missing inputs are n/a, never a pass.** Phases without a target series -> descent metric `None` with a
note. I chose this over judging with every tree as non-target, which would invent violations from absent
data. No phases -> every phase metric `None` (as before).

**R5. Invalid entries are a safety count** (`invalid_entry_frames`: non-finite centre or outline coordinate,
< 3 vertices, zero area). They count as published and never match. Valid -> invalid is a retreat with lost
fraction 1.0. A non-finite height stays in rule 3's own `undefined_height_frames`, so one defect is counted once.

**R6. Undefined heights cost the worst case.** In quality metrics an undefined height on a matched entry counts
in `n_undefined_height` and enters height RMSE / max-abs as 15 m (about scan altitude). Height jumps compare
against the last finite value. Before this, publishing `null` for two bad trees improved height RMSE from 2.45
to 0.10 m.

**R7. First publish for timeliness = first one-to-one match with a finite height.** Negative latencies are
listed and clamped to 0. Before, a single giant polygon covering every tree from t = 0 earned -21 s mean latency.

**R8. Per-frame means are time-weighted.** The latest snapshot is sampled every 0.2 s over first SCAN .. RETURN
(or end). Replaces D9's snapshot-weighted "every snapshot" window. That window reproduced DATA_README's 0.50
only because it counts 287 empty pre-mission snapshots, and it rewards publishing faster when the map is good.
Existing system: 0.58 (window), 0.59 (from first publish).

**R9. ID reuse is decided geometrically.** A reappearing ID whose outline overlaps its pre-gap outline by
< 10 % (intersection / smaller area) is reused, with or without ground truth. In a locked phase it counts as a
new tree (`new_trees_after_scan`). The owner-based count is kept as `id_reuse_by_owner`.

**R10. Hungarian matching zeroes sub-threshold IoUs before assignment.**

**R11. Frames are checked.** `read_ground_truth` returns a list that carries the file's `frame`, and
`score()` raises `ValueError` if any snapshot's frame differs. Without ground truth, one frame per stream.

**R12. Duplicate IDs in one snapshot are a safety count.** Per-ID change checks compare the whole multiset of
entries with that ID.

**R13. Hygiene and additions.** `read_mission` and `score()` sort every series by time. A degenerate truth
outline falls back to a circle (radius, else mean vertex distance), or raises. Also new:
- `locked_id_vanished` (safety; with `_targets`);
- target height range during its own descent (informational);
- `implausible_height_frames` (safety: height <= 0, or above the drone inside the mission window);
- `recall_in_view` (denominator = truth trees already in the camera footprint);
- no-ground-truth mode (`--gt` optional: stability + safety + cost);
- GT height reliability: constant truth heights or `"height_reliable": false` mark height-error metrics
  "GT height unreliable (constant fallback)", kept out of headlines and pass/fail. This is the coordinator's
  request after M2's finding that the Isaac truth is a 4.0 m fallback. Height stability metrics stay fully
  weighted.

Existing-system scorecard: 1 -> 5 safety violations, all from the 145.98 s absorption. The full old -> new table
is in `results/baseline_existing_system_notes.md`.
