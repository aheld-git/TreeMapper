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

**R14. Implausible heights are judged against the truth top when truth heights are reliable (M3, 2026-10-24).**
Both candidates showed the old test (finite height above the drone's altitude) flags accurate heights: the synthetic
descent hover can sit below a neighbouring canopy top (D30). Now, with reliable truth heights, an above-the-drone
height counts only if it is also above its truth top + `implausible_truth_tol_m` (0.5 m); truth top = the tallest truth
tree holding >= `own_min` (20 %) of the entry; an entry no truth tree owns keeps the plain test; `<= 0` always counts.
No truth, or unreliable truth (constant heights, the Isaac recording): unchanged. New field
`implausible_height_excused_entries`. Tests: `test_implausible_height_uses_truth_top_when_reliable`,
`test_implausible_height_merged_entry_uses_tallest_owner`. Effect (held-out suite, old -> new frames): cand_obj hard
624 -> 0, cand_grid hard 527 -> 0, simple unchanged (1605 hard + 65 medium). Existing-system baseline and the pinned
test: unchanged (Isaac truth unreliable); `results/baseline_existing_system.*` regenerated (label text only).

## M2 (2026-10-24, 15:20-; D19 onwards)

**D19. Input contract = one `Frame` per detection message** (`tools/treemapper_tools/frames.py`, mirrored by
`core/include/treemapper/frame.hpp`). A frame carries the receive time, the image stamp, the body pose in the
world at the stamp plus a validity flag, the phase string and the hop target. The target comes in navigation's
own ID space together with its descent point. It also carries the boxes and the disparity sampled on a
stride-4 grid (160 x 100), in full-resolution pixels. Calibration and `T_body_cam` are stored once per file.
- File form `frames.bin`: binary, about 32 kB per frame (u16 disparity x 256), readable from Python and C++.
  The full 1 MB disparity frames are never written; the recorded mission is 226 MB.
- Heartbeat frames (no boxes, no disparity) are written at every phase change and every 0.2 s while the detector
  sleeps, so a system can be driven by this stream alone.
- Alternatives: per-detection depth samples only (rejected: a spatial-evidence candidate or ground modelling
  needs points outside the boxes); full-resolution disparity on disk (rejected: 29 GB).
- Assumption: stride 4 loses nothing that matters at 7-15 m. A tree spans 100-300 px, so >= 600 grid points per box.

**D20. Disparity pairing.** About half of the bag's `/disparity_image_down_float` messages are malformed (step
= width, 256000 bytes of garbage). They are rejected by size and counted; no valid stamp occurs twice.
Detections are paired by exact stamp: 6268 + 89 late of 7414. 1010 more are paired to the nearest valid
disparity within 20 ms, with the pose taken at the disparity stamp so depth and pose stay consistent. The box
offset this causes is <= 3 px at 4 m/s. 47 detections have no disparity and are counted
(`frames_dropped_no_disparity` in the mapper).

**D21. Camera extrinsic = inv(world->body at spawn) * `/tf_static` world->camera.** This is verified against
ground truth: ground flat to 0.01 deg and 1 cm, 99.6 % of canopy points inside true canopies
(`results/geometry_check.md`). M1's guess, the raw transform used as the mount, tilts the camera by 3.37 deg.
Real flight: `/tf_static` has a proper `body -> sensor_down` (pitched ~75 deg about y). The real-flight reader
must use it. **Untested.**

**D22. Replay navigation emulator** (in `treemapper_replay`, not in the mapper). It resolves a recorded target to
the mapper's own ID via the descent point: the published outline containing it, else the nearest centre
< 3 m. When SPRAYING ends, it calls `markSprayed(resolved target)`. The resolved timeline goes to the scorer
as `mission.json` targets (`mission_resolved.json`).
- Alternative: score our map against the recorded IDs. Rejected: meaningless, because the IDs belong to another map.
- Consequence: on the recording our system flies the existing system's hop plan. No re-planning on our map.

**D23. Simple baseline (`SimpleMapper`), deliberately naive.** Per frame:
- points from the central 70 % of each box;
- gates: optical range 0.5-30 m, above drone - 0.3 m, below (per-frame 5th-percentile height + 1 m) = ground;
- one convex-hull observation per box (>= 25 points, area <= 60 m^2), height = p95 of point world z;
- association to the nearest entry within 2 m or containing the centroid;
- grow-only hull union, height = mean of observation heights;
- published after 3 observations; unconfirmed entries expire after 2 s unseen;
- in creation phases only, a containment merge (smaller centroid inside the larger, overlap >= 30 % of the
  smaller); the better-established entry (published > more observations > lower ID) keeps ID and height.

Phase rules live in the pure `phasePolicy()`:
- SCAN / SCAN_APPROACH / ERROR / unknown: create and merge. Unknown is fail-open with a warning.
- PLANNING: no creation, no merge, existing trees may grow.
- HOPPING: additionally only the resolved target may change. No valid target freezes everything (fail closed).
- SPRAYING / IDLE / RETURN: asleep.
- An invalid pose holds everything (fail closed).
- Unconfirmed entries cannot be confirmed in a locked phase. This was a bug found by the synthetic suite: an
  entry created in SCAN reached 3 observations in PLANNING and was first published in HOPPING. Fixed, with a test.
- Height = world z of the canopy top (the descent reference in navigation's frame), not height above a ground
  estimate. On Isaac and on the synthetic data the ground is at about 0. Real flight: world z origin = take-off
  point (assumption).

**D24. Ground-truth heights of the Isaac recording are not usable.** In-canopy points reach +1.35 m (p90) and
+1.88 m (p99) above the constant 4.0 m truth. The coordinator and the scorer fixer handled this (R13:
`gt_height_reliable`). Height rows on the recording are n/a in M2's scorecards.

**D25. Synthetic generator** (`tools/treemapper_tools/synth.py`). Heightfield world at 5 cm with irregular dome
canopies, ray-cast disparity on the same stride-4 grid, three levels. Noise anchored to the recordings
(`results/synthetic_calibration.md`): real stereo is ~0.1 px per pixel, 2-3 % dropouts, 1/16 px quantisation.
- easy / medium / hard = 1.5x / 3x / 5x real disparity noise, plus correlated error, fattening, dropouts and
  outliers, each multiplied 2-4x in corridor turns.
- The detector simulation is far worse than the recorded one: 8-25 % misses, merged boxes over touching trees,
  partial boxes, 0.1-0.5 false positives per frame.
- Frames at 15 Hz while awake, recorded rate ~27 Hz. A cost saving: generation 2 min 40 s for 15 missions on 6
  processes, 438 MB.
- Receive time = stamp, so the phase in a frame is the phase in force when the system sees it. With a 30 ms
  latency, a SCAN frame could arrive after PLANNING began and the scorer would see an unfair new-tree violation.
- Seeded: same (seed, level) -> byte-identical files (test).
- Hard is a stress level: 5x noise at 15 m gives sigma_Z ~ 1.5 m, 6 m in turns. Medium is the main
  discriminating level.

**D26. Noise floor.**
- The C++ systems and the generator are deterministic. Repeated runs give identical scorecards, so the
  run-to-run floor is 0. The meaningful floor is seed-to-seed variation; the evaluation tables report std over
  datasets and per-level mean / worst.
- Candidates must be compared paired by seed.
- Isaac: one run only. The existing system was recorded once. Our replay is deterministic, so the number is
  exact but statistically one sample.

**D27. Environment.** Nothing installed in tm_dev or on the host. numpy, scipy, shapely and rosbag were enough.

**D28. The replay writes an extra snapshot at the last frame of each phase**, besides the steady 5 Hz.
- Why: the snapshot after a phase change carries the new phase's time. Without the extra snapshot, a tree
  published by the last SCAN frame first appears in a PLANNING-time snapshot and is scored as a new tree after
  scanning.
- This is a cadence artefact. The tree did exist when navigation started planning.
- Steady-rate snapshots that would fall before the extra one are skipped, so the stream stays monotonic.
- The scorer's per-frame metrics are time-weighted (R8), so the extra snapshots do not bias them.

**D29. Mapper interface + registry, so candidates develop in parallel without shared edits.**
- `treemapper::Mapper` (`core/include/treemapper/mapper.hpp`): `process`, `publishedTrees`, `markSprayed`,
  `takeStageTiming`, `counters`, optional `setLogger` -- exactly what `treemapper_replay` called on `SimpleMapper`.
  `SimpleMapper` implements it unchanged (verified: map streams byte-identical, PROGRESS.md).
- Configuration is a `string -> string` map (`MapperConfig`) given to the factory, not a Mapper method. `ConfigReader`
  parses typed values and `finish()` rejects unknown keys, so a typo cannot silently run the defaults. Replay:
  `--config "k=v;k=v"`; evaluate: `--set SYSTEM.key=value`.
- Registry in `core/src/mapper_registry.cpp`, one line per system. The candidate defines
  `tm_create_<name>()` in its own .cpp; the line is `TM_REGISTER_CANDIDATE(<name>)`. The reference is weak
  and the candidate objects are linked straight into the executables (OBJECT library `treemapper_registry` +
  `tm_link_registry()`), because the linker never pulls a static-archive member in for a weak reference. Chosen over
  static self-registration (same archive problem) and over a CMake-generated registry (the line would not
  live in a file the candidate owns).
- Why weak: `-DTM_CANDIDATES="cand_a"` (default `all`) builds only the named candidate directories, and the
  registry line of a switched-off candidate registers nothing. Two agents sharing one checkout cannot break each
  other's builds with a half-written file.
- `evaluate.py` validates system names against `treemapper_replay --list-systems` up front and adds a paired
  comparison (per seed, against `--reference`) when given 2+ systems, because run-to-run noise is 0 and
  seed-to-seed variance dominates (D26).

## M3 selection (2026-10-24, 18:00-19:00; D30 onwards)

**D30. Known generator flaw (for M4): the descent hover clears only the target.** `synth.py` flies the spray hover at
z = max(ground under the target + 7 m, target top + 1.5 m). A neighbour that is taller or stands on higher ground can
top out above the hovering drone (hard_s100 at 61.4 s: drone 7.31 m, neighbour top 7.48 m within 8 m; hard_s103 at
103.9 s: 6.28 m vs 6.79 m). Real navigation would not fly that. R14 makes the scorer robust to it; the generator itself
is not changed here and no suite was regenerated. M4: hover z = max over trees within the footprint / rotor reach of
(top + clearance), then generate a fresh suite (leave `suite_holdout` untouched).

**D31. M3 selection: cand_obj is hardened in M4 (close call).** Held-out suite (30 missions, seeds 100-109, never
seen by either candidate) + the recording, frozen configs, paired by seed (`results/m3_comparison.md`).
- Safety: 0 for both everywhere (simple 1670 frames on 11 held-out missions; existing 5 on the recording).
- Medium: within seed noise. cand_obj recall +0.016 (3/0); cand_grid spill -0.016, min IoU +0.10, height tail
  (0 vs 4 trees off by > 1 m). Swallowed, false trees, centre: ties.
- Decided by: hard worst cases (cand_obj recall +0.108, t 4.8, 9/1; swallowed 3.6 vs 5.3; false trees 0 vs 0.8);
  the descent reference on the recording (cand_grid's locked heights sink by up to 1.75 m, its target moves 0.64 m in
  its own descent; cand_obj <= 0.03 m); easy / Isaac outline quality; latency -0.39 s; bounded memory; simpler code.
- Conceded to cand_grid: publication stability (centre jumps 0.7 vs 7.4 per medium mission, height jumps 0 vs 1.1),
  the medium height tail, and range / off-axis / rotation weighting. These are the M4 work list (results/m3_comparison.md).
- Tooling: `tools/treemapper_tools/m3_compare.py` merges per-level evaluate outputs and adds the cand_grid vs cand_obj
  pairing from the same runs (no re-run), plus height-error tails and descent-height ranges.

