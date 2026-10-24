# Decision log

Each entry: decision, alternatives, evidence, assumptions. Newest at the bottom.

**Numbering (consolidated in M5).** One monotonic series **D1-D57** for the build, plus the separate scorer-review series
**R1-R14** (the scorer belongs to its own owner; results files cite R13 / R14 by those numbers, so they are kept). The ROS
entries written as D-R1..D-R7 are D41-D47 (each heading keeps its old name). M4a's hardening decisions were recorded in
`docs/candidates/cand_obj.md` section 10; they are D32-D39 here, each citing its subsection, which keeps the full evidence.

| Milestone | Entries | Topic |
|---|---|---|
| M1 | D1-D18 | environment, repo layout, contract format, scorer definitions, bag facts |
| scorer review (M1-M3) | R1-R14 | scorer fixes found by an independent review; R14 truth-aware implausible heights (M3) |
| M2 | D19-D29 | input contract, pairing, camera geometry, replay emulator, simple baseline, generator, noise floor, Mapper registry |
| M3 | D30-D31 | generator hover flaw, the selection of cand_obj |
| M4a (cand_obj hardening) | D32-D39 | height model, rate limits, skirt trim, weighting, seed guard, tests, held-out verdict |
| M4d (generator) | D40 | hover clears every canopy; suite_v2 |
| M4b (ROS wrapper) | D41-D47 | core linkage, config pointer, pairing, gates, emulator, frame time, real launch defaults |
| M4c (real flight) | D48-D54 | real frame reader, camera chain, disparity offset, no-GT scoring, existing-map conversion, real profile |
| M5 (handoff) | D55-D57 | final evidence sources, verification, what ships |

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

## M4a: cand_obj hardening (2026-10-24, 19:00-20:00; D32-D39)

Recorded by M4a in `docs/candidates/cand_obj.md` section 10 (the full tables and reasoning stay there); consolidated
here in M5. Tuned on suite_v1, suite_v2 and the recording; the held-out suite was run once at the end with the frozen
configuration (`results/cand_obj_m4_holdout.md`, `results/cand_obj_m4_isaac.md`, `results/cand_obj_m4_synthetic.md`, all
paired against the frozen M3 build of commit 89317b2, cand_grid and simple).

**D32. Height = 0.99 quantile of a fused latched surface, not the median of crown-top samples** (cand_obj.md 10.1).
M3's median of per-frame crown-top samples published two Isaac trees at 4.27 / 4.39 m against measured tops of 5.4-5.9 m:
trees entering at the swath edge filled the median with the first visible slice's "top". Now each latched cell carries a
weighted running mean of its per-frame median z (cand_grid's `fuseCell`: weight cap 30, 2 m outlier gate from weight 3,
three rejections re-initialise), and the tree height is the 0.99 quantile over latched cells with a height. 0.97
(cand_grid's) biased easy -0.08 m; 0.99 gives -0.04 / +0.01 / +0.06 m (suite_v1) and 4.83-5.74 m on the recording. On a
merge the loser's cell heights are folded into the winner's cells, which can only raise the quantile (safe direction).

**D33. Heights are fused in creation phases only** (cand_obj.md 10.1). Once the list is locked the descent reference
does not move (except the rate limiter finishing a step in progress). Effect: target height range during its own
descent 0 on every held-out mission (M3 up to 0.62 / 1.79 m medium / hard), height drift since lock 0 on the recording.
Accepted cost: synthetic missions re-enter SCAN for a second corridor, where heights are fused again by design
(max drift since the first lock 0.82 / 1.21 m medium / hard).

**D34. Published centre and height are rate-limited** (cand_obj.md 10.2): <= 0.8 m/s and <= 0.5 m/s, one step per frame,
dt capped at 0.5 s, only for instances the phase policy lets change. The outline is not delayed, so first publication is
unchanged. Centre / height jumps went to 0 on every suite, the recording and the real flight. The alternative, requiring
an observed peak neighbourhood before publication, was not adopted (latency, and the jumps were already gone).

**D35. Relative-height skirt trim (`latch_rel_h`) implemented but shipped off (0)** (cand_obj.md 10.3). It trims spill
(medium 0.166 -> 0.121 at 0.3) but the per-frame peak is a noisy maximum on hard, so swath-edge trees never reach
`min_publish_area`: hard recall 0.82 -> 0.55. cand_grid applies it to a multi-frame surface, where it is cheap.

**D36. Range / off-axis point weights and a rotation-rate frame quality, at `rot_rate_ref` 1.0 rad/s** (cand_obj.md 10.4).
Point weight `(z_ref/Z)^4 / (1 + 10 rho^2)` scales height contributions; frame quality `1/(1+(rate/ref)^2)` with 1.5 s
recovery scales height weights and evidence increments. cand_grid's 0.35 rad/s cost 0.1-0.2 s latency everywhere for no
geometry gain; switching the weighting off was faster but produced split trees on easy (turn-blur fragments). A split tree
is a double target, 0.1-0.3 s is not, so the weighting stays on.

**D37. Overlap matching with a reach limit, plus a seed guard** (cand_obj.md 10.5). A crown covering >= 50 % of an
instance's latched cells with its top within 3 m is that instance; among several, the better-established wins. A crown
that would seed a new instance is credited to a published tree when >= 25 % of its cells lie on its latched cells. Fixes
the Isaac transient duplicate (ID 11 -> 12 -> 11 at 178 s); without the guard the overlap rule alone made it permanent
(precision 0.9). Reach 3 m, not 4 m: hard recall 0.83 vs 0.78. Test `FragmentNeverTakesOverAPublishedTree`.

**D38. Tests and configuration** (cand_obj.md 10.6). 20 cand_obj tests (5 new in M4). Config grew to 45 keys,
`config/cand_obj.cfg` == built-in defaults (test); `height_buffer` removed, `min_height_samples` renamed `min_height_cells`.

**D39. Held-out verdict: ship M4, accepting a small recall and latency loss** (cand_obj.md 10.7). Against the frozen M3
build: mean IoU +0.024 / +0.028 (medium / hard, 10/0), centre RMSE -0.082 m medium (9/1), centre and height jumps to 0,
hard swallowed 3.6 -> 2.8; but recall -0.018 medium (1/3/6, within seed noise) and -0.035 hard (2/6/2, t -2.2), latency
+0.18 / +0.27 s. The lost trees were never confirmed (frame weighting delays latching at row ends; the seed guard
sometimes credits a neighbour's first slice at 4.5 m spacing), not merged away. Against cand_grid the hard recall
advantage is kept (+0.073, 8/0). Stable descent references and 0 jumps were judged worth more than 0.02-0.04 recall.

## M4d: generator fix (2026-10-24, 19:00-; D40)

**D40. Generator hover clears every canopy in the footprint (fixes D30).** `synth.py` used z = max(ground + 7,
target top + 1.5) for the spray hover, which only cleared the target. Now the transit, descent and hover use
`z >= max top of every tree whose canopy lies within FOOTPRINT_R = 8 m of the drone + CLEARANCE = 1.5 m` (the planner adds
PLAN_MARGIN = 2 m so the 1 s path smoothing cannot cut a corner into a canopy). Transit: the whole target-bound path is
checked at 1 m steps; hop height >= hover height + 1 m. Climb-out: above the highest canopy + 1.5 m (a no-op while
the 15 m scan altitude already clears them all). The footprint is 8 m because D30's evidence (hard_s100 at 61.4 s) was a
neighbour within 8 m; an air-frame needs far less, a nadir camera at 7 m sees about 5 m, so 8 m is the generous choice.
- **Nothing else changed, by construction.** The legacy plan is still built (same RNG draws, same segment sample counts)
  and flown as a *shadow*: it keeps the main RNG stream exactly where it was (stereo noise, pose noise, detector
  draws), and the fixed trajectory replays its segment sample counts, so frame count, timing and phases are identical.
  The hover only gets higher, so descents are a little faster than 1 m/s. Frames whose true pose differs (only z) are
  rendered at the new pose with the same stereo noise field; their detector draws come from a separate stream
  (`[seed, level, 32]`) with the sticky miss/merge state copied from the shadow detector, so the main stream, and with
  it every later frame, is untouched. Proof (`hard` seed 100, full size, against the pre-fix generator): world and
  `ground_truth.json` and `camera_info.json` byte-identical; frame count, times, phases, targets identical;
  746 / 1365 frames bit-identical (all of SCAN, PLANNING, IDLE, the heartbeats after the hops); the 619 that differ are
  HOPPING (479), SPRAYING (120) and the RETURN climb-out (20), same noisy x, y and attitude, z higher by 0..1.67 m.
  `overrides={"legacy_hover": True}` reproduces the pre-fix output bit for bit (test pin on the decoded frames).
- **suite_v1 and suite_holdout are untouched** (frozen references, flawed hover and all). Scores on them stay
  comparable among themselves; suite_v2 numbers are not comparable with them. New suite: `/data/work/synthetic/suite_v2`,
  seeds 200-204 x easy/medium/hard, 15 missions, 444 MB (27-37 MB each), the tuning suite from now on.
- Tests (`tools/tests/test_synth_hover.py`): no canopy in the footprint above (drone z - 1.5 m) during the descent, hover
  and climb-out for 3 seeds x 3 levels (> 500 samples each); the fix only raises altitude on the same timeline; the D30
  flaw exists in the legacy plan and not in the fixed one; the legacy flag equals the pre-fix generator and the fixed
  output differs only in the moved frames. The existing reproducibility test compares two runs and has no pin to update.
- Check on the generated suite_v2 from `mission.json` + `ground_truth.json` alone: smallest (drone z - tree top) over all
  trees whose outline is within 8 m, HOPPING and SPRAYING: 1.50 m on 15 / 15 missions (hover z 6.4-9.9 m).
- Baseline on suite_v2 (`simple`): implausible-height frames are 0 on all easy and medium missions and 434 on four hard
  ones (100 / 168 / 117 / 49). They are not generator artefacts: every flagged entry has no owning truth tree and a
  height 0.5-2 m above the highest truth top in the world (e.g. 8.67 m vs 8.19 m), the simple baseline's merged / false
  entries. The drone clears every truth canopy by >= 1.5 m, the scorer's own definition of a real error.

## M4b: ROS wrapper (2026-10-24, 19:00-20:15; D41-D47, written as D-R1..D-R7 and renumbered in M5)

**D41. (formerly D-R1) The node consumes the core by `add_subdirectory`, statically linked.** The registry reaches candidates through
weak symbols, so executables must take the registry's object files (`tm_link_registry`); an installed static core would
silently drop every candidate. The core is located through the package's real path (`../../core`) or `-DTM_CORE_DIR`.
No change to core/CMakeLists.txt.

**D42. (formerly D-R2) Mapper config: the yaml points at `config/<mapper>.cfg`, it never copies values.** One source of truth for the
replay, the evaluation and the drone; overrides exist (`mapper_overrides`) but are logged as WARN and must be empty in
the shipped yaml (test). Unknown keys still fail at node start (dry-run `createMapper`).

**D43. (formerly D-R3) Pairing identical to the offline reader, released in detection order, plus early resolve.** Order keeps the
mapper's input sequence equal to the replay's; early resolve (a newer disparity than stamp + 20 ms has arrived) avoids a
0.5 s stall per unpaired detection (+0.35 s scan latency before) without changing which disparity a frame gets. It
assumes in-order disparity stamps (one publisher); `sync_early_resolve: false` turns it off.

**D44. (formerly D-R4) Gates on missing information (rule 6).** Calibration, mount, pose: fail closed (no mapper / held frames,
counted, health HOLDING or FAULT). Mission phase missing or unknown: fail open (scanning), warned once, listed in
health. `set_tree_sprayed(false)` is refused (sprayed trees are protected). Heartbeat on a wall timer so it survives a
paused sim clock; SLEEPING in IDLE / SCAN_APPROACH / SPRAYING / RETURN, frames still reach the mapper (its phase policy
decides), so the ROS path equals the offline replay.

**D45. (formerly D-R5) Replay-only navigation emulator.** A recorded `/spray_target` is in the recorded system's ID space; feeding it
to the node would freeze the wrong tree. `replay_isaac.launch` remaps it and `nav_emulator.py` resolves targets by
descent point exactly like `replay_main.cpp`, and marks trees sprayed at the end of SPRAYING.

**D46. (formerly D-R6) Frame time is monotonic and the replay clock runs at 1 kHz.** cand_obj's rate limits integrate over frame `t`;
rosbag play's 100 Hz clock and a phase heartbeat fed before older pending frames made the live run diverge
(latency 2.80 vs 2.32 s). Fixed in the node; on the drone the wall clock is continuous anyway.

**D47. (formerly D-R7) Real launch defaults follow M4c's data-derived chain** (mount pitch 82 deg, not tf_static's 75; disparity
offset 4.30 px, not camera_info's 4.92), both from one flight without ground truth; `camera_mount_source:=tf` and
`disparity_offset:=auto` restore the recorded values.


## M4c: real flight (2026-10-24, 19:00-; D48-D54)

M4c recorded its choices in `docs/REAL_FLIGHT.md` and the result files it owns (`results/real_flight_*`); they are
consolidated here by M5 with citations. The real flight has no ground truth, no mission phase and no hop target.

**D48. Real-flight frame reader = flight bag + companion bag, one frame per companion detection message**
(`tools/treemapper_tools/real_frames.py`, `docs/REAL_FLIGHT.md` "What the reader does"). `t` = companion bag time (the
same clock as the flight bag's bag time, the map stream and `mission.json`); `stamp` = header stamp + one constant clock
offset (+6554596.89 s: header stamps are flight time, Oct 2025, the bags were re-recorded in Jan 2026). Pairing as D20
(exact header stamp, else nearest within 20 ms, else no disparity, counted); pose from `/tf` `world_nwu -> body` at the
disparity stamp, a missing pose gives `pose_valid = false` and a counter. Phase = constant SCAN, target -1, so the lock,
descent and spray rules cannot be exercised on this flight (the scorer reports them as n/a). Result: 7164 frames, 7145
paired exactly, 12 without disparity, 7 heartbeats, 0 invalid poses (`results/real_flight_geometry.md`).

**D49. Mount pitch 82 deg about body +y, not tf_static's 75 deg** (`results/real_flight_geometry.md`). Chosen from
competing camera-chain hypotheses (A-F plus a pitch sweep) by ground tilt and by crown position drift between passes at
different altitudes: with 75 deg the ground tilts ~7 deg and crowns move about 0.15 m per metre of altitude along the
heading (about 1 m between the 11.5 m and 18.5 m passes). The cause may be the mount itself or a constant attitude bias
(the body reads +3.2 deg nose-down while hovering); the stereo fits 82 deg either way. Lever arm (0.30, 0.08, -0.20) m
from tf_static, unverified (below the noise); roll unverified beyond +-2 deg. `--mount-pitch-deg tf` / the launch's
`camera_mount_source:=tf` restore the recorded value. This is also D47's launch default.

**D50. Disparity offset 4.30 px, not camera_info's cx_l - cx_r = 4.918 px** (`results/real_flight_geometry.md`,
`results/real_flight_stereo_check.json`). A pose-based motion test puts the depth scale at 0.99 with 4.30 px; with 4.918
depths come out 7-16 % too long. Likely cause: the raw images are not rectified, so the disparity node rectified
differently from camera_info. Baseline 0.1606 m from right `P[3]` in millimetres (same unit convention as Isaac).
`--disparity-offset calib` / the launch's `disparity_offset:=auto` restore the calibration value.

**D51. No-ground-truth scoring of the real flight** (`tools/treemapper_tools/real_score.py`, `results/real_flight_scorecard.md`).
The scorer's no-GT mode (R13) gives stability, safety and cost; tree-level quality is judged against a **proxy**: stereo
crown peaks (local maxima of the stereo canopy-height model, CHM > 1.5 m, at least 3 m apart; 63 on this flight), with
"outlines with 0 peaks" = fragments / false trees and "2+ peaks" = merged crowns, plus agreement between the systems'
final maps. Every build is a clean `git archive` export so uncommitted work cannot leak in. Known artefact: about 57
"above the drone" implausible-height frames per system come from the landing (t > 500 s, drone below 7.5 m over 7-8 m
crowns) because a stream without phases has no end of mission window; not a mapping error.

**D52. The existing system's real-flight map is reconstructed from its RViz topics** (`tools/treemapper_tools/real_existing.py`,
`docs/REAL_FLIGHT.md`). Per MarkerArray the `ns=polygon` LINE_STRIPs are outlines (ID = marker ID); height = the
`/object_fusion/fused_bboxes` median feature height of the same ID (the only height in the companion bag), so its heights
are not comparable in absolute terms with our canopy tops; sprayed = false; frame world_nwu. Its own chain and
configuration in the companion replay are unknown (presumably tf_static's 75 deg and the 4.918 px offset).

**D53. A separate real-flight profile, `config/cand_obj_real.cfg`, chosen by a no-ground-truth sweep** (`results/real_flight_profile.md`,
commits af11268 / ba18073). The sim profile over-fragments the farm flight (102 trees for ~63 stereo crowns, 39 outlines
without a peak). The cand_obj source was frozen, so the fix is configuration only: 18 configurations in two rounds (M4a's
suggested knobs, then instance-level knobs on the best), judged on outlines without a peak, merged outlines, missed peaks,
neighbour spacing, fragment areas, stability, safety and agreement with the other systems. `min_basin_cells` had no effect;
`min_prominence` alone left 16+ fragments and missed 6 crowns; the decisive key was `merge_overlap` 0.5 -> 0.3 (real crowns
split into instances overlapping by 30-50 %). Chosen "P1.0_all" (9 keys: `min_prominence` 1.0, `min_basin_cells` 8,
`seed_truncated` false, `match_radius` 3.0, `match_reach` 4.5, `seed_overlap` 0.1, `min_publish_area` 8.0, `merge_overlap`
0.3, `merge_centre_dist` 2.5): 63 trees, 6 outlines without a peak, 5 peaks missed, 0 jumps. The equivalent "P_all"
(prominence 1.5; 61 trees, 5 fragments) lost on its higher prominence (more missed small trees expected) and drift (3.2 vs
2.6 m). `cell` 0.6 also helped (8 fragments) but was not taken: it changes the whole height model.

**D54. Two profiles, selected by launch file, not one compromise** (`results/real_flight_profile.md`). The real profile
regresses on the synthetic held-out suite (recall 0.930 -> 0.854, merged entries 1.20 -> 2.63, 20 safety frames on
hard_s100, latency 1.89 -> 2.47 s) and costs ~2 s latency on Isaac (4.23 vs 2.32 s): at 4.5-5 m spacing its merge settings
merge real neighbours. So `treemapper_real.launch` defaults `mapper_config_file` to `config/core/cand_obj_real.cfg` (a
one-line launch change); the sim launch, the Isaac replay, `treemapper.yaml`, `scripts/evaluate.sh` and the built-in
defaults keep `config/cand_obj.cfg`. `ConfigAgreement` (M4b, 9100ccb) checks that every launch-default config resolves in
`config/`, parses and is accepted by the mapper. A spacing-aware merge rule could unify the profiles; left as a code change
for later.

## M5: handoff (2026-10-24, 20:15-; D55-D57)

**D55. The shipped system's held-out comparison with `simple` is extracted from M4a's final runs, not re-run.** The
held-out suite is run once per frozen configuration (D31, D39); M4a's final evaluation already ran simple, cand_obj and
cand_grid with `--reference simple` (`/data/work/eval/m4a/final/holdout_*.md`), but `results/cand_obj_m4_holdout.md` kept
only the comparisons with M3 and cand_grid. M5 copied the shipped system's per-level card (with the std over datasets =
seed-to-seed spread) and the paired cand_obj vs simple tables verbatim into `results/final_holdout_vs_simple.md`, with the
source JSON hashes. Re-running would give the same numbers (deterministic, D26) and would only add a second look at the
held-out data.

**D56. Verification before the handoff claims** (`results/REPORT.md` section 11). A fresh build directory for the core
tests (`/data/work/build_m5`) and a fresh catkin workspace (`/data/work/catkin_m5`), the ROS gtests and the rostest smoke,
and one re-run of `evaluate.sh cand_obj` on Isaac compared field by field (and the map stream byte by byte, without the
wall-clock `stage_ms`) with M4a's committed run. All at <= 2 cores (`TM_JOBS=2`, `-j2`, `taskset -c 0,1`).

**D57. What ships.** Mapper `cand_obj` (`ros/treemapper_ros/config/treemapper.yaml`: `mapper: cand_obj`,
`mapper_config_file: config/core/cand_obj.cfg`). Sim / synthetic profile = `config/cand_obj.cfg` = built-in defaults; real
stereo profile = `config/cand_obj_real.cfg`, the default of `treemapper_real.launch` (D54). `cand_grid` and `simple` stay registered and
buildable as references for the evaluation, not for flight. The README parameter table is generated from the two `.cfg`
files (`python3 -m treemapper_tools.param_table`; it fails if a key has no description).
