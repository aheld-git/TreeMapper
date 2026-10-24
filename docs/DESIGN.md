# TreeMapper design (shipped system: `cand_obj`)

This document describes the system that ships: the ROS-free C++17 core library with the `cand_obj` mapper, wrapped by
`treemapper_node` (`ros/treemapper_ros`). It answers the questions of `docs/PROJECT_BRIEF.md` section 10, states how
each rule of section 7 is guaranteed and which test proves it, and gives the behaviour per mission phase (section 6).
Numbers are cited from committed result files; how the design was chosen is in `results/REPORT.md` (section
"History") and `docs/DECISIONS.md` (D31, D32-D39). Candidate-level detail, including what was tried and rejected:
`docs/candidates/cand_obj.md`. The alternative that lost, `cand_grid`: `docs/candidates/cand_grid.md`.

## 1. Architecture in brief

```
 disparity (32FC1) + camera_info   detections (Detection2DArray | DetectionResult)   TF pose   /mission_state   hop target
          |                                     |                                       |            |             |
          +------------------------- treemapper_node (ros/treemapper_ros) ----------------------------------------+
          |  validate calibration (fail closed), pair detection<->disparity by stamp, sample disparity on a        |
          |  stride-4 grid, pose at the stamp (tf2), phase string -> MissionPhase (unknown -> UNKNOWN, fail open)   |
          +-------------------------------------> FrameInput (core/include/treemapper/frame.hpp) ------------------+
                                                         |
                                   treemapper::Mapper (core/include/treemapper/mapper.hpp), registry by name
                                                         |
     cand_obj (core/src/candidates/cand_obj/cand_obj.cpp), per frame:
       1. point gates: invalid, optical range 0.5-30 m, above drone z - 0.5 m, per-frame noise burst (all counted)
       2. canopy height model on a fixed 0.4 m world lattice: per-cell median z, ground plane from the lowest cells,
          3x3 smoothing; canopy = >= 1 m above the plane
       3. crowns = persistence watershed on the CHM (basins with < 0.6 m prominence merge into the neighbour);
          a crown counts only if its top lies inside a (dilated) detection box
       4. match crowns to tree instances by world geometry (top inside outline / near centre / latched overlap)
       5. per-instance evidence grid: +1 canopy-of-this-tree, -1 ground or another tree's canopy, quality-weighted;
          a cell latches permanently after 5 positive frames at >= 75 % positive
       6. publish when confirmed: outline = convex hull of latched cells, centre = their centroid (rate-limited),
          height = 0.99 quantile of the latched cells' fused surface z (rate-limited, finite by construction)
                                                         |
                            publishedTrees() -> ~map, ~markers, ~get_trees;   counters() -> ~counters;
                            takeStageTiming() -> ~stage_timing;   health (AWAKE / SLEEPING / HOLDING / FAULT) -> ~health
```

There are **no 2D tracks** anywhere: identity is decided in the world frame, against each tree's own latched cells.
The detector box is used only as a "this raised surface is a tree" gate for a crown top, never as geometry.
Dependencies: the C++17 standard library only (no Eigen, CGAL, OpenCV); the node needs stock Noetic packages
(`ros/treemapper_ros/README.md`, "Build").

## 2. Answers to the brief's section 10 questions

### 2.1 Separating touching trees when the gap is below the depth noise

Trees are separated **in height, not in xy**. Two touching crowns are two domes with a saddle between their tops. The
per-frame canopy height model takes the *median* of the ~9 points in each 0.4 m cell and smooths 3x3, which cuts the
per-point depth noise (sigma_Z about 0.9 m at 15 m on the medium synthetic level) to about 0.15 m per smoothed cell
(`docs/candidates/cand_obj.md` section 2). A watershed then lets each cell flow uphill to a local maximum, and two
basins are merged only if the lower peak rises less than `min_prominence` (0.6 m) above the saddle. The xy gap can be
zero: as long as the saddle is 0.6 m below the lower top, the trees stay two entries. A merged detector box over two
trees simply contains two crown tops; the box never defines an outline.

The limit is explicit: when the smoothed noise approaches the prominence threshold (about 0.4 m on the hard level,
5x real noise) saddles vanish and neighbours merge. Held-out evidence (`results/cand_obj_m4_holdout.md`): neighbours
swallowed per mission 0 / 1.0 / 2.8 (easy / medium / hard), against 1.8 / 14.3 / 29.3 for the simple baseline and
0 / 0.7 / 5.3 for cand_grid. On the real farm flight the opposite failure dominates (over-fragmentation, section 7).
Test: `CandObjCHM.TouchingDomesUnderOneBoxBecomeTwoCrownsWithoutGroundSpill`, `CandObjRules.TouchingNeighboursStaySeparate`.

### 2.2 Where ground and neighbour points are rejected, given rule 2

Rejection happens **before** anything can reach a published outline, so nothing ever has to be removed from one
(rule 2 forbids shrinking):

| Stage | What is rejected | Mechanism | Counter |
|---|---|---|---|
| point | impossible / unreliable points | invalid disparity; optical range outside [0.5, 30] m; above drone z - 0.5 m; whole frame when the median per-cell MAD exceeds 2 m (noise burst, turns) | `points_dropped_{invalid,range,above_drone,outside_grid_cap}`, `frames_dropped_*` |
| cell | ground and grass | per-cell median z vs a per-frame ground plane fitted to the lowest 40 % of cells (inlier band 0.6 m, slope <= 0.3, else a flat fallback, counted); canopy only >= 1 m above it after smoothing | `cells_dropped_too_few_points`, `frames_ground_plane_fallback` |
| crown | bushes, clutter, partial crowns | top >= `min_tree_height` 1.5 m above ground, >= 4 cells, top inside a detection box (a top within 24 px of the image border is flagged "truncated"; with the shipped `seed_truncated = true` the flag only feeds a counter) | `crowns_dropped_{too_low,too_small,unboxed,truncated_seed}` |
| instance cell | **neighbours** and box corners | each canopy cell belongs to exactly one crown per frame, so a neighbour's crown is negative evidence for this tree's cells; cells > 4 m from the instance centre are never credited; a cell latches only when canopy-of-this-tree in >= 75 % of the (quality-weighted) frames it was judged, and in at least 5 of them | `cells_dropped_too_far_from_centre`, `cells_latched` |

A detection box corner that lands on ground or on the neighbouring tree therefore does nothing: the ground there is not
canopy, and the neighbour's canopy belongs to the neighbour's crown (brief section 8, first failure mode). Only latched
cells form the outline; latched cells are never unlatched, so the published outline is monotone under inclusion. An
optional relative-height trim of the crown skirt (`latch_rel_h`) exists but ships off (it cost hard-level recall 0.82 ->
0.55, `docs/candidates/cand_obj.md` section 10.3).

### 2.3 ID stability through track loss, jumps and merges

- **No detector tracks are used**, so a lost, switched or jumping track changes nothing. A crown is matched to an
  instance by world geometry: its top inside the instance's outline or within `match_radius` (2 m) of its centre, or
  the crown covers >= `match_overlap` (50 %) of the instance's latched cells with its top within `match_reach` (3 m).
  When several instances qualify, the better-established one wins (confirmed > more frames seen > lower ID), so a
  tentative fragment can never take a crown from a published tree.
- **Seed guard**: a crown that would seed a new instance is credited to a published tree instead if >= 25 % of its
  cells lie on that tree's latched cells. This removed a transient duplicate on the recording (`docs/candidates/cand_obj.md`
  section 10.5).
- **Several crowns on one tree in one frame all count** (every crown matched to the instance adds evidence).
- **IDs come from a monotonic counter** and are never reused: expired tentative IDs and merged-away IDs are retired.
  Tentative instances are never published, so a retired tentative ID was never seen by navigation.
- **Merges** happen only in creation phases (SCAN, SCAN_APPROACH, ERROR, unknown), because a merge retires an ID
  navigation may already hold. The sweep runs over every live instance, observed this frame or not, so a stale
  fragment still merges. Two instances merge when their latched cells overlap by >= 50 % of the **smaller** one or their
  centres are < 1.5 m apart: a small fragment inside a large tree merges easily, two large trees need half of the
  smaller one covered (size-aware, brief section 8). The better-established side keeps its ID and its height; the loser's
  cell heights are folded into the winner's cells and the published height moves toward the union's quantile at the
  bounded rate, so a merge never makes a published value jump.
- Evidence: 0 ID switches after lock and 0 new or reused IDs while the list is locked on every held-out mission and on
  the recording; 0 ID reuse and 0 reappearances on the real flight (`results/cand_obj_m4_holdout.md`, `results/cand_obj_m4_isaac.md`,
  `results/real_flight_scorecard.md`).

### 2.4 Confidence model: when a tree becomes publishable, and when its geometry may still change

**Existence.** An instance is *tentative* from its first crown. It is published (confirmed) only when all hold:
`confirm_frames` (3) frames with a matched crown; `min_height_cells` (3) latched cells carrying a fused height;
`min_publish_area` (3 m^2) of latched cells; its centre is not inside a sprayed tree; and the phase allows creation
(an instance cannot be confirmed in a locked phase: that would be a new tree after scanning). Unconfirmed instances unseen
for `tentative_timeout` (1.5 s) are deleted. A confirmed instance is never deleted; it can only be absorbed by a merge in
a creation phase. Because the height is a quantile over >= 3 fused cells, it is finite at the moment of publication
and stays finite (rule 3).

**Height semantics.** `TreeEntry.height` is the canopy **top as a world-frame z coordinate** (m) in the map frame
(`world_nwu` on the drone, `world` in Isaac), the 0.99 quantile of the fused world z of the latched cells. It is **not**
a height above ground: its datum is the world frame's origin, and on the real drone `world_nwu` re-origins at every
boot (origin = power-on position), so it is comparable only with drone poses of the same session. Navigation compares
it with the drone's world z; above-ground height would be `height - local ground z`. `DronePose.msg`'s
`height_above_ground` is unused (the node takes the pose from TF / PoseStamped, the mapper gates on world z).
`TreeEntry.observations` = frames with a matched crown; `confidence` is reserved and always 0.

**Extent.** Per-cell evidence with a permanent latch (5 positive frames, >= 75 % positive). Evidence increments are
quality-weighted: a frame taken during a fast rotation (turn blur) counts as a fraction of a frame
(`1 / (1 + (rate / 1.0 rad/s)^2)`, recovering over 1.5 s). A hover cannot inflate an outline beyond cells that are canopy
in a majority of judged frames, and a cell's height weight saturates at 30, so a long hover does not distort the height
(brief section 8, "hovering inflating confidence"). Tests: `CandObjRules.HoveringDoesNotInflateTheOutline`,
`CandObjM4.LowQualityFramesCarryLessEvidence`.

**When geometry may still change.** The outline may grow, never shrink, whenever the phase policy lets that instance
change (section 4). Cell heights are fused **only in creation phases**; once the list is locked the descent reference
does not move (except for the rate limiter finishing a step already in progress). Published centre and height move
toward their targets at <= 0.8 m/s and <= 0.5 m/s. Sprayed and frozen instances are never stepped. Result: 0 centre and
0 height jumps (> 1 m) on every held-out mission, on the recording and on the real flight; target height range during
its own descent 0 m on every mission (`results/cand_obj_m4_holdout.md`, `results/cand_obj_m4_isaac.md`).
Caveat: synthetic missions re-enter SCAN for a second corridor, where heights are fused again by design; max height drift
since the first lock is 0.82 m (medium) / 1.21 m (hard) worst case there, 0 on the recording.

### 2.5 Map reuse across flights (design only, not implemented)

The world frame is reset every boot and GNSS is off by metres, while orchard rows are periodic, so a single tree can
never anchor a map. Design:

1. **What is stored** after a flight: per tree, its latched cell set and fused heights in that flight's frame, its
   height, sprayed flag and a flight-local ID; plus the flight's GNSS-to-world transform and the GNSS fix quality.
2. **Coarse prior**: the GNSS offset between the sessions bounds the search (a few metres, plus yaw from the heading).
3. **Re-anchoring from the row pattern**: once the new flight has confirmed a few dozen trees, fit the old tree-centre
   constellation to the new one with a 2D rigid transform: RANSAC over centre pairs, scored by the number of centres
   within ~1 m, then a least-squares refinement on the inliers.
4. **Ambiguity test**: the row period makes wrong alignments (shifted by one tree or one row) explicit; accept only when
   the best alignment clearly beats the second best (inlier count and residual), otherwise do not reuse at all (fail
   closed: fly as if no map existed).
5. **Import as prior, not as truth**: old trees enter as *tentative* instances whose cells start with prior evidence, so
   they still need fresh observations to publish, and they get new IDs (rule 1 across flights: an old ID never names a
   different tree). The old sprayed flag is a mission-level fact handed to navigation, not a reason to freeze geometry
   that was never observed in this frame.

## 3. How each rule of brief section 7 is guaranteed, and the test that proves it

Tests live in `core/tests/candidates/cand_obj/test_cand_obj.cpp` (gtest `test_core`, run by `scripts/run_tests.sh`),
`ros/treemapper_ros/test/` (gtest `test_treemapper_ros`) and `tools/tests/` (pytest, the scorer that measures the rules
on whole missions). Each test fails if its rule is broken.

| Rule | Guarantee in the design | Test(s) |
|---|---|---|
| 1 IDs stable, never reused | monotonic `next_id_`; retired IDs never re-issued; geometric matching, no tracks; established side of a merge keeps its ID | `CandObjRules.IdsNeverReusedAndMergeKeepsEstablished`, `CandObjM4.FragmentNeverTakesOverAPublishedTree`; scorer: `test_score_review.py` (ID reuse decided geometrically, R9) |
| 2 no silent damage to published geometry | latch needs >= 5 frames at >= 75 %; latched cells never unlatch (grow-only, monotone outline); height from >= 3 fused cells with a 2 m outlier gate, rate-limited; centre rate-limited | `CandObjRules.OneBadFrameNeverCorruptsAPublishedTree`, `CandObjRules.HoveringDoesNotInflateTheOutline`, `CandObjCHM.NoiseBurstFramesAreDroppedAndCounted`, `CandObjM4.MergeNeverJumpsThePublishedValues`; scorer retreat / jump metrics (R1) |
| 3 height always a real number once published | publication requires >= 3 latched cells with a fused height; non-finite and near-drone heights never fused; `TreeMap::isPublishable` rejects undefined heights and degenerate outlines; the ROS conversion carries the core value | `CandObjRules.PublishesOnlyAfterConfirmationWithFiniteRobustHeight`, `CandObjM4.HeightFromLatchedSurfaceAndRateLimitedPublication`, `TreeMap.RejectsUndefinedHeightAndDegenerateOutline`, `Conversions.TreeEntryRoundTrip` |
| 4 one tree, one entry | watershed split at saddles; size-aware merge (overlap of the smaller / centre distance) over all live instances; seed guard; established side keeps identity and height | `CandObjRules.TouchingNeighboursStaySeparate`, `CandObjCHM.TouchingDomesUnderOneBoxBecomeTwoCrownsWithoutGroundSpill`, `CandObjRules.SeveralCrownsOnOneTreeAllCount`, `CandObjM4.FragmentNeverTakesOverAPublishedTree`, `CandObjRules.IdsNeverReusedAndMergeKeepsEstablished` |
| 5 sprayed trees protected | sprayed instances get no crowns, no evidence, no height steps, no merges; crowns inside a sprayed outline never seed; an instance whose centre is inside a sprayed tree is never confirmed; the node refuses `set_tree_sprayed(false)` | `CandObjRules.SprayedTreesFrozenAndProtected`; scorer sprayed-change / new-inside-sprayed metrics (R2) |
| 6 fail loudly and predictably | every gate has a documented direction (table below); each trigger is counted and its first occurrence logged; the node lists active warnings in `~health` | `CandObjFrames.HeartbeatsHoldsAndSleepAreCounted`, `CandObjRules.OnlyTargetChangesDuringDescentAndPostSprayClimb`, `CandObjConfig.SanitizedClampsAndLogsEveryClamp`, `MapperRegistry.EverySystemRejectsAnUnknownConfigKey`, `Calibration.BuildsFromCameraInfoAndRejectsUncalibrated`, `Disparity.RejectsMalformedAndMismatchedImages` |
| 7 every dropped data point counted by reason | `counters()` covers points, cells, crowns, height samples, instances, frames (list in `docs/candidates/cand_obj.md` section 4); the node adds input / pairing counters; both published on `~counters` | every cand_obj test reads counters; `CandObjCHM.NoiseBurstFramesAreDroppedAndCounted`, `FrameSync.DuplicateDisparityStampIsDroppedAndCounted`, `Conversions.CountersAndTiming` |
| single config, defaults = file | `config/cand_obj.cfg` equals the built-in defaults; clamps in `Config::sanitized()`, each logged; the node's yaml only points at the file | `CandObjConfig.ShippedFileEqualsBuiltInDefaults`, `CandObjConfig.SanitizedClampsAndLogsEveryClamp`, `ConfigAgreement.*` |

**Gates and their failure direction (rule 6)**

| Gate | Missing information | Direction | Where it shows |
|---|---|---|---|
| camera_info | none yet / uncalibrated / implausible baseline | closed: no mapper, frames dropped, retried | `~health` HOLDING / FAULT, `frames_dropped_no_calibration`, `camera_info_rejected` |
| camera mount TF (`camera_mount_source: tf`) | missing | closed, retried | WARN, HOLDING |
| drone pose | not available at the frame stamp / invalid | closed: hold all processing (start-up artefacts, brief section 8) | `pose_unavailable`, `frames_held_no_pose`, HOLDING |
| mission phase | none at all / unknown string | open: behave as SCAN | warn once, `~health` warning "no mission_state received: permissive" |
| hop target in HOPPING / post-spray climb | none (-1) | closed: freeze every tree | warn once, `crowns_dropped_non_target_frozen` |
| post-spray climb | hover position unknown | closed until a valid pose, then until 2 m away | `frames_post_spray_hold`, `post_spray_holds_started` |
| ground plane | fit rejected | open with a flat ground at the lowest decile | warn once, `frames_ground_plane_fallback` |
| disparity | missing / wrong size / malformed | closed for the frame | `frames_dropped_no_disparity`, `disparity_rejected_*`, `sync_frames_without_disparity` |
| detection box | crown top in no box | closed for that crown | `crowns_dropped_unboxed` |
| mapper config | unknown key / bad value | node exits at start (FATAL with the key); values out of range are clamped and logged | stderr / rosout |

## 4. Behaviour across mission phases (brief section 6)

The rules are one pure function, `phasePolicy()` (`core/src/simple_mapper.cpp`, shared by all systems), plus cand_obj's
post-spray hold:

| Phase | Processing | New trees | Merges | Who may change | Heights fused |
|---|---|---|---|---|---|
| SCAN, SCAN_APPROACH | awake | yes | yes | every tree | yes |
| ERROR | awake (a fault is when you most want to see) | yes | yes | every tree | yes |
| unknown / no `/mission_state` | awake, warning (fail open) | yes | yes | every tree | yes |
| PLANNING | awake | no (and no tentative may confirm) | no | every confirmed tree may grow | no |
| HOPPING | awake | no | no | **only the hop target** (no target: nothing) | no |
| post-spray climb (after SPRAYING, until 2 m from the hover point or SCAN) | awake | no | no | only the target | no |
| SPRAYING, IDLE, RETURN | asleep (frames skipped, counted) | no | no | nothing | no |

- **Descent.** `/mission_state` has no descent value, and the transit altitude is below the scan altitude, so the
  design freezes every non-target tree for the whole of HOPPING, not only for the final descent. This is stricter than
  the brief requires (it allows existing trees to grow while hopping) and was chosen because a low-angle look at a
  neighbour during transit is exactly the damage the freeze is for. Planning frames still grow locked trees.
  Test: `CandObjRules.OnlyTargetChangesDuringDescentAndPostSprayClimb`, `CandObjRules.NoNewTreesAfterScanning`.
- **Sleep and heartbeat.** While asleep the node keeps publishing `~health` at 2 Hz from a wall timer (state SLEEPING,
  "phase X: sleeping, not dead") and feeds heartbeat frames to the mapper every 0.2 s while the detector is silent, so
  phase changes and timeouts still run. Test: `CandObjFrames.HeartbeatsHoldsAndSleepAreCounted`; rostest
  `replay_smoke.test` asserts the heartbeat.
- **Hop target ID space.** The target must be in *this* mapper's IDs (navigation reads them from `~get_trees`). In
  replays of recordings the recorded target belongs to another system; `nav_emulator.py` / `treemapper_replay` resolve it
  by the descent point (D22, D45).
- Scorer evidence (all must be 0): new trees after scan, non-target changes in descent, sprayed-tree changes, new trees
  inside sprayed = 0 on all 30 held-out missions and on the recording (`results/cand_obj_m4_holdout.md`,
  `results/cand_obj_m4_isaac.md`). The real flight has no phases, so none of this can be exercised there.

## 5. Known failure modes of brief section 8, and the answer

| Failure mode | Answer in the design |
|---|---|
| box corners on ground / neighbour | boxes never define geometry; ground cells are not canopy; neighbour canopy is the neighbour's crown (negative evidence) |
| points above the drone / far away | point gates (above drone z - 0.5 m, range 30 m), heights never fused within 2 m of the drone altitude |
| start-up artefacts | pose invalid -> hold everything; calibration missing -> no mapper |
| track jumping to another tree | no tracks; only tentative instances ever expire; a confirmed tree is never deleted |
| several tracks on one tree | every matched crown adds evidence; several crowns of one tree in a frame all count |
| size-blind merges | overlap measured against the smaller instance's latched area |
| hovering inflating confidence | latch by majority, height weight capped at 30, evidence quality-weighted |
| simulation too easy | synthetic noise 1.5x / 3x / 5x the real stereo (`results/synthetic_calibration.md`), a hostile detector; tuned on synthetic and on the recording (so the recording is not an unbiased test), reported on a held-out suite (inspected several times: a validation set, not an untouched test); real flight scored without truth |

## 6. Sim vs real is configuration only

Two launch files (`treemapper_sim.launch`: `world`/`body`, verified nadir mount as a parameter; `treemapper_real.launch`:
`world_nwu`/`body`, data-derived 82 deg mount and 4.30 px disparity offset as defaults) and, for the mapper, two
configuration files: `config/cand_obj.cfg` (sim / synthetic profile, the one evaluated on every held-out suite) and the
real-flight profile `config/cand_obj_real.cfg` (the default of `treemapper_real.launch`'s `mapper_config_file`). The core
code is identical in both modes. The real profile changes 9 keys (README parameter table), mainly `merge_overlap` 0.5 -> 0.3,
`min_prominence` 0.6 -> 1.0, `min_publish_area` 3 -> 8 m^2 and wider match / merge radii: real crowns are rough and split
into instances that overlap by 30-50 %, while the trees stand ~8 m apart. On the tightly spaced synthetic orchards
(4.5-5 m) the same settings merge real neighbours, so neither profile fits both (`results/REPORT.md` sections 5-6,
`results/real_flight_profile.md`). The unifying fix would be a spacing-aware merge rule (merge threshold tied to the
estimated crown radius and the local tree spacing); it is a code change and was not made.

## 7. Known weaknesses

- **Real-flight over-fragmentation** with the sim profile: 102 final trees where the stereo shows about 60-63 crowns,
  39 outlines without a crown peak (`results/real_flight_scorecard.md`). The real crowns are rougher than the synthetic
  domes, so a 0.6 m prominence splits one crown into several basins. The real profile fixes it by configuration (63
  trees, 6 outlines without a peak) but regresses on synthetic data (held-out recall 0.93 -> 0.85, 20 safety frames on one
  hard mission, +2 s latency on Isaac): two profiles must be kept apart (section 6).
- **Recall on the hard level** (0.83 held-out) and the crown-entry effect on touching neighbours; recall dropped
  slightly in M4 against M3 (-0.018 medium, -0.035 hard; `results/cand_obj_m4_holdout.md`).
- **Latency**: 1.14 / 2.02 / 2.51 s held-out (easy / medium / hard) and 2.32 s on the recording, against the existing
  system's 1.19 s on the recording; the price of waiting for the crown top and the latch.
- **One ground plane per frame** over the footprint; strong undulation within ~20 m could read as canopy (guarded by the
  1.5 m tree height and the box gate).
- **Height reference** is the 0.99 quantile of the fused latched surface, i.e. near the canopy top; validated against
  synthetic truth only (Isaac truth heights are a constant fallback, R13; the real flight has no truth).
- **Unverified on hardware**: all timings are x86 (about 1 ms per frame); the Orin was not available (README checklist).
