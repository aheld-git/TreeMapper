# cand_obj: object-centric tree instance mapper

Candidate of milestone M3 (family: per-object reasoning). Code: `core/include/treemapper/candidates/cand_obj/cand_obj.hpp`,
`core/src/candidates/cand_obj/cand_obj.cpp`; tests `core/tests/candidates/cand_obj/test_cand_obj.cpp`; frozen
configuration `config/cand_obj.cfg`. Results: `results/cand_obj_isaac.md`, `results/cand_obj_synthetic.md`.

## 1. Approach in one paragraph

Every frame, the stride-4 disparity grid is back-projected and binned onto a fixed world lattice (0.4 m cells) into a
**canopy height model** (CHM): one robust height per cell (median of its points), a per-frame **ground plane** fitted to
the lowest cells, and 3x3 smoothing. Canopy cells (smoothed height >= 1 m above the plane) are split into **crowns** by a
**watershed**: each cell flows uphill to a local maximum, and basins whose peak rises less than `min_prominence` above
the saddle to a neighbour are merged into it. A crown whose top projects inside a (dilated) detection box is an
observation of one tree. Observations are matched to **tree instances** by 3D position (top inside the instance's
outline or within `match_radius` of its centre). Each instance owns a local **evidence grid** on the same lattice: per
cell, how many frames it was that instance's canopy (positive) and how many frames it was seen as ground or as another
instance's canopy (negative). A cell **latches**, permanently, after `latch_frames` positive frames and when positive in
at least `latch_ratio` of the judged frames. The published outline is the convex hull of the latched cells; the centre
is their centroid; the height is the median of the per-frame crown-top samples. There are no 2D tracks anywhere.

## 2. Why this should beat the simple baseline

The baseline's failures (PROGRESS.md, M2) and the mechanism that answers each:

| Failure of `simple` | cand_obj mechanism |
|---|---|
| Merged detector boxes / touching canopies -> neighbours swallowed | Crowns come from the height field, not from boxes: two touching domes have a saddle between their tops, so the watershed splits them at the saddle even when the xy gap is below the depth noise. A merged box simply contains two crown tops. |
| False trees from clutter, FP boxes, noise bursts | A crown must be a local maximum >= `min_tree_height` above the ground plane (bushes are 0.2-0.9 m), inside a box, with >= `min_basin_cells` cells; an instance must then be matched in `confirm_frames` frames, have `min_height_samples` finite tops and `min_publish_area` of latched cells before it exists for navigation. A noise burst seeds tentative instances that expire after `tentative_timeout` without ever being published. |
| Outline spill from raw hulls | Nothing enters an outline below cell level, and cell heights are medians (at 15 m, ~9 points per cell: per-point sigma_Z 0.9 m at medium becomes ~0.4 m per cell and ~0.15 m after smoothing), so ground noise rarely passes the 1 m canopy threshold; a cell also has to be canopy in a majority of the frames it was judged. Cells farther than `max_cell_radius` from the instance centre are refused. |
| Noisy heights, heights above the drone | Height sample = median height of the crown-top cell (not p95 of raw points); the published height is the median of up to `height_buffer` samples; samples above drone altitude - margin are dropped and counted; crowns truncated by the image border give no sample. |
| Publish latency | An instance publishes after `confirm_frames` (4) matched frames with 3 finite tops and 3 m^2 latched: ~0.3 s at 15 Hz once the crown is fully in view. |
| Identity through track loss / jumps | No tracks: matching is by world position against the instance's own outline, so a lost or jumping detector track changes nothing. IDs are monotonic and a merge keeps the better-established ID. |
| Rule 2 | A single frame can change neither an outline (latching needs `latch_frames` >= 3 frames) nor a height (median). Latched cells are never removed, so the hull can only grow. |

## 3. Confidence and identity model

**Existence.** An instance is *tentative* from its first crown until it is confirmed; tentative instances are never
published, cannot be confirmed in a locked phase (that would be a new tree after scanning), and are deleted after
`tentative_timeout` (1.5 s) unseen. Confirmation needs `confirm_frames` frames with a matched crown, `min_height_samples`
finite crown-top samples and `min_publish_area` of latched cells, and the centre must not lie inside a sprayed tree.
A confirmed instance is never deleted; it can only be absorbed by a merge during a creation phase.

**Extent.** Per-cell evidence with a permanent latch. The outline may grow whenever the phase policy allows the instance
to change (SCAN, SCAN_APPROACH, ERROR, unknown: all; PLANNING: confirmed instances; HOPPING: the target only; the post-spray
climb: the target only; SPRAYING/IDLE/RETURN: asleep). It never shrinks: the published outline is monotone under inclusion
of every earlier one. Hovering cannot inflate it beyond cells that are canopy in a majority of judged frames.

**Height.** Median of crown-top samples; finite by construction once >= `min_height_samples` samples exist, which is a
condition of publication (rule 3). A frozen or sprayed instance receives no samples, so its height cannot change.

**Identity.** IDs come from a monotonic counter and are never reused (expired tentative IDs and merged-away IDs are
retired). Matching is geometric (top inside outline / near centre), so detector track changes are invisible. Merges
happen only in creation phases (a merge retires an ID navigation may hold) and only when the latched cells overlap by
>= `merge_overlap` of the smaller instance or the centres are closer than `merge_centre_dist`; the winner is
confirmed > more frames seen > lower ID, and keeps its ID and its height samples (rule 4). The size rule makes a small
fragment inside a large tree merge easily, while two large trees would need half of the smaller one covered.

## 4. How each rule of brief section 7 is guaranteed (and tested)

| Rule | Guarantee | Test |
|---|---|---|
| 1 IDs stable, never reused | monotonic `next_id_`; merges keep the established ID | `IdsNeverReusedAndMergeKeepsEstablished` |
| 2 no silent damage | latch needs >= 3 frames and a majority; latched cells never unlatch; median height | `OneBadFrameNeverCorruptsAPublishedTree`, `HoveringDoesNotInflateTheOutline` |
| 3 height always real | publication requires >= 3 finite samples; samples NaN / above drone dropped | `PublishesOnlyAfterConfirmationWithFiniteRobustHeight` |
| 4 one tree one entry | watershed separation; overlap/centre merge with established winner | `TouchingNeighboursStaySeparate`, `IdsNeverReused...`, `SeveralCrownsOnOneTreeAllCount` |
| 5 sprayed protected | sprayed instances get no crowns, no evidence, no merges; crowns inside a sprayed outline never seed | `SprayedTreesFrozenAndProtected` |
| 6 fail loudly | see the gate table below; every trigger is counted and the first occurrence logged | `HeartbeatsHoldsAndSleepAreCounted`, descent test |
| 7 every drop counted | `counters()` (points, cells, crowns, height samples, instances; listed below) | all tests read counters |
| sec. 6 phases | `phasePolicy()` (shared pure function) + post-spray hold | `NoNewTreesAfterScanning`, `OnlyTargetChangesDuringDescentAndPostSprayClimb` |

**Gates and their failure direction (rule 6):**

| Gate | Missing information | Direction | Logged / counted |
|---|---|---|---|
| mission phase | unknown / none | open (behave as SCAN) | warn once; `frames_total` |
| drone pose | invalid | closed (hold everything) | warn once; `frames_held_no_pose` |
| hop target in HOPPING | none | closed (freeze every tree) | warn once; `crowns_dropped_non_target_frozen` |
| post-spray climb | hover position unknown | closed until the first valid pose after SPRAYING, then until 2 m away | `frames_post_spray_hold`, `post_spray_holds_started` |
| ground plane | fit rejected (slope > 0.3, degenerate) | open with a flat ground at the lowest decile | warn once; `frames_ground_plane_fallback` |
| disparity | missing / wrong size | closed for the frame | `frames_dropped_no_disparity`, `frames_dropped_bad_disparity_size` |
| detections | crown top in no box | closed for that crown | `crowns_dropped_unboxed` |

**Counters** (rule 7): `points_dropped_{invalid,range,above_drone,outside_grid_cap}`, `cells_dropped_too_few_points`,
`frames_dropped_{too_few_points,too_few_cells}`, `frames_without_canopy`, `frames_ground_plane_fallback`,
`basins_merged_low_prominence`, `crowns_dropped_{too_low,too_small,unboxed,truncated_seed,inside_sprayed,sprayed_tree,
new_tree_locked_phase,non_target_frozen,unconfirmed_locked_phase,sleeping}`, `cells_dropped_{outside_window,too_far_from_centre}`,
`height_samples_{dropped_above_drone_or_nan,skipped_truncated_crown}`, `instances_{created,confirmed,expired_unconfirmed,
merged,refused_confirm_inside_sprayed}`, `merge_cells_dropped_outside_window`, `cells_latched`, `crowns_total`,
`frames_{total,skipped_sleeping,held_no_pose,heartbeat_only,post_spray_hold}`, `trees_marked_sprayed`, `mark_sprayed_unknown_id`.

## 5. What was tried (tuning on suite_v1 and the recording; held-out suite never read)

All runs paired by seed against `simple`; the synthetic numbers below are cand_obj's per-level means (easy / medium / hard)
on suite_v1, 5 seeds each. Run-to-run noise is 0 (deterministic), so differences between rows are real for these seeds
but the seed-to-seed spread (std over datasets, in the cards) still applies to any claim about other orchards.

| Build / setting | medium IoU | medium swallowed | hard recall | hard swallowed | false trees (m / h) | spill (e / m / h) | latency medium s | note |
|---|---|---|---|---|---|---|---|---|
| v1: cell-corner hulls, no seeding from truncated crowns (`border_px` 8 on any cell), latch 3 frames at 50 %, prominence 0.8 | 0.550 | 0.8 | 0.50 | 2.8 | 0.4 / 8.4 | 0.25 / 0.41 / 0.60 | 2.95 | Isaac latency 12.3 s: trees whose crown ever touched the image border never seeded |
| + peak-border rule (40 px) and hulls from cell centres | 0.631 | 1.6 | 0.83 | 7.0 | 0.2 / 0.4 | 0.12 / 0.31 / 0.42 | 2.28 | recall +0.37 vs v1 on hard; Isaac latency 2.4 s, spill 0.21 -> 0.10 |
| + frame noise-burst gate (2 m) and height-sample clearance (2 m) = v2 | 0.631 | 1.6 | 0.83 | 7.0 | 0.2 / 0.4 | 0.12 / 0.31 / 0.42 | 2.28 | hard false trees 8.4 -> 0.4, implausible-height frames 126 -> 33 per hard mission |
| v2 + latch_ratio 0.7 | 0.676 | 1.2 | 0.85 | 6.2 | 0.2 / 0.4 | 0.09 / 0.27 / 0.38 | 2.28 | fringe cells that are canopy in only half of the frames no longer latch |
| v2 + seeding from truncated crowns | 0.643 | 2.2 | 0.83 | 8.2 | 0.2 / 0.4 | 0.12 / 0.30 / 0.40 | 2.22 | earlier evidence, but more neighbour spill at crown entry (a neighbour's first visible slice joins the known tree's basin until its own top rises above the saddle) |
| A: truncated seeding + latch 5 frames at 70 % | 0.734 | 1.4 | 0.87 | 3.6 | 0 / 0 | 0.07 / 0.21 / 0.33 | 2.23 | the longer latch window lets the negatives from the neighbour's emergence win |
| B: truncated seeding + latch 3 at 80 %, border 16 px, confirm 3 | 0.715 | 1.4 | 0.87 | 4.8 | 0.2 / 0.6 | 0.08 / 0.23 / 0.33 | 1.61 | fastest; medium height RMSE 0.25 (tops near the border sampled too early) |
| C: latch 5 at 75 %, border 24 px, confirm 3 | 0.749 | 1.4 | 0.86 | 3.2 | 0 / 0 | 0.07 / 0.20 / 0.31 | 1.94 | |
| **D = C + prominence 0.6 m (frozen)** | **0.761** | **1.0** | **0.86** | **2.2** | **0 / 0** | **0.06 / 0.19 / 0.31** | **1.83** | the shipped `config/cand_obj.cfg` |
| E: latch 6 at 80 %, border 24 px, confirm 3 | 0.773 | 1.6 | 0.78 | 2.2 | 0 / 0.2 | 0.05 / 0.17 / 0.29 | 2.23 | best medium IoU but hard recall collapses: 80 % is too strict under 5x noise |

Rejected ideas and why:
- **Points instead of cells.** Per-point ground / height gates (the baseline) cannot work at medium noise: a ground point at 15 m
  has sigma_Z 0.9 m, so 16 % of ground points pass a 1 m canopy gate. The cell median + 3x3 smoothing cuts that to ~0.15 m.
- **Raw hull of a crown as the outline.** One frame would then define geometry (rule 2). The per-cell latch is the whole point.
- **Boxes as the unit of observation.** A merged box over two trees is one observation; splitting by the height field instead
  makes the box only a tree-ness gate for a crown top. The box is still used (a crown top in no box is never credited),
  because in the real world tall things that are not trees exist.
- **A cell-level `truncated` flag** (any crown cell near the border): correct but far too conservative (Isaac latency 12 s).
- **Shrinking the published outline by one cell** to counter stereo foreground fattening (the main remaining spill on synthetic):
  not done, because it would cost IoU on the recording, which has no fattening, and nothing real-stereo exists to decide it.

## 6. Known failure modes and weaknesses

- **Implausible-height frames on synthetic are mostly not height errors.** The generator hovers at a fixed ~7 m world z
  (5.3-6.0 m in hard_s00) while some truth tops are 6.2-8.1 m, and the scorer flags every published height above the drone.
  cand_obj's heights at those trees are within +-0.2 m of the truth (hard_s00: 6.10 vs 6.17, 5.86 vs 5.81, 5.66 vs 5.89;
  hard_s03: 7.60 vs 8.13; medium_s00: 7.15 vs 6.97). A system that under-estimates heights scores "safer" here. One real
  case remains (hard_s01 ID 91: 7.55 m published for a 5.18 m tree, +2.4 m, 23 frames). On the recording the count is 0.
- **Crown entry on touching neighbours.** When a neighbour's crown enters the image next to a known tree, its first
  visible slice has no top of its own yet and joins the known tree's basin until its rise exceeds `min_prominence`. The
  5-frame / 75 % latch absorbs this at 15-27 Hz; a much slower frame rate (< 5 Hz) would let those cells latch.
- **Centre drift during entry.** The centre is the latched centroid, so it moves as the far side of the crown latches
  (scorer "centre jumps": 3-9 per synthetic mission vs 9-33 for `simple`, 2 vs 8 on the recording). Not a safety count,
  but it is visible growth; raising `min_publish_area` trades it against latency.
- **Hard level**: recall 0.86, 2.2 swallowed neighbours per mission, height RMSE 0.59 m. 5x real noise with 4.5 m spacing
  and 60 % merged boxes is a stress test; the saddle between two crowns disappears when the smoothed noise (0.4 m) reaches
  the prominence threshold (0.6 m).
- **Latency** is the price of waiting for the crown top: 1.2 / 1.8 / 2.2 s (easy / medium / hard) and 2.4 s on the
  recording, vs `simple` 0.6 / 0.8 / 0.7 s and 2.4 s, and the existing system's 1.2 s on the recording. Publication needs
  three finite top samples, which exist only once the top is 24 px inside the image.
- **Ground plane per frame**: a single plane over a 21 x 13 m footprint; strong undulation (> ~1 m within the footprint)
  would classify bumps as canopy if they also rise above 1 m. Guarded by `min_tree_height` (1.5 m) and the box gate.
- **Height statistic on rough canopies.** The height sample is the median point height of the crown-top cell. On the
  smooth synthetic domes that is the top (bias -0.002 m, RMSE 0.06 / 0.19 / 0.59 m by level). On the recording the
  published heights are 4.3-5.8 m, between the in-canopy point median (3.84 m) and the p99 top (5.88 m), i.e. lower than
  `simple` (4.8-5.7 m, p95 of raw points) and the existing system (~5.4 m). If navigation needs the extreme top as the
  descent reference, a per-cell high percentile instead of the median is a one-line change, at the price of an upward
  noise bias on real stereo (p80 of 9 points with sigma 0.9 m is +0.75 m). Not decidable without reliable real heights.
- **Unverified**: real stereo (the real flight is not yet readable through the input contract), the Jetson (timings
  below are x86), any detector other than the recorded / simulated ones, a tilted mount.

## 7. Cost

Per processed frame on the x86 workstation (RelWithDebInfo, one core), `stage_ms` in the cards: `chm` (back-projection
of 16000 points, binning, medians + MAD, plane fit, smoothing, watershed) about 0.7-0.9 ms mean, `instances` (matching,
evidence, latching, hulls, merges) about 0.01-0.05 ms. Memory: one 28 x 28 evidence window (about 4 kB) per live instance.
Expect several times the x86 figure on the Orin (not measured); the budget of 10 ms per frame leaves an order of magnitude.

## 8. Results (final frozen configuration; details in `results/cand_obj_*.md`)

Recorded Isaac mission (held out from parameter tuning): recall 9/9 (simple 8, existing 7), mean / min IoU 0.86 / 0.78
(0.73 / 0.46, 0.63 / 0.39), 0 swallowed neighbours (1, 2), centre RMSE 0.21 m (1.10, 1.36), spill 0.05 (0.14, 0.16),
**0 safety violations** (0, 5), 7 trees sprayed, latency 2.2 s (2.4, 1.2), 0.85 ms per frame.

Synthetic suite_v1, paired by seed against `simple` (15 pairs): recall 14/0/1, precision 15/0/0, mean IoU 15/0/0,
neighbours swallowed 15/0/0, false trees 15/0/0 (0 on every mission), centre RMSE 15/0/0, height RMSE 14/1/0, spill
15/0/0, centre jumps 15/0/0, trees sprayed 12/0/3; latency 0/15/0 (1.0 s slower). Medium: recall 0.98, precision 0.99,
IoU 0.76, 1.0 swallowed, 0 false, centre 0.46 m, height 0.19 m, spill 0.19.

## 9. Answers to the brief's section 10 questions, as far as this candidate goes

- **Touching trees with a gap below the depth noise**: separated in height, not in xy. Two crowns have two tops and a
  saddle between them; the watershed on the smoothed CHM splits at the saddle whatever the xy gap. The limit is the
  prominence threshold (0.6 m) against the smoothed noise (0.15 m medium, 0.4 m hard).
- **Ground and neighbour rejection under rule 2**: ground at cell level against a per-frame plane (median of ~9 points,
  3x3 smoothing), neighbours by exclusive crown assignment per frame, and both only ever reach the published outline
  through the latch (5 frames, 75 % majority): rejection happens before anything is published, so nothing has to be
  removed later.
- **ID stability through track loss, jumps and merges**: no tracks; geometric matching against the instance's own
  outline; monotonic IDs; merges keep the established ID and happen only in creation phases.
- **Confidence model**: tentative until 3 matched frames, 3 finite tops and 3 m^2 latched; then published forever;
  geometry may grow (never shrink) whenever the phase policy allows that instance to change; height is a median that
  stops moving when the instance is frozen or sprayed.
- **Map reuse across flights** (design only): the evidence grids are on a world lattice, so a previous map is a set of
  (latched cell set, height) per tree in the old frame. With metres of GNSS offset and repetitive rows, re-anchoring must
  come from the row *pattern*, not from single trees: fit the old tree-centre constellation to the new one with a 2D
  rigid transform (RANSAC over centre pairs, scored by the number of centres within ~1 m), accept only when the match is
  unambiguous (second-best alignment clearly worse, which the row period makes explicit), and then import old trees as
  *tentative* instances with their outlines as prior evidence, so they still need fresh observations before they are
  published with new IDs. Old IDs are never reused (rule 1 across flights).
