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
is the centroid of the cells whose observation-weighted evidence (each frame counts min(point weight, 1) x frame
quality) currently says "this tree's canopy" (D66: latching is permanent, and a far edge view latches cells displaced
toward its track; held within `centre_max_outside` of the outline, the latched centroid while the outline has fewer than
3 points; `centre_from_support = false` gives the latched centroid); the height is a high quantile (0.99) of the latched cells' fused surface heights (M4; M3 used the
median of per-frame crown-top samples, see section 10). Published centre and height move at a bounded rate. There are no
2D tracks anywhere.

## 2. Why this should beat the simple baseline

> **Historical (M3).** Sections 2-9 describe the M3 candidate as it was selected. They are **superseded** by section 10
> (M4 hardening) and by `docs/DESIGN.md` (the shipped system). In particular the M3 height model (median of up to
> `height_buffer` crown-top samples, `min_height_samples`) no longer exists: M4 publishes the 0.99 quantile of the fused
> latched surface and gates publication on `min_height_cells` (section 10.1); the final review added the sticky lock and the
> non-truncated-height guard (`docs/DESIGN.md` sections 2.4 and 4).


The baseline's failures (PROGRESS.md, M2) and the mechanism that answers each:

| Failure of `simple` | cand_obj mechanism |
|---|---|
| Merged detector boxes / touching canopies -> neighbours swallowed | Crowns come from the height field, not from boxes: two touching domes have a saddle between their tops, so the watershed splits them at the saddle even when the xy gap is below the depth noise. A merged box simply contains two crown tops. |
| False trees from clutter, FP boxes, noise bursts | A crown must be a local maximum >= `min_tree_height` above the ground plane (bushes are 0.2-0.9 m), inside a box, with >= `min_basin_cells` cells; an instance must then be matched in `confirm_frames` frames, have `min_height_samples` (M3; now `min_height_cells`) finite tops and `min_publish_area` of latched cells before it exists for navigation. A noise burst seeds tentative instances that expire after `tentative_timeout` without ever being published. |
| Outline spill from raw hulls | Nothing enters an outline below cell level, and cell heights are medians (at 15 m, ~9 points per cell: per-point sigma_Z 0.9 m at medium becomes ~0.4 m per cell and ~0.15 m after smoothing), so ground noise rarely passes the 1 m canopy threshold; a cell also has to be canopy in a majority of the frames it was judged. Cells farther than `max_cell_radius` from the instance centre are refused. |
| Noisy heights, heights above the drone | Height sample = median height of the crown-top cell (not p95 of raw points); the published height is the median of up to `height_buffer` samples (M3 only; removed in M4, section 10.1); samples above drone altitude - margin are dropped and counted; crowns truncated by the image border give no sample. |
| Publish latency | An instance publishes after `confirm_frames` (4) matched frames with 3 finite tops and 3 m^2 latched: ~0.3 s at 15 Hz once the crown is fully in view. |
| Identity through track loss / jumps | No tracks: matching is by world position against the instance's own outline, so a lost or jumping detector track changes nothing. IDs are monotonic and a merge keeps the better-established ID. |
| Rule 2 | A single frame can change neither an outline (latching needs `latch_frames` >= 3 frames) nor a height (median). Latched cells are never removed, so the hull can only grow. |

## 3. Confidence and identity model

> Historical (M3), superseded by section 10 and `docs/DESIGN.md` (see the note under section 2).

**Existence.** An instance is *tentative* from its first crown until it is confirmed; tentative instances are never
published, cannot be confirmed in a locked phase (that would be a new tree after scanning), and are deleted after
`tentative_timeout` (1.5 s) unseen. Confirmation needs `confirm_frames` frames with a matched crown, `min_height_samples` (M3; now `min_height_cells`)
finite crown-top samples and `min_publish_area` of latched cells, and the centre must not lie inside a sprayed tree.
A confirmed instance is never deleted; it can only be absorbed by a merge during a creation phase.

**Extent.** Per-cell evidence with a permanent latch. The outline may grow whenever the phase policy allows the instance
to change (SCAN, SCAN_APPROACH, ERROR, unknown: all; PLANNING: confirmed instances; HOPPING: the target only; the post-spray
climb, until 2 m from the hover point or SCAN, SCAN_APPROACH included (D67): the target only; SPRAYING/IDLE/RETURN: asleep). It never shrinks: the published outline is monotone under inclusion
of every earlier one. Hovering cannot inflate it beyond cells that are canopy in a majority of judged frames.

**Height (M4).** Every latched cell carries a fused surface height (weighted running mean of its per-frame median z,
saturating weight 30, outlier gate 2 m once established); the tree height is the 0.99 quantile of those cells, finite by
construction once >= `min_height_cells` latched cells carry a height, which is a condition of publication (rule 3). Cell
heights are fused in creation phases only and the published value moves at <= 0.5 m/s, so a locked, frozen or sprayed
instance's height cannot drift or jump (section 10).

**Identity.** IDs come from a monotonic counter and are never reused (expired tentative IDs and merged-away IDs are
retired). Matching is geometric (top inside outline / near centre), so detector track changes are invisible. Merges
happen only in creation phases (a merge retires an ID navigation may hold) and only when the latched cells overlap by
>= `merge_overlap` of the smaller instance or the centres are closer than `merge_centre_dist`; the winner is
confirmed > more frames seen > lower ID, and keeps its ID and its height samples (rule 4). The size rule makes a small
fragment inside a large tree merge easily, while two large trees would need half of the smaller one covered.

## 4. How each rule of brief section 7 is guaranteed (and tested)

> Historical (M3), superseded by section 10 and `docs/DESIGN.md` (see the note under section 2).

| Rule | Guarantee | Test |
|---|---|---|
| 1 IDs stable, never reused | monotonic `next_id_`; merges keep the established ID | `IdsNeverReusedAndMergeKeepsEstablished` |
| 2 no silent damage | latch needs >= 3 frames and a majority; latched cells never unlatch; median height | `OneBadFrameNeverCorruptsAPublishedTree`, `HoveringDoesNotInflateTheOutline` |
| 3 height always real | publication requires >= 3 latched cells with a fused height; NaN / near-drone cell heights dropped; the 0.99 quantile of the latched surface is rate-limited | `PublishesOnlyAfterConfirmationWithFiniteRobustHeight`, `HeightFromLatchedSurfaceAndRateLimitedPublication` |
| 4 one tree one entry | watershed separation; overlap/centre merge with established winner; a crown covering a published tree's cells is that tree (seed guard) | `TouchingNeighboursStaySeparate`, `IdsNeverReused...`, `SeveralCrownsOnOneTreeAllCount`, `FragmentNeverTakesOverAPublishedTree`, `MergeNeverJumpsThePublishedValues` |
| 5 sprayed protected | sprayed instances get no crowns, no evidence, no merges; crowns inside a sprayed outline never seed | `SprayedTreesFrozenAndProtected` |
| 6 fail loudly | see the gate table below; every trigger is counted and the first occurrence logged | `HeartbeatsHoldsAndSleepAreCounted`, descent test |
| 7 every drop counted | `counters()` (points, cells, crowns, height samples, instances; listed below) | all tests read counters |
| sec. 6 phases | `phasePolicy()` (shared pure function) + post-spray hold | `NoNewTreesAfterScanning`, `OnlyTargetChangesDuringDescentAndPostSprayClimb`, `PostSprayHoldOutlastsScanApproachUntilTheDroneLeaves` |

**Gates and their failure direction (rule 6):**

| Gate | Missing information | Direction | Logged / counted |
|---|---|---|---|
| mission phase | unknown / none | open (behave as SCAN) | warn once; `frames_total` |
| drone pose | invalid | closed (hold everything) | warn once; `frames_held_no_pose` |
| hop target in HOPPING | none | closed (freeze every tree) | warn once; `crowns_dropped_non_target_frozen` |
| post-spray climb | hover position unknown | closed until the first valid pose after SPRAYING, then until 2 m away from it or SCAN starts; SCAN_APPROACH does not end it (D67) | `frames_post_spray_hold`, `post_spray_holds_started` |
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

> Historical (M3), superseded by section 10 and `docs/DESIGN.md` (see the note under section 2).

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

> Historical (M3), superseded by section 10 and `docs/DESIGN.md` (see the note under section 2).

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

> Historical (M3), superseded by section 10 and `docs/DESIGN.md` (see the note under section 2).

Per processed frame on the x86 workstation (RelWithDebInfo, one core), `stage_ms` in the cards: `chm` (back-projection
of 16000 points, binning, medians + MAD, plane fit, smoothing, watershed) about 0.7-0.9 ms mean, `instances` (matching,
evidence, latching, hulls, merges) about 0.01-0.05 ms. Memory: one 28 x 28 evidence window (about 4 kB) per live instance.
Expect several times the x86 figure on the Orin (not measured); the budget of 10 ms per frame leaves an order of magnitude.

## 8. Results (final frozen configuration; details in `results/cand_obj_*.md`)

> Historical (M3), superseded by section 10 and `docs/DESIGN.md` (see the note under section 2).

Recorded Isaac mission (used during tuning; not an unbiased test; it was tuned on, section 5): recall 9/9 (simple 8, existing 7), mean / min IoU 0.86 / 0.78
(0.73 / 0.46, 0.63 / 0.39), 0 swallowed neighbours (1, 2), centre RMSE 0.21 m (1.10, 1.36), spill 0.05 (0.14, 0.16),
**0 safety violations** (0, 5), 7 trees sprayed, latency 2.2 s (2.4, 1.2), 0.85 ms per frame.

Synthetic suite_v1, paired by seed against `simple` (15 pairs): recall 14/0/1, precision 15/0/0, mean IoU 15/0/0,
neighbours swallowed 15/0/0, false trees 15/0/0 (0 on every mission), centre RMSE 15/0/0, height RMSE 14/1/0, spill
15/0/0, centre jumps 15/0/0, trees sprayed 12/0/3; latency 0/15/0 (1.0 s slower). Medium: recall 0.98, precision 0.99,
IoU 0.76, 1.0 swallowed, 0 false, centre 0.46 m, height 0.19 m, spill 0.19.

## 9. Answers to the brief's section 10 questions, as far as this candidate goes

> Historical (M3), superseded by section 10 and `docs/DESIGN.md` (see the note under section 2).

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

## 10. M4 changes (hardening, 2026-10-24 19:00-22:00)

Work list: `results/m3_comparison.md` ("M4 work list"). Tuned on suite_v1, the fixed-hover suite_v2 and the recorded
Isaac mission; the held-out suite was run once at the end with the frozen configuration (`results/cand_obj_m4_holdout.md`,
`results/cand_obj_m4_isaac.md`, `results/cand_obj_m4_synthetic.md`; all paired against the M3 cand_obj build of commit
89317b2, cand_grid and simple). Configuration: 45 keys, `config/cand_obj.cfg` == built-in defaults (test).

### 10.1 Height statistic (task 1): quantile of the latched surface

*Problem.* M3's height was the median of per-frame crown-top samples. On the recording two trees published 4.27 / 4.39 m
against measured canopy tops of 5.4-5.9 m. The map stream shows why: both entered at the swath edge, the first visible
slice has its own "top" (the highest cell of the slice, not of the tree), those samples fill the median first, and the
sample buffer is kept centred on the median, so the early low samples never leave. The +-2.5 m tails on medium
(held-out seed 105, IoU 0.25 / 0.40 entries) are the same mechanism with a neighbour's cells in the slice.

*Change.* Each instance's evidence window now carries, per cell, a fused surface height: a weighted running mean of the
cell's per-frame median z (cand_grid's `fuseCell` model: weight saturates at `cell_weight_cap` 30 so hovering cannot
inflate confidence; once a cell has weight >= `gate_min_weight` 3 a frame value more than `outlier_gate_m` 2 m away is
rejected, three rejections in a row re-initialise it). The tree height is the `height_quantile` 0.99 quantile over the
latched cells with a height. Height fusion of a cell needs its z below drone z - `height_sample_clearance` (2 m), as
before per sample. Heights are fused in **creation phases only** (SCAN, SCAN_APPROACH, ERROR, unknown): once the list is
locked the descent reference does not move, except by the rate limiter finishing a step already in progress.

*Quantile.* 0.97 (cand_grid's value) left synthetic heights biased -0.08 m on easy (the 97 % quantile of a dome's cells
sits at top x sqrt(0.97)) and the recording at 4.18-5.44 m; 0.99 gives -0.04 / +0.01 / +0.06 m (easy / medium / hard,
suite_v1) and 4.83-5.74 m on the recording (cand_grid 4.75-5.76, simple 4.8-5.7, p99 point tops 5.4-5.9). The two
formerly low trees now publish 5.47 and 5.62 m.

*Merges.* The loser's cell heights are merged into the winner's cells (weighted), so the merged surface includes what
the fragment measured; the published height then moves toward the union's quantile at the bounded rate. The winner keeps
its ID (rule 4). A union can only raise the quantile relative to "winner keeps its own surface", which is the safe direction.

### 10.2 Publication stability (task 2): rate-limited centre and height

The published centre moves toward its target (the latched centroid here; since D66 the centroid of the supported cells) at <= `max_centre_rate` 0.8 m/s and the published height toward
the quantile at <= `max_height_rate` 0.5 m/s (one step per frame, dt capped at 0.5 s, only for instances the phase
policy lets change; sprayed / frozen instances are never stepped). The outline is not delayed, so the scorer's
first-publish time (IoU match + finite height) is unchanged by this: centre and height jumps went from 3.2 / 0.6
(easy), 8.6 / 0.2 (medium), 6.4 / 1.0 (hard) per suite_v1 mission and 7 / 4 on the recording to **0 everywhere**, at no
latency cost (the latency changes below come from the other items). The observed-peak-neighbourhood gate was not
adopted: it costs latency and the jumps were already gone.

### 10.3 Relative-height latch (task 3): implemented, shipped off

`latch_rel_h`: a crown cell whose smoothed height above ground is below this fraction of the crown's peak is never
claimed (unknown for the instance, so it neither latches nor counts against the tree). Measured (suite_v1, 5 seeds;
recording):

| latch_rel_h | easy IoU | medium IoU | medium spill | hard recall | Isaac IoU | Isaac spill |
|---|---|---|---|---|---|---|
| 0 (shipped) | 0.913 | 0.787 | 0.166 | 0.823 | 0.867 | 0.064 |
| 0.3 | 0.913 | 0.811 | 0.121 | 0.549 | - | - |
| 0.5 | 0.813 | 0.796 | 0.042 | 0.515 | 0.799 | 0.011 |

It trims spill as advertised, but on hard the per-frame peak height is a noisy maximum, the threshold rises with it and
trees at the swath edge never reach `min_publish_area` before leaving the view: recall 0.82 -> 0.55. cand_grid applies the
same rule to a multi-frame fused surface, where it is cheap. Left in as an option at 0.

### 10.4 Weighting (task 4): range, off-axis and rotation rate

Point weight `(z_ref/Z)^4 / (1 + lateral_k rho^2)` (`z_ref` 15 m, `lateral_k` 10) and the frame quality
`1/(1+(rate/rot_rate_ref)^2)` with a 1.5 s linear recovery after a fast rotation (cand_grid's `frameQuality`, now a pure
function in cand_obj too). The point weight scales each cell's height contribution; the frame quality scales both the
height weight and the per-cell evidence increments (`npos` / `nneg` are now quality-weighted frame counts: a frame taken
in a fast turn counts as a fraction of a frame, `latch_frames` is unchanged for clean frames). `rot_rate_ref` 0.35
(cand_grid's) cost 0.1-0.2 s latency on every dataset for no geometry gain, so it ships at 1.0 rad/s; switching the
weighting off entirely (sweep D) was faster still (easy 0.87 vs 1.17 s, Isaac 2.21 vs 2.32 s) but produced split trees
on easy (0.4 / 0.2 per mission on suite_v1 / suite_v2; turn-blur fragments at the row ends) and lower min IoU
(0.79 vs 0.87 easy). A split tree is a double target, 0.1-0.3 s is not, so the weighting stays on.

### 10.5 Transient duplicate (task 5): overlap matching and seed guard

*Problem.* On the recording, truth tree 2 was matched by ID 11 -> 12 -> 11 within 0.6 s at 178 s: ID 11 was a 6 m^2
slice born at the swath edge, the next crowns of the same tree had their top 2.06 m from the slice centroid (outside
`match_radius` 2.0 and outside the slice's outline), so a second instance (12) was born, published, and merged back
0.6 s later.

*Change.* A crown that covers >= `match_overlap` 0.5 of an instance's latched cells, with its top within `match_reach`
3 m of the instance centre, is that instance wherever its top lies; among several such instances the better-established
one wins (confirmed > frames seen > lower ID), so a tentative fragment can never take a crown away from a published
tree. **Seed guard:** a crown that would otherwise seed a new instance is credited to a published tree if >= `seed_overlap`
0.25 of its cells lie on that tree's latched cells (top within reach). Without the guard, the first overlap rule alone
turned the transient duplicate into a permanent one (the second instance was born from a crown covering only part of
the slice, then starved once the tree took every later crown, and stayed a published 4 m^2 fragment: precision 0.9 on the
recording). The reach limit matters on hard (4.5 m spacing): a merged basin whose top is on the neighbour must not be
credited to the tree (hard recall 0.78 with a 4 m reach, 0.83 with 3 m). Rule test `FragmentNeverTakesOverAPublishedTree`:
a tree published from a slice, then shown whole with its top > match_radius from the slice centroid and outside the
slice's outline, gets no second ID. Recording: 0 ID switches after lock, precision 1.0.

### 10.6 Tests (task 6)

20 cand_obj tests (15 M3 + 5 M4: fragment takeover, latched-surface height + rate limit, skirt trim, frame quality
weighting incl. the pure `frameQuality`, merge without jumps). The config-file test now covers 45 keys. Removed keys:
`height_buffer`; renamed: `min_height_samples` -> `min_height_cells`.

### 10.7 Held-out verdict (30 missions, run once with the frozen configuration) and what got worse

Paired per seed against the frozen M3 build (`results/cand_obj_m4_holdout.md`; W / L / T = seeds M4 better / worse / equal):

| level | IoU | min IoU | centre RMSE | spill | centre / height jumps | swallowed | split | recall | latency |
|---|---|---|---|---|---|---|---|---|---|
| easy | +0.004 (8/2) | -0.008 (4/4) | -0.001 | -0.011 (10/0) | 2.5 / 0.1 -> 0 / 0 | 0 = 0 | 0.1 -> 0.2 | 1 = 1 | -0.03 s |
| medium | +0.024 (10/0, t 4.0) | +0.093 (7/3) | -0.082 (9/1, t -3.6) | -0.011 (9/1) | 7.4 / 1.1 -> 0 / 0 | 1.0 = 1.0 | 0.4 -> 0.1 | **0.977 -> 0.960** (1/3/6, t -1.2) | **+0.18 s** (1/9) |
| hard | +0.028 (10/0, t 3.6) | +0.037 (6/4) | -0.044 (7/3) | -0.017 (9/1) | 6.3 / 0.8 -> 0 / 0 | 3.6 -> 2.8 (6/0) | 0.5 -> 0.1 (5/1) | **0.866 -> 0.831** (2/6/2, t -2.2) | **+0.27 s** (3/7) |

Heights (held-out): RMSE 0.050 / 0.257 / 0.480 m -> 0.038 / 0.232 / 0.405 m; matched trees off by > 1 m (under-estimates)
4 (2) -> 5 (0) on medium and 10 (5) -> 7 (1) on hard; max |error| 2.53 -> 2.09 m (medium), 2.30 -> 1.93 m (hard); bias
+0.02 / +0.06 / +0.18 m by level (M3 +0.02 / +0.02 / +0.11): the remaining tail is on the high side, the safe direction.
Target height range during its own descent: 0 on every mission (M3 up to 0.62 / 1.79 m on medium / hard, cand_grid
0.49 / 1.60). Safety: 0 everywhere. Recording: IoU 0.859 -> 0.867, min IoU 0.779 -> 0.817, ID switches 2 -> 0, centre /
height jumps 7 / 4 -> 0 / 0, heights 4.27-5.83 -> 4.83-5.74 m, height drift since lock 0.03 -> 0 m.

**Worse, and why:**

- **Recall**: medium -0.018 (within seed noise, but the worst seed drops from 0.94 to 0.82: medium_s100 publishes 14 of
  16 trees, M3 15) and hard -0.035 (t -2.2; 6 seeds lose 1-3 trees, 2 gain one). In every losing seed the published count
  is lower while swallowed neighbours are equal or lower: the missing trees were never confirmed, not merged away. Two
  mechanisms, both deliberate: the frame-quality weighting makes cells latch later at the row ends (latency up on the
  same seeds, e.g. medium_s100 2.45 -> 3.77 s, hard_s104 1.91 -> 3.53 s), so a tree seen only around a turn can leave
  the view before `min_publish_area`; and the seed guard credits a crown that lies >= 25 % on a published tree's cells to
  that tree, which on hard (4.5 m spacing, merged boxes) sometimes is a neighbour's first slice. Against cand_grid the
  hard recall advantage is kept (+0.073, 8/0; swallowed -2.5, false trees -0.8, precision +0.048).
- **Latency**: medium +0.18 s, hard +0.27 s, recording 2.23 -> 2.32 s (cand_grid 2.30 s). Easy is 0.03 s faster. Still
  below cand_grid on every held-out level (1.14 / 2.02 / 2.51 vs 1.40 / 2.38 / 2.66 s); on the recording 0.02 s above it.
  The rate limiter itself costs nothing (sweep D without the frame weighting: Isaac 2.21 s, easy 0.87 s, but split trees).
- **Max height drift since lock** is not 0 on the synthetic suites (0.82 m worst on medium, 1.21 m on hard; M3 1.37 /
  1.91): those missions re-enter SCAN for a second corridor, where heights are fused again by design (the scorer counts
  from the first locked phase). On the recording and inside every descent it is 0.
- **Spill on the recording** 0.052 -> 0.064 (neighbour spill 0.022 -> 0.056): the overlap matching lets a tree's crown
  reach the known cells a little earlier; still a third of cand_grid's 0.093.
- **Cost**: `chm` 0.93 ms per frame vs 0.85 (x86).

### 10.8 Cost

`chm` 0.93 ms (was 0.85) per processed frame on x86 (point weights, three more per-cell arrays); `instances` 0.01 ms.
Memory per live instance: 28 x 28 cells x (2 x 4 B evidence + 1 B latched + 4 B height + 4 B weight + 1 B gate) ~ 14 kB.
