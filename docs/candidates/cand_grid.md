# cand_grid: map-centric candidate (M3b)

Code: `core/include/treemapper/candidates/cand_grid/cand_grid.hpp`, `core/src/candidates/cand_grid/`
(`cand_grid_pure.cpp` = configuration, pure decision functions, tiled grid, segmentation;
`cand_grid_mapper.cpp` = the mapper). Tests: `core/tests/candidates/cand_grid/test_cand_grid.cpp`.
Configuration: `config/cand_grid.cfg` (equals the built-in defaults; a test enforces it).
Scorecards: `results/cand_grid_synthetic.md`, `results/cand_grid_isaac.md`.

## 1. The idea in one paragraph

Every valid stereo point of every frame, not only the points inside a detection box, is fused into a
world-fixed, sparse, tiled **2.5D surface grid** (a DSM: 0.25 m cells; per cell a fused surface height, a
saturating confidence weight, a frame count, the spread of its per-frame measurements and a count of frames
in which a detection box covered it). Trees are then **extracted from the map**, not from detections:
ground = a robust low surface, CHM = DSM − ground, and a **persistence watershed** on the smoothed CHM yields
one basin per canopy dome. Basins become persistent **instances**. An unpublished instance is re-derived from
scratch at every extraction, so early segmentation mistakes cost nothing. A published instance owns
**latched cells**. Its outline is the convex hull of those cells and only grows. The detector boxes give
semantic support ("this raised surface is a tree"), never geometry.

## 2. Pipeline per frame

1. **Gates** (each counted, rule 7): invalid disparity, optical range outside [0.5, 25] m, point above
   drone z − 0.3 m (physically impossible), outside the target region during a descent.
2. **Point weight** `w = (z_ref/Z)^4 / (1 + k·ρ²)` with k = 10, where ρ is the off-axis tangent. Stereo depth error grows
   like Z², and its lateral component like ρ·σ_Z, so oblique far points weigh little. A **frame weight**
   `q = 1/(1+(ω/ω_ref)²)` from the body rotation rate ω (from consecutive pose quaternions), which recovers
   linearly over 1.5 s after a fast rotation. This handles the brief's blur and noise bursts in corridor turns.
3. **Per-frame, per-cell robust measurement**: the points of one cell in one frame are reduced to their
   **median** z. Their weight is the mean point weight × min(n, 6), so dense or close views cannot buy
   extra confidence by point count.
4. **Cell fusion** (`fuseCell`): a weighted running mean with a **saturating weight** (cap 30). After the
   cap it behaves like an exponential average, so hovering cannot inflate confidence. A weighted running
   variance gives the spread. Once a cell is established (w ≥ 6), a frame median more than 2 m off is rejected.
   Three rejections in a row re-initialise the cell, because then the established value was the outlier.

Every 0.1 s an **extraction** runs over the window covered since the last one, plus 6 m:

5. **Ground**: the 20 % quantile of the well-observed cells (≥ 3 frames, w ≥ 3) per 1 m coarse cell, then a
   min filter over 4.5 m as a first guess. The guess is refined to the mean of cells within 0.6 m above it, in
   a 1.5 m neighbourhood. This removes the min filter's bias from noise, slope and undulation. Without the
   refinement, outlines were 2-3× too large: ground cells passed the canopy threshold.
6. **CHM and canopy mask**: CHM ≥ 0.5 m. The CHM is smoothed 3×3.
7. **Persistence watershed** (`segmentCanopy`): flood from the highest cell downwards with union-find. When
   two basins meet, the lower one survives only if its peak rises ≥ 0.8 m above the saddle. Otherwise it
   merges, as a noise bump on one dome would. Cells more than 3.6 m from the basin peak are cut. Per segment:
   peak CHM, box-supported fraction, open fraction (boundary cells next to unobserved cells), whether the
   peak neighbourhood is fully observed, and whether the segment touches the window border (then it is
   skipped).
8. **Instances**:
   - A segment belongs to the **published** instance that owns most of its cells, provided that instance owns
     the peak cell, or has its peak within 1.5 m, or owns ≥ 50 % of the segment. Otherwise it is a different
     dome next to a neighbour's cells.
   - Failing that, it belongs to the nearest **unpublished** instance with its peak within 1.5 m.
   - Failing that, a new instance is created, only in creation phases and never with its peak inside a
     sprayed tree.
9. **Publication** (`publishDecision`). The confidence model is in section 3.
10. **Growth of a published tree**: a cell latches only if it is confident and assigned to the same tree in
    two consecutive extractions, from a segment that is not too open.
11. **Fragment merge**: two published instances whose peaks fall into one basin for 3 consecutive
    extractions merge, if the smaller is ≤ 0.6 × the larger, neither is sprayed and the phase allows merges.

## 3. Confidence model: when a tree is published, and when its geometry may still change

Publishable = all of the following, each a different kind of evidence (pure `publishDecision`, tested):

| Evidence | Gate (default) | Defends against |
|---|---|---|
| geometric: latchable cell area | ≥ 5 m² of cells with w ≥ 3, ≥ 3 frames, spread ≤ 1.0 m, CHM ≥ 0.6 × peak CHM | noise bursts, single bad frames, clutter fragments |
| shape: peak height above ground | smoothed peak CHM ≥ 1.8 m | bushes and ground clutter (0.2-0.9 m) |
| observation completeness | peak neighbourhood (1 m) fully observed, open fraction ≤ 0.6 | partial views at the swath edge |
| semantic | ≥ 30 % of the segment's cells covered by a detection box in ≥ 2 frames | non-tree raised objects |
| temporal | seen as such in 2 consecutive extractions (0.2 s) | one-extraction artefacts |
| phase | creation allowed (SCAN, SCAN_APPROACH, ERROR, unknown) | new trees after scanning |
| height | finite | rule 3 |

Why this is single-pass and hover-invariant: every gate is reached in a fraction of one cruise pass.

- At 15 m altitude and 4 m/s a tree stays in view for about 3 s, that is about 45 frames at 15 Hz.
- The cell gates need 3 frames.
- Confidence is a saturating weight per cell, not a count of observations of the tree. A tree seen for a
  minute and one seen once at cruise speed therefore end with the same confidence and the same
  (exponentially averaged) surface. There is a test for this: hover 30 s versus one pass gives an outline area
  within 25 % and a height within 0.3 m.

When geometry may change after publication:

- **Outline**: grow-only. It is the convex hull of all latched cells plus the previous outline. A cell
  latches only when it has multi-frame (≥ 3), weighted (w ≥ 3), consistent (spread ≤ 1.0 m) evidence. It must
  also belong to the same tree in two consecutive extractions, from a segment that is closed enough. A single
  frame cannot latch a cell: one frame contributes at most min(n, 6) × weight, and in a turn q is about 0.1. A
  few bad frames cannot shrink anything, because nothing is ever removed.
- **Height**: recomputed every extraction as the 97 % quantile of the latched cells' surface heights. Each
  cell height is a capped-weight average, so one frame moves it by at most w_f / (W + w_f). The published
  value may move at most 0.5 m/s (`max_height_rate`, independent of the extraction period), so it never jumps. An undefined recomputation keeps
  the last finite value.
- **Centre**: the outline centroid. It moves only through growth.
- Rule 2 test: after a good pass, 3 frames with every disparity +2 px and 2 frames at 3 px noise leave every
  outline containing its old outline, the centre within 0.5 m, the height within 0.6 m, and the area below
  1.3×.

## 4. Identity model

- IDs are handed out **only at publication**, from a monotonic counter, and never decrease. Unpublished
  instances use internal keys, so candidates that never confirm burn no IDs.
- A published instance is bound to its **latched cells**, a set of world cells. It is not bound to a track or
  a detection. Re-segmentation every extraction cannot steal an ID: a new segment inherits the ID of the
  published tree that owns its cells, and a basin elsewhere cannot.
- There are no tracks, so track loss or a track jumping to another tree does not exist in this design. The
  brief's failure mode, deleting the old tree when its track jumps, cannot happen: nothing published is ever
  deleted.
- **Merge** (rule 4): a fragment's ID is retired forever. The survivor ranks published > more latched area >
  lower key (`mergeSurvivor`). It keeps its ID and height and absorbs the fragment's cells. Merges are
  size-aware (`mergeAllowedBySize`): a small fragment merges into a large tree, two large trees never merge on
  overlap alone. Two neighbours are never merged, because they are two basins (a valley ≥ 0.8 m below the lower
  peak).

## 5. Answers to the brief's section 10 questions (for this design)

- **Touching trees with a gap below the depth noise.** Separation happens on the fused surface, not on
  points. A cell that hundreds of points fall into has a height error far below the per-point σ_Z: about 0.9 m
  per point at 15 m for medium-level noise, about 0.1-0.2 m per fused cell. The valley between two domes
  (the base height, 0.5-1.5 m above ground, versus tops 2.5-6.5 m) is a property of the surface even with a
  zero gap. The persistence threshold (0.8 m) separates real domes from noise bumps. Merged detector boxes do
  not matter, because boxes never shape segments. Test: two domes with a 0.2 m gap at 0.3 px noise, under a
  single image-wide box, give 2 entries.
- **Ground and neighbour points, given rule 2.** Ground points are not discarded per point. They define the
  ground and the CHM, and a cell is canopy only if its fused surface is ≥ 0.5 m above the ground. Neighbour
  points land in the neighbour's basin. A box corner on a neighbour or on the ground therefore cannot add to
  an outline. Latching additionally requires CHM ≥ 0.6 × the tree's peak CHM. That trims the low skirt
  where stereo noise smears canopy points into occluded ground, the main spill source in this design. Each
  of these is a pre-publication filter on cells, never a removal from a published outline.
- **IDs through track loss, jumps and merges.** See section 4: there are no tracks, IDs bind to cells.
- **Confidence model.** See section 3.
- **Reusing an earlier flight's map.** A design only, not implemented.
  - Store the confidence-latched DSM tiles and the published instances: outline, height, peak.
  - Next flight: build a fresh DSM during the first corridor. Register the new peak constellation against the
    old one with a 2D rigid transform, using RANSAC on peak triples plus height and crown-area descriptors.
  - The orchard grid is repetitive, so accept a registration only if it is unique: the best hypothesis must
    clearly beat the second best, which must be off by at least one row or tree spacing. Missing trees, height
    differences and the field edge break the symmetry. Accept it only if the GNSS offset prior (a few metres)
    agrees.
  - Until a registration is accepted, use the old map only as a prior for the search, never as published
    geometry. Even then, re-confirm each tree in the current flight before it is published.

## 6. How each brief rule is guaranteed (and tested)

| Rule | Mechanism | Test(s) |
|---|---|---|
| §7.1 IDs stable, never reused | IDs only at publication, monotonic counter; IDs bind to latched cells; merges retire the loser's ID | `Rule1_IdsAreStableAndNeverReused`, `MergeRulesAreSizeAware...` |
| §7.2 no silent damage | grow-only hull of latched cells; multi-frame, persistent, consistent latching; height step limit | `Rule2_BadFramesNever...`, `OutlierFrameRejected...`, `HoveringSaturates...` |
| §7.3 height always real | publish needs a finite height; an undefined recomputation keeps the last value; `publishedTrees()` filters as a last line | `Rule3_HeightIsAlwaysFinite...`, `PublishDecisionNeeds...` |
| §7.4 one tree = one entry | persistence watershed on the fused surface; size-aware fragment merge; survivor ranking | `Rule4_TouchingTrees...MergedBox`, `SegmentationSeparatesTouchingDomes...`, `MergeRules...` |
| §7.5 sprayed protected | sprayed instances never change (`instanceMayChange`), never merge; a basin whose peak is inside a sprayed outline creates nothing | `Rule5_SprayedTreeNeverChanges...` |
| §7.6 fail loudly | see the gate table below; every trigger is counted and the first one logged | `Rule6_InvalidPose...`, `DuringADescentOnlyTheTargetChanges` (no target), `UnknownKeyIsRejectedAndClampsAreLogged` |
| §7.7 drops counted | `counters()`: `points_dropped_*`, `segments_*`, `publish_blocked_*`, `frames_*` | `Rule7_DroppedPointsAreCountedByReason` |
| §6 scan / plan / hop | `phasePolicy()` (shared), plus: unpublished instances frozen in locked phases | `Section6_NoNewTreesAfterScanning...` |
| §6 descent | only the target changes; the DSM is updated only within 1.5 m of the target's bounding box; the freeze also holds through the **climb-out** after SPRAYING until the drone has moved 1.5 m (`climbOutPolicy`) | `Section6_DuringADescent...`, `Section6_ClimbOut...`, `PhaseRulesForInstanceChanges` |
| §6 sleeping, unknown phase | SPRAYING/IDLE/RETURN: no processing, `frames_skipped_sleeping` (the heartbeat); unknown phase: scanning + warning | `Section6_SleepsWithHeartbeat...` |
| §8 start-up | invalid pose holds everything (nothing enters the grid) | `Rule6_InvalidPose...` |
| §8 hovering | saturating weights; per-frame per-cell point cap | `Section8_Hovering...`, `HoveringSaturates...` |
| config | defaults == `config/cand_grid.cfg`; unknown keys throw; clamps in `validate()`, each logged | `ShippedConfigFileEqualsBuiltInDefaults`, `UnknownKeyIsRejected...` |

Gates, and whether each fails open or closed:

| Gate | Missing information | Behaviour |
|---|---|---|
| pose | invalid | **closed**: hold all processing, log once, `frames_held_no_pose` |
| phase | missing / unknown | **open**: scanning behaviour, log once (brief §6) |
| hop target | HOPPING without target, or target unknown to the map | **closed**: freeze everything, `frames_dropped_frozen_no_target`, log once |
| rotation rate | no previous pose (first frame, after sleep) | **open**: frame weight 1, `frames_quality_unknown_rate` |
| ground | no well-observed cell nearby | **closed**: CHM undefined, so no canopy there |
| segment at the window border | tree may continue outside | **closed**: skipped this extraction, `segments_skipped_window_border` |
| partially observed tree | open boundary / peak not observed | **closed** for publication and growth (`publish_blocked_open`, `growth_skipped_open_segment`) |
| frame footprint absurdly large | > 4 M cells | **closed**: frame dropped and counted |

## 7. Cost

- Per frame, x86 workstation, one core, RelWithDebInfo (`results/cand_grid_synthetic.md`):
  - integrate 0.31-0.50 ms mean: back-projection of 16 000 grid points, counting sort into cells, fusion;
  - extract 0.30-0.69 ms mean, amortised over one extraction every 0.1 s;
  - total about 0.6-1.2 ms per frame, against the 10 ms budget; the worst 0.2 s snapshot means are
    1.03 ms (integrate) and 1.97 ms (extract).
  - On Isaac (27 Hz): 0.47 + 0.11 ms.
- Expect several times this on the Orin. **Not measured on the Orin.**
- **Memory**: 28 B per cell, so 28.7 kB per 8 m x 8 m tile of 32 x 32 cells. Only observed tiles exist
  (`gauge_grid_tiles` and `gauge_grid_bytes` in counters.json). Measured: hard_s00 108 tiles (3.1 MB),
  medium_s00 90 tiles (2.6 MB), Isaac 38 tiles (1.1 MB); the largest mission is 3.2 MB. That is about 450 B per observed m², so 10 ha is
  about 45 MB. Tiles are never evicted, so a much larger area would need finished tiles evicted, which is not
  implemented.

## 8. Known failure modes

- **Latency**: publication waits for an observed peak neighbourhood, a mostly closed dome and 2 extractions.
  - A trace on medium_s03 shows the sequence for a typical tree, measured from its centre entering the
    scorer's footprint circle. Its basin appears after about 0.7 s. It is blocked as "open" (peak
    neighbourhood not yet observed) for about 0.5 s. Then come 2 confirmations, and it is published at
    1.2-1.7 s. `simple` publishes at 0.5-1.1 s.
  - A few trees first seen only at a swath edge are published in the next corridor, 11-21 s later. Those
    dominate the mean scan latency: about 2.5 s on medium, 2.3 s on Isaac, against 1.2 s for `existing`.
  - Single-pass capture holds: recall comes from one pass, no hover is needed. But this design is not the
    fastest to publish.
  - Extracting every 0.1 s instead of every 0.2 s (frozen) bought 0.1-0.3 s and also raised precision.
  - Looser gates (peak neighbourhood 0.5 m, 1 confirmation) bought a further 0.1-0.4 s and cost precision on
    hard.
- **Spill on oblique, occluded skirts**: points near occluded ground beyond the far silhouette are smeared by
  the line-of-sight error. Those cells never see ground, so they look raised. The relative-height latch rule
  and the lateral weighting (k = 10) trim most of it. Final spill: 0.06 (easy), 0.18 (medium), 0.28 (hard),
  0.09 on Isaac.
- **Hard is still hard**: recall 0.78, IoU 0.63 and 3.8 neighbours swallowed per mission (simple: 25), under
  5× real stereo noise with mostly overlapping canopies.
- **The 0.6 × peak-CHM latch rule makes outlines a "core plus"**: low crown edges (a dome's last ~10 % of
  radius) are not latched, so outlines are slightly smaller than the true crown on clean data.
- **Implausible-height safety counts.** This is the only non-zero safety count: medium 7.4 (one seed),
  hard 17 (three seeds). simple has 132 on hard.
  - Every event involves a tree whose true top is above the hovering drone's world z (sloped ground: hard_s00,
    hard_s01, hard_s03) or within 0.02-0.13 m of it (medium_s00, three entries in hard_s00).
  - cand_grid's error on those trees is between -0.73 m and +0.49 m. The per-event table is in
    `results/cand_grid_synthetic.md`.
  - Clipping heights to the drone altitude would publish a too-low descent reference, so it is not done.
  - A lower height quantile (0.93) halves the counts, but it biases heights by -0.1 m and underestimates the
    Isaac canopy tops by up to about 1 m. It was rejected.
  - This needs a scorer / scenario decision: the scorer test is ill-posed on sloped ground.
- **Fragment merge outside the window**: two published fragments merge only while both are inside the
  extraction window (the recent footprint plus 6 m). Fragments far from any current view are not
  consolidated until the drone sees them again.
  - Merges do occur: `instances_merged` per mission is 0 on easy, 0-8 on medium, 1-6 on hard and 0 on Isaac.
  - `merge_blocked_size` (two large instances refused) reaches 14.
  - `merge_blocked_phase` is 53 on Isaac: merge votes during PLANNING / HOPPING, correctly refused.
  - The merge rules have pure tests only. There is no end-to-end fragment test.
- **The ground model assumes the ground is visible somewhere within ~4.5 m** of every canopy. A continuous
  hedge wider than ~9 m would lose its ground and its CHM. That fails closed: no tree is published there.
- **Real stereo**: tuned on synthetic data only, with Isaac as a check. The real flight's 15° off-nadir mount
  goes through the same world-frame maths but is **untested**.
- **Grid memory** is unbounded in area. There is no tile eviction.

## 9. What was tried and did not work

- **Plain min-filter ground** gave outlines 2-3× too large and IoU about 0.39 on medium, because the
  ground was biased low. The band-mean refinement fixed that.
- **CHM ≥ 0.5 m as the only outline criterion** smeared the skirts. The relative latch rule at 0.3 / 0.45 / 0.6
  gave medium IoU 0.50 / 0.59 / 0.67. At 0.7, IoU reached 0.73, but heights from the smaller core rose by about
  0.1 m. That put the tallest medium tree (6.97 m, drone hover at 6.99 m) above the drone. It was rejected
  as fragile.
- **Height quantile.** 0.93 biased heights low by about 0.1 m on synthetic. On Isaac it gave 4.5-5.6 m against
  measured canopy tops of 5.4-5.9 m, which is the unsafe direction for a descent reference. 0.99 / 1.0 (the max)
  biased synthetic heights up by 0.1-0.3 m. 0.97 with a 0.5 m/s rate limit was best: synthetic RMSE 0.07 / 0.17 /
  0.34 m, and on Isaac 4.7-5.8 m with 0 height jumps.
- **Height step per extraction.** A 0.1 m step per extraction became a 1 m/s limit once extractions ran every
  0.1 s, and Isaac then showed 5 height jumps. It was replaced by a rate in m/s; 0.5 m/s gives 0 jumps.
- **Cell-consistency (spread) gate.**
  - With no gate, hard had 32 implausible-height frames and 2 false trees per mission.
  - 1.5 m removed most of them.
  - The remaining false trees on medium (about 1 per mission) lay 3.6-10.5 m from any canopy, beyond the row
    ends: corridor-turn noise bursts. Their frame medians spread about 1.4 m.
  - 1.0 m, the frozen value, took medium precision from 0.94 to 0.97 and false trees from 1.0 to 0.4, at a cost
    of -0.03 recall on hard.
  - This gate is noise-scale dependent. Real stereo (about 0.1 px) gives a much smaller spread, so 1.0 m is
    permissive there. A 5×-real sensor starts to lose genuine cells.
- **Strong turn down-weighting** (rotation reference 0.2 rad/s, 2.5 s recovery) lost recall on hard
  (0.71 → 0.60): trees at the row ends are seen mainly while turning.
- **Disabling the per-cell outlier gate** cost precision (medium 0.97 → 0.87, a split tree on Isaac) and
  raised hard implausible heights.
- **Looser publication gates** (peak neighbourhood 0.5 m, or 1 confirmation at 10 Hz extractions) cut latency
  by about 0.3 s but cost precision and created false trees on hard.
- **Smaller minimum area** (2 m²) let small noise "trees" with 7 m heights through on hard. 5 m² removed
  them without changing medium.
