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
