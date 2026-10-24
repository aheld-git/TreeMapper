# Baseline: existing system's recorded map, Isaac sim mission 2026-10-23-15-34-29

Final map at t = 288.37 s. Per-frame column = mean over every snapshot of the stream (1393) / mean over snapshots from the first non-empty one (1106).

## Detection

| Metric | Final map | Per-frame mean (all / from first publish) |
|---|---|---|
| Trees in truth / published / matched | 9 / 7 / 7 | — |
| Recall | 0.78 | 0.50 / 0.63 |
| Precision | 1.00 | 1.00 / 1.00 |
| Missed (absorbed by a merge / unseen) | 2 (2 / 0) | 4.52 / 3.36 |
| Merged entries / trees inside them / neighbours swallowed | 2 / 4 / 2 | 1.27 / 1.60 (swallowed) |
| Split trees (duplicates / fragments) | 0 (0 / 0) | 0.00 / 0.00 |
| False trees | 0 | 0.00 / 0.00 |

Missed truth IDs: [0, 3]. Merged entries (map id: truth ids covered): {'7': [3, 4], '1': [0, 1]}.

## Geometry (matched pairs)

| Metric | Final map | Per-frame mean (all / from first publish) |
|---|---|---|
| Mean IoU (min) | 0.63 (0.39) | 0.60 / 0.60 |
| Centre error mean / RMSE / max (m) | 1.09 / 1.35 / 2.36 | 1.33 / 1.33 RMSE |
| Height bias / RMSE / max abs (m) | 1.41 / 1.46 / 1.89 | 1.49 / 1.49 RMSE |
| Spill outside all true canopies | 0.16 | 0.15 / 0.15 |
| Neighbour spill mean / max | 0.15 / 0.41 | 0.13 / 0.13 |

| Truth id | Map id | IoU | Centre err (m) | Height err (m) | Neighbour spill |
|---|---|---|---|---|---|
| 1 | 1 | 0.43 | 2.08 | 1.14 | 0.33 |
| 2 | 29 | 0.86 | 0.22 | 1.87 | 0.00 |
| 4 | 7 | 0.39 | 2.36 | 1.35 | 0.41 |
| 5 | 31 | 0.55 | 1.28 | 1.28 | 0.15 |
| 6 | 9 | 0.86 | 0.13 | 1.89 | 0.00 |
| 7 | 10 | 0.65 | 0.58 | 1.55 | 0.00 |
| 8 | 32 | 0.66 | 0.96 | 0.81 | 0.15 |

## Spray outcome (final map)

Sprayed entries 7; truth trees sprayed 7 of 9; sprayed twice 0; orphan sprayed entries 0; not sprayed: [0, 3]. (truth tree sprayed iff a sprayed entry's centre lies inside its canopy.)

## Stability

| Metric | Value |
|---|---|
| ID switches, total / after the list locks (t = 87.35 s) | 1 / 0 |
| ID meaning changes, total / after lock | 1 / 0 |
| ID reappearances / reuse | 0 / 0 |
| Geometry retreats (>2% of outline lost in one update); max lost fraction | 0; 0.000 |
| Centre jumps (> 1.0 m) | 2 |
| Height jumps (> 1.0 m) | 14 |
| IDs vanished (of which held by navigation) | [2, 8] ([8]) |
| Truth trees ever split into >= 2 entries | [] |
| Truth trees ever inside a merged entry | [0, 1, 3, 4] |

## Safety (all must be 0)

| Metric | Value |
|---|---|
| Frames publishing an undefined height | 0 |
| New trees while the list is locked (PLANNING/HOPPING/SPRAYING) | 0 |
| Non-target changes during descent (370 frames judged) | 0 |
| Sprayed tree changed after spraying | 1 |
| New tree centred inside a sprayed tree | 0 |
| **Total violations** | **1** |

- `sprayed_tree_change_events` (first 1): [{"t": 145.976, "id": 7}]
- `id_switch_events` (first 1): [{"t": 81.382, "truth_id": 1, "from": 2, "to": 1, "after_lock": false}]
- `id_meaning_change_events` (first 1): [{"t": 81.187, "id": 1, "from": 0, "to": 1, "after_lock": false}]
- `centre_jump_events` (first 2): [{"t": 77.385, "id": 1, "moved_m": 2.731}, {"t": 145.976, "id": 7, "moved_m": 1.271}]
- `height_jump_events` (first 12): [{"t": 81.772, "id": 9, "dh_m": 1.284}, {"t": 82.179, "id": 1, "dh_m": 1.317}, {"t": 113.379, "id": 10, "dh_m": -2.354}, {"t": 113.785, "id": 10, "dh_m": 2.602}, {"t": 128.182, "id": 7, "dh_m": -1.285}, {"t": 131.774, "id": 7, "dh_m": 1.361}, {"t": 156.376, "id": 1, "dh_m": 1.249}, {"t": 179.581, "id": 31, "dh_m": 1.567}, {"t": 182.782, "id": 31, "dh_m": -2.602}, {"t": 194.174, "id": 32, "dh_m": -1.014}, {"t": 200.771, "id": 32, "dh_m": -1.682}, {"t": 210.375, "id": 31, "dh_m": 4.044}]

## Timeliness

first snapshot covering the tree (cov >= cover_min) minus the first time its centre came within the nadir footprint's half-diagonal (yaw-free bound); 'scan' variant counts view time only while the phase is SCAN.

Latency mean / max: 15.06 s / 52.68 s from first view in any phase; 2.86 s / 11.38 s from first view while scanning. Never published: [].

| Truth id | First in view (s) | First in view while scanning (s) | First published (s) | Latency (s) | Latency from scan view (s) |
|---|---|---|---|---|---|
| 0 | 60.19 | 65.80 | 67.98 | 7.78 | 2.18 |
| 1 | 59.40 | 65.80 | 77.17 | 17.78 | 11.38 |
| 2 | 170.90 | 174.80 | 178.38 | 7.47 | 3.58 |
| 3 | 59.10 | 78.80 | 80.78 | 21.68 | 1.98 |
| 4 | 56.80 | 79.50 | 80.78 | 23.98 | 1.28 |
| 5 | 178.10 | 178.10 | 179.78 | 1.67 | 1.67 |
| 6 | 80.60 | 80.60 | 81.98 | 1.38 | 1.38 |
| 7 | 81.30 | 81.30 | 82.37 | 1.07 | 1.07 |
| 8 | 128.30 | 179.79 | 180.98 | 52.68 | 1.19 |

## Cost

map stream carries no stage_ms

## Notes: inputs, definitions and comparison with DATA_README

**Inputs.** `/validation/fusion_snapshot` (1393 snapshots, ~5 Hz) and `/validation/ground_truth`
(9 trees, identical in all 557 non-empty messages) from `isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag`,
plus `/mission_state`, `/spray_target` and `/height_estimator/drone_position` for phases, targets and pose.
All non-empty snapshots are already in frame `world`; only the 287 empty pre-mission snapshots carry
`world_nwu` (the extractor converts per message anyway). Published height = `max_height` (canopy top,
the existing system's descent reference). The message's `centroid` is used as the centre (falls back to the
area centroid when it is zero = unset). Ground truth = the recorded 12-gon outlines, centre = vertex mean
(equals the `/tf` `world->Tree_XXXX` positions; the message `centroid` field is unset in the truth).
The existing system ran once, so no run-to-run noise floor exists for it.

**Comparison with DATA_README's table** (README value -> this scorer):

| README metric | README | Here | Why they differ |
|---|---|---|---|
| Trees truth / map / matched | 9 / 7 / 7 | 9 / 7 / 7 | same |
| Recall / precision, final | 0.78 / 1.00 | 0.78 / 1.00 | same |
| Recall / precision, per-frame | 0.50 / 1.00 | 0.50 / 1.00 | same when averaged over every snapshot of the stream (empty pre-mission frames score recall 0); 0.63 from the first publication on |
| Mean outline IoU final / per-frame | 0.61 / 0.59 | 0.63 / 0.60 | small definition difference; a true-circle truth gives 0.63 / 0.60 too, so not the 12-gon. Not chased further |
| Centre RMSE final / per-frame | 0.97 / 0.93 m | 1.35 / 1.33 m | **not reproduced.** Our pairs come from IoU matching; the two merged entries are matched to the tree their centre is nearest and contribute 2.08 m and 2.36 m. Dropping them gives 0.77 m, vertex-mean centres give 1.95 m; neither is 0.97. Treat ours as the definition of record |
| ID switches | 0 | 1 total, **0 after the list locks** | the one switch is during the first scan (t = 81.4 s): truth tree 1 was first published as ID 2, ID 2 was merged into ID 1 at 77.4 s. Before planning nobody holds IDs, so 0 after lock is the navigation-relevant number and matches |
| Merged neighbours (over-merge) | 4 | 2 merged entries / 4 trees inside them / 2 neighbours swallowed | README counts trees inside merged entries (4); we also report how many trees were swallowed (2) |
| Fragmented trees | 3 | 0 (final), 0 truth trees ever split into >= 2 entries | **not reproduced.** Our split test needs >= 2 entries each holding >= 20 % of their area inside the same tree; no frame meets it. README's definition is unknown (it may count trees covered by an entry below some overlap) |
| Outline spill | 0.13 | 0.16 | spill = published area outside every true canopy / published area. Against true circles it is 0.14; the remainder is definition |
| Height bias / RMSE | 1.41 / 1.46 m | 1.41 / 1.46 m | same |
| Trees sprayed | 7 of 9, no revisits, no orphans | 7 of 9, 0 twice, 0 orphans | same with "sprayed iff a sprayed entry's centre lies inside the true canopy" |

**What this scorer finds that the README table does not show:**

- **Safety: 1 violation.** At t = 145.98 s the already-sprayed tree 7 absorbed the newly selected hop target 8
  (the existing system's "sprayed polygon absorbs the target" carve-out): a sprayed tree changed shape
  (rule 5), its centre moved 1.27 m, and **ID 8 vanished while navigation held it as the hop target**.
  It happened in hop transit (navigation then re-planned at 151.2 s), so it is not counted as a
  descent-window change.
- **Height jumps: 14** single-update height changes > 1 m on published trees, up to +4.0 m (tree 31 at
  210.4 s, while it was the target). The descent reference of a published tree is not stable.
- 0 undefined heights, 0 new trees while the list is locked, 0 non-target changes in the 370 descent-window
  snapshots, 0 geometry retreats (outlines are grow-only), 0 ID reuse.
- **Timeliness:** scan latency median 1.7 s, mean 2.9 s, but trees seen first in the approach (`SCAN_APPROACH`) wait for
  the scan phase (the existing detector sleeps in SCAN_APPROACH), and tree 1 took 11.4 s after it was in
  view in the scan. Footprint test uses the image half-diagonal (yaw-free), so latencies are upper bounds.
- **Cost:** not available; the recorded map carries no per-stage timing.
