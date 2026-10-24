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
