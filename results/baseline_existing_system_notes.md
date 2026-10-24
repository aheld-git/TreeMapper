## Notes: inputs, definitions, what changed in the 2026-10-24 scorer review

**Inputs.** `/validation/fusion_snapshot` (1393 snapshots, ~5 Hz) and `/validation/ground_truth`
(9 trees, identical in all 557 non-empty messages) from `isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag`,
plus `/mission_state`, `/spray_target` and `/height_estimator/drone_position` for phases, targets and pose.
Every snapshot is in frame `world` after extraction, and the scorer now checks that against the truth frame.
Published height = `max_height` (canopy top, the existing system's descent reference). Centre = the message's
`centroid` (falls back to the area centroid when it is zero = unset). Ground truth = the recorded 12-gon outlines,
centre = vertex mean. The existing system ran once, so no run-to-run noise floor exists for it.
`tools/tests/test_score_baseline.py` pins the key numbers below.

**Ground-truth heights are unreliable (M2, `results/geometry_check.md`).** Every truth height is the
publisher's constant 4.0 m fallback. Verified stereo puts real canopy points +1.35 m (p90) to +1.88 m (p99)
above it, so the +1.41 m height bias / 1.46 m RMSE is mostly an artefact of the ground truth. The scorer detects
this (all truth heights identical) and labels every height-error number "GT height unreliable (constant fallback)".
They are reported but are **not a headline and not pass/fail**. The height *stability* metrics (jumps, drift,
target height range in descent) do not use ground truth and stay fully weighted. The ground truth itself was not
modified.

**Old scorecard (commit c34bc5e) -> this one, and why it moved:**

| Metric | Old | New | Why |
|---|---|---|---|
| Final map: recall / precision / IoU / centre RMSE / spill | 0.78 / 1.00 / 0.63 / 1.35 m / 0.16 | same | final-map definitions unchanged (Hungarian now ignores sub-threshold IoUs; no effect here) |
| Per-frame recall (window / from first publish) | 0.50 / 0.63 | **0.58 / 0.59** | was a mean over every snapshot of the stream, including 287 empty pre-mission ones. Now it is time-weighted: the latest snapshot sampled every 0.2 s from the first SCAN (65.75 s) to RETURN (241.64 s). README's 0.50 was reproduced only by the old, rate-dependent definition |
| Per-frame recall over trees already in view | — | 0.71 | new: denominator = truth trees whose centre has entered the camera footprint by that time |
| Per-frame IoU / centre RMSE / spill | 0.60 / 1.33 / 0.15 | 0.60 / 1.33 / 0.14 | same window change |
| Descent snapshots judged | 370 | **553** (approach 230, spraying 140, climb-out 183) | the window now includes the climb out of every spray: from the end of SPRAYING until the drone leaves 1.5 m of the hover point, in whatever phase follows (HOPPING, SCAN_APPROACH, RETURN) |
| Non-target changes in descent | 0 | **3** | all in the climb out of tree 7's spray: at 145.98 s sprayed tree 7 absorbed the new target 8. The drone was then at 8.4 m, 0.1 m from the hover point, *not* in hop transit as the old notes said. With target 8 gone, the existing freeze failed open, and tree 1 changed at 150.98 / 151.18 s while the drone was still over tree 7 (10.8 m) |
| IDs vanished while the list is locked | — | **1** | new metric: target ID 8 vanished at 145.98 s (HOPPING) while navigation held it |
| Sprayed tree changed / vanished / lost flag | 1 | 1 | ID 7 at 145.98 s, now judged against the record taken when it was first sprayed |
| **Total safety violations** | **1** | **5** | 1 sprayed change + 1 locked-ID vanish + 3 non-target changes; the same event at 145.98 s and its consequences |
| Centre jumps | 2 | **8** | now > 1 m within any 1 s window (rate-independent), counted per onset. The extra 6 are 1.00-1.54 m moves spread over several 0.2 s updates during the first scans (77-81 s) and the second scan (179-182 s) |
| Height jumps | 14 | **19** | same windowing, and compared with the last finite value; the +2.35 / -2.6 m pair at 113.4 s is now one onset |
| Max height drift per ID since lock | — | **4.21 m** (ID 31) | new; ID 10 drifts 2.60 m, ID 1 1.82 m: the descent reference of a locked tree is not stable |
| Max centre drift per ID since lock | — | 1.44 m (ID 7, the absorption) | new |
| Geometry retreats | 0 | 0 | now measured against each ID's historic outline union: the outlines really are grow-only (max lost fraction 0.000) |
| Spray outcome | 7 of 9, 0 twice, 0 orphans | same | now over the whole stream, with the centre at spray time and distinct IDs per truth tree |
| Scan latency mean / max | 2.86 / 11.38 s | **1.19 / 2.18 s** | first publish is now the first one-to-one match with a finite height, not the first covering entry. Tree 1 was matched (as fragment ID 2) at 67.98 s, long before any entry covered 30 % of it (77.17 s). No negative latencies |
| Latency from first view in any phase, mean / max | 15.06 / 52.68 s | 13.39 / 52.68 s | same change |
| Target height range during its own descent | — | max 1.84 m (ID 32) | new, informational |

**Comparison with DATA_README's table** (still valid for the final map): trees 9 / 7 / 7, recall 0.78,
precision 1.00, height bias / RMSE 1.41 / 1.46 m (GT-unreliable, see above), 7 of 9 sprayed, all the same. Not
reproduced, as before: README's centre RMSE 0.97 m (ours 1.35 m: the two merged entries are matched to the tree
their centre is nearest), "3 fragmented trees" (no frame has >= 2 entries each holding >= 20 % of their area in one
tree), spill 0.13 (ours 0.16). README per-frame recall 0.50 matches only the retired snapshot-weighted definition.

**What the scorecard says about the existing system:**

- **Safety: 5 violations, all from one design choice.** The "sprayed polygon absorbs the hop target" carve-out
  changes a sprayed tree (rule 5), deletes the ID navigation is flying to while the list is locked, and, with the
  target gone, the hop freeze fails open and a non-target tree changes during the climb out.
- **Height stability is the weakest area:** 19 height jumps of more than 1 m within 1 s, and locked trees drift up
  to 4.2 m from their height at lock time. The target's height moves up to 1.84 m during its own descent.
- 0 undefined heights, 0 invalid entries, 0 duplicate IDs, 0 implausible heights, 0 new or reused IDs while
  locked, 0 geometry retreats, 0 ID reuse.
- **Timeliness:** about 1.2 s from first view in the scan to the first finite-height match. Trees first seen in
  SCAN_APPROACH wait for the scan, because the existing detector sleeps in SCAN_APPROACH.
- **Cost:** not available; the recorded map carries no per-stage timing.
