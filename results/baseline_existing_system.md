# Baseline: existing system's recorded map, Isaac sim mission 2026-10-23-15-34-29

Mode: ground truth; frame `world`; 1393 snapshots.

Final map at t = 288.37 s. Per-frame column = time-weighted mean of the latest snapshot sampled every 0.20 s over the mission window: first SCAN (65.75 s) .. first RETURN (241.64 s) (880 samples) / from the first non-empty snapshot to the window end (872 samples).

## Detection

| Metric | Final map | Per-frame mean (window / from first publish) |
|---|---|---|
| Trees in truth / published / matched | 9 / 7 / 7 | — |
| Recall | 0.78 | 0.58 / 0.59 |
| Recall over trees already in view | 0.78 (9 trees ever in view) | 0.71 / 0.72 |
| Precision | 1.00 | 1.00 / 1.00 |
| Missed (absorbed by a merge / unseen) | 2 (2 / 0) | 3.78 / 3.73 |
| Merged entries / trees inside them / neighbours swallowed | 2 / 4 / 2 | 1.48 / 1.49 (swallowed) |
| Split trees (duplicates / fragments) | 0 (0 / 0) | 0.00 / 0.00 |
| False trees / invalid entries | 0 / 0 | 0.00 / 0.00 |

Missed truth IDs: [0, 3]. Merged entries (map id: truth ids covered): {'7': [3, 4], '1': [0, 1]}.

## Geometry (matched pairs)

Truth heights: GT height unreliable (constant fallback): every truth height is 4.000 m.

| Metric | Final map | Per-frame mean (window / from first publish) |
|---|---|---|
| Mean IoU (min) | 0.63 (0.39) | 0.60 / 0.60 |
| Centre error mean / RMSE / max (m) | 1.09 / 1.35 / 2.36 | 1.33 / 1.32 RMSE |
| Height bias / RMSE / max abs (m) **(GT height unreliable (constant fallback): not a headline, not pass/fail)** | 1.41 / 1.46 / 1.89 | 1.50 / 1.50 RMSE |
| Matched entries with undefined height (charged 15 m each) | 0 | 0.00 / 0.00 |
| Spill outside all true canopies | 0.16 | 0.14 / 0.14 |
| Neighbour spill mean / max | 0.15 / 0.41 | 0.12 / 0.12 |

| Truth id | Map id | IoU | Centre err (m) | Height err (m) (GT unreliable) | Neighbour spill |
|---|---|---|---|---|---|
| 1 | 1 | 0.43 | 2.08 | 1.14 | 0.33 |
| 2 | 29 | 0.86 | 0.22 | 1.87 | 0.00 |
| 4 | 7 | 0.39 | 2.36 | 1.35 | 0.41 |
| 5 | 31 | 0.55 | 1.28 | 1.28 | 0.15 |
| 6 | 9 | 0.86 | 0.13 | 1.89 | 0.00 |
| 7 | 10 | 0.65 | 0.58 | 1.55 | 0.00 |
| 8 | 32 | 0.66 | 0.96 | 0.81 | 0.15 |

## Spray outcome (whole stream)

Sprayed IDs 7; truth trees sprayed 7 of 9; sprayed twice 0; orphan sprayed IDs 0; not sprayed: [0, 3]; sprayed by (truth: ids): {'1': [1], '2': [29], '4': [7], '5': [31], '6': [9], '7': [10], '8': [32]}. (over the whole stream: a truth tree is sprayed by every distinct ID whose centre, when its sprayed flag first appeared, lies inside the canopy.)

## Stability

| Metric | Value |
|---|---|
| ID switches, total / after the list locks (t = 87.35 s) | 1 / 0 |
| ID meaning changes, total / after lock | 1 / 0 |
| ID reappearances / geometric reuse / reuse by truth owner | 0 / 0 / 0 |
| Geometry retreats (> 2% of the ID's historic outline union lost; onsets) / max lost fraction | 0 / 0.000 |
| Centre jumps (> 1.0 m within 1.0 s; onsets) | 8 |
| Height jumps (> 1.0 m within 1.0 s or vs last finite; onsets) | 19 |
| Max centre drift per ID from first publish / since lock (m) | 3.21 / 1.44 |
| Max height drift per ID from first publish / since lock (m) | 2.58 / 4.21 |
| IDs vanished (of which held by navigation) | [2, 8] ([8]) |
| Truth trees ever split into >= 2 entries | [] |
| Truth trees ever inside a merged entry | [0, 1, 3, 4] |

Per ID: max lost fraction, centre drift from first / since lock, height drift from first / since lock (m):

| ID | First published (s) | Max lost | Centre drift first / lock | Height drift first / lock |
|---|---|---|---|---|
| 1 | 67.38 | 0.000 | 3.21 / 0.03 | 1.29 / 1.82 |
| 2 | 67.98 | 0.000 | 1.24 / n/a | 1.30 / n/a |
| 7 | 80.38 | 0.000 | 2.28 / 1.44 | 0.92 / 1.32 |
| 8 | 80.38 | 0.000 | 1.48 / 0.08 | 0.67 / 0.00 |
| 9 | 81.58 | 0.000 | 1.70 / 0.10 | 1.66 / 0.70 |
| 10 | 81.77 | 0.000 | 1.46 / 0.25 | 2.47 / 2.60 |
| 29 | 174.98 | 0.000 | 1.70 / 0.05 | 1.33 / 1.02 |
| 31 | 179.37 | 0.000 | 2.67 / 0.68 | 2.58 / 4.21 |
| 32 | 180.78 | 0.000 | 2.03 / 0.21 | 1.14 / 1.20 |

## Safety (all must be 0; n/a = not judgeable from the inputs)

| Metric | Value |
|---|---|
| Frames publishing an undefined height | 0 |
| Frames with an invalid entry (non-finite centre/outline, < 3 vertices, zero area) | 0 |
| Frames with a duplicate ID | 0 |
| Frames with an implausible height (<= 0 or above the drone) | 0 |
| New (or reused) IDs while the list is locked (PLANNING/HOPPING/SPRAYING) | 0 |
| IDs vanished while the list is locked (of which the hop target) | 1 (1) |
| Non-target changes during descent (553 snapshots judged: approach 230, spraying 140, climb-out 183) | 3 |
| Sprayed tree changed / vanished / lost its flag after spraying | 1 |
| New tree inside a sprayed outline | 0 |
| **Total violations** | **5** |

Informational: target height range during its own approach/spraying window, max 1.84 m; per target {'1': 0.804, '7': 0.434, '9': 0.14, '10': 0.466, '29': 0.109, '31': 0.223, '32': 1.84}.

- `locked_id_vanished_events` (first 1 of 1 listed): [{"t": 145.976, "id": 8, "phase": "HOPPING", "was_target": true}]
- `nontarget_change_events` (first 3 of 3 listed): [{"t": 145.976, "id": 7, "target": 8, "kind": "changed", "window": "climb"}, {"t": 150.981, "id": 1, "target": 8, "kind": "changed", "window": "climb"}, {"t": 151.176, "id": 1, "target": 8, "kind": "changed", "window": "climb"}]
- `sprayed_tree_change_events` (first 1 of 1 listed): [{"t": 145.976, "id": 7, "kind": "changed"}]
- `id_switch_events` (first 1 of 1 listed): [{"t": 81.382, "truth_id": 1, "from": 2, "to": 1, "after_lock": false}]
- `id_meaning_change_events` (first 1 of 1 listed): [{"t": 81.187, "id": 1, "from": 0, "to": 1, "after_lock": false}]
- `centre_jump_events` (first 8 of 8 listed): [{"t": 77.174, "id": 2, "moved_m": 1.238}, {"t": 77.385, "id": 1, "moved_m": 2.731}, {"t": 80.781, "id": 7, "moved_m": 1.181}, {"t": 80.976, "id": 8, "moved_m": 1.08}, {"t": 145.976, "id": 7, "moved_m": 1.271}, {"t": 179.175, "id": 29, "moved_m": 1.001}, {"t": 180.182, "id": 31, "moved_m": 1.536}, {"t": 181.58, "id": 32, "moved_m": 1.323}]
- `height_jump_events` (first 12 of 19 listed): [{"t": 77.174, "id": 2, "dh_m": 1.299}, {"t": 81.382, "id": 1, "dh_m": -1.04}, {"t": 81.772, "id": 9, "dh_m": 1.284}, {"t": 113.379, "id": 10, "dh_m": -2.604}, {"t": 128.182, "id": 7, "dh_m": -1.285}, {"t": 131.774, "id": 7, "dh_m": 1.361}, {"t": 151.582, "id": 1, "dh_m": -1.016}, {"t": 156.376, "id": 1, "dh_m": 1.249}, {"t": 178.184, "id": 29, "dh_m": 1.006}, {"t": 179.581, "id": 31, "dh_m": 1.567}, {"t": 180.784, "id": 29, "dh_m": -1.076}, {"t": 181.385, "id": 32, "dh_m": 1.054}]

## Timeliness

first snapshot where the tree is one-to-one matched to a valid entry with a finite height, minus the first time its centre came within the nadir footprint's half-diagonal (yaw-free bound); 'scan' variant counts view time only while the phase is SCAN; negative latencies are listed and clamped to 0 in mean/max.

Latency mean / max: 13.39 s / 52.68 s from first view in any phase; 1.19 s / 2.18 s from first view while scanning. Never published: []. Negative latencies (clamped to 0): [].

| Truth id | First in view (s) | First in view while scanning (s) | First published (s) | Latency (s) | Latency from scan view (s) |
|---|---|---|---|---|---|
| 0 | 60.19 | 65.80 | 67.37 | 7.18 | 1.58 |
| 1 | 59.40 | 65.80 | 67.98 | 8.58 | 2.18 |
| 2 | 170.90 | 174.80 | 174.98 | 4.08 | 0.18 |
| 3 | 59.10 | 78.80 | 80.37 | 21.27 | 1.58 |
| 4 | 56.80 | 79.50 | 80.37 | 23.58 | 0.88 |
| 5 | 178.10 | 178.10 | 179.58 | 1.48 | 1.48 |
| 6 | 80.60 | 80.60 | 81.58 | 0.97 | 0.97 |
| 7 | 81.30 | 81.30 | 81.98 | 0.68 | 0.68 |
| 8 | 128.30 | 179.79 | 180.98 | 52.68 | 1.19 |

## Cost

map stream carries no stage_ms

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
| Centre jumps | 2 | **8** | now > 1 m within any 1 s window (rate-independent), counted per onset. The extra 6 are 1.0-1.5 m moves spread over several 0.2 s updates during the first scans (77-81 s) and the second scan (179-182 s) |
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
