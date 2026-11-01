# corridor_nav closed loop in Isaac Sim

Isaac Sim 5.1 orchard (`sim/isaac/orchard_sim.py`, `tm_isaac`) <-> ros1_bridge <-> Noetic SGBM + YOLOX detector,
TreeMapper and the corridor navigator (`tm_sim_ros`), one command: `scripts/sim_mission.sh <spec.json>`.

## Detector parity

Question (Task 14, open until the model was on hand): do `sim_perception`'s YOLOX pre/post-processing and the Isaac
model give the boxes the existing detector gave on the Orin, and with which input and score threshold?

Model: `/data/work/models/yolox_s_isaac/yolox_s_isaac_epoch190_416_static.onnx` (the model the Orin's Isaac launch runs
as a TensorRT engine; sha256 3f6ec5a0...504b). Input `images` [1, 3, 416, 640], output [1, 5460, 6]
(5460 = 52*80 + 26*40 + 13*20 anchors, rows cx, cy, w, h, obj, cls).

Data: the recorded Isaac mission `cloud_bundle_isaac_2026-10-23-15-34-29.bag` (read-only) and the existing detector's
boxes extracted from it (`/data/work/extracted/isaac_2026-10-23-15-34-29/detections.jsonl`). 80 frames spread evenly
over the mission (60 with recorded boxes, 123 boxes; 20 without), each left `image_grey` taken from the bag at the
identical header stamp. Our pipeline: `sim_perception.yolox` letterbox (640x400 -> 640x416, ratio 1, pad 114), raw
0..255, onnxruntime-gpu 1.19.2 (CUDA), per-class NMS 0.45. Greedy IoU matching against the recorded boxes.
Tool: `ros/sim_perception/scripts/detector_parity.py`; raw output: `/data/work/sim_runs/parity/parity.json`.

| score threshold | recorded boxes matched (IoU >= 0.5) | our boxes | ours on frames with recorded boxes, unmatched | ours on frames without recorded boxes | mean IoU | mean / max centre offset (px) |
|---|---|---|---|---|---|---|
| 0.30 | 123 / 123 (100 %) | 219 | 77 | 19 | 0.979 | 1.43 / 11.2 |
| 0.70 | 123 / 123 (100 %) | 157 | 26 | 8 | 0.979 | 1.43 / 11.2 |
| 0.75 | 123 / 123 (100 %) | 142 | 12 | 7 | 0.979 | 1.43 / 11.2 |
| **0.80** | **123 / 123 (100 %)** | **128** | **0** | **5** | **0.979** | **1.43 / 11.2** |
| 0.85 | 91 / 123 (74 %) | 96 | 0 | 5 | 0.982 | 1.31 / 8.2 |

Mean score difference of the matched pairs (ours - recorded): +0.0001. Lowest recorded score: 0.8004.

Findings and decisions:
- **The output is already grid-decoded** (centres span 3.8 .. 635 px of the 640 px input): `~decoded true`. Decoding it
  again would scramble every box. (The anchor count alone does not tell; the value range does.)
- **Score threshold 0.8**: the existing detector keeps boxes with score >= 0.8 (lowest recorded 0.8004). At 0.8 every
  recorded box is reproduced (IoU 0.98, centre offset 1.4 px mean; the same score to 1e-4) and no extra box appears on
  a frame where the existing detector reported boxes. On 19 of the 20 frames without recorded boxes we report none
  either; all 5 extra boxes (scores 0.87 .. 0.90) are on one frame at 61.13 s in SCAN_APPROACH, where the
  recorded detector published an empty result - as it did on every SCAN_APPROACH frame (46 of 46) and
  every IDLE / RETURN frame of the recording: it is gated off outside the scan and hop phases, not disagreeing on
  boxes. 0.3 (the `sim_perception.launch` default) would add 77 boxes the existing detector does not report on frames
  where it did report.
- **Grey replicated to 3 channels**: the drone's cameras are grey (the recording has only `image_grey`), the Orin
  feeds grey, and the model input is fixed at 3 channels. Grey replicated reproduces the Orin's boxes exactly, so it is
  the input for the sim too (Isaac publishes rgb8; the detector converts to grey first). An RGB input cannot be
  compared with the Orin's detections at all, so it was not chosen.
- Input size **[416, 640]** (h, w), class 0, NMS 0.45. `sim_closed_loop.launch` passes these explicitly.

## Smoke (`sim/specs/3x3_smoke.json`, strategy corridor)

Command: `scripts/sim_mission.sh sim/specs/3x3_smoke.json --out /data/work/sim_runs/smoke16b_b --timeout 900`.
Isaac `--v-max-xy 4.0 --v-max-z 1.5 --max-rtf 1.0`; navigator = `config/corridor_nav.cfg` with the sim-only
`carrot_lead_s 1.2`; detector as above; TreeMapper `treemapper_sim.launch` unchanged (cand_obj.cfg).

**Gate: passed** (attempt 2, run dir `/data/work/sim_runs/smoke16b_b/run_1`, commit 0b9f740 + the harness of this
commit): mission DONE, every safety row passes, 9 of 9 truth trees sprayed.

| Scorecard row | Value |
|---|---|
| Mission completed (DONE, aborts 0, hold timeouts 0, never ERROR) | PASS |
| Minimum clearance over the truth canopy | 2.313 m (PASS, > 0) |
| Nozzle open outside DOSE | 0 samples (PASS) |
| Sprayed tree targeted again | 0 episodes (PASS) |
| Truth trees sprayed (>= 50 % of an episode's intended dose) | 9 of 9 (missed none) |
| Dose episodes / without a matched truth target | 11 / 2 |
| Double-sprayed (>= 20 % in two episodes) | 1 (tree 2) |
| On-target fraction; off-target litres (ground) | 0.686; 0.777 L (0.651 L) of 2.479 L |
| Mission time; path length; seconds per sprayed tree | 218.0 s; 331.9 m; 24.2 s |
| Map (final): published / matched; centre RMSE; mean IoU; height bias; spill outside canopies | 11 / 9; 0.48 m; 0.62; +0.61 m; 0.36 |
| Navigator step / SGBM / detector mean | 0.01 / 13.2 / 10.3 ms |
| Real-time factor | 1.000 (226 sim s / 226 wall s) |

Notes on the two unmatched episodes (no truth tree missed, so no miss needs explaining):
- TreeMapper id 7 at (5.98, 2.38), a fragment of tree 2 split off 2.4 m from its centre (beyond the scorer's 2 m match
  radius): 0.086 L landed on tree 2, which is why tree 2 reads double-sprayed (its own episode delivered 0.20 L).
- TreeMapper id 65 at (9.92, 4.96) with a published height of **12.98 m** (no tree there; truth tops are 2.3 .. 3.1 m):
  a phantom from stereo outliers. The navigator dosed it from 15.3 m (published height + spray_clearance); 0.04 L of
  the 0.15 L reached canopies, the rest the ground. Not a safety event (clearance 12 m). Since the controller ruling
  after this smoke the navigator skips a target published above home + `expected_canopy_top_max` +
  `target_height_margin` (7 + 2 = 9 m here), counted `target_height_implausible`: this phantom would now be skipped.

### Re-run after review fix round 1: `--strategy corridor --runs 2` (gate passed in both runs)

`scripts/sim_mission.sh sim/specs/3x3_smoke.json --strategy corridor --runs 2 --out /data/work/sim_runs/smoke16b_fr1
--timeout 900` at commit e7db18f (implausible-height skip, failure containment, run inputs from `sim_inputs.py`):
exit 0, `compare.md` written.

| Scorecard row | run_1 | run_2 |
|---|---|---|
| Mission completed (DONE, aborts 0, hold timeouts 0, never ERROR) | PASS | PASS |
| Minimum clearance over the truth canopy | 2.565 m PASS | 2.562 m PASS |
| Nozzle open outside DOSE | 0 PASS | 0 PASS |
| Sprayed tree targeted again | 0 PASS | 0 PASS |
| Truth trees sprayed | 9 of 9 | 9 of 9 |
| Dose episodes / unmatched / double-sprayed | 9 / 0 / 0 | 9 / 0 / 0 |
| On-target fraction; off-target litres | 0.889; 0.278 L | 0.860; 0.331 L |
| Mission time; path length; seconds per sprayed tree | 197.5 s; 307.1 m; 21.9 s | 199.0 s; 310.0 m; 22.1 s |
| Map (final): published / matched; centre RMSE; mean IoU; height bias; spill | 9 / 9; 0.35 m; 0.55; +0.65 m; 0.43 | 9 / 9; 0.51 m; 0.59; +0.56 m; 0.36 |
| target_height_implausible | 0 | 0 |
| Real-time factor; wall time of the run; bag | 1.000; 368 s; 2.4 MB | 1.000; 370 s; 2.4 MB |

No phantom was published in either run, so the new skip did not act; it is covered by the navigator test
(`Navigator.ImplausiblyHighTargetIsNeverVisited`).

### Smoke attempt 1 (failed the spray criterion) and its fix

`/data/work/sim_runs/smoke16b_a/run_1`: DONE and every safety row passed, but 6 of 9 trees sprayed (missed 1, 3, 4),
13 episodes (2 unmatched), on-target 0.597, 1.63 L on the ground; map centre RMSE 0.91 m, mean IoU 0.42, spill 0.60,
13 entries for 9 trees (2 phantoms, trees 3 and 4 split, 1 and 4 merged), heights up to +3.8 m.

Root cause (evidence, not a navigator problem): the detector matched every fully visible crown (1109 of 1109, box IoU
0.77 against the projected truth crowns, centre offset (0.2, 1.6) px), so the error came from the disparity. The sim
ground was a flat-colour mesh (`clutter.grass` was not implemented). A stereo capture hovering over tree 5 at 15 m
(`/data/work/sim_runs/diag16b`) run through the node's SGBM: 24 % of the ground pixels came out more than 1 m above
the ground (95th percentile +2.9 m, as high as the canopies); the ground's mean |Laplacian| was 2.3 against 17 on the
crowns. SGBM filled the textureless ground by propagation from the crowns, and TreeMapper built phantom crowns and
over-wide outlines from it. The recorded Isaac mission had grass on the ground (and its disparity is rendered depth:
100 % valid and noise-free, see below), so TreeMapper had never met this.

Fix `0b9f740` (sim): `clutter.grass` is rendered as a periodic band-limited brightness texture (3 .. 40 cm, 8 m tile)
on the terrain colour. Same capture afterwards: ground valid 0.87, share above 1 m 0.09, ground |Laplacian| 13.4.

### Extra check of the harness: `--strategy field_first --runs 2`

`/data/work/sim_runs/smoke16b_ff/{run_1,run_2,compare.md}`: both DONE, all safety rows pass (minimum clearance 2.39 /
2.24 m), 8 of 9 sprayed in each, on-target 0.837 / 0.843, mission 167.5 / 170.0 s. The misses are map errors, one per
run: run 1 tree 7 (its TreeMapper entry 1.79 m off the truth centre, 43 % of the intended dose landed on it), run 2
tree 5 (published 5.76 m for a 2.34 m tree, dosed from 8.4 m, 36 %). With 2 runs per strategy on 9 trees these numbers
are not a strategy comparison.

### What this smoke says about the sim (for the comparison runs)

- The recorded Isaac mission's `/disparity_image_down_float` is rendered depth (100 % valid, noise-free; ground at 15 m
  4.8 px everywhere), so every earlier Isaac result of TreeMapper (`results/ros_replay_isaac.md`: centre RMSE 0.19 m,
  IoU 0.87) was on perfect stereo. In this loop the disparity is SGBM on rendered images; with the textured ground the
  map is clearly worse than on the recording (centre RMSE 0.35 .. 0.70 m, IoU 0.55 .. 0.62, height bias +0.4 .. +0.65 m,
  outlines spill 0.32 .. 0.44 outside the canopies over the five runs on the textured ground). Spray outcomes are limited by that map.
- Per run (3x3, RTF 1.0): ~150 s Isaac startup, the mission (170 .. 220 sim s), ~5 s teardown, ~10 s scoring;
  340 .. 395 s wall in total (3x3; 153 s to /clock in the re-run). Bag 2.0 .. 2.6 MB, run directory 5 .. 6 MB.

## Comparison: corridor vs field_first (Task 16 Step 3, 3 runs per spec and strategy)

Runs: `scripts/sim_mission.sh sim/specs/<spec>.json --strategy <corridor|field_first> --runs 3 --out
/data/work/sim_runs/cmp16c_<spec>_<strategy>`, the four batches one after another at commit 6e6aeb9 (the one dirty
file is `.claude/settings.json`, the user's tool config; no code), Isaac `--v-max-xy 4.0 --v-max-z 1.5 --max-rtf 1.0`,
navigator = `config/corridor_nav.cfg` with the strategy and the sim-only `carrot_lead_s 1.2`, detector as in "Detector
parity" (sha256 3f6ec5a0...504b, score threshold 0.8), TreeMapper `treemapper_sim.launch` with `cand_obj.cfg` unchanged.
Batch log: `/data/work/logs/cmp_16c_20261031T192531Z.log`. Specs: `5x5_tropical` (25 trees, 7 m x 7 m grid, jitter
1.5 m, flat, seed 1001) and `rows_8x12` (86 trees after 8 % missing, 6 m rows x 4.5 m, jitter 0.5 m, 0.03 rad slope,
30 rocks, seed 7). Every run uses the spec's seed, so the three runs of a batch fly the same orchard: the run-to-run
spread is the loop's own non-determinism (rendering, SGBM, detector, timing), not orchard variation.

The tables below are generated, not typed:

```bash
python3 -m treemapper_tools.sim_compare_report /data/work/sim_runs/cmp16c_5x5_tropical_corridor \
  /data/work/sim_runs/cmp16c_5x5_tropical_field_first /data/work/sim_runs/cmp16c_rows_8x12_corridor \
  /data/work/sim_runs/cmp16c_rows_8x12_field_first --out <md> --json <json>
```

(`tools/treemapper_tools/sim_compare_report.py`: it reads each run's `mission_score.json` - the scorer's own metrics
through `mission_score.compare_runs`, which also wrote each batch's `compare.md` - plus `truth.json`, `mission.json`
and `map.jsonl` for the miss analysis.) Per spec and strategy: mean over the 3 runs / worst run (in the bad direction) /
spread (max - min). **Noise floor**: a difference of the means no larger than the larger of the two spreads (and never
below 1 % of the larger mean) is reported as "no measurable difference"; a bias is compared by its magnitude. With 3 runs per side this is a coarse floor; a difference just above it is a hint,
not a proof.

### Setup

| Spec | Strategy | Runs | Commit (dirty files) | Seed | Detector model | RTF mean (min) | Mission wall time per run (s) | Footprint radius r_fp |
|---|---|---|---|---|---|---|---|---|
| 5x5_tropical | corridor | 3 | 6e6aeb9 (1) | 1001 | `yolox_s_isaac_epoch190_416_static.onnx` | 1.000 (1.000) | 442 .. 455 | 0.80 m (nozzle 2.2 m above the published top, cone 20 deg) |
| 5x5_tropical | field_first | 3 | 6e6aeb9 (1) | 1001 | `yolox_s_isaac_epoch190_416_static.onnx` | 1.000 (1.000) | 368 .. 372 | 0.80 m (nozzle 2.2 m above the published top, cone 20 deg) |
| rows_8x12 | corridor | 3 | 6e6aeb9 (1) | 7 | `yolox_s_isaac_epoch190_416_static.onnx` | 1.000 (1.000) | 1000 .. 1019 | 0.80 m (nozzle 2.2 m above the published top, cone 20 deg) |
| rows_8x12 | field_first | 3 | 6e6aeb9 (1) | 7 | `yolox_s_isaac_epoch190_416_static.onnx` | 1.000 (1.000) | 901 .. 909 | 0.80 m (nozzle 2.2 m above the published top, cone 20 deg) |

### Result in short

- **Efficiency: field_first is faster on both orchards**, well above the noise floor: mission time 362.8 s vs 442.8 s
  on 5x5_tropical (-80 s, -18 %; floor 13.5 s) and 897.5 s vs 1002.7 s on rows_8x12 (-105 s, -10.5 %; floor 18.5 s),
  path 139 m / 189 m shorter. In this loop corridor mode saves no flight time; it costs 10-18 %.
- **Spray outcome: no measurable difference** on either orchard (trees sprayed 21.3 vs 21.0 of 25 and 83.3 vs 84.7 of
  86; on-target fraction 0.73 vs 0.70 and 0.93 vs 0.93; spreads of 3-6 trees between runs of the same strategy).
- **Safety: all 12 runs completed** (DONE, no abort, no hold, never ERROR) and **pass every safety row**: minimum true
  clearance over the canopy 1.46-2.28 m (> 0), nozzle never open outside DOSE, no sprayed tree targeted again, no
  clearance-guard trigger.
- **Misses (35 in 12 runs, 5.3 % of the 666 tree visits) are map errors, not navigation errors**: 24 (a) map centre
  further from the truth than the 0.80 m footprint (21 on 5x5, where a tree first confirmed at the far edge of the
  image keeps a centre biased toward that track), 7 (e) other (6 published heights 1.8-4.1 m too high, so the footprint
  was wider than the crown; 1 tree never published), 3 (d) navigator skips (`target_height_implausible`, from the same
  height over-estimate crossing 9 m), 1 (b) merged entry, 0 (c) phantom. Details in "Why trees were missed".

### 5x5_tropical: per-strategy statistics

| Group | Metric | corridor mean / worst / spread | field_first mean / worst / spread | field_first - corridor (noise floor) | Verdict |
|---|---|---|---|---|---|
| efficiency | Mission time (s) | 442.83 / 448.00 / 13.50 | 362.83 / 364.50 / 4.50 | -80.00 (13.50) | field_first better |
| efficiency | Path length (m) | 784.42 / 792.53 / 20.23 | 645.79 / 650.00 / 8.36 | -138.63 (20.23) | field_first better |
| efficiency | Seconds per sprayed tree | 20.91 / 23.58 / 4.69 | 17.64 / 21.41 / 5.76 | -3.28 (5.76) | no measurable difference |
| efficiency | Litres emitted | 5.38 / 5.59 / 0.34 | 5.68 / 6.02 / 0.73 | +0.30 (0.73) | no measurable difference |
| efficiency | Glide climbs inserted | 3.33 / 5.00 / 3.00 | 2.67 / 7.00 / 7.00 | -0.67 (7.00) | no measurable difference |
| efficiency | Time in scan (s) | 95.00 / 95.00 / 0.00 | 93.83 / 94.00 / 0.50 | -1.17 (0.95) | field_first better |
| efficiency | Time in settle (s) | 16.00 / 16.00 / 0.00 | 16.00 / 16.00 / 0.00 | +0.00 (0.16) | no measurable difference |
| efficiency | Time in transit (s) | 120.17 / 123.00 / 7.00 | 51.17 / 51.52 / 0.52 | -68.99 (7.00) | field_first better |
| efficiency | Time in glide (s) | 134.83 / 137.00 / 4.00 | 123.33 / 130.50 / 11.50 | -11.50 (11.50) | field_first better |
| efficiency | Time in dose (s) | 73.33 / 76.50 / 6.00 | 78.50 / 83.50 / 10.50 | +5.17 (10.50) | no measurable difference |
| efficiency | Time in hold (s) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| spray | Truth trees sprayed | 21.33 / 19 / 4 | 21.00 / 17 / 6 | -0.33 (6) | no measurable difference |
| spray | Truth trees missed | 3.67 / 6 / 4 | 4.00 / 8 / 6 | +0.33 (6) | no measurable difference |
| spray | Double-sprayed | 1.00 / 1 / 0 | 0.33 / 1 / 1 | -0.67 (1) | no measurable difference |
| spray | On-target fraction | 0.73 / 0.73 / 0.01 | 0.70 / 0.65 / 0.10 | -0.03 (0.10) | no measurable difference |
| spray | Off-target litres | 1.43 / 1.46 / 0.08 | 1.69 / 2.10 / 0.69 | +0.26 (0.69) | no measurable difference |
| spray | Dose episodes | 25.67 / 26.00 / 1.00 | 25.67 / 27.00 / 2.00 | +0.00 (2.00) | no measurable difference |
| spray | Episodes without a matched truth tree | 0.67 / 1.00 / 1.00 | 0.67 / 1.00 / 1.00 | +0.00 (1.00) | no measurable difference |
| spray | Litres on the ground | 1.37 / 1.46 / 0.18 | 1.67 / 2.07 / 0.65 | +0.31 (0.65) | no measurable difference |
| spray | Trees skipped (navigator) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| safety | Minimum clearance (m) | 2.23 / 2.15 / 0.13 | 2.04 / 2.01 / 0.04 | -0.19 (0.13) | corridor better |
| safety | Nozzle open outside DOSE | 0.00 / 0 / 0 | 0.00 / 0 / 0 | +0.00 (0.00) | no measurable difference |
| safety | Completed (1 = DONE, no abort / hold timeout / ERROR) | 1.00 / 1.00 / 0.00 | 1.00 / 1.00 / 0.00 | +0.00 (0.01) | no measurable difference |
| safety | All safety rows pass (1 = yes) | 1.00 / 1.00 / 0.00 | 1.00 / 1.00 / 0.00 | +0.00 (0.01) | no measurable difference |
| safety | Sprayed tree targeted again (episodes) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | Clearance-guard triggers | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | Guard-blind counter | 6.33 / 7.00 / 1.00 | 1.00 / 1.00 / 0.00 | -5.33 (1.00) | field_first lower (info only) |
| guards | Holds (all reasons) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | IDs re-resolved | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | IDs dropped (vanished) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | Late trees (cleanup list) | 0.33 / 1.00 / 1.00 | 0.00 / 0.00 / 0.00 | -0.33 (1.00) | no measurable difference |
| guards | Targets skipped as implausibly high | 0.33 / 1.00 / 1.00 | 0.00 / 0.00 / 0.00 | -0.33 (1.00) | no measurable difference |
| guards | Spray points raised (split neighbour) | 0.67 / 1.00 / 1.00 | 0.33 / 1.00 / 1.00 | -0.33 (1.00) | no measurable difference |
| map | Map recall (final) | 1.00 / 1.00 / 0.00 | 1.00 / 1.00 / 0.00 | +0.00 (0.01) | no measurable difference |
| map | Map precision (final) | 0.96 / 0.96 / 0.00 | 0.98 / 0.93 / 0.07 | +0.01 (0.07) | no measurable difference |
| map | Split trees (final) | 1.00 / 1.00 / 0.00 | 0.33 / 1.00 / 1.00 | -0.67 (1.00) | no measurable difference |
| map | False trees (final) | 0.00 / 0.00 / 0.00 | 0.33 / 1.00 / 1.00 | +0.33 (1.00) | no measurable difference |
| map | Mean IoU (final) | 0.54 / 0.54 / 0.01 | 0.54 / 0.51 / 0.06 | -0.00 (0.06) | no measurable difference |
| map | Centre RMSE (m, final) | 0.68 / 0.81 / 0.25 | 0.85 / 1.02 / 0.26 | +0.17 (0.26) | no measurable difference |
| map | Centre error max (m, final) | 1.74 / 2.13 / 0.64 | 1.90 / 2.48 / 0.89 | +0.16 (0.89) | no measurable difference |
| map | Height bias (m, final) | 0.69 / 0.80 / 0.26 | 0.30 / 0.37 / 0.13 | -0.39 (0.26) | field_first better |
| map | Height RMSE (m, final) | 0.89 / 1.05 / 0.36 | 0.49 / 0.59 / 0.20 | -0.41 (0.36) | field_first better |
| map | Spill outside all canopies (final) | 0.34 / 0.36 / 0.04 | 0.40 / 0.45 / 0.10 | +0.06 (0.10) | no measurable difference |
| map | Max height drift per ID since lock (m) | 3.68 / 5.90 / 4.19 | 0.06 / 0.17 / 0.17 | -3.62 (4.19) | no measurable difference |
| map | Frames with an implausible height | 84.67 / 139.00 / 139.00 | 6.00 / 18.00 / 18.00 | -78.67 (139.00) | no measurable difference |
| map | Map safety violations (total) | 87.33 / 139.00 / 137.00 | 6.00 / 18.00 / 18.00 | -81.33 (137.00) | no measurable difference |
| cost | Navigator step mean (ms) | 0.01 / 0.01 / 0.00 | 0.01 / 0.01 / 0.00 | +0.00 (0.00) | no measurable difference |
| cost | Navigator step p99 (ms) | 0.05 / 0.06 / 0.04 | 0.06 / 0.06 / 0.00 | +0.01 (0.04) | no measurable difference |
| cost | SGBM mean (ms) | 13.33 / 13.45 / 0.19 | 13.38 / 13.42 / 0.08 | +0.05 (0.19) | no measurable difference |
| cost | Detector mean (ms) | 9.75 / 9.75 / 0.01 | 9.89 / 9.90 / 0.05 | +0.13 (0.10) | field_first higher (info only) |
| cost | Real-time factor | 1.00 / 1.00 / 0.00 | 1.00 / 1.00 / 0.00 | -0.00 (0.01) | no measurable difference |

### rows_8x12: per-strategy statistics

| Group | Metric | corridor mean / worst / spread | field_first mean / worst / spread | field_first - corridor (noise floor) | Verdict |
|---|---|---|---|---|---|
| efficiency | Mission time (s) | 1002.67 / 1011.50 / 18.50 | 897.50 / 901.00 / 7.50 | -105.17 (18.50) | field_first better |
| efficiency | Path length (m) | 1444.29 / 1452.31 / 15.21 | 1254.86 / 1259.78 / 10.86 | -189.42 (15.21) | field_first better |
| efficiency | Seconds per sprayed tree | 12.03 / 12.11 / 0.21 | 10.60 / 10.86 / 0.41 | -1.43 (0.41) | field_first better |
| efficiency | Litres emitted | 22.87 / 23.12 / 0.57 | 23.09 / 23.35 / 0.46 | +0.22 (0.57) | no measurable difference |
| efficiency | Glide climbs inserted | 13.67 / 15.00 / 2.00 | 15.00 / 18.00 / 6.00 | +1.33 (6.00) | no measurable difference |
| efficiency | Time in scan (s) | 173.67 / 174.50 / 2.00 | 171.17 / 171.50 / 0.50 | -2.50 (2.00) | field_first better |
| efficiency | Time in settle (s) | 20.00 / 20.00 / 0.00 | 20.00 / 20.00 / 0.00 | +0.00 (0.20) | no measurable difference |
| efficiency | Time in transit (s) | 135.00 / 139.00 / 7.00 | 59.50 / 60.00 / 1.50 | -75.50 (7.00) | field_first better |
| efficiency | Time in glide (s) | 342.67 / 343.00 / 0.50 | 330.33 / 331.50 / 2.52 | -12.34 (3.43) | field_first better |
| efficiency | Time in dose (s) | 309.83 / 312.00 / 6.50 | 316.51 / 322.02 / 10.02 | +6.67 (10.02) | no measurable difference |
| efficiency | Time in hold (s) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| spray | Truth trees sprayed | 83.33 / 82 / 3 | 84.67 / 83 / 3 | +1.33 (3) | no measurable difference |
| spray | Truth trees missed | 2.67 / 4 / 3 | 1.33 / 3 / 3 | -1.33 (3) | no measurable difference |
| spray | Double-sprayed | 0.33 / 1 / 1 | 1.33 / 3 / 3 | +1.00 (3) | no measurable difference |
| spray | On-target fraction | 0.93 / 0.91 / 0.03 | 0.93 / 0.92 / 0.03 | +0.00 (0.03) | no measurable difference |
| spray | Off-target litres | 1.58 / 1.99 / 0.72 | 1.57 / 1.80 / 0.67 | -0.01 (0.72) | no measurable difference |
| spray | Dose episodes | 86.00 / 87.00 / 3.00 | 87.67 / 90.00 / 4.00 | +1.67 (4.00) | no measurable difference |
| spray | Episodes without a matched truth tree | 0.33 / 1.00 / 1.00 | 1.00 / 1.00 / 0.00 | +0.67 (1.00) | no measurable difference |
| spray | Litres on the ground | 1.52 / 1.86 / 0.60 | 1.53 / 1.76 / 0.69 | +0.02 (0.69) | no measurable difference |
| spray | Trees skipped (navigator) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| safety | Minimum clearance (m) | 1.63 / 1.60 / 0.06 | 1.55 / 1.46 / 0.16 | -0.08 (0.16) | no measurable difference |
| safety | Nozzle open outside DOSE | 0.00 / 0 / 0 | 0.00 / 0 / 0 | +0.00 (0.00) | no measurable difference |
| safety | Completed (1 = DONE, no abort / hold timeout / ERROR) | 1.00 / 1.00 / 0.00 | 1.00 / 1.00 / 0.00 | +0.00 (0.01) | no measurable difference |
| safety | All safety rows pass (1 = yes) | 1.00 / 1.00 / 0.00 | 1.00 / 1.00 / 0.00 | +0.00 (0.01) | no measurable difference |
| safety | Sprayed tree targeted again (episodes) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | Clearance-guard triggers | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | Guard-blind counter | 10.33 / 12.00 / 4.00 | 1.33 / 2.00 / 1.00 | -9.00 (4.00) | field_first lower (info only) |
| guards | Holds (all reasons) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | IDs re-resolved | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | IDs dropped (vanished) | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | +0.00 (0.00) | no measurable difference |
| guards | Late trees (cleanup list) | 1.33 / 2.00 / 1.00 | 0.00 / 0.00 / 0.00 | -1.33 (1.00) | field_first lower (info only) |
| guards | Targets skipped as implausibly high | 0.33 / 1.00 / 1.00 | 0.33 / 1.00 / 1.00 | +0.00 (1.00) | no measurable difference |
| guards | Spray points raised (split neighbour) | 0.33 / 1.00 / 1.00 | 2.33 / 3.00 / 1.00 | +2.00 (1.00) | field_first higher (info only) |
| map | Map recall (final) | 1.00 / 0.99 / 0.01 | 1.00 / 1.00 / 0.00 | +0.00 (0.01) | no measurable difference |
| map | Map precision (final) | 0.99 / 0.99 / 0.01 | 0.98 / 0.96 / 0.03 | -0.01 (0.03) | no measurable difference |
| map | Split trees (final) | 0.67 / 1.00 / 1.00 | 1.33 / 3.00 / 3.00 | +0.67 (3.00) | no measurable difference |
| map | False trees (final) | 0.00 / 0.00 / 0.00 | 0.67 / 1.00 / 1.00 | +0.67 (1.00) | no measurable difference |
| map | Mean IoU (final) | 0.69 / 0.68 / 0.02 | 0.69 / 0.68 / 0.02 | +0.00 (0.02) | no measurable difference |
| map | Centre RMSE (m, final) | 0.44 / 0.48 / 0.09 | 0.45 / 0.47 / 0.04 | +0.01 (0.09) | no measurable difference |
| map | Centre error max (m, final) | 1.42 / 1.91 / 0.82 | 1.81 / 1.99 / 0.31 | +0.39 (0.82) | no measurable difference |
| map | Height bias (m, final) | -0.01 / -0.03 / 0.03 | -0.22 / -0.26 / 0.10 | -0.21 (0.10) | corridor better |
| map | Height RMSE (m, final) | 0.68 / 0.73 / 0.10 | 0.52 / 0.85 / 0.50 | -0.16 (0.50) | no measurable difference |
| map | Spill outside all canopies (final) | 0.28 / 0.29 / 0.01 | 0.30 / 0.30 / 0.01 | +0.01 (0.01) | no measurable difference |
| map | Max height drift per ID since lock (m) | 3.93 / 5.45 / 3.28 | 0.00 / 0.00 / 0.00 | -3.93 (3.28) | field_first better |
| map | Frames with an implausible height | 2230.67 / 2639.00 / 658.00 | 2003.67 / 3392.00 / 3392.00 | -227.00 (3392.00) | no measurable difference |
| map | Map safety violations (total) | 2243.00 / 2642.00 / 650.00 | 2003.67 / 3392.00 / 3392.00 | -239.33 (3392.00) | no measurable difference |
| cost | Navigator step mean (ms) | 0.01 / 0.01 / 0.00 | 0.02 / 0.02 / 0.00 | +0.00 (0.00) | field_first higher (info only) |
| cost | Navigator step p99 (ms) | 0.06 / 0.07 / 0.00 | 0.11 / 0.11 / 0.00 | +0.04 (0.00) | field_first higher (info only) |
| cost | SGBM mean (ms) | 13.41 / 13.44 / 0.04 | 13.58 / 13.63 / 0.09 | +0.17 (0.14) | field_first higher (info only) |
| cost | Detector mean (ms) | 9.58 / 9.64 / 0.10 | 9.73 / 9.79 / 0.10 | +0.15 (0.10) | field_first higher (info only) |
| cost | Real-time factor | 1.00 / 1.00 / 0.00 | 1.00 / 1.00 / 0.00 | -0.00 (0.01) | no measurable difference |

### Corridor vs field_first: what differs and why

- **Where the time goes**: settle and dose times show no measurable difference; scan time is 1.2 s (5x5) and 2.5 s
  (8x12) shorter in field_first, out of 94-95 s and 171-174 s: above the noise floor, negligible in size. The
  difference is transit (120 vs 51 s on 5x5,
  135 vs 60 s on 8x12) and, less, glide (-12 s on each). Corridor mode climbs back to the 15 m scan altitude after
  each corridor's spray-back and transits to the next corridor's entry (8 and 10 corridors); field_first scans all
  corridors in one boustrophedon sequence at scan altitude, descends once, and glides from corridor to corridor at
  spray altitude. Corridor mode's planned advantage - spraying a corridor while its map is fresh - shows up in
  neither the spray outcome nor the map's centre error (no measurable difference), and it has a cost in the map's
  heights (finding 2 below).
- **Spray and map centres**: no measurable difference in trees sprayed / missed / double-sprayed, on-target fraction,
  off-target and ground litres, map recall, precision, IoU and centre RMSE.
- **Map heights**: on 5x5 field_first's final heights are better (bias +0.30 vs +0.69 m, RMSE 0.49 vs 0.89 m); on
  8x12 corridor's bias is smaller (-0.01 vs -0.22 m) with no measurable RMSE difference. The per-ID height drift after
  the lock (3.7 / 3.9 m in corridor mode, <= 0.17 m in field_first) is structural - field_first has no scan after its
  only lock, corridor mode scans again after every lock but the last - but with 3 runs it is not statistically
  separable on 5x5 (difference 3.6 m, noise floor 4.2 m); on 8x12 it is just above the floor (3.9 vs 3.3 m). The
  mechanism (finding 2) rests on the traces, not on this statistic. The TreeMapper scorer's implausible-height frames
  and map safety violations show no measurable difference on either orchard.
- **Safety margins**: minimum true clearance 2.23 vs 2.04 m on 5x5 (corridor higher by 0.19 m, just above the 0.13 m
  floor), no measurable difference on 8x12 (1.63 vs 1.55 m; worst run 1.46 m in field_first). Both are above zero
  in every run; the clearance guard never had to act.
- **Guards and counters**: no holds, no re-resolved or dropped IDs in any run. `guard_blind` scales with the number of
  descents from scan altitude (6.3 and 10.3 per corridor run vs 1.0 and 1.3 per field_first run; not analysed
  further, no effect on the dose). Late trees (cleanup list) occur only in corridor mode (0.3 and 1.3 per run).
  `spray_point_raised` (a split neighbour within reach) is 2.3 per run in field_first on 8x12 vs 0.3: a difference
  of 2.0 against a noise floor of 1.0 (the larger 3-run spread). These counters, like the cost rows, are tagged
  "info only" in the tables: they describe how the mission was flown, not its quality.
- **Cost**: the navigator step is 0.01-0.02 ms mean, <= 0.11 ms p99 (field_first holds the whole field's list: ~2x,
  irrelevant at 20 Hz); SGBM 13.3-13.6 ms and detector 9.6-9.9 ms per frame on the workstation GPU; RTF 1.000 in all
  runs (the sim was capped at 1.0).

### Why trees were missed (diagnosis; nothing in the navigator, perception or TreeMapper was changed)

A truth tree is *missed* when no dose episode put >= 50 % of its intended dose on it (scorer definition). **Spray
footprint**: the navigator flies the nozzle to `published height + spray_clearance` (2.5 m) and the nozzle sits 0.3 m
below the body (`nozzle.offset_body`), so it is 2.2 m above the published canopy top; with the 20 deg cone half-angle
of the specs the footprint radius on the canopy-top plane is **r_fp = 2.2 x tan 20 deg = 0.80 m** (when the published
height is right). Each missed tree gets one class, in this precedence (exact rules in the module doc of
`sim_compare_report.py`): (d) navigator skip with the counter that explains it, (b) split / fragment / merged entry,
(c) phantom took the dose, (a) the aimed map entry's centre (at the episode start) further from the truth centre than
r_fp, (e) other with the reason. An episode the scorer left unmatched (entry centre outside every canopy and > 2 m from
every truth centre) is attributed to the nearest truth tree when it is within that tree's crown radius + r_fp + 0.5 m
(these are displaced copies of one tree, the scorer's "orphan sprayed IDs", so they land in (a), not (c)); otherwise it
is a phantom episode.

The first-sighting columns: where the drone was, across track, when the aimed entry was **first published**. The camera
flies with its wide image axis across track (hfov 70.3 deg), so at 15 m over a ~2.6 m canopy the view reaches
(15 - 2.6) x tan 35.15 deg = **8.7 m** to either side; an offset of 7-8.4 m means the crown (radius 1.3-1.8 m) was
partly outside the image when the entry was confirmed.

#### Missed trees by class

Classes: (a) map centre error > spray footprint; (b) split / fragment / merged entry; (c) phantom took the dose; (d) navigator skip; (e) other.

| Spec | Strategy | (a) | (b) | (c) | (d) | (e) | Missed |
|---|---|---|---|---|---|---|---|
| 5x5_tropical | corridor | 9 | 0 | 0 | 1 | 1 | 11 |
| 5x5_tropical | field_first | 12 | 0 | 0 | 0 | 0 | 12 |
| rows_8x12 | corridor | 0 | 1 | 0 | 1 | 6 | 8 |
| rows_8x12 | field_first | 3 | 0 | 0 | 1 | 0 | 4 |
| all | all | 24 | 1 | 0 | 3 | 7 | 35 |

| Spec | Strategy | Run | Truth tree | Class | Aimed entry (episodes) | Best dose fraction | Map centre error at spray (m) | Crown radius (m) | Published - true top (m) | Nozzle above true top (m) | Actual footprint (m) | First-sighting lateral offset (m) | First-sighting centre error (m) | Final IoU | Mission time fraction | Gap to nearest crown (m) | Note |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 5x5_tropical | corridor | run_1 | 2 | d | None (0) | n/a | n/a | 1.54 | n/a | n/a | n/a | n/a | n/a | 0.40 | n/a | 4.13 | target_height_implausible (entry 12 peaked at 9.06 m > 9.00 m) |
| 5x5_tropical | corridor | run_1 | 13 | a | 65 (1) | 0.37 | 1.38 | 1.39 | +0.20 | 2.40 | 0.87 | +7.8 | 1.95 | 0.25 | 0.46 | 2.95 | 1.38 m > 0.80 m |
| 5x5_tropical | corridor | run_1 | 17 | a | 79 (1) | 0.39 | 1.41 | 1.45 | +0.44 | 2.64 | 0.96 | +7.9 | 1.06 | 0.25 | 0.58 | 1.33 | 1.41 m > 0.80 m |
| 5x5_tropical | corridor | run_1 | 21 | a | 115 (1) | 0.44 | 1.43 | 1.62 | +0.50 | 2.70 | 0.98 | +8.1 | 1.69 | 0.25 | 0.85 | 3.06 | 1.43 m > 0.80 m |
| 5x5_tropical | corridor | run_1 | 22 | a | 116 (1) | 0.29 | 2.13 | 1.76 | +0.53 | 2.73 | 0.99 | +8.2 | 1.75 | 0.25 | 0.84 | 3.06 | 2.13 m > 0.80 m |
| 5x5_tropical | corridor | run_1 | 24 | a | 121 (1) | 0.43 | 1.17 | 1.32 | +0.34 | 2.54 | 0.92 | +7.7 | 1.22 | 0.21 | 0.81 | 3.59 | 1.17 m > 0.80 m |
| 5x5_tropical | corridor | run_2 | 16 | e | 122 (1) | 0.48 | 0.61 | 1.33 | +1.78 | 3.98 | 1.45 | +5.3 | 0.37 | 0.48 | 0.70 | 4.64 | height over-estimate: footprint 1.45 m > crown radius 1.33 m; height +1.8 m |
| 5x5_tropical | corridor | run_2 | 21 | a | 135 (1) | 0.39 | 1.58 | 1.62 | +0.36 | 2.56 | 0.93 | +8.1 | 1.82 | 0.25 | 0.83 | 3.06 | 1.58 m > 0.80 m |
| 5x5_tropical | corridor | run_3 | 12 | a | 67 (1) | 0.31 | 1.43 | 1.51 | +2.32 | 4.52 | 1.64 | +8.4 | 1.46 | 0.33 | 0.47 | 1.33 | 1.43 m > 0.80 m; height +2.3 m |
| 5x5_tropical | corridor | run_3 | 13 | a | 83 (1) | 0.46 | 1.22 | 1.39 | +0.26 | 2.46 | 0.89 | +7.8 | 1.20 | 0.25 | 0.44 | 2.95 | 1.22 m > 0.80 m |
| 5x5_tropical | corridor | run_3 | 21 | a | 128 (1) | 0.43 | 1.49 | 1.62 | +0.04 | 2.24 | 0.82 | +8.1 | 1.66 | 0.44 | 0.83 | 3.06 | 1.49 m > 0.80 m |
| 5x5_tropical | field_first | run_1 | 11 | a | 57 (1) | 0.34 | 1.62 | 1.54 | -0.22 | 1.98 | 0.72 | +6.0 | 2.30 | 0.30 | 0.66 | 3.14 | 1.62 m > 0.80 m |
| 5x5_tropical | field_first | run_1 | 13 | a | 79 (1) | 0.43 | 1.22 | 1.39 | +0.64 | 2.84 | 1.03 | +7.8 | 0.99 | 0.31 | 0.71 | 2.95 | 1.22 m > 0.80 m |
| 5x5_tropical | field_first | run_2 | 12 | a | 65 (1) | 0.35 | 1.57 | 1.51 | +0.05 | 2.25 | 0.82 | +8.4 | 1.58 | 0.23 | 0.67 | 1.33 | 1.57 m > 0.80 m |
| 5x5_tropical | field_first | run_2 | 16 | a | 126 (1) | 0.43 | 1.21 | 1.33 | +0.16 | 2.36 | 0.86 | +5.3 | 1.20 | 0.29 | 0.51 | 4.64 | 1.21 m > 0.80 m |
| 5x5_tropical | field_first | run_3 | 10 | a | 28 (1) | 0.46 | 1.94 | 2.15 | -0.38 | 1.82 | 0.66 | +5.4 | 2.47 | 0.38 | 0.74 | 1.45 | 1.94 m > 0.80 m |
| 5x5_tropical | field_first | run_3 | 11 | a | 59 (1) | 0.29 | 1.77 | 1.54 | -0.21 | 1.99 | 0.72 | +6.1 | 2.54 | 0.32 | 0.67 | 3.14 | 1.77 m > 0.80 m |
| 5x5_tropical | field_first | run_3 | 13 | a | 79 (1) | 0.47 | 1.15 | 1.39 | +0.40 | 2.60 | 0.95 | +7.8 | 1.40 | 0.26 | 0.71 | 2.95 | 1.15 m > 0.80 m |
| 5x5_tropical | field_first | run_3 | 15 | a | 95 (1) | 0.47 | 1.42 | 1.58 | -0.12 | 2.08 | 0.76 | +3.9 | 1.64 | 0.26 | 0.61 | 2.36 | 1.42 m > 0.80 m |
| 5x5_tropical | field_first | run_3 | 16 | a | 129 (1) | 0.16 | 1.92 | 1.33 | +0.14 | 2.34 | 0.85 | +5.9 | 2.07 | 0.20 | 0.52 | 4.64 | 1.92 m > 0.80 m |
| 5x5_tropical | field_first | run_3 | 21 | a | 153 (1) | 0.49 | 1.22 | 1.62 | +0.90 | 3.10 | 1.13 | +8.1 | 1.34 | 0.24 | 0.50 | 3.06 | 1.22 m > 0.80 m |
| 5x5_tropical | field_first | run_3 | 24 | a | 147 (1) | 0.39 | 1.34 | 1.32 | +0.43 | 2.63 | 0.96 | +7.7 | 1.45 | 0.21 | 0.46 | 3.59 | 1.34 m > 0.80 m |
| 5x5_tropical | field_first | run_3 | 25 | a | 144 (1) | 0.08 | 2.48 | 1.56 | +0.11 | 2.31 | 0.84 | +8.0 | 3.10 | 0.25 | 0.44 | 3.59 | 2.48 m > 0.80 m |
| rows_8x12 | corridor | run_1 | 24 | e | 78 (1) | 0.36 | 0.37 | 1.60 | +3.73 | 5.93 | 2.16 | +3.9 | 0.83 | 0.42 | 0.36 | 1.27 | height over-estimate: footprint 2.16 m > crown radius 1.60 m; height +3.7 m |
| rows_8x12 | corridor | run_2 | 1 | e | 0 (1) | 0.34 | 0.22 | 1.66 | +4.11 | 6.31 | 2.30 | +3.4 | 0.49 | 0.57 | 0.16 | 0.69 | height over-estimate: footprint 2.30 m > crown radius 1.66 m; height +4.1 m |
| rows_8x12 | corridor | run_2 | 13 | d | None (0) | n/a | n/a | 1.67 | n/a | n/a | n/a | n/a | n/a | 0.44 | n/a | 0.68 | target_height_implausible (entry 11 peaked at 10.92 m > 9.00 m) |
| rows_8x12 | corridor | run_2 | 15 | e | None (0) | n/a | n/a | 1.52 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.33 | never published by TreeMapper |
| rows_8x12 | corridor | run_2 | 24 | e | 88 (1) | 0.47 | 0.23 | 1.60 | +2.81 | 5.01 | 1.82 | +3.9 | 1.09 | 0.44 | 0.36 | 1.27 | height over-estimate: footprint 1.82 m > crown radius 1.60 m; height +2.8 m |
| rows_8x12 | corridor | run_3 | 15 | b | 100 (1) | 0.48 | 1.06 | 1.52 | +1.36 | 3.56 | 1.30 | -1.6 | 0.75 | 0.42 | 0.25 | 0.33 | aimed entry 1.06 m off; height +1.4 m |
| rows_8x12 | corridor | run_3 | 24 | e | 95 (1) | 0.47 | 0.61 | 1.60 | +2.78 | 4.98 | 1.81 | +3.9 | 0.25 | 0.47 | 0.36 | 1.27 | height over-estimate: footprint 1.81 m > crown radius 1.60 m; height +2.8 m |
| rows_8x12 | corridor | run_3 | 36 | e | 133 (1) | 0.49 | 0.36 | 1.88 | +3.52 | 5.72 | 2.08 | +5.1 | 1.56 | 0.43 | 0.47 | 0.19 | height over-estimate: footprint 2.08 m > crown radius 1.88 m; height +3.5 m |
| rows_8x12 | field_first | run_1 | 65 | a | 208 (1) | 0.45 | 1.68 | 1.77 | -0.42 | 1.78 | 0.65 | +7.4 | 2.27 | 0.28 | 0.37 | 1.14 | 1.68 m > 0.80 m |
| rows_8x12 | field_first | run_2 | 30 | d | None (0) | n/a | n/a | 1.32 | n/a | n/a | n/a | n/a | n/a | 0.73 | n/a | 1.66 | target_height_implausible (entry 105 peaked at 10.32 m > 9.00 m) |
| rows_8x12 | field_first | run_2 | 65 | a | 183 (1) | 0.41 | 1.77 | 1.77 | -0.41 | 1.79 | 0.65 | +7.5 | 2.21 | 0.31 | 0.37 | 1.14 | 1.77 m > 0.80 m |
| rows_8x12 | field_first | run_2 | 85 | a | 229 (1) | 0.49 | 1.43 | 1.57 | -0.31 | 1.89 | 0.69 | +3.1 | 1.69 | 0.27 | 0.30 | 1.28 | 1.43 m > 0.80 m |

#### Patterns over every attributed dose episode

First-sighting lateral offset = across-track distance from the drone to the truth centre when the aimed entry was first published. 'Toward the first track' = the across-track part of the map error points back toward where the drone was at that moment.

| Spec | Strategy | Episodes | offset < 5 m: n, mean error, share > r_fp, miss share | offset >= 5 m: same | Errors > r_fp pointing toward the first track | Across-track share of those errors | Miss share by mission third | Height over-estimate > 1 m: n, miss share |
|---|---|---|---|---|---|---|---|---|
| 5x5_tropical | all | 153 | 87, 0.32 m, 6 %, 1 % | 66, 1.04 m, 59 %, 36 % | 43 of 44 | 0.89 | 0 % / 18 % / 21 % | 21, 10 % |
| 5x5_tropical | corridor | 77 | 42, 0.27 m, 7 %, 0 % | 35, 0.94 m, 49 %, 34 % | 19 of 20 | 0.91 | 0 % / 16 % / 28 % | 17, 12 % |
| 5x5_tropical | field_first | 76 | 45, 0.36 m, 4 %, 2 % | 31, 1.15 m, 71 %, 39 % | 24 of 24 | 0.86 | n/a / 19 % / 15 % | 4, 0 % |
| rows_8x12 | all | 519 | 245, 0.32 m, 7 %, 3 % | 274, 0.39 m, 9 %, 3 % | 24 of 42 | 0.75 | 3 % / 4 % / 2 % | 17, 47 % |
| rows_8x12 | corridor | 258 | 125, 0.34 m, 9 %, 4 % | 133, 0.39 m, 7 %, 2 % | 7 of 20 | 0.81 | 3 % / 5 % / 1 % | 14, 50 % |
| rows_8x12 | field_first | 261 | 120, 0.30 m, 5 %, 2 % | 141, 0.39 m, 11 %, 3 % | 17 of 22 | 0.70 | 4 % / 2 % / 3 % | 3, 33 % |

| Spec | Strategy | Episodes with err > r_fp that still got >= 50 %: n, error range | Max error / crown radius of an episode that got >= 50 % | Min error / crown radius of an episode with err > r_fp that got < 50 % | Height over-estimate > 1.5 m at spray: n, along-track distance of the tree from the field's entry edge |
|---|---|---|---|---|---|
| 5x5_tropical | all | 20, 0.81 .. 1.82 m | 0.85 | 0.75 | 9, 3.8 .. 26.5 m |
| 5x5_tropical | corridor | 9, 0.81 .. 1.82 m | 0.85 | 0.88 | 8, 3.8 .. 19.7 m |
| 5x5_tropical | field_first | 11, 0.83 .. 1.62 m | 0.79 | 0.75 | 1, 26.5 .. 26.5 m |
| rows_8x12 | all | 32, 0.80 .. 1.91 m | 0.93 | 0.70 | 12, 5.1 .. 18.8 m |
| rows_8x12 | corridor | 17, 0.80 .. 1.91 m | 0.93 | 0.70 | 10, 5.1 .. 18.8 m |
| rows_8x12 | field_first | 15, 0.83 .. 1.77 m | 0.90 | 0.91 | 2, 9.1 .. 18.6 m |

#### Dose aim accuracy

Aim error = distance from the drone's mean xy during a dose episode to the centre of the map entry the navigator aimed at (its `/spray_target` id, centre in the latest map before the episode). It measures the navigator and the drone, not the map.

| Spec / strategy | Episodes | Median (m) | p95 (m) | Max (m) | Share <= 0.1 m |
|---|---|---|---|---|---|
| 5x5_tropical/corridor | 77 | 0.000 | 0.106 | 0.172 | 92.2 % |
| 5x5_tropical/field_first | 77 | 0.000 | 0.000 | 0.028 | 100.0 % |
| rows_8x12/corridor | 258 | 0.000 | 0.065 | 0.328 | 96.5 % |
| rows_8x12/field_first | 263 | 0.000 | 0.010 | 0.075 | 100.0 % |
| all/all | 675 | 0.000 | 0.043 | 0.328 | 97.8 % |

Phantom episodes (dose on an entry that is not a displaced copy of any truth tree): 5x5_tropical field_first run_1 id 149 at 175 s, rows_8x12 field_first run_2 id 82 at 634 s, rows_8x12 field_first run_3 id 86 at 635 s.

**Findings** (numbers from the generated tables above and the generated height traces in "Appendix: height traces";
the few derived readings are marked as such; "TreeMapper" = `cand_obj` as configured by `config/cand_obj.cfg`):

1. **First sighting at the image edge -> centre bias (5x5_tropical, both strategies; 21 of the 23 5x5 misses).**
   Over every attributed 5x5 episode, entries first published with the tree < 5 m across track were 0.32 m off at spray
   time (6 % beyond r_fp, 1 % missed); entries first published at >= 5 m were 1.04 m off (59 % beyond r_fp, 36 %
   missed). In 43 of the 44 episodes whose error exceeded r_fp, the error points back toward the track the drone flew
   when the entry was first published, and 89 % of its length is across track. The corridors are scanned in +y order
   and the view is ~17 m wide across track, so a tree is first confirmed from a scan line one or two corridors before
   its own, at the far edge of the image, with part of the crown outside it: the published centre sits toward the
   drone. Later passes much closer to the tree (its own corridor's scan, 3-4 m) did not pull it back far enough
   (first-sighting error 1.0-3.1 m, at spray 1.15-2.48 m, final IoU 0.20-0.44). (Derived reading, from the truth
   positions in `truth.json` and the 8 equal strips of 4.75 m the navigator planned over the 38 m field: the trees
   concerned are the 5x5 rows at y ~ 13-15 and y ~ 29, plus single trees at y ~ 7.5 / 19.5 / 21.7, all within ~1.8 m
   of a strip boundary, i.e. 7-8 m from the scan line two corridors back.) It hits corridor and field_first alike
   (5x5 miss share of far-sighted episodes 34 % vs 39 %): it is a perception property, not a strategy effect.
   On rows_8x12 it is much weaker (far-sighted entries 0.39 m off vs 0.32 m near, 9 % vs 7 % beyond r_fp); it still
   caused two of the three rows_8x12 field_first (a) misses (tree 65 in runs 1 and 2: first published 7.4-7.5 m
   across track, 2.2-2.3 m off, 1.7-1.8 m off at spray time). Why the dense rows orchard shows it less was not
   investigated. Centre error > r_fp is necessary for a class (a) miss but not sufficient (threshold table under "Patterns"): on
   5x5 every episode that got >= 50 % had an error below 0.85 of the crown radius and every episode beyond r_fp that
   got < 50 % was at 0.75 or more; 20 5x5 episodes with errors of 0.81-1.82 m still delivered >= 50 % (8x12: 0.93 /
   0.70, 32 episodes at 0.80-1.91 m; its lowest misses are the height over-estimates of finding 2).
2. **Published height ramps up after the drone has passed, and the lock freezes it (mostly corridor mode;
   5 of the 8 rows_8x12 corridor misses, 1 of the 11 5x5 corridor misses, plus all three navigator skips).** TreeMapper rate-limits a
   published height to `max_height_rate` = 0.5 m/s. In the traces the height of an entry near the start of a scan
   climbs at exactly that rate while the drone flies away from it along the corridor (out of view, 10-25 m away), and
   keeps climbing until that scan line ends (outside SCAN TreeMapper holds its entries; in corridor mode the list
   then locks); a later scan line that sees the tree again brings it back down at the same rate. Examples (appendix traces): 5x5 corridor run_1 entry 12
   (tree 2, true top 2.31 m) 4.35 m at 21.63 s, 25 m behind the drone at 8.39 m by 29.83 s, 8.99 m at the first lock
   (31.03 s, PLANNING; peak 9.06 m), back down from 79 s when the next scan line saw it, 3.10 m at the end (the
   "5.9 m height drift after lock" in that scorecard); rows_8x12 corridor run_1 entry 78 (tree 24, top 2.83 m)
   3.63 m when the drone passed it at 283.53 s, 6.37 m at 289.93 s with the drone 23.8 m away, 6.57 m frozen from the
   lock on, nozzle 5.93 m above the true top at its dose (miss table). Consequences: (i) a frozen height above
   home + `expected_canopy_top_max` + `target_height_margin` = 9 m makes the navigator skip the tree
   (`target_height_implausible`; it is never revisited when the height later comes down - the known Task 16B minor);
   (ii) a frozen height 2.8-4.1 m too high puts the nozzle 5-6.3 m above the true top, the footprint radius grows to
   1.8-2.3 m, wider than the crown, and half the dose lands on the ground (class (e) "height over-estimate").
   Over-estimates > 1.5 m at spray time (threshold table): 18 episodes in corridor mode (8 on 5x5 on trees
   3.8-19.7 m from the field's entry edge, 10 on 8x12 at 5.1-18.8 m; the corridors are 38 m and 59.5 m long, so
   these trees are in the part of the corridor scanned first), against 3 in field_first (1 on 5x5 at 26.5 m, 2 on
   8x12). field_first locks only after every corridor has been scanned, so a later scan
   line usually sees the tree again before the lock and pulls the height back (5x5 final height bias +0.30 m vs
   +0.69 m, RMSE 0.49 vs 0.89 m). When no later line sees it, field_first freezes it too: rows_8x12 field_first run_2,
   tree 30, entry 105: 3.62 m when the drone passed at 89.93 s, 10.31 m when that scan line ended (103.33 s,
   SCAN_APPROACH), 10.32 m to the end of the mission; skipped at the single lock (`target_height_implausible`).
   Why the height target is too high once the tree is behind the drone is inside `cand_obj` and was not diagnosed.
3. **Controller's run_1 observation, checked**: 5x5 corridor run_1 sprayed 19 of 25 (confirmed). Five of the six
   misses (13, 17, 21, 22, 24) are class (a): map centre error at spray time 1.17-2.13 m, final IoU 0.21-0.25,
   first published 7.7-8.2 m across track with the error pointing back to that track (finding 1); they are late in
   the mission (time fraction 0.46-0.85) only because they are in the rows scanned last, and they have no crowded
   neighbours (crown gaps 1.3-3.6 m). The sixth, tree 2, is different: centre error 0.35 m, but its entry 12 was
   frozen at 8.99-9.06 m (finding 2) and skipped (`target_height_implausible` 1); the 5.9 m per-ID height drift after lock is
   that same entry coming back down to 3.1 m in later scans.
4. **Phantoms** cost three doses in 12 runs, all in field_first (5x5 run_1 entry 149; 8x12 run_2 entry 82, run_3
   entry 86; list under the patterns table) and caused no miss: class (c) is empty.

### Known weaknesses of this comparison

- **One orchard per spec, 3 runs per side.** The spread is the loop's own non-determinism on a fixed seed; orchard
  variation (other seeds, other layouts) is not sampled. The noise-floor rule (difference vs the larger spread of 3
  runs) is coarse: differences just above it are hints.
- **The map, not the navigator, limits the spray outcome.** The drone's mean xy during a dose was within 0.043 m of the centre of
  the map entry it aimed at (its `/spray_target` id, centre in the map before the dose) in 95 % of the 675 episodes,
  max 0.33 m, 97.8 % within 0.1 m (table "Dose aim accuracy"), against map centre errors of 1.15-2.48 m on the
  class (a) misses; every miss traces back to a map entry: its published centre (24) or height (9; findings 1 and 2), a merged entry
  (1), or a tree never published (1). The map runs on SGBM over rendered images with a periodic grass texture; the earlier Isaac
  results of TreeMapper were on rendered depth (see "What this smoke says about the sim"), so map quality here is
  neither the recording's nor a real flight's.
- **The two TreeMapper behaviours behind most misses are not fixed** (TreeMapper is out of scope here): the
  post-pass height ramp frozen by the lock (mostly in corridor mode), and the edge-of-image centre bias. A corridor-vs-field_first
  verdict on spray quality may change once they are; the efficiency result does not depend on them.
- **The navigator never retries a target skipped as implausibly high** once its height comes back down (3 trees in
  12 runs).
- **TreeMapper's own scorer rows** (frames with an implausible height, non-target changes during descent, new tree
  inside a sprayed outline; "Map safety violations" in the tables) are high on rows_8x12 in both strategies (no measurable
  difference); they are map-quality rows of the TreeMapper scorer, not the mission safety rows, which all pass.
- **Sim simplifications**: no wind or drift, ideal rectified stereo pair, nozzle as a ray cone onto collider meshes,
  RTF 1.0 on a workstation GPU, detector trained on real images (parity with the Orin checked above), the Isaac
  drone's controller instead of ArduPilot.
- Minimum true clearance over the canopy is 1.6 m on rows_8x12 (taller, denser trees) against 2.0-2.3 m on 5x5; it
  passes (> 0) and no clearance-guard trigger occurred in any run.

### Appendix: height traces

Generated with `python3 -m treemapper_tools.sim_compare_report /data/work/sim_runs/cmp16c_5x5_tropical_corridor --trace /data/work/sim_runs/cmp16c_5x5_tropical_corridor/run_1:12 --trace /data/work/sim_runs/cmp16c_rows_8x12_corridor/run_1:78 --trace /data/work/sim_runs/cmp16c_rows_8x12_field_first/run_2:105` (one row whenever the published height moved >= 0.25 m, plus the last snapshot).

#### Height trace: /data/work/sim_runs/cmp16c_5x5_tropical_corridor/run_1

| Entry | t (s) | Published height (m) | Phase | Drone x, y, z (m) | Drone - entry distance (m) |
|---|---|---|---|---|---|
| 12 | 21.63 | 4.35 | SCAN | 6.4, -2.6, 15.0 | 2.0 |
| 12 | 22.43 | 4.69 | SCAN | 9.8, -2.6, 15.0 | 2.7 |
| 12 | 23.23 | 5.09 | SCAN | 12.7, -2.6, 15.0 | 5.3 |
| 12 | 23.83 | 5.39 | SCAN | 15.3, -2.6, 15.0 | 7.8 |
| 12 | 24.43 | 5.69 | SCAN | 17.8, -2.6, 15.0 | 10.2 |
| 12 | 25.03 | 6.00 | SCAN | 20.0, -2.6, 15.0 | 12.4 |
| 12 | 25.63 | 6.30 | SCAN | 22.5, -2.6, 15.0 | 14.9 |
| 12 | 26.23 | 6.58 | SCAN | 24.6, -2.6, 15.0 | 17.0 |
| 12 | 26.83 | 6.88 | SCAN | 27.1, -2.6, 15.0 | 19.5 |
| 12 | 27.43 | 7.20 | SCAN | 29.6, -2.6, 15.0 | 21.9 |
| 12 | 28.03 | 7.49 | SCAN | 31.2, -2.6, 15.0 | 23.6 |
| 12 | 28.63 | 7.79 | SCAN | 32.5, -2.6, 15.0 | 24.8 |
| 12 | 29.23 | 8.09 | SCAN | 32.9, -2.6, 15.0 | 25.3 |
| 12 | 29.83 | 8.39 | SCAN | 33.0, -2.6, 15.0 | 25.4 |
| 12 | 30.43 | 8.69 | SCAN | 33.0, -2.6, 15.0 | 25.4 |
| 12 | 31.03 | 8.99 | PLANNING | 33.0, -2.6, 15.0 | 25.4 |
| 12 | 79.03 | 8.72 | SCAN | 8.3, 2.1, 15.0 | 3.3 |
| 12 | 79.63 | 8.42 | SCAN | 10.5, 2.1, 15.0 | 4.3 |
| 12 | 80.23 | 8.12 | SCAN | 12.7, 2.1, 15.0 | 6.0 |
| 12 | 80.83 | 7.82 | SCAN | 15.3, 2.1, 15.0 | 8.2 |
| 12 | 81.43 | 7.52 | SCAN | 17.5, 2.1, 15.0 | 10.3 |
| 12 | 82.03 | 7.22 | SCAN | 20.1, 2.1, 15.0 | 12.9 |
| 12 | 82.63 | 6.92 | SCAN | 22.3, 2.1, 15.0 | 15.0 |
| 12 | 83.23 | 6.62 | SCAN | 24.5, 2.1, 15.0 | 17.2 |
| 12 | 83.83 | 6.32 | SCAN | 27.1, 2.1, 15.0 | 19.7 |
| 12 | 84.43 | 6.02 | SCAN | 29.3, 2.1, 15.0 | 21.9 |
| 12 | 85.03 | 5.72 | SCAN | 31.3, 2.1, 15.0 | 23.9 |
| 12 | 85.63 | 5.42 | SCAN | 32.4, 2.1, 15.0 | 25.0 |
| 12 | 86.23 | 5.17 | SCAN | 32.9, 2.1, 15.0 | 25.5 |
| 12 | 86.63 | 4.92 | SCAN | 33.0, 2.1, 15.0 | 25.5 |
| 12 | 87.23 | 4.62 | SCAN | 33.0, 2.1, 15.0 | 25.5 |
| 12 | 87.77 | 4.32 | SCAN | 33.0, 2.1, 15.0 | 25.5 |
| 12 | 88.27 | 4.07 | PLANNING | 33.0, 2.1, 15.0 | 25.5 |
| 12 | 100.83 | 3.76 | SCAN_APPROACH | 28.4, -0.1, 5.1 | 20.7 |
| 12 | 124.23 | 3.43 | SCAN | 12.5, 6.9, 15.0 | 9.3 |
| 12 | 124.83 | 3.18 | SCAN | 15.0, 6.9, 15.0 | 10.8 |
| 12 | 453.83 | 3.10 | IDLE | -3.0, -3.0, 0.0 | 10.8 |

#### Height trace: /data/work/sim_runs/cmp16c_rows_8x12_corridor/run_1

| Entry | t (s) | Published height (m) | Phase | Drone x, y, z (m) | Drone - entry distance (m) |
|---|---|---|---|---|---|
| 78 | 179.33 | 4.29 | SCAN | 6.5, 8.0, 15.0 | 4.0 |
| 78 | 179.93 | 4.56 | SCAN | 8.6, 8.0, 15.0 | 3.7 |
| 78 | 181.53 | 4.26 | SCAN | 14.7, 8.0, 15.0 | 7.4 |
| 78 | 182.13 | 3.95 | SCAN | 17.4, 8.0, 15.0 | 9.7 |
| 78 | 283.53 | 3.63 | SCAN | 7.5, 13.2, 15.0 | 1.6 |
| 78 | 285.13 | 3.97 | SCAN | 13.6, 13.2, 15.0 | 5.0 |
| 78 | 285.73 | 4.27 | SCAN | 15.8, 13.2, 15.0 | 7.2 |
| 78 | 286.33 | 4.57 | SCAN | 18.5, 13.2, 15.0 | 9.8 |
| 78 | 286.93 | 4.87 | SCAN | 20.7, 13.2, 15.0 | 12.0 |
| 78 | 287.53 | 5.17 | SCAN | 23.3, 13.2, 15.0 | 14.6 |
| 78 | 288.13 | 5.47 | SCAN | 25.5, 13.2, 15.0 | 16.8 |
| 78 | 288.73 | 5.76 | SCAN | 27.7, 13.2, 15.0 | 19.0 |
| 78 | 289.33 | 6.06 | SCAN | 30.3, 13.2, 15.0 | 21.6 |
| 78 | 289.93 | 6.37 | SCAN | 32.6, 13.2, 15.0 | 23.8 |
| 78 | 1017.53 | 6.57 | IDLE | -3.0, -3.0, 0.0 | 19.2 |

#### Height trace: /data/work/sim_runs/cmp16c_rows_8x12_field_first/run_2

| Entry | t (s) | Published height (m) | Phase | Drone x, y, z (m) | Drone - entry distance (m) |
|---|---|---|---|---|---|
| 105 | 73.33 | 3.33 | SCAN | 34.4, 8.0, 15.0 | 4.3 |
| 105 | 89.93 | 3.62 | SCAN | 36.1, 13.2, 15.0 | 1.0 |
| 105 | 90.73 | 4.02 | SCAN | 33.1, 13.2, 15.0 | 2.7 |
| 105 | 91.33 | 4.32 | SCAN | 30.5, 13.2, 15.0 | 5.3 |
| 105 | 91.93 | 4.62 | SCAN | 28.3, 13.2, 15.0 | 7.5 |
| 105 | 92.53 | 4.92 | SCAN | 26.1, 13.2, 15.0 | 9.7 |
| 105 | 93.13 | 5.21 | SCAN | 23.5, 13.2, 15.0 | 12.3 |
| 105 | 93.73 | 5.51 | SCAN | 21.3, 13.2, 15.0 | 14.4 |
| 105 | 94.33 | 5.83 | SCAN | 18.6, 13.2, 15.0 | 17.1 |
| 105 | 94.93 | 6.12 | SCAN | 16.4, 13.2, 15.0 | 19.3 |
| 105 | 95.53 | 6.42 | SCAN | 14.2, 13.2, 15.0 | 21.5 |
| 105 | 96.13 | 6.72 | SCAN | 11.6, 13.2, 15.0 | 24.1 |
| 105 | 96.73 | 7.02 | SCAN | 9.4, 13.2, 15.0 | 26.3 |
| 105 | 97.33 | 7.32 | SCAN | 6.8, 13.2, 15.0 | 28.9 |
| 105 | 97.93 | 7.61 | SCAN | 4.6, 13.2, 15.0 | 31.1 |
| 105 | 98.53 | 7.91 | SCAN | 2.4, 13.2, 15.0 | 33.3 |
| 105 | 99.13 | 8.22 | SCAN | -0.2, 13.2, 15.0 | 35.9 |
| 105 | 99.73 | 8.52 | SCAN | -2.2, 13.2, 15.0 | 37.9 |
| 105 | 100.33 | 8.82 | SCAN | -3.9, 13.2, 15.0 | 39.6 |
| 105 | 100.93 | 9.11 | SCAN | -4.8, 13.2, 15.0 | 40.5 |
| 105 | 101.53 | 9.42 | SCAN | -5.0, 13.2, 15.0 | 40.7 |
| 105 | 102.13 | 9.72 | SCAN | -5.0, 13.2, 15.0 | 40.7 |
| 105 | 102.73 | 10.02 | SCAN | -5.0, 13.2, 15.0 | 40.7 |
| 105 | 103.33 | 10.31 | SCAN_APPROACH | -5.0, 13.4, 15.0 | 40.7 |
| 105 | 908.13 | 10.32 | IDLE | -3.0, -3.0, 0.0 | 41.6 |

Not verified on the Orin or on a real flight
