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
  the 0.15 L reached canopies, the rest the ground. Not a safety event (clearance 12 m), but see "Concerns".

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
  map is clearly worse than on the recording (centre RMSE 0.38 .. 0.70 m, IoU 0.55 .. 0.62, height bias +0.4 .. +0.6 m,
  outlines spill 0.32 .. 0.44 outside the canopies over the three good runs). Spray outcomes are limited by that map.
- Per run (3x3, RTF 1.0): ~150 s Isaac startup, the mission (170 .. 220 sim s), ~5 s teardown, ~10 s scoring;
  340 .. 395 s wall in total. Bag 2.0 .. 2.6 MB, run directory 5 .. 6 MB.
