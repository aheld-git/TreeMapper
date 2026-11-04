# Miss reduction: replay dataset and baseline

Spec: `docs/superpowers/specs/2026-11-03-miss-reduction-design.md` (sections 2 and 4.1). This file holds the replay dataset
(four Isaac closed-loop runs recorded with `--record-frames`), the proof that `treemapper_replay` reproduces those closed
loops, and the "today" values of the acceptance criteria A1-A3 measured on it. Nothing here changes TreeMapper; the numbers
are the baseline every fix is measured against.

All paths below are inside the work container (`/data/work` = `/media/alex/waspsim/cloud_build_bundle/work` on the host).

## Replay dataset

One closed-loop run of each spec x strategy, recorded by `scripts/sim_mission.sh sim/specs/<spec>.json --strategy <s> --runs 1
--record-frames --out /data/work/sim_runs/mr_frames_<spec>_<s>` (batch log `/data/work/logs/rec_mr_20261104T094126Z.log`, all four
`rc=0`, mission end state `DONE`, nothing skipped, `rtf.json` RTF 1.00). Each run directory holds `frames.bin` (TMF1, from
`tools/treemapper_tools/sim_frames.py`), `frames.bin.meta.json`, `truth.json`, `mission.json`, `map.jsonl` (the live
TreeMapper's map stream), the filtered `mission.bag`, `scorecard.md`.

| Dataset (`/data/work/sim_runs/<name>/run_1/`) | Frames (with disparity) | Duration (s) | frames.bin | mission.bag | Recorded at commit | Truth trees | Sprayed |
|---|---|---|---|---|---|---|---|
| `mr_frames_5x5_tropical_corridor` | 4289 (3907) | 448.7 | 125.6 MB | 13.6 MB | `23b0957` | 25 | 27 |
| `mr_frames_5x5_tropical_field_first` | 3748 (3435) | 384.7 | 110.4 MB | 12.3 MB | `7298b89` | 25 | 25 |
| `mr_frames_rows_8x12_corridor` | 9611 (8951) | 998.3 | 287.8 MB | 94.9 MB | `7284ba6` | 86 | 86 |
| `mr_frames_rows_8x12_field_first` | 8568 (7936) | 889.2 | 255.2 MB | 93.6 MB | `7284ba6` | 86 | 88 |

Seeds: 1001 (5x5_tropical), 7 (rows_8x12). The commits differ because the tools-only commits `7298b89` and `7284ba6`
landed while the batch was running; `git diff 23b0957 HEAD -- core nav ros sim config scripts` is empty, so the code that
ran (navigation, simulator, TreeMapper node, config) is identical for all four. Every `check()` in `frames.bin.meta.json`:
`pose_invalid 0`, `disparity_rejected 0`, `grid_lost 0`.

A count that is not zero but does not show a loss: the disparity-grid node logged `stamp_gap` 420 / 288 / 815 / 735 (a stamp
interval more than 1.5x the previous one). Evidence that no image the live node used is missing from `frames.bin`: in every run the
grid node's `received == published` (3923 / 3923 on the first), the grid messages recorded are within 2 of the images the SGBM
published (`grid_lost 0`), and the replay reproduces the live map to 0.0004 m at every lock (next section), which it could not
if frames were missing. The cause of the large gaps themselves (upstream stamp spacing) was not investigated.
Two things in the closed loops themselves: the 5x5 runs have 27 / 26 map entries for 25 truth trees (corridor: truth trees 7 and
22 carry two entries each; field_first: truth tree 12 does) and `rows_8x12_field_first` sprayed 88 for 86 trees (one entry, id 79,
never matches a truth tree; the same in live and replay).

### Replay setup

* Binary: `treemapper_replay` built fresh from HEAD `7284ba6ceac67c7910ad2c973fd1ace2999e4cf8` (no uncommitted change under
  `core/` or `config/`):
  `cmake -S core -B /data/work/build_mr_core -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build /data/work/build_mr_core --target treemapper_replay -j4`.
* Config: exactly the file the closed-loop node loaded. `treemapper_sim.launch` loads `ros/treemapper_ros/config/treemapper.yaml`
  (`mapper: cand_obj`, `mapper_config_file: config/core/cand_obj.cfg`, `mapper_overrides: {}`); `config/core` is a symlink to the
  repository's `config/`, so it is `config/cand_obj.cfg`, **unchanged** (last commit `960ccb7`, 45 keys). Every run's `ros.log`
  reads `treemapper_node: mapper 'cand_obj' (config .../config/core/cand_obj.cfg, 45 keys)` and contains no override line. The
  launch passes no `mapper_config_file` argument. The `.cfg` is given to the replay as the `k=v;k=v` string
  (comments and spaces stripped, 45 pairs; stored next to each output as `config_kv.txt`).
* Replay command (per dataset `<ds>`, default `--rate 5`):

  ```
  CFG=$(sed -e 's/#.*//' -e 's/[[:space:]]//g' config/cand_obj.cfg | grep -v '^$' | paste -sd';')
  /data/work/build_mr_core/treemapper_replay --frames /data/work/sim_runs/<ds>/run_1/frames.bin --system cand_obj --config "$CFG" \
    --out-map /data/work/replay/baseline/<ds>/map.jsonl --out-targets /data/work/replay/baseline/<ds>/targets.json \
    --out-counters /data/work/replay/baseline/<ds>/counters.json
  ```
* Scoring (always with the replay's own targets, controller ruling R10; the closed loop without `--replay`):

  ```
  PYTHONPATH=tools python3 -m treemapper_tools.replay_metrics --map /data/work/sim_runs/<ds>/run_1/map.jsonl \
    --truth .../truth.json --mission .../mission.json --json live_metrics.json --md live_metrics.md
  PYTHONPATH=tools python3 -m treemapper_tools.replay_metrics --map /data/work/replay/baseline/<ds>/map.jsonl \
    --truth .../truth.json --mission .../mission.json --replay --targets /data/work/replay/baseline/<ds>/targets.json \
    --json metrics.json --md metrics.md
  ```
  Outputs: `/data/work/replay/baseline/<ds>/{map.jsonl,targets.json,counters.json,metrics.json,metrics.md,live_metrics.json,live_metrics.md,equivalence.json}`
  (`metrics.md` has the per-entry table). The map streams are 10-94 MB each (they carry `stage_ms`).
* Determinism: `mr_frames_5x5_tropical_corridor` replayed twice into separate directories. `map.jsonl` is byte-identical once
  the `stage_ms` field (wall-clock timings) is removed (`sed -E 's/,"stage_ms":\{[^}]*\}//'`; the raw files differ only in
  those timings), and `targets.json` and `counters.json` are byte-identical.

## Replay equals the closed loop (hard gate, spec 4.1)

The replay is judged against the same run's live `map.jsonl`, with the same `truth.json` and `mission.json`. Entry ids are the
mapper's own: live and replay entries are paired by (truth tree, rank by first publication), which is the same pairing as by
id except for two entries of one dataset (below). "Final" = the last snapshot the entry appears in.

| Dataset | Entries live / replay | Paired | Final `dh` within 0.15 m | Final `dc` within 0.15 m | Max abs final `dh` / `dc` (m) | Ramping entries: lock `dh` live -> replay | Attached doses live / replay |
|---|---|---|---|---|---|---|---|
| 5x5 corridor | 27 / 27 | 27 | 27 / 27 (100 %) | 27 / 27 (100 %) | 0.0000 / 0.0000 | truth 1: 1.802 -> 1.802; truth 12: 3.633 -> 3.633; truth 18: 1.913 -> 1.913 | 27 / 27 (of 27 episodes) |
| 5x5 field_first | 26 / 26 | 26 | 26 / 26 (100 %) | 26 / 26 (100 %) | 0.0000 / 0.0000 | truth 12: 6.539 -> 6.539 | 25 / 25 (of 25) |
| rows corridor | 86 / 86 | 86 | 86 / 86 (100 %) | 86 / 86 (100 %) | 0.0004 / 0.0252 | truth 4: 4.963 -> 4.963 | 86 / 86 (of 86) |
| rows field_first | 88 / 88 | 88 | 88 / 88 (100 %) | 88 / 88 (100 %) | 0.0002 / 0.0000 | truth 25: 1.806 -> 1.806 | 87 / 87 (of 88; entry 79 unmatched to truth in both) |

Thresholds of the brief: at least 90 % of matched entries within 0.15 m on both `dh` and `dc`; ramping entries (here: every
entry whose closed-loop lock `dh` is above 1.5 m, the A1 offenders, since these runs have no appendix traces) reproduce their
lock `dh` within 0.3 m; attached dose episodes equal. All met, with a margin of two orders of magnitude on the first two
tolerances. The maximum lock `dh` difference over all paired entries (not only the ramping ones) is 0.0004 m, and the maximum
difference over all dose values 0.025 m (`dc`, rows corridor).

Time-aligned over the whole run (every replay snapshot against the live map's latest snapshot at or before it, each entry
present in both) the streams are identical for 99.9 %, 97.5 %, 96.3 % and 98.1 % of the entry-snapshots (5x5 corridor, 5x5
field_first, rows corridor, rows field_first). The residual is the transient: the replay snapshots every 0.2 s from the first frame, the live
node every 0.2 s from its own clock (3.033 s vs 3.05 s start), so during a height ramp one stream is one frame ahead.
Worst transient differences: `dh` 0.09 / 0.20 / 0.11 / 0.19 m, `dc` 0.14 / 0.24 / 0.17 / 0.31 m, while the values at the
locks and doses differ by at most 0.0004 m (`dh`) and 0.025 m (`dc`). No entry has a height in one stream and none in the other.

One real difference, `5x5 field_first`: 24 of 26 pairs have equal ids; two entries (truth 21: live 142 / replay 144, truth 22:
live 146 / replay 148) got ids two apart. Most likely a tentative instance created and expired in one stream and not in the
other before t = 116 s (first publication 116.73 s vs 116.67 s; not traced further); the entries' values are identical, so the
gate is unaffected, but ids are not stable across live and replay to the last digit.

The TMF1 differences expected beforehand (disparity stored as u16/256, heartbeats on a 0.2 s grid, nlerp pose interpolation)
show at most as the sub-millimetre and transient differences above (not separated further). Replay and live agree at every lock
and dose, so fixes can be developed on the replay.

## Baseline (the "today" values of A1-A3)

`replay_metrics --md` summaries of the replay map (`/data/work/replay/baseline/<ds>/metrics.md`); the closed-loop `live_metrics.md`
has the same values to the printed digits, except three figures that differ in the last digit: rows corridor, A3 far / near mean
`dc` (live 0.349 / 0.366 m vs replay 0.347 / 0.365 m); rows field_first, A2 worst `dh` (live -0.638 m vs replay -0.637 m). Rows
corridor's worst `dh` is -0.596 m in both.

| Metric | 5x5 corridor | 5x5 field_first | rows corridor | rows field_first |
|---|---|---|---|---|
| A1 max `dh` at lock (m) | 3.633 | 6.539 | 4.963 | 1.806 |
| A1 max `dh` at dose (m) | 3.633 | 0.879 | 4.963 | 1.806 |
| A1 lock + dose values over 1.5 m | 6 (3 + 3) | 1 (1 + 0) | 2 (1 + 1) | 2 (1 + 1) |
| A2 share of entries with a value below -0.3 m | 0.000 (0 / 27) | 0.038 (1 / 26) | 0.349 (30 / 86) | 0.460 (40 / 87) |
| A2 worst `dh` (m) | -0.189 | -0.344 | -0.596 | -0.637 |
| A3 far (first published >= 5 m across): entries with a dose | 13 | 10 | 45 | 51 |
| A3 far: mean `dc` at dose (m) | 1.008 | 1.061 | 0.347 | 0.445 |
| A3 far: share `dc` > 0.8 m | 0.615 | 0.700 | 0.067 | 0.137 |
| A3 near: entries with a dose | 14 | 15 | 41 | 36 |
| A3 near: mean `dc` at dose (m) | 0.220 | 0.337 | 0.365 | 0.237 |
| A3 near: share `dc` > 0.8 m | 0.000 | 0.067 | 0.098 | 0.028 |

A1 offenders (entries whose lock or dose `dh` exceeds 1.5 m; lock time, `dh` at the lock, `dh` at the dose; first
publication offset across track):

| Dataset | Entry (truth) | Offset (m) | Lock t (s) | Lock `dh` | Dose `dh` |
|---|---|---|---|---|---|
| 5x5 corridor | 0 (1) | 1.5 | 33.1 | 1.802 | 1.802 (t 72.2) |
| 5x5 corridor | 59 (12) | 8.4 | 195.6 | 3.633 | 3.633 (t 214.3) |
| 5x5 corridor | 111 (18) | 4.4 | 300.1 | 1.913 | 1.913 (t 320.0) |
| 5x5 field_first | 55 (12) | 8.4 | 146.4 | 6.539 | no dose (truth 12 was dosed under its second entry 119: `dh` 0.436) |
| rows corridor | 23 (4) | 2.8 | 100.9 | 4.963 | 4.963 (t 140.8) |
| rows field_first | 95 (25) | 3.7 | 234.7 | 1.806 | 1.806 (t 644.2) |

What the baseline shows against the spec's "today" column:

* A1: 11 lock/dose values over 1.5 m on these four runs (5 of them at a dose), max 6.54 m. The 5x5 corridor dose list is the
  spec's height ramp (3 entries); the same tree (truth 12, first seen 8.4 m across track) is the worst in both 5x5 strategies.
  The rows runs have 1-2 offenders each, so A1 is carried by the 5x5 datasets.
* A2: the under-estimate side is already a problem on the rows datasets (35 % and 46 % of entries more than 0.3 m low, worst
  -0.60 / -0.64 m), almost absent on 5x5 (0 % and 3.8 %). A fix that lowers heights must stay at or below these numbers.
* A3: the far-sighted centre bias is reproduced on the 5x5 datasets, 1.008 m / 61.5 % (corridor; the spec's closed-loop figure
  is 1.04 m / 59 %) and 1.061 m / 70 % (field_first); near entries are 0.22 / 0.34 m. On rows_8x12 it is mild (0.35 / 0.45 m,
  7 % / 14 %), and A3 only applies to 5x5 anyway.

## Reproducing and pinning

The replay metrics that later tasks diff against are committed byte for byte as
`results/miss_reduction/baseline/<dataset>/metrics.json` (output of `replay_metrics --replay --targets ... --json`, per-entry
lock and dose values plus the `a1` / `a2` / `a3` summaries). The frame files and the scratch scripts live only under `/data/work`;
these hashes pin them (`sha256sum <file>`):

| File | sha256 |
|---|---|
| `/data/work/sim_runs/mr_frames_5x5_tropical_corridor/run_1/frames.bin` | `b267923d2cc1c35bd14a139976cf85e8fd55b7128511c9e576ec85f222092b7b` |
| `/data/work/sim_runs/mr_frames_5x5_tropical_field_first/run_1/frames.bin` | `24e438d92cb0bc8a63e3fdb624700912ed78e520fe9a3d8a8e9ccd754f153163` |
| `/data/work/sim_runs/mr_frames_rows_8x12_corridor/run_1/frames.bin` | `2a70367e5ef514a9af126fcc8b021c21ed2303f15ed1532d20bd76b3f5b66d32` |
| `/data/work/sim_runs/mr_frames_rows_8x12_field_first/run_1/frames.bin` | `9873721ff4fdddd2343ac8ffe77a621919982106bf0a40436b15e1df6e4ad4eb` |
| `results/miss_reduction/baseline/mr_frames_5x5_tropical_corridor/metrics.json` | `5d95548ad2cdef0c3675091c14f1c01642ed4694e40c86cbc696592db6e09103` |
| `results/miss_reduction/baseline/mr_frames_5x5_tropical_field_first/metrics.json` | `1ca4bc65d0f455cc4a44a3f912ac9384efe270daf3a04129e8163ba2085ebd6c` |
| `results/miss_reduction/baseline/mr_frames_rows_8x12_corridor/metrics.json` | `2776bb665c8348f5de6a4cad7b3ba21ae1075ab5bf417844fff54232b87c91e2` |
| `results/miss_reduction/baseline/mr_frames_rows_8x12_field_first/metrics.json` | `58bb6e92a12bc7f7f614411cf064684f38ebe6dced6dff4b2ba3f63518b821e6` |
| `/data/work/replay/replay_one.sh` | `18779292e11d60cc0dbf42015162486e2e4f12692163ce53c1e7b44a523bea9b` |
| `/data/work/replay/score_one.sh` | `6c4735c614440b765426ee9a7843844fb293fcca4c9ea358d604d616fd1e6c44` |
| `/data/work/replay/process_one.sh` | `65fab91612db57f554b024846532dfde8cc2558385e4e93610718fbec28d2d17` |
| `/data/work/replay/equiv.py` | `ebb401561f3e0ac172c36599b96b846a8d2f89f819fdf2c3387b57a8237163fc` |
| `/data/work/replay/trace_diff.py` | `7df715333b125f51c0738b4378a1e03d224424be13889e3c4c455786693f7c1c` |

Re-running (inside the work container, cwd = repository root, `PYTHONPATH=tools`): `/data/work/replay/process_one.sh <ds>` runs
`replay_one.sh` (the replay command above), `score_one.sh` (both scorings above), `equiv.py` and `trace_diff.py`. The replay
metrics regenerated from the pinned `frames.bin` and the HEAD `7284ba6` binary must hash to the committed `metrics.json`.

### Appendix: the equivalence gate, enough to re-implement it

Inputs: the closed loop's `metrics.json` (`replay_metrics` on the run's `map.jsonl`, no `--replay`) and the replay's `metrics.json`
(`--replay --targets <its own out-targets>`), both against the run's `truth.json` and `mission.json`; plus the two map streams.

1. **Pairing.** Each `metrics.json` lists entries (map ids matched to a truth tree, sticky at first match; see the
   `replay_metrics` docstring). Group the entries of each side by `truth_id`, sort each group by `first_pub_t`, and pair the
   k-th live entry with the k-th replay entry of the same truth tree. An entry left over on either side is a gate failure. (Entry ids
   are the mapper's own; they are reported as equal or not, but pairing does not use them.)
2. **Values compared.**
   * Final `dh` / `dc`: for each entry, its last snapshot in its own map stream (live `map.jsonl` / replay `map.jsonl`):
     `dh` = published height - true top (`mission_score._tree_top` of the paired truth tree), `dc` = distance from the published
     centre to the truth centre (`mission_score._tree_centre`). A pair is within tolerance when `|dh_replay - dh_live| <= 0.15 m`
     (resp. `dc`); an undefined height counts as a failure.
   * Lock `dh` of the ramping entries: the pairs whose live lock `dh` (`entries[].lock.dh`, the lock rule of the
     `replay_metrics` docstring) exceeds 1.5 m; the replay's lock `dh` must be within 0.3 m.
   * Attached dose episodes: `n_dose_episodes_attached` of the two `metrics.json` must be equal.
3. **Pass rule.** At least 90 % of the pairs within 0.15 m on `dh` and at least 90 % on `dc` (computed separately), every ramping
   pair within 0.3 m, equal attached-dose counts, no unpaired entry. All four datasets pass: 100 % / 100 %, ramping differences 0.0 m,
   attached doses equal.
4. **Diagnostic, not a gate (`trace_diff.py`).** At every replay snapshot stamp `t`, compare each entry id present in both streams with
   the live stream's latest snapshot with stamp <= `t`: share of identical (height and centre) entry-snapshots, max `|dh|`, max `dc`.
   Values in the table of the previous section.

## Height

Task 5 (spec section 4.2). Diagnosed on the replay dataset above; no fix shipped (controller ruling R17: the centre fix
goes first, the height fix is parked). `cand_obj`, `config/` and `DECISIONS.md` are unchanged.

### Height / diagnosis

Method: temporary instrumentation in `cand_obj.cpp` (env `TM_DIAG_IDS` / `TM_DIAG_OUT`, in the patch below). It logs every
height sample of the listed instances (cell, sample `zc`, weight `wf`, the cell's `hz` / `hw` before the sample, mean pixel,
mean optical depth Z, point count and z range), every latch and every `height_target` change. The `frames.bin` points
were also read directly. With the variables unset the build reproduces the four baseline `metrics.json` byte for byte.

Mechanism, in the order it happens:

1. **Stereo mismatches put samples metres above the crowns.**
   - The raw points have Z 3-10 m where the canopy is at 12-15 m below the drone, so they land at z 5-13 m over trees 2.3-3.3 m tall.
   - About 0.7 % of all points are impossibly high, 2-4x more in the top and bottom 50 image rows (the leading and trailing edges of the view).
   - Around tree 12 at 186.4-188.0 s, alternate frames have 16-21 % of their points impossibly high.
   - A mismatch sits on the true ray at a shorter range. It moves about 0.17 m in the world per 0.4 m of flight, so the same 0.4 m cell gets consistent wrong heights for 3-4 consecutive frames: a ghost surface.
2. **The sample weight rewards its own error.**
   - The point weight `(z_ref/Z)^4 / (1 + 10 rho^2)` is computed from the measured, mismatched Z, and a cell's weight is the mean over its points. One near point therefore dominates.
   - On the crown cells of 5x5 corridor (41,724 samples), the samples more than 1 m above the true top are **4 % of the samples and carry 59 % of the height weight**.
   - Samples with `wf` >= 3 are 36-68 % garbage; samples with `wf` < 1 are 1 %. Single mismatched samples reach `wf` 37, 742 and 2822.
3. **Three rejections re-initialise a cell, uncapped.**
   - Three consecutive rejected samples replace the established value with the third sample, at its uncapped weight.
   - This applies whatever the established weight is (up to the cap of 30) and whether or not the three samples agree with each other.
   - In 5x5 corridor, 27 of 32 upward resets land on garbage.
4. **Young cells are ungated.** While `hw` is below `gate_min_weight` (3), any sample is fused. A cell seen only from far away (about 0.25 per frame) stays ungated for 10+ frames, and one heavy mismatch then takes it over.
5. **The 0.99 quantile is the top cell.** The index is `floor(0.99 (n-1) + 0.5)`: the maximum cell below 51 latched cells and the second highest up to about 150. The offenders have 21-112 latched cells, so one corrupted latched cell sets `height_target`. `stepPublished` then walks the published height up at 0.5 m/s after the pass, and the lock freezes it. The rate limiter is not the bug.

All six baseline A1 offenders follow this mechanism:

| Dataset, entry (truth) | Lock dh | What set the top cell |
|---|---|---|
| 5x5 corridor, 59 (12) | +3.63 | Cell (20,33) at 0.79 m, `hw` 10.3, rejects 3.13 / 6.44 / 6.86 (trailing edge, v 139 -> 14). The third rejection re-initialises it to 6.86, and a further 5.63 is fused ungated (`hw` 2.9 < 3), giving 6.41. `height_target` goes 2.76 -> 6.86 at t 187.77; the lock at 195.58 freezes 6.41. |
| 5x5 corridor, 0 (1) | +1.80 | Cells (2,-3) and (1,-3) at 1.87 m with `hw` 30 (the cap); 4.13 / 3.92 / 4.42 (`wf` about 2.5 each) re-initialise them to about 4.4 m. |
| 5x5 corridor, 111 (18) | +1.91 | Cell (30,51) with `hw` 30; three mutually inconsistent rejections (-1.86 / 4.42 / 4.70) re-initialise it to 4.70. |
| 5x5 field_first, 55 (12) | +6.54 | Cell (24,32): an ungated 8.60 (`wf` 4.4) gives 5.43, then a reset to 10.69 with uncapped `wf` 37.2. Cell (13,34): an ungated 10.40 (`wf` 23.7) gives 9.31. |
| rows corridor, 23 (4) | +4.96 | Cell (33,6): an ungated 8.47 (`wf` 43.5) gives 8.12. Cells (30,5) and (32,6) are re-initialised to 6.25 and 5.98 (`wf` up to 43). |
| rows field_first, 95 (25) | +1.81 | Cell (31,24) with `hw` 23.7; 7.49 / 6.05 / 5.84 re-initialise it to 5.84. |

Entry 55 (controller ruling R16) is the duplicate entry of truth 12; truth 12 was dosed under entry 119 at +0.44 m. Its
+6.54 m is this height mechanism, not the duplicate: under the best variant below its lock dh is +0.02 m, while 119 stays
at +0.44 m. The duplicate itself (a rule-4 merge failure) is a separate open cause and was not touched.

### Height / status: parked

Fifteen variants were run on all four datasets (replay, `--replay --targets`). A3 is unchanged in every variant, to three
decimals. The variants that matter:

| Variant | What it changes |
|---|---|
| wq | sample weight capped at the frame quality q |
| medw | cell weight = median point weight instead of the mean |
| dbl | one frame at most doubles a cell's weight; the first counts at most q |
| hgate | a sample heavier than the cell's whole weight is gated even before `gate_min_weight` |
| ch | rejected samples form a challenger that replaces the cell when it outweighs it |
| ch4 | as ch, plus the challenger needs 3 consecutive, mutually consistent rejections |
| mode | two hypotheses per cell; the one with more frame support (`sum of min(wf, q)`) wins |
| hgsupcap | hgate + ch4 decided on frame support instead of weight + an accepted sample counts at most the cell's weight |

Each cell gives A1 max dh at the lock / at a dose (m), then A2 share of entries below -0.3 m / worst dh (m). Values
outside A1 or A2 are in bold. ch and ch4 have no A2 figures because their A1 is already far out.

| Variant | 5x5 corridor | 5x5 field_first | rows corridor | rows field_first |
|---|---|---|---|---|
| baseline | 3.63 / 3.63; 0.000 / -0.189 | 6.54 / 0.88; 0.038 / -0.344 | 4.96 / 4.96; 0.349 / -0.596 | 1.81 / 1.81; 0.460 / -0.637 |
| wq | **2.10**; **0.037** / **-0.358** | 0.76; **0.077** / **-0.417** | 1.23; **0.698** / **-0.900** | 1.13; **0.805** / **-0.874** |
| medw | **3.63**; 0.000 / **-0.194** | **6.60**; **0.077** / **-0.418** | **4.57**; **0.372** / -0.592 | 1.28; **0.506** / -0.636 |
| dbl | **2.10**; 0.000 / **-0.214** | **2.60**; 0.038 / -0.344 | **1.93**; **0.360** / -0.596 | 1.41; **0.494** / **-0.647** |
| hgate | **3.63**; 0.000 / -0.181 | 0.88; 0.038 / -0.344 | **2.54**; **0.360** / -0.596 | 1.44; 0.460 / -0.637 |
| ch | **10.35** | **10.15** | **9.22** | **9.86** |
| ch4 | **9.75** | **9.38** | **7.78** | **3.58** |
| mode | **1.74**; 0.000 / **-0.210** | **2.07**; 0.038 / -0.315 | 1.31; **0.372** / -0.596 | **2.95**; **0.494** / -0.564 |
| hgsupcap | **1.88**; 0.000 / -0.189 | 1.11; 0.038 / -0.315 | 1.00; 0.349 / -0.596 | **1.57**; **0.494** / -0.564 |

What the table shows:
- **The range weight cannot simply be capped.** On rows the near-nadir samples, the ones that see the crown top, legitimately weigh 2-6 against 0.3-1 for oblique samples of the crown sides. wq doubles the rows under-estimates: an anti-goal.
- **A reset test on weight alone or on sample count alone is worse than today** (ch, ch4). Single mismatches weigh up to thousands, and ghost bursts are consistent for 3-4 frames. The old rule was also what pulled back cells that one heavy mismatch had corrupted.

**Why hgsupcap is not shipped.**
- It removes every runaway, and its replay is deterministic:

  | Entry | Baseline lock dh (m) | hgsupcap lock dh (m) |
  |---|---|---|
  | 59 | 3.63 | 0.05 |
  | 0 | 1.80 | -0.02 |
  | 111 | 1.91 | 0.53 |
  | 55 | 6.54 | 0.02 |
  | 23 | 4.96 | 0.62 |
  | 95 | 1.81 | -0.03 |

- **A1 fails on 5x5 corridor, 1.88 m.** Entries 10 and 13 go from +1.47 and +1.33 to +1.63 and +1.88. Their top cells are set by four consistent, moderately weighted readings of 3.3-4.8 m from the first views at the leading image edge. The old reset used to pull these cells back down.
- **A1 fails on rows field_first, 1.57 m.** Entry 151 goes from +1.44 to +1.57. Its top cell lies about 1 m outside the crown, with 7 consistent readings of 4.1-5.7 m over 1 s.
- **A2 fails on rows field_first:** share 0.494 against 0.460.
- **Correct heights are lowered by 0.3-0.5 m**, the unsafe direction: entry 8 goes from +0.06 to -0.50 (rows field_first), entries 149 and 191 to -0.38 and -0.44 (rows corridor).

**Open design step.** After the runaway, what is left is a tail of +0.5-1.9 m from ghost surfaces that stay consistent
over 3-4 consecutive frames. No per-cell temporal rule separates them from a real top without lowering correct
heights. A later attempt needs a surface check across widely separated views: a ghost moves in the world as the camera
moves, a real surface does not. Alternatively, it needs a ruling on how A1 treats that tail.

**Experimental source.** `results/miss_reduction/height_experiments.patch` contains all variants and the diagnostic dump,
behind environment flags. Apply it with `git apply results/miss_reduction/height_experiments.patch` on `294254f`. hgsupcap is
`TM_EXP_HGATE=1 TM_EXP_CH4=1 TM_EXP_SUP=1 TM_EXP_HCAP=1`; with no flags set the build equals the baseline byte for byte.
Re-applied and re-run, the four hgsupcap `metrics.json` are byte-identical to the ones the table was made from.
