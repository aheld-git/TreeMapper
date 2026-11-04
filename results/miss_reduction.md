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

## Centre

Task 6 (spec section 4.3). Diagnosed on the 5x5 replay datasets, fixed in `cand_obj` (D66), measured on all four.

### Centre / diagnosis

Method: temporary instrumentation in a scratch copy of `cand_obj` (not committed: no new drop reason). For every
instance it logged each latch (cell, drone pose, the cell's mean pixel and height above ground in that frame), each
crown credited (cells, peak, drone pose), and at the dose the instance's whole evidence grid; for a few far entries
also every per-cell, per-frame judgement (canopy of this tree / ground / another tree's canopy). With the logging the
four `metrics.json` are byte-identical to the baseline. Scripts: `/data/work/replay/t6/scripts/` (`latched.py`,
`evid.py`, `farlatch.py`, `grid.py`, `passmap.py`, `agg.py`).

Camera geometry that matters here: 640 x 400 px, fy 454, rows of the image across track, so the half field of view
across track is 24.8 deg and a crown 12.5 m below the camera is in view to about 5.8 m either side of the track. A tree
first published 8 m across track is first seen at the image edge, at about 15 m range along an oblique ray. Baseline
0.16 m: one pixel of disparity is about 3 m of range there.

**Which latched cells pull the centre, and from which views.** The published centre is the mean of the latched cells,
and latching is permanent. For the far entries (first published >= 5 m across track) of the 5x5 datasets, almost every
latched cell latched while the drone was on the first, far track; those cells sit outside the true crown, displaced
toward that track:

| Latched cells of far entries, by the drone's across-track distance when they latched | 5x5 corridor | 5x5 field_first | rows corridor | rows field_first |
|---|---|---|---|---|
| drone 0-3 / 3-5 m across: share of the latched cells | 0.01 / 0.01 | 0.01 / 0.00 | 0.12 / 0.05 | 0.12 / 0.06 |
| ... their mean offset from the truth centre toward the drone (m) | +0.33 / +0.02 | +0.01 / -0.48 | -1.25 / -1.06 | -0.96 / -0.92 |
| drone 5-7 m across: share / outside the truth crown / offset toward the drone | 0.48 / 0.23 / +0.22 | 0.69 / 0.55 / +1.17 | 0.57 / 0.30 / +0.15 | 0.49 / 0.28 / +0.05 |
| drone >= 7 m across: share / outside the truth crown / offset toward the drone | 0.50 / 0.58 / +1.47 | 0.29 / 0.59 / +1.12 | 0.26 / 0.32 / +1.04 | 0.32 / 0.37 / +0.87 |
| latched centroid error at the dose, far / near entries (m) | 1.01 / 0.22 | 1.06 / 0.34 | 0.35 / 0.37 | 0.45 / 0.24 |

Two entries in detail (5x5 corridor):

- **Entry 59 (truth 12, first published 8.4 m across).** 21 latched cells, all latched from the track at y = 6.87; 14 lie
  outside the crown, south of it. At the latch their mean pixel row was v = 79 (the image edge) and their height above
  ground 3.9 m, above the true top of 2.77 m: points moved back along the oblique ray toward the camera (a range
  under-estimate) land both higher and closer to the drone. The near views later judge these cells ground (final
  evidence 19.6 positive against 30.8 negative frames on average), but they stay latched. Of the 44 cells of the true
  crown, 37 never latch: 26.7 positive against 18.5 negative frames is 0.59 < `latch_ratio` 0.75. The negative frames
  come from the far view (it sees the true crown's north half as ground, the crown being drawn 1.5 m south) and from the
  near pass itself: in that pass (3.7 m across) truth tree 17 is 7.9 m across, and its own displaced crown, credited to
  its instance (85), lands on tree 12's north half ("another tree's canopy"). Centre at the dose 1.53 m off.
- **Entry 8 (truth 7, first published 9.0 m across).** 54 latched cells, 52 latched from the first track; 43 outside the
  crown (mean pixel row 118, height above ground 3.0 m against a 2.65 m top), later judged ground in 42.8 frames against
  16.5. 31 of the 42 truth-crown cells never latch (21.3 positive / 25.1 negative). Centre 2.0 m off.

So the designer's guess holds, with one addition: the far view does not only latch a ghost band toward its track, its
(equal-weight) ground judgements on the true far half keep that half from ever latching once near views see it.
Both come from treating a far, oblique frame at the image edge as one full frame of evidence. The cause in the stereo
(range error at the image edge on foliage) is outside this work; the same edge views are what Task 5 found behind the
height bursts.

**Why rows_8x12 shows it less.**
- Its far entries are less far: 27 of 45 (corridor) and 27 of 51 (field_first) far entries with a dose were first
  published 5-7 m across, against 5 of 13 and 5 of 10 on 5x5, and on rows the 5-7 m band is only mildly displaced
  (+0.15 / +0.05 m).
- Near views still latch cells on rows far entries: 17-18 % of their latched cells latch from 0-5 m, and those lie on
  the far side of the crown (-0.9 to -1.25 m), pulling the centroid back. On 5x5 near views latch 1-2 %. The rows
  crowns are taller (true top 4.0 m against 2.7 m) and larger (radius 1.8 against 1.6 m), so more of a crown is in view
  and better resolved from the near track. This last point is consistent with the data, not separately proven.
- Rows outside cells are spread round the crown (28 % outside for far and near entries alike), not on one side.

### Centre / fix (D66)

The published centre is now the centroid of the cells whose **observation-weighted** evidence currently says "this
tree's canopy", instead of the centroid of the latched cells.

- Each instance keeps a second pair of evidence counts per cell, `wpos` / `wneg`, from the same per-frame judgements as
  `npos` / `nneg`. Each frame counts min(the cell's mean point weight, 1) x frame quality. The point weight is the
  existing `(z_ref / Z)^4 / (1 + lateral_k rho^2)`, so a far, oblique view at the image edge counts about 0.15 and a
  near-nadir view about 1.
- A cell supports the centre while `wpos >= latch_frames` and `wpos >= centre_support_ratio (0.5) x (wpos + wneg)`.
  Unlatched cells count (the far half of the crown), latched cells that the better views call ground drop out.
- Fewer than `min_height_cells` supported cells: the latched centroid, as before (fails open, counted
  `centre_support_fallback_latched`).
- The supported centroid is held within `centre_max_outside` (1.0 m) of the latched outline (counted
  `centre_clamped_to_outline`): the navigator accepts a clearance-guard release over its target only within 1.3 m of
  the target's outline (`raiseSprayFloor`). Added in review fix round 1 (ruling R20).
- The published centre still moves toward its target at `max_centre_rate`; the rate limiter is unchanged.
- **Unchanged:** latching, the outline (convex hull of the latched cells, grow-only, rule 2), heights, the internal centre
  used for matching, merges, the spill guard and the sprayed checks. Three new keys, `centre_from_support` (true),
  `centre_support_ratio` (0.5) and `centre_max_outside` (1.0), in both profiles. `centre_from_support = false`
  reproduces the baseline map streams, targets, counters and `metrics.json` byte for byte on all four replays.

Checked on the four replays: every map snapshot is identical to the baseline except the centre (outline, height, id and
`sprayed` of all 40 532 / 43 554 / 287 942 / 363 793 entry-snapshots), the targets files are identical, the counters
differ only in `centre_steps_clamped` and the new `centre_support_fallback_latched` / `centre_clamped_to_outline`
(0 / 0 / 8 / 56). The clamp changes no lock or dose value: the four `metrics.json` are byte-identical with and without it.

### Centre / result

Replay, `replay_metrics --replay --targets <own>` against the committed baseline. The table is the centre fix alone
(`1cab792`). The committed `results/miss_reduction/centre_fix/<ds>/metrics.json` are HEAD (D66 + D67): identical to this
table on both field_first datasets, and on the two corridor datasets as in "With D67" below.

| Metric | 5x5 corridor | 5x5 field_first | rows corridor | rows field_first |
|---|---|---|---|---|
| A3 far: mean `dc` at dose (m), before -> after | 1.008 -> **0.099** | 1.061 -> **0.114** | 0.347 -> 0.116 | 0.445 -> 0.128 |
| A3 far: share `dc` > 0.8 m | 0.615 -> **0.000** | 0.700 -> **0.000** | 0.067 -> 0.000 | 0.137 -> 0.000 |
| A3 near: mean `dc` at dose (m) | 0.220 -> 0.106 | 0.337 -> 0.089 | 0.365 -> 0.140 | 0.237 -> 0.102 |
| A3 near: share `dc` > 0.8 m | 0.000 -> 0.000 | 0.067 -> 0.000 | 0.098 -> 0.000 | 0.028 -> 0.000 |
| A1 max `dh` lock / dose (m), n over 1.5 m | 3.63 / 3.63, 6 (unchanged; **8 with D67**, see below) | 6.54 / 0.88, 1 (unchanged) | 4.96 / 4.96, 2 (unchanged) | 1.81 / 1.81, 2 (unchanged) |
| A2 share below -0.3 m / worst `dh` | 0.000 / -0.189 (unchanged) | 0.038 / -0.344 (unchanged) | 0.349 / -0.596 (unchanged) | 0.460 / -0.637 (unchanged) |

A3 holds on both 5x5 datasets (target: far <= 0.5 m and <= 0.15 beyond 0.8 m; near not worse). A1 and A2 are
identical to the baseline entry by entry (every lock and dose `dh`): the fix does not touch heights. Every 5x5 far
entry is within 0.22 m at its dose; before, 8 of 13 (corridor) and 7 of 10 (field_first) were beyond the 0.8 m footprint.

**With D67 (the post-spray hold no longer ends on SCAN_APPROACH; HEAD, the committed `metrics.json`).** Field_first
replays are byte-identical. On the corridor replays the hold now lasts through the SCAN_APPROACH climb out of each dose
(held frames 255 -> 767 on 5x5 corridor, 1140 -> 1810 on rows corridor), which shifts when other instances get evidence:

| Corridor replays, D66 alone -> D66 + D67 | 5x5 corridor | rows corridor |
|---|---|---|
| A3 far: mean `dc` at dose (m), share > 0.8 m | 0.099, 0.000 -> 0.133, 0.000 | 0.116, 0.000 -> 0.108, 0.000 |
| A3 near: mean `dc` at dose (m), share > 0.8 m | 0.106, 0.000 -> 0.095, 0.000 | 0.140, 0.000 -> 0.137, 0.000 |
| far / near at the start of the dosing leg (mean, share > 0.8 m) | 0.452, 0.154 / 0.151, 0 -> 0.472, 0.154 / 0.143, 0 | 0.146, 0 / 0.167, 0.024 -> 0.138, 0 / 0.161, 0.024 |
| A1 max `dh` lock / dose (m), n over 1.5 m | 3.63 / 3.63, 6 -> 3.63 / 3.63, **8** (lock 3 -> 4, dose 3 -> 4) | 4.96 / 4.96, 2 (unchanged) |
| A2 share below -0.3 m / worst `dh` | unchanged (0.000 / -0.189) | unchanged (0.349 / -0.596) |
| centre outside its own outline: at the doses / any snapshot max | 5 of 27, 0.82 -> 4 of 27, 0.79 / 0.817 -> 0.794 m | 1 of 86, 0.99 / 1.000 m (unchanged) |

**Accepted A1 count change (ruling R25).** The new A1 value on 5x5 corridor is entry 46 (truth 7, the rule 4 duplicate
of entry 8): lock `dh` +1.454 -> +2.454 m, dose `dh` +1.454 -> +2.178 m. The first map difference is at t 81.65 s, entry 8
frozen in the climb after its 72.2 s spray; entry 46 is then confirmed 0.6 s later (126.45 s instead of 125.85 s) and its
first published height is 9.22 m (3.70 m before): one garbage latched cell, the parked height-runaway mechanism ("Height"
above). It falls at 0.5 m/s and the lock at 135.03 s takes 4.86 m (4.10 m before). The hold change alone, without D66, gives
the same values. Too high is the flight-safe direction, the maximum is unchanged and A2 is identical; the height fix removes
it. On rows corridor entry 111 moves lock / dose `dh` -0.055 -> -0.258 m (not below -0.3 m, so A2 is unchanged).

**Where the correction happens.** The navigator plans the glide to the target's centre when it starts the leg and
replans only when a height changes (`needsReplan`) or the clearance guard releases; it does not follow a centre that
moves during the leg. The centre at the start of the HOPPING leg that ends in each dose is therefore the better guess of
where the closed-loop drone will dose:

| `dc` at the start of the dosing leg, far / near (mean, share > 0.8 m) | 5x5 corridor | 5x5 field_first | rows corridor | rows field_first |
|---|---|---|---|---|
| baseline | 1.018, 0.615 / 0.221, 0.000 | 1.066, 0.700 / 0.337, 0.067 | 0.367, 0.067 / 0.377, 0.098 | 0.446, 0.137 / 0.237, 0.028 |
| fix | 0.452, 0.154 / 0.151, 0.000 | 0.172, 0.000 / 0.121, 0.000 | 0.146, 0.000 / 0.167, 0.024 | 0.154, 0.000 / 0.117, 0.000 |

On 5x5 corridor two far entries are still beyond 0.8 m at the start of their leg: entry 59 (1.53 m) and entry 8
(1.27 m). Both are corridor pairs where the tree's own pass is also the far pass of the next row's tree, whose ghost
covers the crown's far half (see entry 59 above); their centres converge only from the views on the way to the dose
(entry 59: 1.53 m at 211.8 s, 0.12 m at the dose, 214.3 s). In the closed loop the drone would dose where it planned
unless a height replan picks up the new centre. That is a navigator question (replan on a centre move), outside this task.

**Centre and outline are now separate.** The outline still holds the far view's latched ghost band (grow-only), so a
corrected centre can lie outside it.

| Centre outside its own outline (m) | 5x5 corridor | 5x5 field_first | rows corridor | rows field_first |
|---|---|---|---|---|
| any snapshot, before the clamp (round 0) | 0.817 (entry 59, t 214.27) | 0 | 1.006 (entry 23, t 140.53) | 1.067 (entry 20, t 25.15) |
| any snapshot, with `centre_max_outside` 1.0 | 0.817 | 0 | 1.000 | 1.000 |
| at the doses (attached dose episodes) | 5 of 27, max 0.82 | 0 of 25 | 1 of 86, 0.99 | 0 of 87 |

Nothing bounded the distance before the clamp (in principle up to `max_cell_radius`); beyond 1.3 m the navigator
ignores a guard release over the target (`spray_floor_release_ignored`) and the attempt fails after its climbs.

A centre can also sit inside **another** entry's outline. The baseline has no such case on 5x5 corridor, 5x5
field_first and rows corridor, and one on rows field_first (253 inside 252, t 209.15-219.15, both truth 85, neither
sprayed), which stays. The fix adds three, each two entries of one physical tree (pre-existing rule 4 duplicates):
- 5x5 corridor, entry 8 inside 46 (truth 7) from t 125.85 to the end: 8 was sprayed at 107.3, 46 is born at 125.85 and dosed at 162.5;
- 5x5 corridor, entry 121 inside 139 (truth 22) from t 401.85: 121 sprayed at 379.1, 139 born at 401.9;
- rows field_first, entry 20 inside 137 (truth 14) from t 796.75: 137 sprayed at 763.0, 20 is on its dosing leg and dosed at 797.7.

What it means for the navigator: `reresolveId` prefers the entry whose outline contains the lost target's centre but
skips sprayed entries, so a vanished target is never re-resolved onto its sprayed duplicate; `sprayPoint` raises the
spray point over any other entry whose outline is within 1.3 m of the target centre, so a target lying on a duplicate's
outline is dosed from at least that duplicate's height plus clearance. That is the safe direction; it may weaken the dose.

**Alternatives measured** (scratch build, env flags; outputs `/data/work/replay/t6/<ds>/<variant>/`):

| Variant | 5x5 corridor far | 5x5 field_first far | Why not |
|---|---|---|---|
| support over latched cells only (centre stays inside the outline) | 0.470 / 0.077 | 0.250 / 0.000 | passes with no margin on corridor; the far half of the crown never latches, so it cannot come in |
| as the fix, but also as the internal centre (matching, merges) | 0.093 / 0.000 | 0.118 / 0.000 | changes matching: A1 count 6 -> 8 on 5x5 corridor, near entries change |
| weighted evidence as a second latch path | 0.829 / 0.462 | 1.028 / 0.636 | new latched cells feed the height: 5x5 field_first A1 dose 0.88 -> 5.72 m |
| the fix with unweighted (frame-quality) evidence | 0.220 / 0.000 | 0.167 / 0.000 | works, but the weighting halves the far error |

Sensitivity of the fix (far mean / share): `centre_support_ratio` 0.4 / 0.5 / 0.6 / 0.75 gives 0.191 / 0.099 / 0.116 /
0.382 m on 5x5 corridor (share 0.077 / 0 / 0 / 0.154), rows unaffected within 0.05 m; a minimum of 3 or 10 weighted
frames instead of `latch_frames` (5) changes the far means by less than 0.01 m (3 changes which entries the
navigation emulator resolves on 5x5 corridor).

**Determinism.** 5x5 corridor and rows field_first replayed twice: `targets.json`, `counters.json`, `metrics.json`
byte-identical, `map.jsonl` identical once `stage_ms` is removed.

**Real flight (no truth; the section 6 check is Task 7).** The real farm log replayed with both profiles, old and new
binary. Real profile: 63 final trees, outlines and heights identical, every centre inside its own outline; final
centres move by a median 0.48 m (p90 1.18, max 3.17); distance from the centre to the stereo crown peak inside its
outline (`results/real_flight_crowns.json`, a proxy: a crown's top is not its centre) mean 1.76 -> 1.63 m, p90 3.02 ->
2.66 m, 22 trees closer by more than 0.3 m and 11 farther. Sim profile on the same log: 101 trees, mean 2.40 -> 2.35 m,
p90 4.25 -> 4.43 m, 23 closer and 15 farther, 100 of 101 centres inside their outline. Largest per-snapshot centre step
unchanged (0.26 / 0.30 m, the rate limit). With the outline clamp the real profile's numbers are unchanged (no centre
leaves its outline on the whole log); two centres pass briefly through a touching neighbour's outline (2.4 s and 0.4 s,
none before). The sim profile on the real log: the clamp acts 19 times, largest distance outside 1.000 m; centres inside
another entry's outline in 28 pairs (20 before).

## TreeMapper regression check

Spec section 6 (A6) and the section 3 anti-goal "regressing the real-flight profile". The height fix is parked, so the only
TreeMapper behaviour change is the centre fix (D66). **Before** = `be0e5bd` (the commit before the centre fix; the core is
identical to `294254f` / `7284ba6`), **after** = `1cab792`. Each is a clean `git archive` export built with
`cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo` under `/data/work/mr_t7/{before,after}`, build dirs `/data/work/mr_t7/build_{before,after}`.
`tools/` is identical in both (the scorer is the same). Raw outputs under `/data/work/eval/mr_t7/`; the small result files and the
scripts are kept in `results/miss_reduction/regression/` (named below).

**Re-run with D67 (Task 7 fix round 1).** Every evaluation below was run again with after = D66 + D67 (`a0e2de3`; built from
`git stash create` `1e8543f`, whose `core/` and `config/` equal that commit) and a third build, **hold** = `be0e5bd` plus only
the one-line D67 change (`d67_hold_only.diff`), under `/data/work/mr_t7f/`, outputs `/data/work/eval/mr_t7f/` (job lines
`stage_d67.sh`, analysis `analyse_d67.sh`). The committed files in `regression/` are from that re-run. Every number is
identical to the `1cab792` run except on the recorded Isaac mission: `SAFETY total violations` is 0 (it was 1, see the table)
and one fewer snapshot has a centre move (309 instead of 310 of 1410).

**Noise floor and how "beyond noise" is decided.** Run-to-run noise was measured on suite_v2 only: `TM_REPEAT=2` gives a
run-to-run difference of 0 for every metric in both builds (and identical maps, r0 against r1, on 15 of 15 in the D67 re-run).
The held-out suite, the recorded Isaac mission and the real log were run once per build; they use the same deterministic
binary, so a difference between builds on one dataset is taken as real. What
decides whether a difference matters is the dataset-to-dataset spread of the paired difference (the paired rows of
`evaluate.py`: t = mean diff / (std diff / sqrt n), |t| < 2 is within seed noise, W / L / T counted per dataset). A single
dataset (the recorded Isaac mission) has no spread; there it is compared with the paired spread of the synthetic suites
(std of the per-dataset centre RMSE difference 0.047 - 0.056 m) and every discrete counter change is reported as it is.
`evaluate.py` timing fields (`stage_ms`) are wall clock and differ between any two runs; map comparisons drop them.

### Commands

```
# 1 synthetic suite_v2 + recorded Isaac mission, both builds (cwd = the build's own export; 15 datasets x 2 repeats)
cd /data/work/mr_t7/<before|after> && TM_BUILD=/data/work/mr_t7/build_<before|after> \
  TM_OUT=/data/work/eval/mr_t7/suite_v2_<before|after> TM_REPEAT=2 ./scripts/evaluate.sh cand_obj suite_v2
# 2 held-out suite (30 missions), as results/cand_obj_m4_holdout.md ran it
cd /ws/tools && TM_REPLAY=/data/work/mr_t7/build_<before|after>/core/treemapper_replay python3 -m treemapper_tools.evaluate \
  --system cand_obj --suite /data/work/holdout/suite_holdout --runs /data/work/eval/mr_t7/holdout_<before|after>/runs \
  --out-md .../holdout_<before|after>.md --out-json .../holdout_<before|after>.json
# 3 real flight scorecard, as results/real_flight_scorecard.md ran it (real_score), sim and real profile, both builds
python3 -m treemapper_tools.real_score --dataset /data/work/extracted/real_farm --replay <after binary> \
  --systems "sim_before=cand_obj@<before bin>,sim_after=cand_obj@<after bin>,sim_after_off=cand_obj@<after bin>,real_before=...,real_after=...,real_after_off=...,existing" \
  --set real_<x>.<key>=<value> for every key of that build's config/cand_obj_real.cfg ... --runs /data/work/eval/mr_t7/real/runs
```
The exact job lines are `results/miss_reduction/regression/stage2.sh` (`1cab792` run) and `stage_d67.sh` (D67 re-run, which
adds the hold build, the rollback on the Isaac mission and the four miss-reduction replays). Paired tables: `compare.py` (`compare_suite_v2.md`,
`compare_isaac.md`, `compare_holdout.md`, same folder). The built-in defaults equal `config/cand_obj.cfg` (the sim profile).

### Result per evaluation

| Evaluation | Before | After | Difference against its noise floor | Verdict |
|---|---|---|---|---|
| Synthetic suite_v2, 15 missions, run-to-run noise 0 (`compare_suite_v2.md`) | recall 0.954, precision 0.991, IoU 0.773, swallowed 1.067, height RMSE 0.155, sprayed 4.867, SAFETY 0 | the same for every metric but one | centre RMSE 0.426 -> 0.420 m: -0.005 m, t -0.4, 9 better / 6 worse; easy group 0.140 -> 0.154 (+0.014, t +1.7, 4 worse / 1 better), medium -0.009, hard -0.021 | no regression |
| Held-out suite, 30 missions (`compare_holdout.md`) | centre RMSE 0.443 m, sprayed 4.867, all else as after | centre RMSE 0.425 m, sprayed 4.933 | centre RMSE -0.018 m (t -1.8, 19 better / 11 worse); easy 0.133 -> 0.147 (+0.015, t +2.2, 6 worse / 4 better); medium 0.441 -> 0.395 (-0.046, t -2.3); hard -0.023 (t -1.2); sprayed +0.067 (2 / 0 / 28). Recall, precision, IoU, swallowed, split, heights, jumps, SAFETY: 0 difference on all 30 | no regression overall; easy scenes +1.5 cm is beyond the noise rule (t +2.2), a small regression (see Verdict) |
| Recorded Isaac mission, 9 trees, 1 dataset (`compare_isaac.md`) | recall 1, precision 1, IoU 0.867 / min 0.817, centre RMSE 0.191 m (max 0.311), SAFETY 0 | same, but centre RMSE 0.255 m (max 0.472); SAFETY 1 at `1cab792`, 0 with D67 | centre RMSE +0.064 m: 4 trees closer (-0.04 to -0.09), 5 farther (+0.025 to +0.18); about 1.2 x the synthetic paired spread, so within noise for one dataset. SAFETY at `1cab792`: one non-target change in a climb-out window, entry 7 at t = 169.196 s (no target, SCAN_APPROACH), its centre moved 0.01-0.02 m (the scorer's threshold is 1 mm); cause: the post-spray hold ended on SCAN_APPROACH (D67), fixed | no regression with D67 (centre RMSE within noise) |
| Real-flight scorecard, real profile (`cand_obj_real.cfg`; `real/real_score.json`) | 63 trees, 68 IDs, area p10/50/90 21.94 / 37.84 / 47.30, vanished 5, jumps 0, implausible-height frames 59, max centre drift from first 2.586 m | identical trees, IDs, outlines, heights, counters; max centre drift from first 3.455 m; nearest-crown-peak distance p10 / p50 5.76 / 7.46 -> 4.26 / 7.83 | outlines, heights, IDs byte-identical for all 63 trees; centre drift from first publication +0.87 m (the centre now travels toward the crown, jumps stay 0, per-snapshot step 0.265 -> 0.264 m) | no regression in a bounded quantity |
| Real-flight scorecard, sim profile | 101 trees, 111 IDs, drift 3.342 m | 101, 111, drift 4.246 m | same pattern (+0.90 m drift), outline / heights identical | as above |

The scorecard of the real flight has no ground truth, no phases: recall, centre error, spray outcome and the locked-phase
rules are n/a there (as in `results/real_flight_scorecard.md`). The earlier numbers are reproduced: sim profile before = 101
final trees / 111 IDs (as `results/real_flight_scorecard.md` after the S8 guard), real profile 63 / 68.

### Rollback: `centre_from_support = false` reproduces the before build

* Synthetic: the after build with `--config centre_from_support=false` on all 15 suite_v2 datasets: `map.jsonl` identical to the
  before build's (without `stage_ms`) and `targets.json` byte-identical on 15 of 15 (`1cab792` run). With D67 the reference is
  the hold build (`be0e5bd` + the D67 line): after-off equals hold on the 15 suite_v2 datasets and the recorded Isaac mission
  (maps and targets), and hold equals before on all 16, so `centre_from_support = false` still reproduces `be0e5bd` there
  (`rollback.txt`, written by `analyse_d67.sh` with `same_maps.py`, which drops `stage_ms`; the `1cab792` run's file was
  written the same way, not by the `cmp` line in `stage2.sh`).
* Real log, both profiles: `sim_before` = `sim_after_off`, `real_before` = `real_after_off`, 2559 of 2559 snapshots identical
  (`maps_equal.txt`; with D67 also `*_hold` = `*_before` = `*_after_off`: the log has no mission phases, so no hold starts);
  the same stability, crown and cost summaries in `real_score.json`.
* The fix is not vacuous: with it on, the map differs from snapshot 81 (sim) / 87 (real).
* (The four Isaac replays were shown byte-identical earlier, section "Centre / result".)

### Real log, every published snapshot (2559 snapshots; `centre_outline_real_log.txt`, script `centre_outline.py`)

| | sim before | sim after | real before | real after |
|---|---|---|---|---|
| entry-snapshots | 137092 | 137092 | 89602 | 89602 |
| centre outside its own outline | 0 | 857 (0.625 %), 6 entries | 0 | 0 |
| distance outside: median / p90 / max | - | 0.113 / 0.366 / 1.000 m | - | - |
| `centre_clamped_to_outline` (counter, frames) | 0 | 19 | 0 | 0 |
| `centre_support_fallback_latched` | 0 | 13907 | 0 | 342 |
| centre inside ANOTHER entry's outline: pairs / episodes | 20 / 21 | 28 / 34 | 0 / 0 | 2 / 2 (16 entry-snapshots; 2.4 s and 0.4 s) |
| snapshots with at least one such pair | 2022 | 2000 | 0 | 16 |

So with the real profile no centre is ever outside its outline over the whole log and the clamp never acts; the sim profile
reproduces the Task 6 review values exactly (20 -> 28 pairs, 19 clamps, maximum outside 1.000 m, which is `centre_max_outside`).
Final trees (`realcmp.txt`): real profile centres shift by a median 0.48 m (p90 1.18, max 3.17), distance to the stereo
crown peak mean 1.76 -> 1.63 m, p90 3.02 -> 2.66; sim profile 2.40 -> 2.35 m, p90 4.25 -> 4.43 m.
Centre churn (`churn.txt`, `churn.py`): entry-snapshots in which the published centre moves by more than 1 mm:
sim profile 1.40 % -> 3.88 %, real profile 1.89 % -> 4.44 %, recorded Isaac 2.13 % -> 4.74 %; the largest single step is
unchanged (0.30 / 0.265 / 0.182 m, the rate limit).

### Verdict

* Detection, outlines, heights, IDs, jumps, every other safety counter, the real-flight maps (except the centre) and the cost
  per frame: no change beyond the noise floor on any evaluation (45 synthetic missions with a zero difference on everything but
  the centre RMSE, which improves by 0.005 - 0.018 m on average).
* What gets worse with the centre fix, not fixed in this task (the centre now follows the observation support and moves about
  twice as often):
  1. **Easy scenes:** the centre RMSE is 1.4 - 1.5 cm worse: held-out easy 0.133 -> 0.147 m, t +2.2, beyond the |t| < 2 rule,
     so a small regression beyond noise; suite_v2 easy +0.014 m, t +1.7, within it. Medium / hard scenes improve by
     0.01 - 0.05 m (held-out medium t -2.3).
  2. **Recorded Isaac mission:** centre RMSE 0.191 -> 0.255 m (5 of 9 trees farther), about 1.2 x the synthetic paired spread,
     so within noise for a single dataset (plausibly the mission's own observation geometry, not a defect).
* **Corrected (fix round 1):** the `1cab792` run also showed `SAFETY total violations` 0 -> 1 on the Isaac mission (a 1 - 2 cm
  centre move of non-target entry 7 in SCAN_APPROACH, inside the scorer's climb-out window), first reported here as a price of
  the centre fix. The cause was cand_obj's post-spray hold, which ended on SCAN_APPROACH (since M3) against its documented
  "until 2 m away"; D66 only made a centre move in that window more likely. D67 fixes the hold: SAFETY is 0 on the mission and
  on all 45 synthetic missions, and nothing else in this section changes (`climbout.txt`: SCAN never starts inside the window,
  the closest SCAN start is 5.07 m from the hover point). D67 changes the 5x5 corridor replay's A1 count 6 -> 8 (section
  "Centre / result", ruling R25).
* Raised to the controller (spec section 6); numbers also in `docs/DECISIONS.md` (addendum to D66, D67).
