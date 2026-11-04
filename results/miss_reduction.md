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
  repository's `config/`, so it is `config/cand_obj.cfg`, **unchanged** (last commit `c375a5a`, 45 keys). Every run's `ros.log`
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
has the same values to the printed digits, except three rows-corridor figures that differ in the last digit (live A3 far / near
mean `dc` 0.349 / 0.366 m, worst `dh` -0.638 m).

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

## Reproducing

```
# replay + both scorings + equivalence per dataset (scratch scripts, not in the repository):
/data/work/replay/process_one.sh <ds>      # replay_one.sh, score_one.sh, equiv.py, trace_diff.py
```
`equiv.py` pairs entries (truth tree, rank by first publication), takes each entry's last snapshot value against the run's
`truth.json` (`mission_score._tree_top` / `_tree_centre`) and the lock / dose values from the two `metrics.json`; `trace_diff.py`
does the time-aligned comparison. Both import only the reviewed tools and write nothing into the repository.
