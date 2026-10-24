# PROGRESS

Autonomous, time-boxed build of the on-board tree mapper (see `docs/PROJECT_BRIEF.md`,
`docs/LOCAL_RUN.md`, `docs/DATA_README.md`). Launch ~15:00, end ~00:00 (2026-10-24).
**All commands run inside the `tm_dev` container** (`docker exec tm_dev bash -lc '... cd /ws && ...'`).

## Milestone plan

| # | Window | Scope | Status |
|---|---|---|---|
| M1 | 15:00–17:00 | Repo skeleton (core lib, tools package, msgs package, test runner); bag extraction; scorer; **baseline scorecard of the existing system** | **done** |
| M2 | 17:00–18:30 | Synthetic orchard generator (hard noise, seeded, full missions) + a deliberately simple baseline system; both scored on synthetic and recorded data | **done** (15:20-16:15) |
| M3 | 18:30–21:30 | >= 2 genuinely different architectures (e.g. per-object tracking+fusion vs spatial evidence grid), compared with the scorer over many seeds + noise floor; pick one | **done** — **cand_obj chosen** (D31, `results/m3_comparison.md`) |
| M4 | 19:00–20:30 | Hardening of the chosen design; a test per rule of brief section 7; single config file + defaults test; ROS 1 wrappers + launch files (real-log and sim modes) built in tm_dev | **done**, as four parallel parts: M4a cand_obj hardening (D32-D39), M4d generator fix + suite_v2 (D40), M4b ROS wrapper (D41-D47), M4c real flight (D48-D54); M4c added the real-flight profile `config/cand_obj_real.cfg`, the default of the real launch (D53, D54) |
| M5 | 20:15–22:45 | Handoff: README, results report, design doc (section 10 answers), decision log; everything committed | **done** (D55-D57); independent review follows |

If behind: cut alternatives, never the scorer, the safety-rule tests or the handoff.

## Final state (M5, 2026-10-24)

**Start here**: `README.md` (build, run, score, parameters, hardware checklist), `results/REPORT.md` (all results, with
sources), `docs/DESIGN.md` (design, brief section 10 answers, rule-by-rule tests), `docs/DECISIONS.md` (D1-D57, R1-R14).

| Deliverable (brief section 12) | Where | Status |
|---|---|---|
| 1 buildable ROS 1 workspace: core library, thin node, interface package, launch files real + sim | `core/`, `ros/treemapper_ros/`, `ros/treemapper_msgs/`, `ros/treemapper_ros/launch/` | done; built and tested in tm_dev (x86); **never built on the Orin** |
| 2 synthetic world, scorer, one-command evaluation | `tools/treemapper_tools/{synth,score,evaluate}.py`, `scripts/evaluate.sh` | done |
| 3 results report (vs simple, vs cand_grid, vs the existing system; worst cases, noise floor, weaknesses) | `results/REPORT.md` | done |
| 4 design document answering section 10 | `docs/DESIGN.md` | done (map reuse: design only) |
| 5 handoff README | `README.md` | done |
| 6 decision log | `docs/DECISIONS.md` | done, consolidated and renumbered |

**Verification at handoff** (M5, fresh build dirs, `results/REPORT.md` section 11): `scripts/run_tests.sh` ALL TESTS
PASSED (63 gtest cases in `test_core`, 102 pytest); catkin build of both packages clean; `test_treemapper_ros` 28/28 (incl. M4b's launch-config test 9100ccb);
`rostest replay_smoke.test` SUCCESS; `evaluate.sh cand_obj` on Isaac reproduces the committed scorecard exactly
(byte-identical map stream without wall-clock timing).

**Headline** (`results/cand_obj_m4_isaac.md`): on the recorded Isaac mission cand_obj finds 9/9 trees (existing 7/9),
mean IoU 0.867 (0.629), centre RMSE 0.19 m (1.36), 0 safety violations (5), 0 centre/height jumps (8/19); scan latency
2.32 s (1.19 s).

### Open items

1. **Hardware**: Orin build, timing next to the detector, live camera chain, navigation integration: all unverified
   (README checklist, in priority order).
2. **Two mapper profiles**: the real profile (63 trees on the farm flight) regresses on synthetic data (held-out recall 0.93 -> 0.85, 20 safety frames on hard_s100, +2 s latency on Isaac), so the real launch uses it and everything else the sim profile. It is tuned on one flight against a stereo-peak proxy, no ground truth. A spacing-aware merge rule could unify the two (code change, not done).
3. **Latency** ~1.1 s behind the existing system on the recording; publication thresholds are the lever (README table).
4. **Hard-level recall** 0.83 (M4 traded 0.02-0.04 recall for 0 jumps, D39).
5. **Map reuse across flights** is a design only (`docs/DESIGN.md` section 2.5).
6. **Stale parameter**: with `seed_truncated = true` (shipped), `peak_border_px` only feeds a counter; the comments in
   `cand_obj.hpp` and `docs/candidates/cand_obj.md` sections 2 / 6 still describe the M3 meaning ("no height sample").
   Harmless, but a reader tuning it will see no effect.
7. Heights are world z and inherit any pose-z drift of the flight controller (unknown on the real flight).
8. The Isaac truth heights are a constant fallback, so heights are validated on synthetic data only.

The sections below are the milestone history, kept as written (earlier "next milestone" notes are superseded by this
section).

## M1 delivered

- `core/` — ROS-free C++17 library `treemapper_core`: `types.hpp` (TreeEntry = the contract entry, MissionPhase +
  parser, DronePose, Detection2D), `tree_map.hpp` (TreeMap: allTrees / findTree / markSprayed / upsert, and
  `isPublishable` which rejects undefined heights and degenerate outlines). gtest suite `test_core`. Plain CMake,
  `package.xml` with `build_type cmake` for catkin later.
- `ros/treemapper_msgs/` — TreeEntry, TreeMap, Detection2D(Array), DronePose, MissionPhase, Health, DropCounters,
  StageTiming; services GetTrees, SetTreeSprayed. Every field documented. Test-built once in a scratch catkin
  workspace (`/data/work/catkin_m1`), not integrated yet.
- `tools/treemapper_tools/` (Python 3.8):
  - `contract.py` — file formats (map stream JSONL, ground truth, mission), readers/writers, nwu->world.
  - `extract_bag.py` — reads the Isaac bag with the real `rosbag` API.
  - `score.py` — the scorer (detection, geometry, stability, safety, timeliness, cost; final + per-frame), CLI
    + markdown/json report. Definitions: `docs/SCORER.md` and the module docstring.
- `tools/tests/` — 27 pytest cases on hand-made maps, each safety metric shown firing (and not firing on the
  allowed counterpart: target changes, transit growth, spray-flag flips, new trees during SCAN).
- `scripts/run_tests.sh` (all tests, one command), `scripts/score_baseline.sh` (extract once + scorecard).
- `results/baseline_existing_system.{md,json}` — **the number to beat**.
- `docs/DECISIONS.md` (D1–D18), `docs/SCORER.md`.

Test result (M1 end): `ALL TESTS PASSED` — ctest 1/1 (4 gtest cases), pytest 27 passed.

## Baseline scorecard (existing system, recorded Isaac mission) — summary

**Stale (M1, pre-fix scorer).** The scorer review fixes (commit e079bff, DECISIONS R1-R13) changed several
numbers: the existing system now has 5 safety violations, time-weighted per-frame recall 0.58, and height rows
n/a (the truth heights are a constant 4.0 m fallback). Current numbers: `results/baseline_existing_system.md`
and the side-by-side in `results/simple_baseline_isaac.md`.

Full card: `results/baseline_existing_system.md`.

| Metric | Final map | Per-frame (all snapshots) |
|---|---|---|
| Truth / published / matched | 9 / 7 / 7 | |
| Recall / precision | 0.78 / 1.00 | 0.50 / 1.00 |
| Merged entries / trees in them / swallowed | 2 / 4 / 2 | |
| Split / false trees | 0 / 0 | |
| Mean IoU (min) | 0.63 (0.39) | 0.60 |
| Centre RMSE / max | 1.35 / 2.36 m | 1.33 m |
| Height bias / RMSE | +1.41 / 1.46 m | 1.49 m RMSE |
| Spill outside canopies / neighbour spill max | 0.16 / 0.41 | 0.15 |
| ID switches total / after lock | 1 / 0 | |
| Geometry retreats / centre jumps / height jumps (>1 m) | 0 / 2 / 14 | |
| **Safety violations** | **1** (sprayed tree 7 absorbed hop target 8 at 145.98 s; target ID vanished) | |
| Undefined heights / new trees when locked / non-target descent changes | 0 / 0 / 0 (370 descent frames) | |
| Sprayed truth trees | 7 of 9, 0 twice, 0 orphans | |
| Scan latency mean / max | 2.9 s / 11.4 s (upper bounds) | |
| Cost | n/a (not in recording) | |

Matches DATA_README on recall, precision, height, sprayed count, "4 over-merged" (= trees in merged entries),
0 ID switches after lock. Differs (definitions, documented in the card's notes): IoU 0.63 vs 0.61, spill 0.16 vs
0.13, centre RMSE 1.35 vs 0.97 m (**not reproduced**), fragmented 0 vs 3 (**not reproduced**), ID switches total 1
(pre-lock merge) vs 0.

## M2 delivered

- **Input contract** — `tools/treemapper_tools/frames.py` (Python), `core/include/treemapper/frame.hpp` + `frame_io.hpp` (C++).
  - One `Frame` per detection message: receive t, stamp, body pose + validity, phase, hop target (navigation's ID +
    descent point), boxes, and disparity on a stride-4 grid (160 x 100).
  - File `frames.bin`: about 32 kB per frame; heartbeat frames while the detector sleeps.
  - Bag reader: `bag_frames.py`. It streams via the rosbag API, rejects the malformed half of the disparity
    messages, pairs by exact stamp with a 20 ms nearest fallback, and takes the pose from `/tf` at the stamp.
  - Recorded mission: `/data/work/extracted/isaac_2026-10-23-15-34-29/frames.bin` (symlink to
    `/data/work/frames/isaac_full.bin`, 226 MB, ~1 min to rebuild).
- **Camera geometry verified** — `results/geometry_check.md`.
  - `/tf_static` is the camera pose AT SPAWN, not the mount. The mount = inv(world->body at spawn) * tf_static,
    an exact nadir mount with lever arm (0.15, 0.08, 0).
  - Ground flat to 0.01 deg and 1 cm; 99.6 % of canopy points inside true canopies; per-tree canopy centroid
    0.35 m mean from the true centre, no bias.
  - **The truth heights are ~1.4-1.9 m too low** (constant 4.0 m). The scorer now marks them unreliable (R13).
- **Simple baseline** — `core/include/treemapper/simple_mapper.hpp`, `SimpleMapper`, deliberately naive; see D23.
  - Rules: the pure `phasePolicy()`, plus rule tests in `core/tests/test_simple_mapper.cpp` (14 gtest cases in total).
  - Runner: `treemapper_replay` (`core/apps/replay_main.cpp`) writes the contract JSONL at 5 Hz plus phase-boundary
    snapshots. It also runs a navigation emulator: target resolution by descent point, mark-sprayed when SPRAYING ends.
- **Synthetic generator** — `tools/treemapper_tools/synth.py`, levels easy / medium / hard.
  - Writes the same files as the recorded mission: frames.bin, ground_truth.json, mission.json, camera_info.json.
  - Seeded and byte-reproducible.
  - Calibration against the recordings, including the real flight's disparity: `results/synthetic_calibration.md`.
  - Suite `suite_v1` = 5 seeds x 3 levels at `/data/work/synthetic/suite_v1` (438 MB, 2 min 40 s to regenerate).
- **One-command evaluation** — `scripts/evaluate.sh [system] [suite]`.
  - Builds, extracts or generates whatever is missing, runs the system on the recording (side by side with the
    existing system) and on the suite, and prints the scorecards.
  - Python entry point: `python3 -m treemapper_tools.evaluate --system a,b --dataset D | --suite S [--repeat N]`.
- **Scorecards**: `results/simple_baseline_isaac.{md,json}`, `results/simple_baseline_synthetic.{md,json}`, both
  from the fixed scorer.

### Scorecards (fixed scorer)

Recorded Isaac mission (9 trees):

| Metric | simple baseline | existing system |
|---|---|---|
| Recall / precision (final) | 0.89 / 1.00 | 0.78 / 1.00 |
| Per-frame recall (time-weighted) | 0.64 | 0.58 |
| Mean / min IoU | 0.73 / 0.46 | 0.63 / 0.39 |
| Merged entries / neighbours swallowed | 1 / 1 | 2 / 2 |
| Split / false trees | 0 / 0 | 0 / 0 |
| Centre RMSE | 1.10 m | 1.36 m |
| Height error | n/a (GT unreliable); publishes 4.8-5.7 m vs measured tops 5.4-5.9 m | n/a |
| Spill / neighbour spill max | 0.14 / 0.38 | 0.16 / 0.42 |
| Centre jumps / height jumps | 8 / 3 | 8 / 19 |
| **Safety violations** | **0** | **5** |
| Truth trees sprayed | 7 | 7 |
| Scan latency mean | 2.4 s | 1.2 s |
| Cost | 0.50 ms extract + 0.004 ms integrate per frame (x86) | n/a |

Synthetic `suite_v1` (mean / worst over 5 seeds):

| Metric | easy | medium | hard |
|---|---|---|---|
| Recall (final) | 0.90 / 0.79 | 0.50 / 0.30 | 0.27 / 0.09 |
| Precision (final) | 0.89 / 0.86 | 0.53 / 0.43 | 0.25 / 0.09 |
| Mean IoU | 0.58 / 0.49 | 0.37 / 0.33 | 0.22 / 0.14 |
| Neighbours swallowed | 2.2 / 4 | 14.6 / 19 | 25.2 / 29 |
| False trees | 1.6 / 3 | 9.6 / 13 | 22 / 26 |
| Centre RMSE | 1.24 / 1.50 m | 2.06 / 2.85 m | 2.65 / 3.37 m |
| Height RMSE | 0.25 / 0.39 m | 0.56 / 0.82 m | 0.83 / 1.38 m |
| Spill | 0.43 / 0.53 | 0.64 / 0.69 | 0.83 / 0.86 |
| Safety violations | 0 / 0 | 0 / 0 | 132 / 225 (all implausible heights above the drone) |

**Noise floor**: run-to-run is 0 (deterministic system and data; `--repeat 2` gives identical numbers). The
real floor is seed-to-seed: the std over datasets is in the cards, e.g. medium final recall spans 0.30-0.67.
**Compare candidates paired by seed.**

Tests at M2 end: `./scripts/run_tests.sh` -> ALL TESTS PASSED (ctest: 14 gtest cases, 10 of them new; pytest: 76 passed, 4 of them new in `tools/tests/test_frames_synth.py`).

## Post-M2: Mapper interface + registry (D29)

`treemapper::Mapper` + a name-keyed registry, so two candidates can be built in parallel with no shared edit but one
registration line; `evaluate` compares any registered systems paired by seed. See "How to add a candidate".

**No behaviour change of `simple`** (before = commit 1a5a72e build, after = this change, both via `./scripts/evaluate.sh simple`,
outputs in `/data/work/eval/before_refactor` and `/data/work/eval/after_refactor`): for all 16 runs (recorded Isaac
mission + the 15 suite_v1 datasets) the map stream is byte-identical once the wall-clock `stage_ms` field is removed
(sha256 equal), `targets.json` and `counters.json` are byte-identical, `score.json` is identical in every metric
(the `cost` section is wall-clock timing and differs), and the `isaac.md` / `synthetic.md` scorecards differ only in
their timing rows (plus the new paired section in `isaac.md`). `./scripts/run_tests.sh`: ALL TESTS PASSED
(ctest 22 gtest cases, pytest 80).

## How to add a candidate

Everything below runs inside `tm_dev` (`docker exec tm_dev bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws && ...'`).
A candidate edits **no shared file except one registration line**.

**The interface** (`core/include/treemapper/mapper.hpp`, class `treemapper::Mapper`; read the header, it is short):
`process(const FrameInput&)`, `publishedTrees() const`, `markSprayed(TreeId)`, `takeStageTiming()`,
`counters() const`, and an optional `setLogger()`. Configuration is a `MapperConfig` (`map<string,string>`)
handed to the factory; `ConfigReader` reads typed values and `finish()` rejects unknown keys. `SimpleMapper`
(`simple_mapper.hpp/.cpp`) is the worked example; `phasePolicy()` is a reusable pure function for the
phase rules.

**Steps** (name = `cand_a`):

1. Create your own files, nothing else:
   - `core/include/treemapper/candidates/cand_a/*.hpp`
   - `core/src/candidates/cand_a/*.cpp` (all `*.cpp` in the directory are globbed; no CMake edit)
   - optional tests: `core/tests/candidates/cand_a/*.cpp` (globbed into `test_core`; plain gtest)
   - optional `core/src/candidates/cand_a/candidate.cmake` for extra `find_package` / link lines (runs after
     `treemapper_core` exists: `target_link_libraries(treemapper_core PUBLIC ...)`)
2. In one of your .cpp files, in `namespace treemapper`, define the factory (this exact name, `tm_create_` + your name):
   ```cpp
   #include "treemapper/mapper.hpp"
   namespace treemapper {
   std::unique_ptr<Mapper> tm_create_cand_a(const MapperConfig& config, const StereoCalibration& cal) {
     return std::make_unique<CandA>(CandAConfigFrom(config), cal);   // throw std::invalid_argument on a bad config
   }
   }  // namespace treemapper
   ```
3. The one shared edit: add one line to `registry()` in `core/src/mapper_registry.cpp`, after `TM_REGISTER_BUILTIN(simple)`:
   ```cpp
   TM_REGISTER_CANDIDATE(cand_a)
   ```
   Two agents each add their own line on adjacent lines; git merges that cleanly (keep them alphabetical).
4. Build and unit-test. The generic `test_mapper_registry.cpp` already runs every registered system through the
   interface (heartbeats, unknown config key, unknown ID), so your candidate gets that smoke test for free.
   While another candidate is half-written, build only yours (the other candidates' sources are then not even
   compiled; their registry lines register nothing): `TM_CANDIDATES=cand_a TM_BUILD=/data/work/build_cand_a ./scripts/run_tests.sh`.
   Always use your own `TM_BUILD` and `TM_OUT`, so two agents never overwrite each other's build or output.
5. Evaluate, paired against `simple` (first name = reference; `TM_REF=` changes it):
   ```
   TM_CANDIDATES=cand_a TM_BUILD=/data/work/build_cand_a TM_OUT=/data/work/eval/cand_a ./scripts/evaluate.sh simple,cand_a
   TM_SET="cand_a.alpha=0.3 cand_a.k=5" ./scripts/evaluate.sh simple,cand_a      # configuration entries
   ./scripts/evaluate.sh simple,cand_a,cand_b suite_v2                           # all of them, another suite
   ```
   Outputs `$TM_OUT/isaac.{md,json}` (recorded mission, also vs the existing system) and
   `$TM_OUT/synthetic.{md,json}`; `treemapper_replay --list-systems` lists what the binary knows. Direct form:
   `python3 -m treemapper_tools.evaluate --system simple,cand_a --suite /data/work/synthetic/suite_v1 --reference simple`.

**Paired output** (2+ systems, per non-reference system `S`, after the usual per-system cards):
`## Paired: S vs ref (difference = S - ref, same dataset = same seed)`, then
1. a summary table, one row per headline metric: `ref mean | S mean | mean diff | std diff | t | win / loss / tie`
   over the pairs (a win = S better than ref; t = mean diff / (std diff / sqrt n), |t| < 2 is seed noise);
2. mean paired difference per group (easy / medium / hard);
3. **per seed**: one row per dataset (`medium_s3`, ...), columns = signed differences `S - ref` of recall,
   precision, mean IoU, neighbours swallowed, false trees, centre RMSE, height RMSE, spill and safety violations.
The JSON (`paired` key) carries the per-seed difference for every headline metric.

## What M3 needs to know

- **Run and score a candidate in one command:** see "How to add a candidate" below (interface + registry landed
  after M2, D29; the runner no longer needs editing).
- `TM_REPEAT=2 ./scripts/evaluate.sh <system>` reruns every synthetic dataset (the run-to-run floor stays 0 while
  everything is deterministic). For more seeds: `python3 -m treemapper_tools.synth --out /data/work/synthetic/suite_v2 --seeds 0-9`,
  then `./scripts/evaluate.sh <system> suite_v2`. About 18 s per mission per core; mind the 40 GB budget, 30 MB per mission.
- **Medium is the discriminating level**: touching canopies, 3x real stereo noise. **Hard** is a stress test: 5x
  noise gives sigma_Z ~1.5 m at 15 m and ~6 m in turns. Tune on synthetic, report on the recording (later also tuned on: used during tuning; not an unbiased test).
- **Where the simple baseline fails**, i.e. what M3 must beat:
  - merges across touching canopies and merged boxes;
  - false trees from noise, FP boxes and partial boxes;
  - spill from grow-only hulls of raw noisy observations;
  - heights from p95 of noisy points: implausible above-the-drone heights on hard;
  - scan latency 2.4 s on the recording (3 observations to publish);
  - target resolution failing when the hop target falls in a merged blob.
- **Hop targets in replay**: recorded / synthetic targets are in navigation's ID space. The emulator resolves
  them to the system's IDs by descent point (D22), and the scorer judges against `mission_resolved.json` in the
  run directory. On the recording every system flies the existing system's hop plan.
- **Phase-boundary snapshots** (D28): the runner writes one at the last frame of every phase; keep that when
  changing the runner.
- **Isaac height metrics are n/a by design** (`gt_height_reliable` false). Use synthetic for height accuracy;
  the measured Isaac canopy tops are 5.4-5.9 m.
- **Real flight**: not yet read through the input contract. Its `/tf_static` has a real `body -> sensor_down`
  mount (~75 deg about y). A real-flight frame reader (frames from the main bag + detections from the companion
  bag) is still to be written; the stride-4 disparity grid and calibration path should carry over. Its disparity
  is 1/16 px quantised, ~0.1 px noise, 97-98 % valid.
- Python stays 3.8-compatible. Scorer files belong to the scorer owner: change them only with a decision-log entry.

## M3 delivered (selection, 2026-10-24 18:00-19:00)

- Candidates (frozen, commit d5ce8be): `cand_obj` (per-instance crowns + evidence latching, `docs/candidates/cand_obj.md`)
  and `cand_grid` (fused 2.5D DSM + persistence watershed, `docs/candidates/cand_grid.md`).
- **Scorer R14**: implausible heights judged against the truth top + 0.5 m when truth heights are reliable (tests added;
  baseline pin unchanged). Candidates' hard-level "violations" (624 / 527 frames) were all accurate heights under a
  low hover -> 0; simple's 1670 remain.
- **Final comparison on the held-out suite** `/data/work/holdout/suite_holdout` (30 missions) + the recording:
  `results/m3_comparison.{md,json}`; rerun: per level `python3 -m treemapper_tools.evaluate --system simple,cand_obj,cand_grid
  --suite <level dir of symlinks> --reference simple`, then `python3 -m treemapper_tools.m3_compare --eval <dir>`.

| held-out, mean (medium / hard) | simple | cand_obj | cand_grid |
|---|---|---|---|
| recall | 0.39 / 0.11 | 0.98 / 0.87 | 0.96 / 0.76 |
| precision | 0.48 / 0.12 | 0.98 / 0.99 | 0.99 / 0.95 |
| mean IoU | 0.32 / 0.34 | 0.76 / 0.59 | 0.77 / 0.64 |
| neighbours swallowed | 14.3 / 29.3 | 1.0 / 3.6 | 0.7 / 5.3 |
| false trees | 10 / 18.7 | 0 / 0 | 0 / 0.8 |
| height RMSE m | 0.59 / 0.86 | 0.26 / 0.48 | 0.15 / 0.42 |
| spill | 0.68 / 0.81 | 0.17 / 0.25 | 0.15 / 0.24 |
| centre jumps | 15 / 24 | 7.4 / 6.3 | 0.7 / 3.7 |
| safety violations | 6.5 / 160.5 | 0 / 0 | 0 / 0 |
| scan latency s | 1.18 / 0.80 | 1.85 / 2.24 | 2.38 / 2.66 |

Recording: cand_obj / cand_grid / simple / existing recall 1 / 1 / 0.89 / 0.78, IoU 0.86 / 0.85 / 0.73 / 0.63, safety
0 / 0 / 0 / 5, latency 2.23 / 2.30 / 2.39 / 1.19 s.

## What M4 must do (cand_obj hardening)

Work list with evidence in `results/m3_comparison.md` ("M4 work list"); in short:
1. Height statistic: high quantile of latched surface instead of the crown-top-cell median (two Isaac trees published
   ~1.1-1.5 m below the others; +-2.5 m tails on medium_s105), keep locked heights frozen.
2. Publication stability from cand_grid: rate-limit centre/height motion and/or require an observed peak
   neighbourhood (7.4 centre jumps per medium mission vs 0.7).
3. cand_grid's relative-height latch (0.6 x peak CHM) for spill; its range / off-axis / rotation-rate weighting.
4. Test that a fragment can never take over a published tree's match (Isaac transient duplicate at 178 s).
5. Fix the generator's hover altitude (D30), generate a new tuning suite; never touch `suite_holdout`.
6. Then the original M4 scope: a test per brief section 7 rule, single config + defaults test, ROS 1 wrappers + launch
   files (real-log and sim) built in tm_dev.

## M1 notes: bag facts and contract details (still valid)

**Commands** (in tm_dev): `./scripts/run_tests.sh`; `./scripts/score_baseline.sh`;
`python3 -m treemapper_tools.score --map M.jsonl --gt G.json --mission mission.json --camera camera_info.json --out-md X.md --out-json X.json`
(run from `/ws/tools`, or `sys.path` insert `/ws/tools`). Python must stay 3.8-compatible
(`from __future__ import annotations`, no `match`, no PEP 604 at runtime).

**Extracted recorded mission** — `/data/work/extracted/isaac_2026-10-23-15-34-29/` (host:
`/media/alex/waspsim/cloud_build_bundle/work/extracted/...`), all in sim frame `world` (ENU):
`ground_truth.json`, `map_existing.jsonl` (existing system), `mission.json` (phases, targets, pose = drone
position from `/height_estimator/drone_position`, ~10 Hz), `active_target.json`, `detections.jsonl`
(`{"t","stamp","boxes":[[x,y,w,h,label,score],..]}`, pixels of the half-res left image, ~27 Hz when awake),
`body_tf.csv` (world->body, t,x,y,z,qx,qy,qz,qw), `gt_tree_tf.json`, `camera_info.json`, `meta.json`.
Re-extract: `python3 -m treemapper_tools.extract_bag --bag /data/isaac/<bag> --out <dir>` (~1 min).

**Scene facts for the synthetic generator** (from the recording): 3x3 trees, 6 m spacing with up to 1 m jitter,
canopy radii 2.46 / 2.81 m (gaps between canopies ~0.7–1.2 m), every canopy top 4.0 m. Scan at 15 m, 4 m/s,
corridors along y (x = -6, then x = +6 in the second scan); hop transit ~10.5 m; spray hover ~7 m.
Mission: IDLE -> SCAN_APPROACH -> SCAN (twice with an approach between) -> PLANNING -> HOPPING/SPRAYING x3 ->
PLANNING -> HOPPING/SPRAYING -> SCAN_APPROACH -> SCAN -> PLANNING -> HOPPING/SPRAYING x3 -> RETURN -> IDLE.
Camera (half res) 640x400, fx = fy = 454.53, cx = 346.71, cy = 209.68; stereo baseline 0.16 m (right
`P[3] = -72724.8` = -fx·B in **millimetres**).

**The output contract a system must emit for scoring:** a map stream JSONL (see `docs/SCORER.md`), one snapshot at
a steady rate (~5 Hz) from the start of the run including empty ones, same clock as `mission.json`; plus
optional `stage_ms` per snapshot for cost. The synthetic generator should write `ground_truth.json`,
`mission.json`, `camera_info.json` in the same formats so `score.py` works unchanged.

**Watch out / unverified:**
- `/tf_static` in the bag parents `bottom_left_camera`/`bottom_right_camera` directly to `world` (Isaac quirk).
  The translation (0.146, ±0.080, 0.071) is clearly the mount offset relative to `body`, and the rotation
  (q = 0.707, -0.707, 0.021, -0.021) maps optical z to -z (nadir). Likely meaning: parent should be `body`.
  **Unverified** — check by reprojecting disparity of a known tree against the truth before relying on it.
  Images and disparity carry frame_id `body_bottom`, 640x400 (mono8 / 32FC1).
- Stamps (checked in M1, one pass over the bag): left 14975 / right 15115 images (~54 Hz, unique stamps);
  `/disparity_image_down_float` 28874 messages but only 14630 unique stamps — **every disparity frame is
  published ~twice** (one publisher, `/depth_to_disparity_node`, i.e. simulator depth, not stereo matching);
  de-duplicate by stamp. 90 % of disparity stamps equal a left-image stamp exactly (max offset 32.5 ms).
  Detections (~27 Hz when awake): 99.5 % of stamps equal a left-image stamp exactly (max 32.5 ms).
- The detector in the recording sleeps outside SCAN/PLANNING/HOPPING (no detections in SCAN_APPROACH/SPRAYING).
- Scorer thresholds (`ScoreConfig`) are fixed for comparability; change them only with a decision-log entry and
  re-score every system.
- The existing system's map exists for one run only; no noise floor for it. M3 must measure our own noise floor
  (same build, different seeds / repeated runs) before claiming improvements.
- Real flight (`/data/real/`) not touched in M1.

## M4b delivered: ROS 1 wrapper `ros/treemapper_ros` (19:00-22:00)

- Node `treemapper_node` (C++17) around any registered `Mapper` (`~mapper`, default cand_obj), Mapper interface +
  registry only. Inputs: disparity + left/right camera_info, detections (treemapper_msgs/Detection2DArray or the
  existing spot_spraying_msgs/DetectionResult, decoded via ShapeShifter, no dependency), pose (TF or PoseStamped),
  /mission_state, hop target (Int32, our IDs). Outputs `/treemapper/{map,markers,health,counters,stage_timing}`,
  services `/treemapper/{get_trees,set_tree_sprayed}`. Pairing = bag_frames.py rules (exact stamp, 0.5 s wait,
  nearest 20 ms, de-dup, malformed rejected) + early resolve. Topics/params/gates/Jetson build: `ros/treemapper_ros/README.md`.
- Config: `config/treemapper.yaml` points at `config/core/<mapper>.cfg` (symlink to the repo's `config/`), never copies
  it; `test_config_agreement` enforces it. Core consumed by `add_subdirectory` (weak-symbol registry needs
  `tm_link_registry`); no change to core/CMakeLists.txt was needed.
- Launch: `treemapper_sim.launch` (world/body, verified nadir mount as param), `treemapper_real.launch` (world_nwu,
  all names args; M4c's data-derived mount 82 deg + disparity offset 4.30 px as defaults, tf mount selectable),
  `replay_isaac.launch` (bag --clock, nav_emulator.py, optional record).
- Verified in tm_dev: 0 warnings (-Wall -Wextra), gtest 27/27, rostest `replay_smoke.test` SUCCESS, full Isaac
  mission ROS vs offline replay of the same build: same 9 trees + IDs, scorecards equal within transport timing
  (`results/ros_replay_isaac.md`); 45 s real-flight smoke of the real launch runs (25 trees, not scored).
- Run it: build per the README (scratch ws `/data/work/catkin_m4b`, profile `snap` = git-archive export of the core),
  then `ros/treemapper_ros/scripts/ros_replay_check.sh OUT 45 0 0.00390625` (~5 min).
- Unverified: Orin build/timing, real camera chain on hardware, navigation integration (only the emulator drove it).

## Blockers

None.

## M4d note

suite_v2 (fixed hover) at /data/work/synthetic/suite_v2 is the tuning suite from now on (D40; suite_v1 and suite_holdout stay frozen).
