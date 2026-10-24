# PROGRESS

Autonomous, time-boxed build of the on-board tree mapper (see `docs/PROJECT_BRIEF.md`,
`docs/LOCAL_RUN.md`, `docs/DATA_README.md`). Launch ~15:00, end ~00:00 (2026-10-24).
**All commands run inside the `tm_dev` container** (`docker exec tm_dev bash -lc '... cd /ws && ...'`).

## Milestone plan

| # | Window | Scope | Status |
|---|---|---|---|
| M1 | 15:00–17:00 | Repo skeleton (core lib, tools package, msgs package, test runner); bag extraction; scorer; **baseline scorecard of the existing system** | **done** |
| M2 | 17:00–18:30 | Synthetic orchard generator (hard noise, seeded, full missions) + a deliberately simple baseline system; both scored on synthetic and recorded data | **done** (15:20-16:15) |
| M3 | 18:30–21:30 | >= 2 genuinely different architectures (e.g. per-object tracking+fusion vs spatial evidence grid), compared with the scorer over many seeds + noise floor; pick one | next |
| M4 | 21:30–23:10 | Hardening of the chosen design; a test per rule of brief section 7; single config file + defaults test; ROS 1 wrappers + launch files (real-log and sim modes) built in tm_dev | |
| M5 | 23:10–00:00 | Handoff: README, results report, design doc (section 10 answers), decision log; everything committed | |

If behind: cut alternatives, never the scorer, the safety-rule tests or the handoff.

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

## What M3 needs to know

- **Run and score a candidate in one command:** `docker exec tm_dev bash -lc 'cd /ws && ./scripts/evaluate.sh <system>'`
  (output in `/data/work/eval/<system>/`, ~3 min). To add a candidate:
  1. Implement it in the core: a class with `process(FrameInput)`, `publishedTrees()`, `markSprayed(id)` and
     `takeStageTiming()`, like `SimpleMapper`.
  2. Register its name in `core/apps/replay_main.cpp` (`--system`). Today the runner only knows `simple`, so
     factor a small interface when adding the second system.
- `TM_REPEAT=2 ./scripts/evaluate.sh <system>` reruns every synthetic dataset (the run-to-run floor stays 0 while
  everything is deterministic). For more seeds: `python3 -m treemapper_tools.synth --out /data/work/synthetic/suite_v2 --seeds 0-9`,
  then `./scripts/evaluate.sh <system> suite_v2`. About 18 s per mission per core; mind the 40 GB budget, 30 MB per mission.
- **Medium is the discriminating level**: touching canopies, 3x real stereo noise. **Hard** is a stress test: 5x
  noise gives sigma_Z ~1.5 m at 15 m and ~6 m in turns. Tune on synthetic, report on the recording (held out).
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

## Blockers

None.
