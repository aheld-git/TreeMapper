# PROGRESS

Autonomous, time-boxed build of the on-board tree mapper (see `docs/PROJECT_BRIEF.md`,
`docs/LOCAL_RUN.md`, `docs/DATA_README.md`). Launch ~15:00, end ~00:00 (2026-10-24).
**All commands run inside the `tm_dev` container** (`docker exec tm_dev bash -lc '... cd /ws && ...'`).

## Milestone plan

| # | Window | Scope | Status |
|---|---|---|---|
| M1 | 15:00–17:00 | Repo skeleton (core lib, tools package, msgs package, test runner); bag extraction; scorer; **baseline scorecard of the existing system** | **done** |
| M2 | 17:00–18:30 | Synthetic orchard generator (hard noise, seeded, full missions) + a deliberately simple baseline system; both scored on synthetic and recorded data | next |
| M3 | 18:30–21:30 | >= 2 genuinely different architectures (e.g. per-object tracking+fusion vs spatial evidence grid), compared with the scorer over many seeds + noise floor; pick one | |
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

## What M2 needs to know

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
- `/tf_static` in the bag parents `bottom_left_camera`/`bottom_right_camera` directly to `world` (Isaac quirk);
  the camera->body extrinsic for reprojection has to be worked out (or assume nadir, image axes vs body yaw
  unknown). Not investigated in M1.
- `/disparity_image_down_float` has 28874 messages vs 14975 left images (~2x): probably two publishers or
  duplicates; check stamps before using it. Detection stamps vs image stamps not checked.
- The detector in the recording sleeps outside SCAN/PLANNING/HOPPING (no detections in SCAN_APPROACH/SPRAYING).
- Scorer thresholds (`ScoreConfig`) are fixed for comparability; change them only with a decision-log entry and
  re-score every system.
- The existing system's map exists for one run only; no noise floor for it. M3 must measure our own noise floor
  (same build, different seeds / repeated runs) before claiming improvements.
- Real flight (`/data/real/`) not touched in M1.

## Blockers

None.
