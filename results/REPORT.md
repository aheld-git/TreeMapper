# TreeMapper results report

Shipped system: **`cand_obj`** (M4 build, `config/cand_obj.cfg`; real-flight profile `config/cand_obj_real.cfg`, section 6),
in the ROS node `treemapper_node`. Every number below is copied from a committed results file, cited per table; nothing
here comes from a run that is not committed. All timings are x86 (tm_dev, one core per process); nothing ran on the Orin.
Definitions of every metric: `docs/SCORER.md` and `tools/treemapper_tools/score.py`. Design: `docs/DESIGN.md`.

## 1. Headline

- **Recorded Isaac mission (held out from tuning) against the existing system**: 9 / 9 trees found vs 7 / 9, mean outline
  IoU 0.867 vs 0.629, 0 neighbours swallowed vs 2, centre RMSE 0.19 vs 1.36 m, **0 safety violations vs 5**, 0 centre /
  height jumps vs 8 / 19, 7 truth trees sprayed by each. Worse: scan latency 2.32 s vs 1.19 s. (`results/cand_obj_m4_isaac.md`)
- **Held-out synthetic suite** (30 missions never used for tuning): beats the simple baseline on every level on recall,
  precision, IoU, swallowed neighbours, false trees, centre error and spill (10/0 seeds on most rows); against the
  alternative `cand_grid` it is a tie on medium and wins the hard worst cases (recall +0.073, 8/0; swallowed -2.5, 8/1;
  false trees -0.8). Safety 0 on all 30 missions. (`results/final_holdout_vs_simple.md`, `results/cand_obj_m4_holdout.md`)
- **Real farm flight** (no ground truth): no retreats, no ID reuse, 0 undefined / invalid / duplicate entries, 0 centre and
  0 height jumps for cand_obj; but with the sim profile it **over-fragments** (102 trees where the stereo shows ~60-63
  crowns and the other systems publish 54-59). The real profile is section 6. (`results/real_flight_scorecard.md`)

## 2. Final scorecard against the existing system: recorded Isaac mission

Source: `results/cand_obj_m4_isaac.md` (9 trees, 7 hops; every system flies the existing system's hop plan, targets resolved to
each system's own IDs by descent point, D22). Height-error rows are n/a by design: the recording's truth heights are a
constant 4.0 m fallback (R13, D24); measured canopy tops are 5.4-5.9 m.

| Metric | **cand_obj (shipped)** | existing system | simple baseline | cand_grid |
|---|---|---|---|---|
| recall / precision (final) | **1 / 1** | 0.778 / 1 | 0.889 / 1 | 1 / 1 |
| recall, per-frame mean (time-weighted) | **0.738** | 0.580 | 0.636 | 0.738 |
| mean / min IoU (final) | **0.867 / 0.817** | 0.629 / 0.393 | 0.730 / 0.462 | 0.850 / 0.746 |
| neighbours swallowed / split / false trees | **0 / 0 / 0** | 2 / 0 / 0 | 1 / 0 / 0 | 0 / 0 / 0 |
| centre RMSE m | **0.191** | 1.355 | 1.103 | 0.191 |
| spill / neighbour spill max | **0.064 / 0.056** | 0.164 / 0.415 | 0.141 / 0.378 | 0.093 / 0.077 |
| centre jumps / height jumps (> 1 m) | **0 / 0** | 8 / 19 | 8 / 3 | 0 / 0 |
| ID switches after lock | **0** | 0 | 0 | 0 |
| **SAFETY total violations** | **0** | **5** | 0 | 0 |
| truth trees sprayed | 7 | 7 | 7 | 7 |
| scan latency mean s | 2.319 | **1.190** | 2.386 | 2.297 |
| max height drift since lock m | **0** | 4.212 | 0.366 | 1.751 |
| target height range in own descent m | **0** | 1.840 | 0.295 | 0.641 |
| final heights published m | 4.83-5.74 | not listed in the source | 4.77-5.70 | 4.75-5.76 |
| cost ms / frame, mean (worst) | 0.860 (1.118) | n/a (not recorded) | 0.615 (0.980) | 0.580 (1.950) |

The existing system's 5 violations are one event (`results/baseline_existing_system.md`): at 145.98 s, during the climb-out
after spraying tree 7, the sprayed tree 7 absorbed the hop target 8 (sprayed-tree change, locked ID vanished while it was the
target), the freeze then failed open and tree 1 changed twice more before the drone left (3 non-target descent changes).

**ROS path = offline replay** (`results/ros_replay_isaac.md`): the full mission through `rosbag play` + the node +
`nav_emulator.py` gives the same 9 trees with the same 9 IDs, outline IoU 0.98-1.00 per tree, identical safety and stability
rows, mean IoU 0.865 vs 0.867, scan latency 2.335 vs 2.319 s.

## 3. Held-out synthetic suite: shipped system vs simple baseline vs cand_grid

Suite `/data/work/holdout/suite_holdout`: 30 missions, seeds 100-109 x easy / medium / hard (1.5x / 3x / 5x real stereo
noise, a hostile detector: misses, false positives, merged and partial boxes, D25), generated before M3 and never used for
tuning; run once per frozen configuration. Cells: **mean / worst seed** (worst = min for recall, precision, IoU, sprayed;
max otherwise). W/L/T = seeds where cand_obj is better / worse / equal, paired by seed. Sources: means, worsts and the paired
comparison with cand_grid `results/cand_obj_m4_holdout.md`; paired comparison with simple `results/final_holdout_vs_simple.md`
(same runs).

### easy (n = 10)

| Metric | simple | **cand_obj** | cand_grid | W/L/T vs simple | W/L/T vs cand_grid |
|---|---|---|---|---|---|
| recall (final) | 0.966 / 0.867 | **1 / 1** | 1 / 1 | 4/0/6 | 0/0/10 |
| precision (final) | 0.861 / 0.778 | 0.988 / 0.938 | 1 / 1 | 10/0/0 | 0/2/8 |
| mean IoU | 0.583 / 0.551 | **0.908 / 0.892** | 0.872 / 0.848 | 10/0/0 | 10/0/0 |
| min IoU | 0.274 / 0.167 | **0.840 / 0.672** | 0.761 / 0.661 | 10/0/0 | 9/1/0 |
| neighbours swallowed | 1.8 / 3 | 0 / 0 | 0 / 0 | 10/0/0 | 0/0/10 |
| split trees | 1.5 / 2 | 0.2 / 1 | 0 / 0 | 10/0/0 | 0/2/8 |
| false trees | 0.8 / 2 | 0 / 0 | 0 / 0 | 5/0/5 | 0/0/10 |
| centre RMSE m | 1.048 / 1.249 | 0.140 / 0.214 | 0.153 / 0.262 | 10/0/0 | 5/5/0 |
| height RMSE m | 0.169 / 0.240 | **0.038 / 0.055** | 0.056 / 0.110 | 10/0/0 | 10/0/0 |
| spill | 0.434 / 0.482 | **0.049 / 0.058** | 0.064 / 0.079 | 10/0/0 | 10/0/0 |
| centre / height jumps | 7.5 / 0 | 0 / 0 | 0 / 0 | 10/0/0, 0/0/10 | 0/0/10 |
| **safety violations** | 0 / 0 | **0 / 0** | 0 / 0 | 0/0/10 | 0/0/10 |
| truth trees sprayed | 3.6 / 3 | 4 / 4 | 4 / 4 | 4/0/6 | 0/0/10 |
| scan latency s | **0.564 / 0.629** | 1.142 / 1.200 | 1.400 / 1.600 | 0/10/0 | 10/0/0 |

### medium (n = 10): the discriminating level

| Metric | simple | **cand_obj** | cand_grid | W/L/T vs simple | W/L/T vs cand_grid |
|---|---|---|---|---|---|
| recall (final) | 0.393 / 0.235 | 0.960 / 0.824 | 0.961 / 0.882 | 10/0/0 | 3/3/4 |
| precision (final) | 0.478 / 0.250 | 0.995 / 0.950 | 0.989 / 0.938 | 10/0/0 | 2/1/7 |
| mean IoU | 0.315 / 0.199 | 0.779 / 0.731 | 0.771 / 0.758 | 10/0/0 | 6/4/0 |
| min IoU | 0.151 / 0.111 | 0.523 / 0.232 | 0.529 / 0.423 | 10/0/0 | 5/5/0 |
| neighbours swallowed | 14.3 / 18 | 1 / 3 | 0.7 / 2 | 10/0/0 | 3/4/3 |
| split trees | 0.1 / 1 | 0.1 / 1 | 0.2 / 1 | 1/1/8 | 2/1/7 |
| false trees | 10 / 15 | 0 / 0 | 0 / 0 | 10/0/0 | 0/0/10 |
| centre RMSE m | 2.137 / 3.864 | **0.441 / 0.656** | 0.514 / 0.721 | 10/0/0 | 8/2/0 |
| height RMSE m | 0.586 / 1.125 | 0.232 / 0.549 | **0.153 / 0.260** | 8/2/0 | 5/5/0 |
| spill | 0.679 / 0.764 | 0.158 / 0.185 | 0.153 / 0.178 | 10/0/0 | 4/6/0 |
| centre / height jumps | 15 / 0.8 | **0 / 0** | 0.7 / 0 | 10/0/0, 6/0/4 | 6/0/4, 0/0/10 |
| **safety violations** | 6.5 / 65 | **0 / 0** | 0 / 0 | 1/0/9 | 0/0/10 |
| truth trees sprayed | 2.6 / 1 | 5.1 / 5 | 5.1 / 5 | 9/0/1 | 0/0/10 |
| scan latency s | **1.178** / 4.031 | 2.024 / 3.771 | 2.383 / 4.440 | 1/9/0 | 7/3/0 |

### hard (n = 10): stress test (5x real noise, 4.5 m spacing, 60 % merged boxes)

| Metric | simple | **cand_obj** | cand_grid | W/L/T vs simple | W/L/T vs cand_grid |
|---|---|---|---|---|---|
| recall (final) | 0.105 / 0 | **0.831 / 0.750** | 0.758 / 0.679 | 10/0/0 | 8/0/2 |
| precision (final) | 0.117 / 0 | **1 / 1** | 0.952 / 0.850 | 10/0/0 | 7/0/3 |
| mean IoU | 0.344 / 0.173 | 0.621 / 0.597 | **0.640 / 0.597** | 7/1/0 (8 pairs) | 3/7/0 |
| min IoU | 0.268 / 0.108 | 0.312 / 0.116 | **0.375 / 0.304** | 4/4/0 | 3/7/0 |
| neighbours swallowed | 29.3 / 41 | **2.8 / 5** | 5.3 / 8 | 10/0/0 | 8/1/1 |
| split trees | 0.1 / 1 | 0.1 / 1 | 0.2 / 1 | 1/1/8 | 2/1/7 |
| false trees | 18.7 / 26 | **0 / 0** | 0.8 / 3 | 10/0/0 | 5/0/5 |
| centre RMSE m | 2.097 / 3.775 | **0.754 / 0.885** | 0.875 / 1.247 | 7/1/0 | 7/3/0 |
| height RMSE m | 0.859 / 1.278 | 0.405 / 0.595 | 0.420 / 0.581 | 7/1/0 | 7/3/0 |
| spill | 0.807 / 0.866 | 0.233 / 0.283 | 0.238 / 0.306 | 10/0/0 | 4/6/0 |
| centre / height jumps | 24 / 2.9 | **0 / 0** | 3.7 / 0 | 10/0/0, 9/0/1 | 9/0/1, 0/0/10 |
| **safety violations** | 160.5 / 331 | **0 / 0** | 0 / 0 | 10/0/0 | 0/0/10 |
| truth trees sprayed | 1.6 / 0 | 5.5 / 4 | 5.8 / 5 | 10/0/0 | 3/5/2 |
| scan latency s | **0.800** / 1.373 | 2.510 / 4.604 | 2.655 / 5.365 | 0/10/0 | 4/6/0 |

(In the jumps rows, the first figure of each cell is centre jumps, the second height jumps, both means; the paired
W/L/T are given in that order.)

Reading: cand_obj and cand_grid are both far ahead of the simple baseline. Between the two candidates, medium is within seed
noise (|t| < 2 on every row except centre jumps, `results/cand_obj_m4_holdout.md`); cand_grid keeps a tighter height tail on
medium (0 of 175 matched trees off by > 1 m vs 5 of 175) and a better hard IoU; cand_obj wins the hard detection worst cases
and has the stable descent reference (target height range during its own descent 0 m on every mission; cand_grid up to 0.49 /
1.60 m medium / hard). Tuning-suite results (suite_v1, suite_v2) are in `results/cand_obj_m4_synthetic.md`; they are not
comparable across suites (D40).

## 4. Real flight (no ground truth): stability, safety, tree count and fragmentation

Source: `results/real_flight_scorecard.md` (`BB_treehopping_farm`, 511 s, verified camera chain D49/D50, constant SCAN phase;
cand_obj M4 = the shipped sim profile, core 960ccb7). The stereo shows about 60-63 crowns (63 stereo crown peaks, D51);
"outlines with 0 peaks" are fragments or false trees, "2+ peaks" are merged crowns.

| Metric | simple | **cand_obj (sim profile)** | cand_grid | existing |
|---|---|---|---|---|
| final trees / IDs ever published | 59 / 67 | 102 / 115 | 55 / 80 | 54 / 77 |
| outlines with 0 / 1 / 2+ crown peaks | 1 / 53 / 5 | **39** / 61 / 2 | 6 / 42 / 7 | 4 / 42 / 8 |
| crown peaks in no outline (missed) | 2 | 3 | 5 | 5 |
| outline area m^2 p50 (p90) | 67.57 (92.17) | 30.64 (44.86) | 51.47 (110.56) | 41.13 (73.02) |
| nearest-neighbour centre distance m p10 | 7.08 | **3.15** | 5.76 | 7.02 |
| geometry retreats / ID reuse / reappearances | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| centre jumps / height jumps (> 1 m) | 58 / 6 | **0 / 0** | 34 / 2 | 75 / 296 |
| IDs vanished (merges) | 8 | 13 | 25 | 23 |
| undefined-height / invalid / duplicate-ID frames | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| implausible heights: <= 0 / above the drone | 0 / 57 | 0 / 59 | 0 / 59 | **3232** / 0 |
| cost ms / frame mean (p95) | 0.547 + 0.33 | 0.981 (1.215) + 0.15 (0.287) | 0.846 + 0.668 | n/a |

- **Stability and safety** hold on real stereo for cand_obj: no retreats, no reuse, no jumps, every height finite.
- **The "above the drone" counts are a landing artefact** of a phase-less stream (t > 500 s, drone below 7.5 m over 7-8 m
  crowns; D51), the same for all our systems. The existing system's 3232 frames publish heights <= 0 m (its median feature
  height at or below the take-off point), which would be a descent to ground level if navigation used it.
- **Fragmentation is the one real-data failure** the synthetic and Isaac results did not predict: 6-9 m^2 pieces at crown
  edges and in the gaps between touching crowns. cand_obj has the tightest per-crown outlines and the fewest merged crowns, but
  ~40 surplus fragments. The cause is crown roughness: a real canopy has several local maxima more than 0.6 m above their
  saddles. The fix is a real-stereo configuration profile (section 6).
- The M3 cand_obj build behaves the same on this flight (100 trees, 37 fragment outlines) but jumps (42 centre / 39 height).

<!-- REAL_PROFILE_SECTION -->

## 7. Safety counts (all must be 0)

| Data | cand_obj | existing | simple | cand_grid | source |
|---|---|---|---|---|---|
| Isaac recording (undefined heights, new trees after scan, non-target descent changes, sprayed changes, new inside sprayed, invalid, duplicate, implausible, locked ID vanished) | **0** | 5 (1 sprayed change, 1 locked target vanished, 3 non-target descent changes) | 0 | 0 | `results/cand_obj_m4_isaac.md`, `results/baseline_existing_system.md` |
| Held-out suite, 30 missions | **0** | n/a | 1670 frames (65 medium + 1605 hard, implausible heights) | 0 | `results/cand_obj_m4_holdout.md` |
| Real flight (undefined / invalid / duplicate) | **0** | 0 | 0 | 0 | `results/real_flight_scorecard.md` |
| Real flight, heights <= 0 | 0 | 3232 frames | 0 | 0 | same |

The descent, lock and spray rules can only be judged where phases exist (synthetic and Isaac); on the real flight they are n/a.

## 8. Latency and cost

| | cand_obj | existing | simple | cand_grid | source |
|---|---|---|---|---|---|
| scan latency mean s, Isaac | 2.319 | 1.190 | 2.386 | 2.297 | `results/cand_obj_m4_isaac.md` |
| scan latency mean s, held-out easy / medium / hard | 1.142 / 2.024 / 2.510 | n/a | 0.564 / 1.178 / 0.800 | 1.400 / 2.383 / 2.655 | `results/cand_obj_m4_holdout.md` |
| mapper cost ms / frame mean (worst snapshot max), held-out easy / medium / hard | 1.018 (21.1) / 1.049 (18.8) / 0.960 (2.4) | n/a | 0.546 / 0.494 / 0.432 | 0.851 / 0.964 / 1.017 | same |
| mapper cost ms / frame, Isaac | 0.860 (max 1.118) | n/a | 0.615 | 0.580 | `results/cand_obj_m4_isaac.md` |
| node total per frame (ROS, Isaac) | 0.84 mean, 1.5 p99, 6.7 max | | | | `results/ros_replay_isaac.md` |

Latency is cand_obj's main price: a tree publishes only after 3 frames with a matched crown, 3 latched cells with a fused
height and 3 m^2 of latched cells, each cell needing 5 consistent frames, with crown tops at least 24 px inside the image.
On the recording that is about 1.1 s later than the existing system (latency is an upper-bound measure, D15).
Cost is about 1 ms per frame on x86, nearly all of it in the `chm` stage (`instances` ~0.01 ms on Isaac); the worst-snapshot maxima of ~20 ms on easy / medium are
single snapshots (the mean stays ~1 ms). Assume several times that on the Orin (unmeasured); memory is bounded (~14 kB per
live instance, `docs/candidates/cand_obj.md` section 10.8).

## 9. Noise floor

- **Run-to-run: 0.** Generator, replay and mappers are deterministic (D26). M5 re-ran the shipped system on the recording in a
  fresh build: the map stream (without the wall-clock `stage_ms`), targets and counters are byte-identical to M4a's committed
  run, and every scorecard metric is equal (section 11).
- **ROS path**: not bit-deterministic (message timing): mapper counters differ by 0.1-1 % run to run and the ID allocator once
  ended one tentative instance apart; IDs never changed meaning within a run (`results/ros_replay_isaac.md`).
- **Seed-to-seed** is the real floor, hence every comparison is paired by seed. Std over the 10 held-out datasets for cand_obj
  (`results/final_holdout_vs_simple.md`), easy / medium / hard: recall 0 / 0.053 / 0.044, mean IoU 0.008 / 0.026 / 0.021,
  centre RMSE 0.033 / 0.115 / 0.087 m, height RMSE 0.011 / 0.170 / 0.090 m, spill 0.005 / 0.014 / 0.033, scan latency
  0.042 / 0.724 / 0.980 s. A paired |t| < 2 is treated as seed noise throughout.
- **Isaac and the real flight are one sample each**; the existing system was recorded once, so no floor exists for it.

## 10. Known weaknesses

1. **Real-stereo fragmentation** with the sim profile (section 4); the real profile is tuned on one flight without ground
   truth (section 6).
2. **Latency** 2.32 s vs the existing system's 1.19 s on the recording; up to 4.6 s on the worst hard seed.
3. **Recall on hard noise** 0.83 (worst 0.75); M4 lost 0.018 / 0.035 recall on medium / hard against M3 in exchange for 0
   jumps and frozen descent references (D39).
4. **Height tail on medium**: 5 of 175 matched trees off by > 1 m (all over-estimates, the safe direction), cand_grid 0
   (`results/cand_obj_m4_holdout.md`). Isaac heights cannot be judged (truth is a constant fallback); real heights have no truth.
5. **Re-fusion in a second scan**: heights are fused again when the mission re-enters SCAN (max drift since first lock 0.82 /
   1.21 m on medium / hard synthetic). Inside every descent the target height is constant.
6. **Whole of HOPPING is frozen for non-targets**, stricter than the brief (DESIGN section 4): transit views do not improve
   neighbours.
7. **One ground plane per frame**; strongly undulating terrain is untested.
8. **Everything hardware-related is unverified**: Orin build and timing, the real camera chain live (mount 82 vs 75 deg,
   disparity offset 4.30 vs 4.918 px), detector pixel frame (raw vs rectified, up to ~13 px), navigation integration.

## 11. Verification done for the handoff (M5)

Run by M5 on 2026-10-24 in tm_dev (x86_64, Ubuntu 20.04, Noetic, g++ 9.4; at most 2 cores), core at commit 8dac005 (no
core change since). Logs: `/data/work/m5/`.

| Check | Command | Result |
|---|---|---|
| all tests, fresh build, all candidates | `TM_BUILD=/data/work/build_m5 TM_JOBS=2 ./scripts/run_tests.sh` (empty build dir) | **ALL TESTS PASSED**: C++ build clean (no warnings printed), ctest 1/1 (`test_core`, 63 gtest cases), pytest 102 passed |
| ROS package build | fresh catkin workspace `/data/work/catkin_m5` (symlinked `treemapper_msgs`, `treemapper_ros`), `catkin build -j2` + `--make-args tests` | all packages succeeded, no warnings in the log |
| ROS gtests | `devel/lib/treemapper_ros/test_treemapper_ros` | **27 / 27 passed** |
| ROS end-to-end smoke | `rostest treemapper_ros replay_smoke.test` (60 s of the Isaac bag at rate 1.0) | **SUCCESS** (1 test, 0 errors, 0 failures) |
| committed Isaac scorecard reproduces | `TM_BUILD=/data/work/build_m5 TM_OUT=/data/work/eval/m5_cand_obj TM_SKIP_SYNTH=1 ./scripts/evaluate.sh cand_obj` | every scorecard metric equal to M4a's committed run (`results/cand_obj_m4_isaac.md`, raw `/data/work/eval/m4a/final/isaac.json`); map stream (without `stage_ms`), `targets.json` and `counters.json` byte-identical; only wall-clock cost differs (chm 0.85 ms mean this run) |

<!-- VERIFICATION_REAL -->

## 12. How the decision was reached

| Step | What | Evidence |
|---|---|---|
| M1 | Contract, bag extraction, scorer; **baseline scorecard of the existing system** on the recorded Isaac mission. An independent review then fixed 13 scorer defects that let a system look better than it is (R1-R13); the existing system went from 1 to 5 safety violations. | `results/baseline_existing_system.md`, `results/baseline_existing_system_notes.md`, D1-D18, R1-R13 |
| M2 | Synthetic orchard generator, noise calibrated against both recordings and made 1.5-5x harder; a **deliberately simple baseline** (raw box hulls, grow-only). Already 0 safety violations on Isaac, but it merges touching trees and invents trees as noise rises. | `results/synthetic_calibration.md`, `results/simple_baseline_isaac.md`, `results/simple_baseline_synthetic.md`, D19-D29 |
| M3 | **Two genuinely different candidates**: `cand_obj` (per-instance crowns + per-cell evidence latching) and `cand_grid` (one fused 2.5D DSM + persistence watershed + latched instances), developed in parallel behind one Mapper interface, tuned on suite_v1 + the recording. | `docs/candidates/cand_obj.md`, `docs/candidates/cand_grid.md`, `results/cand_obj_{isaac,synthetic}.md`, `results/cand_grid_{isaac,synthetic}.md` |
| M3 selection | **Held-out comparison** (30 new missions + the recording, frozen configs, paired by seed). Safety tie (0 / 0); medium within noise; cand_obj chosen on hard worst cases (recall +0.108, 9/1), a stable descent reference on the recording (cand_grid's locked heights sank by up to 1.75 m), easy / Isaac outline quality, latency, bounded memory. Conceded to cand_grid: publication stability and the height tail, which became the M4 work list. A scorer fix (R14) was needed first. | `results/m3_comparison.md`, D30, D31, R14 |
| M4a | **Hardening of cand_obj**, carrying cand_grid's strengths over: latched-surface height quantile, heights frozen outside creation phases, rate-limited centre / height, range / off-axis / rotation weighting, overlap matching + seed guard. Held-out run once at the end: jumps 0 everywhere, IoU and centre better, recall slightly lower. | `docs/candidates/cand_obj.md` section 10, `results/cand_obj_m4_holdout.md`, `results/cand_obj_m4_isaac.md`, `results/cand_obj_m4_synthetic.md`, D32-D39 |
| M4d | Generator fix (the descent hover cleared only the target) and a new tuning suite suite_v2; held-out suites untouched. | D40 |
| M4b | **ROS 1 wrapper** + launch files for sim and real; ROS path reproduces the offline replay on the full Isaac mission. | `ros/treemapper_ros/README.md`, `results/ros_replay_isaac.md`, D41-D47 |
| M4c | **Real flight**: frame reader, camera chain verified from data (82 deg mount, 4.30 px offset), no-GT scorecard of all systems; found the over-fragmentation and tuned a real profile. | `docs/REAL_FLIGHT.md`, `results/real_flight_geometry.md`, `results/real_flight_scorecard.md`, D48-D54 |
| M5 | Handoff: README, this report, design document, consolidated decision log; verification in a fresh build. | D55-D57 |

## 13. Index of result files

| File | Content |
|---|---|
| `baseline_existing_system.{md,json}`, `baseline_existing_system_notes.md` | the existing system on the recording; old -> new scorer |
| `geometry_check.{md,json}` | Isaac camera mount verified against truth |
| `synthetic_calibration.{md,json}` | synthetic noise anchored to the recordings |
| `simple_baseline_{isaac,synthetic}.{md,json}` | M2 baseline |
| `cand_obj_{isaac,synthetic}.{md,json}`, `cand_grid_{isaac,synthetic}.{md,json}` | M3 candidates (tuning data) |
| `m3_comparison.{md,json}` | M3 held-out selection |
| `cand_obj_m4_{holdout,isaac,synthetic}.md` | M4 hardened cand_obj vs M3, cand_grid, simple (and existing on Isaac) |
| `final_holdout_vs_simple.md` | shipped cand_obj: per-level card with seed spread, paired vs simple (M5 extract) |
| `ros_replay_isaac.{md,json}` | ROS path vs offline replay |
| `real_flight_geometry.{md,json}`, `real_flight_stereo_check.json` | real camera chain |
| `real_flight_scorecard.{md,json}`, `real_flight_crowns.json`, `real_flight_plots/` | real flight, all systems, no GT |
| `real_flight_profile.md` (+ M4c's sweep files) | the real-flight configuration profile |
