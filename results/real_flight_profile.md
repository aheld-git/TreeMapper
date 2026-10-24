# cand_obj on real data: a separate real-flight profile (`config/cand_obj_real.cfg`)

**Problem.** On the real farm log the default (sim) profile over-fragments: 102 final trees where the stereo canopy
model shows about 63 crowns, and 39 outlines contain no crown peak (`real_flight_scorecard.md`).

**Constraint.** The cand_obj source is frozen, so this sweep is configuration only. It used build 960ccb7, the M4a
hardened cand_obj, on `/data/work/extracted/real_farm/frames.bin` (verified chain).

**Criteria (no ground truth):**
- outlines without a stereo crown peak (target ~0);
- fragment areas;
- outlines holding 2+ peaks (merges);
- peaks in no outline (misses);
- nearest-neighbour centre distance (the orchard spacing is ~8 m);
- stability: jumps, drift, vanished IDs;
- safety;
- overlap with the other systems' final trees, as a sanity check only.

The crown peaks (63) come from `real_flight_crowns.json`.

Command:
```
python3 -m treemapper_tools.real_profile_sweep [--round 2] --dataset /data/work/extracted/real_farm \
  --replay /data/work/eval/m4c/build_960ccb7/treemapper_replay --system cand_obj --crowns results/real_flight_crowns.json \
  --refs simple=<map>,cand_grid=<map>,existing=/data/work/extracted/real_farm/map_existing.jsonl --out /data/work/eval/m4c/sweep
```

## Sweep (18 configs: 12 in round 1 = M4a's knobs, 6 in round 2 = instance-level knobs on top of round 1's best)

| round | config (changes vs default) | trees | outlines w/o peak | outlines 2+ peaks | peaks missed | NN centre p10/p50 m | fragment area p50/max m2 | area p10/p50/p90 m2 | centre/height jumps | max centre drift m | IDs vanished | undef/invalid/dup | in simple / cand_grid / existing |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | `default` | 102 | 39 | 2 | 3 | 3.15 / 4.12 | 15.84 / 36.48 | 8.88 / 30.64 / 44.86 | 0 / 0 | 3.34 | 13 | 0/0/0 | 101/102 / 99/102 / 92/102 |
| 1 | `min_prominence=1.0` | 86 | 26 | 2 | 5 | 3.26 / 4.65 | 20.48 / 36.64 | 15.52 / 35.12 / 49.4 | 0 / 0 | 3.37 | 14 | 0/0/0 | 86/86 / 84/86 / 79/86 |
| 1 | `min_prominence=1.5` | 79 | 21 | 2 | 5 | 3.36 / 5.2 | 17.68 / 38.32 | 12.62 / 35.6 / 47.15 | 0 / 0 | 3.36 | 10 | 0/0/0 | 79/79 / 77/79 / 76/79 |
| 1 | `min_basin_cells=8` | 102 | 39 | 2 | 3 | 2.97 / 4.09 | 15.84 / 36.48 | 8.88 / 31.24 / 45.38 | 0 / 0 | 3.34 | 13 | 0/0/0 | 102/102 / 99/102 / 94/102 |
| 1 | `min_basin_cells=12` | 102 | 39 | 2 | 3 | 2.97 / 4.04 | 15.84 / 36.48 | 8.88 / 31.4 / 45.38 | 0 / 0 | 3.34 | 13 | 0/0/0 | 102/102 / 99/102 / 94/102 |
| 1 | `min_prominence=1.0; min_basin_cells=8` | 86 | 25 | 2 | 5 | 3.36 / 4.62 | 17.68 / 40.64 | 12.04 / 36.64 / 49.36 | 0 / 0 | 3.37 | 14 | 0/0/0 | 86/86 / 84/86 / 80/86 |
| 1 | `min_prominence=1.0; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5` | 92 | 29 | 1 | 6 | 3.38 / 4.3 | 19.76 / 40.64 | 13.74 / 34.84 / 46.77 | 0 / 0 | 3.37 | 8 | 0/0/0 | 92/92 / 90/92 / 89/92 |
| 1 | `min_prominence=1.5; min_basin_cells=12; match_radius=3.0; merge_centre_dist=2.5` | 82 | 22 | 1 | 6 | 3.5 / 4.7 | 15.64 / 40.64 | 12.04 / 36.48 / 47.21 | 0 / 0 | 3.35 | 8 | 0/0/0 | 82/82 / 80/82 / 80/82 |
| 1 | `min_prominence=1.0; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=6.0` | 88 | 25 | 1 | 6 | 3.42 / 4.64 | 21.52 / 40.64 | 17.1 / 35.24 / 47.09 | 0 / 0 | 3.06 | 7 | 0/0/0 | 88/88 / 87/88 / 86/88 |
| 1 | `min_prominence=1.0; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=8.0` | 82 | 21 | 1 | 7 | 3.56 / 4.98 | 24.4 / 40.64 | 19.89 / 35.44 / 51.97 | 0 / 0 | 2.69 | 4 | 0/0/0 | 82/82 / 81/82 / 81/82 |
| 1 | `min_prominence=1.5; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=8.0` | 77 | 16 | 1 | 6 | 3.64 / 5.57 | 27.2 / 40.64 | 19.14 / 37.76 / 51.78 | 0 / 0 | 2.84 | 6 | 0/0/0 | 77/77 / 76/77 / 75/77 |
| 1 | `min_basin_cells=12; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=8.0` | 88 | 24 | 1 | 4 | 3.4 / 4.72 | 20.12 / 51.2 | 16.4 / 34.92 / 48.52 | 0 / 0 | 2.53 | 8 | 0/0/0 | 88/88 / 87/88 / 86/88 |
| 2 | `min_prominence=1.5; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=8.0; merge_overlap=0.3` | 66 | 9 | 1 | 6 | 4.25 / 6.97 | 26.8 / 37.44 | 20.84 / 39.8 / 62.64 | 0 / 0 | 2.84 | 10 | 0/0/0 | 66/66 / 65/66 / 65/66 |
| 2 | `min_prominence=1.5; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=8.0; seed_truncated=false` | 77 | 15 | 2 | 4 | 3.52 / 4.84 | 24.0 / 38.32 | 19.46 / 37.68 / 49.04 | 0 / 0 | 2.48 | 5 | 0/0/0 | 77/77 / 76/77 / 73/77 |
| 2 | `min_prominence=1.5; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=8.0; seed_overlap=0.1` | 77 | 16 | 1 | 6 | 3.64 / 5.57 | 27.2 / 40.64 | 19.14 / 37.76 / 51.78 | 0 / 0 | 2.84 | 6 | 0/0/0 | 77/77 / 76/77 / 75/77 |
| 2 | `min_prominence=1.5; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=8.0; cell=0.6` | 70 | 8 | 1 | 7 | 3.69 / 6.97 | 29.61 / 32.76 | 25.13 / 38.79 / 48.53 | 0 / 0 | 3.10 | 7 | 0/0/0 | 70/70 / 69/70 / 69/70 |
| 2 | `min_prominence=1.5; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=8.0; merge_overlap=0.3; seed_truncated=false; seed_overlap=0.1; match_reach=4.5` | 61 | 5 | 2 | 5 | 5.84 / 7.65 | 29.84 / 34.72 | 24.16 / 38.32 / 46.32 | 0 / 0 | 3.22 | 5 | 0/0/0 | 61/61 / 60/61 / 60/61 |
| 2 | `min_prominence=1.0; min_basin_cells=8; match_radius=3.0; merge_centre_dist=2.5; min_publish_area=8.0; merge_overlap=0.3; seed_truncated=false; seed_overlap=0.1; match_reach=4.5` | 63 | 6 | 2 | 5 | 5.76 / 7.46 | 25.4 / 36.96 | 21.94 / 37.84 / 47.3 | 0 / 0 | 2.59 | 5 | 0/0/0 | 63/63 / 62/63 / 62/63 |

Above-drone height frames are 58-59 for every config: the landing artefact described in the scorecard.
Repeatability across passes is read here as stability, since cand_obj_m4 rate-limits every published centre:
- **centre jumps 0 in every config**;
- max centre drift from first publication 2.5-3.4 m (2.6 m for the chosen config).

**Reading:**
- M4a's suggested knobs (round 1) reduce the fragments but cannot remove them. `min_basin_cells` has no effect.
  `min_prominence` 1.5 plus `min_publish_area` 8 gets down to 16 fragments, but misses 6 crowns.
- The decisive knob is **`merge_overlap` 0.5 -> 0.3** (round 2). Real crowns are split into two or three instances
  that overlap by 30-50 %, and they only merge at the lower threshold.
- `seed_truncated=false`, `seed_overlap` 0.1 and `match_reach` 4.5 add a little: P_all gets 5 fragments
  against 9 with merge_overlap 0.3 alone.
- `cell` 0.6 also helps (8 fragments). It was not taken, because it changes the whole height model.

## Decision: `config/cand_obj_real.cfg` = row "P1.0_all"

It changes 9 keys from `config/cand_obj.cfg`:
- min_prominence 1.0
- min_basin_cells 8
- seed_truncated false
- match_radius 3.0
- match_reach 4.5
- seed_overlap 0.1
- min_publish_area 8.0
- merge_overlap 0.3
- merge_centre_dist 2.5

On the real flight (`real_flight_profile_scorecard.json`, plot `real_flight_plots/final_cand_obj_real.png`):
- **63 trees** for 63 crown peaks; 55 outlines with exactly one peak, 6 without, 2 with two;
- 5 peaks missed, against 3 with the default;
- nearest-neighbour centre distance p10 5.8 m (was 3.2 m), outline area p50 38 m2;
- 0 jumps and 0 safety counts besides the landing artefact; 5 IDs vanished (was 13).

P1.0_all and P_all are equivalent: P_all gives 61 trees and 5 fragments, P1.0_all 63 and 6. P1.0_all was chosen for
its lower prominence (fewer missed small trees on synthetic data) and its lower drift (2.6 vs 3.2 m).

**Which profile goes with which launch:**

| Use | Profile |
|---|---|
| `ros/treemapper_ros/launch/treemapper_real.launch` (real drone / real logs) | `config/cand_obj_real.cfg`: the default of its `mapper_config_file` arg, a one-line change (af11268); M4b's launch test 9100ccb checks that it resolves, parses and is accepted |
| `treemapper_sim.launch`, `replay_isaac.launch`, `treemapper.yaml`'s pointer, `scripts/evaluate.sh`, the built-in defaults | `config/cand_obj.cfg` (sim / default) |

## Does one profile serve both? No: the real profile regresses on the synthetic held-out suite

Same build (960ccb7). Both runs are ours, under `/data/work/eval/m4c/profile_{default,real}`.

| held-out suite (30 missions), mean / worst | default `cand_obj.cfg` | real `cand_obj_real.cfg` |
|---|---|---|
| recall (final) | 0.930 / 0.750 | 0.854 / 0.607 |
| precision (final) | 0.994 / 0.938 | 1 / 1 |
| mean IoU | 0.769 / 0.597 | 0.726 / 0.511 |
| merged entries / neighbours swallowed | 1.20 / 1.27 | 2.63 / 2.77 |
| split trees | 0.13 | 0.07 |
| centre RMSE m | 0.445 / 0.885 | 0.632 / 1.213 |
| height RMSE m | 0.225 / 0.595 | 0.334 / 1.288 |
| spill | 0.146 | 0.167 |
| safety violations | 0 / 0 | 0.67 / 20 (hard_s100: 20 frames with a height 0.65 m above the truth top and above a 7.35 m hover) |
| scan latency mean s | 1.89 / 4.60 | 2.47 / 7.52 |
| by level, recall: easy / medium / hard | 1.00 / 0.96 / 0.83 | 0.99 / 0.88 / 0.68 |
| by level, merged entries: easy / medium / hard | 0 / 1.0 / 2.6 | 0.1 / 2.6 / 5.2 |

| recorded Isaac mission (9 trees) | default | real |
|---|---|---|
| recall / precision | 1 / 1 | 1 / 1 |
| mean IoU | 0.867 | 0.855 |
| centre RMSE m | 0.19 | 0.21 |
| safety violations | 0 | 0 |
| scan latency mean s | 2.32 | **4.23** |

**Conclusion: two profiles are genuinely needed.**
- The synthetic medium and hard levels have touching canopies at 4.5-5 m spacing. There the lower `merge_overlap`
  and the larger match / merge radii merge real neighbours. On the real orchard, where crowns are about 8 m apart,
  the same settings repair the fragments.
- On the well-separated Isaac trees the real profile costs only latency: ~2 s more, from `min_publish_area` 8 and
  the stricter seeding.
- What separates the two cases is **tree spacing relative to crown roughness**. A spacing-aware merge rule in code,
  for example merging when the overlap exceeds a share tied to the estimated crown radius, could unify them. That
  is a code change, left for after the freeze.

**Unverified:**
- No ground truth on the real flight. "Fragment" means no stereo crown peak inside the outline (3 m peak
  separation), which is a proxy.
- The 5 missed peaks are not checked by eye one by one.
- The profile is tuned on this single log, one orchard and one flight. It is not validated on another real flight.
