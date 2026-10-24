# Final-review fixes: re-score, mutation check, cost scaling

Run by the final-fix pass on 2026-10-24 in tm_dev (x86, at most 4 cores). **Before** = commit 90f1926 (cand_obj behaviour
unchanged since M4a / M5), **after** = this pass: sticky lock (S4), non-truncated height guard (S8), uniform-grid merge
candidates + reach-first matching (S5, output-identical). Scratch: `/data/work/final_fix/` (`eval/{base,new,s5}`,
`real/`, `bench/`, `mutation/`). Machine-readable: `results/final_review_rescore.json`. **The held-out suite was not
re-run** (its numbers in `results/cand_obj_m4_holdout.md` / `final_holdout_vs_simple.md` are from the M4a build, i.e.
"before"; S4 cannot change them, see below, S8 can).

## 1. Re-score on Isaac and suite_v2 (cand_obj, `config/cand_obj.cfg`)

### Recorded Isaac mission (used during tuning)

| metric | before (mean) | after (mean) |
|---|---|---|
| recall (final) | 1 | 1 |
| precision (final) | 1 | 1 |
| recall (per-frame mean) | 0.738 | 0.738 |
| mean IoU (final) | 0.867 | 0.867 |
| min IoU (final) | 0.817 | 0.817 |
| merged entries (final) | 0 | 0 |
| split trees (final) | 0 | 0 |
| false trees (final) | 0 | 0 |
| centre RMSE m (final) | 0.191 | 0.191 |
| height bias m (final) | n/a | n/a |
| height RMSE m (final) | n/a | n/a |
| spill (final) | 0.064 | 0.064 |
| centre jumps | 0 | 0 |
| height jumps | 0 | 0 |
| SAFETY total violations | 0 | 0 |
| scan latency mean s | 2.319 | 2.319 |

### suite_v2 (15 missions, tuning suite)

| metric | before (mean) | after (mean) | easy before / after | hard before / after | medium before / after |
|---|---|---|---|---|---|
| recall (final) | 0.954 | 0.954 | 1 / 1 | 0.886 / 0.886 | 0.976 / 0.976 |
| precision (final) | 0.987 | 0.991 | 1 / 1 | 0.984 / 0.984 | 0.978 / 0.988 |
| recall (per-frame mean) | 0.85 | 0.849 | 0.892 / 0.889 | 0.786 / 0.786 | 0.871 / 0.871 |
| mean IoU (final) | 0.772 | 0.773 | 0.905 / 0.906 | 0.621 / 0.621 | 0.791 / 0.791 |
| min IoU (final) | 0.584 | 0.586 | 0.857 / 0.863 | 0.361 / 0.361 | 0.533 / 0.533 |
| merged entries (final) | 1.067 | 1.067 | 0 / 0 | 2.8 / 2.8 | 0.4 / 0.4 |
| split trees (final) | 0.267 | 0.2 | 0 / 0 | 0.4 / 0.4 | 0.4 / 0.2 |
| false trees (final) | 0 | 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| centre RMSE m (final) | 0.426 | 0.426 | 0.141 / 0.14 | 0.729 / 0.729 | 0.407 / 0.407 |
| height bias m (final) | 0.055 | 0.055 | -0.012 / -0.012 | 0.097 / 0.097 | 0.079 / 0.079 |
| height RMSE m (final) | 0.155 | 0.155 | 0.054 / 0.054 | 0.224 / 0.224 | 0.187 / 0.187 |
| spill (final) | 0.151 | 0.151 | 0.05 / 0.05 | 0.25 / 0.25 | 0.154 / 0.153 |
| centre jumps | 0 | 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| height jumps | 0 | 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 0 | 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| scan latency mean s | 1.769 | 1.795 | 1.185 / 1.247 | 2.573 / 2.573 | 1.547 / 1.566 |
- **Isaac**: every scorecard metric unchanged. The map stream differs in 30 of 1410 snapshots, all on one tree (ID 6):
  the S8 guard holds back confirmation until a non-truncated crown has sampled the top (counter
  `instances_waiting_untruncated_height` = 6 frames), so its first published height is 4.29 m instead of 3.56 m
  (measured canopy tops are 5.4-5.9 m; the rate limiter then converges as before). It is published in the same snapshot.
  The sticky lock triggers for 15 frames (SCAN_APPROACH after the spraying, `frames_locked_after_planning`) and
  changes nothing there.
- **suite_v2**: final recall unchanged (0.954); precision +0.004 and split trees -0.07 (medium: one fragment fewer); per-frame
  recall -0.001; scan latency +0.026 s mean (easy +0.06 s). No synthetic mission enters ERROR / unknown / SCAN_APPROACH
  after the lock, so S4 does not act there; the differences are S8's.
- **S5 is output-identical**: the S5 build against the S4+S8 build, map streams without the wall-clock `stage_ms`,
  `targets.json` and `counters.json` compared byte for byte: Isaac 3 / 3 files identical, suite_v2 45 / 45, scorecards
  equal apart from the cost row (`/data/work/final_fix/cmp_streams.py`).

## 2. Real flight (no ground truth), `treemapper_replay` on `/data/work/extracted/real_farm/frames.bin`

| profile | before: final trees / IDs ever | after |
|---|---|---|
| real `cand_obj_real.cfg` (the real launch default) | 63 / 68 | 63 / 68, **map stream identical** (2559 snapshots, without `stage_ms`) |
| sim `cand_obj.cfg` | 102 / 115 | 101 / 111 (S8: 4 fewer IDs ever published, 1 fewer final tree; the sim profile seeds from truncated crowns) |

So `results/real_flight_profile*.md` / `real_flight_scorecard.*` stay valid for the real profile; the sim-profile row
of the real-flight scorecard is from the "before" build.

## 3. Mutation check of the rule tests (S6)

`scripts/mutation/run_mutants.sh` (mutants in `scripts/mutation/mutants.py`) breaks one rule per mutant in
`cand_obj.cpp`, rebuilds the cand_obj gtests and runs them. The five mutants the final review found surviving are now
killed:

| reviewer's surviving mutant | killed by |
|---|---|
| `insideSprayed` always false (rule 5: no new tree inside a sprayed tree) | `CandObjFinal.NoNewTreeConfirmedInsideASprayedTree` |
| sprayed trees not skipped in merges | `CandObjFinal.SprayedTreeNeverMerges` |
| merge winner not confirmed > tentative (inverted, or the rung removed) | `CandObjFinal.MergeWinnerIsTheConfirmedInstance` |
| heights fused in locked phases | `CandObjFinal.HeightsNotFusedInLockedPhases` |
| `markSprayed` accepts unpublished instances | `CandObjFinal.MarkSprayedRefusesUnpublishedInstances` |

Full run (20 mutants): 17 killed, 3 survive and are **equivalent** (the mutated check is redundant with another one, so no
test can tell): the seed-time `insideSprayed(c.peak)` (a crown whose top is inside a sprayed tree is matched to that tree
by `findInstanceForCrown` with the same hull / radius test and dropped first), `refreshHeight` in locked phases (no cell
height is fused there, so it is a no-op), and `pol.may_create` in the confirmation test (unconfirmed instances are not
updatable in locked phases). They stay as defence in depth. Also killed: merges in locked phases (new test
`CandObjFinal.NoMergeWhileTheListIsLocked`, which the old suite missed too), the sticky lock, the S8 guard, the NaN config
rejection, freeze, post-spray hold, rate limit, locked-phase creation / growth.

## 4. Per-frame cost against map size (S5)

The reviewer's benchmark (`Adv.CostScalingWithMapSize`: N published trees on a 6 m grid, then 50 frames seeing 6 crowns;
`integrate()` only, x86, -O2; `/data/work/final_fix/bench/`):

| N trees | before ms / frame | after ms / frame |
|---|---|---|
| 100 | 0.207 | 0.123 |
| 400 | 1.469 | 0.469 |
| 1600 | 18.591 | 1.292 |

Before, the merge sweep tested all N^2 / 2 pairs every frame. After, merge candidates come from a uniform grid on the
centres (cell = the 8 m reach), visited in the old order, and the crown matching tests the O(1) reach before counting cell
overlaps. What remains grows linearly (per-instance bookkeeping each frame), about 0.8 us per tree.
