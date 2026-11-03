# Fewer missed trees: TreeMapper height and centre fixes, navigator retry, re-measured comparison

Status: design, 2026-11-03. Follows `2026-10-30-corridor-spray-nav-design.md` and its results
(`results/corridor_nav_isaac.md`, branch `spec/corridor-spray-nav`). Build on that branch or a branch cut from it.

## 1. Why

The Isaac comparison (12 missions: 5x5_tropical and rows_8x12, corridor and field_first, 3 runs each) missed 35 of
666 tree visits. The navigator is not the cause: during a dose the drone sat within 0.043 m of the map centre it aimed
at in 95 % of 675 doses. 33 of the 35 misses trace to two behaviours of TreeMapper (`cand_obj`), and 3 to a navigator
gap:

| Cause | Misses | Where it comes from |
|---|---|---|
| Map centre further from the true centre than the 0.80 m spray footprint | 24 | Trees first confirmed at the far edge of the image keep a centre biased toward the track they were first seen from (5x5: mean error 1.04 m for entries first published ≥ 5 m across track, 59 % beyond the footprint; 0.32 m for entries first published closer). |
| Published height far too high | 6 | The published height climbs at exactly `max_height_rate` (0.5 m/s) after the drone has passed the tree, and the lock freezes it. Nozzle 5–6.3 m above the true top, footprint wider than the crown. 18 doses > 1.5 m too high in corridor mode, 3 in field_first. |
| Navigator skip, `target_height_implausible` | 3 | Same height ramp pushed the tree past the plausibility limit; the navigator never revisits it after the height comes back down. |
| Merged entry / never published | 2 | Not addressed here. |

Corridor mode, the point of the earlier design, came out 10–18 % slower than field_first with no measurable spray
benefit. The height ramp hurts corridor mode more (it locks each corridor right after scanning it), so the strategy
comparison is not settled until the map is fixed. This work fixes the map, closes the navigator gap, and measures again.

## 2. Goals

1. **Height:** TreeMapper's published height no longer runs away from the true canopy top after the tree leaves the
   good part of the view.
2. **Centre:** a tree first confirmed at the image edge ends up with a centre as good as one first confirmed near
   nadir, once the drone has seen it from closer.
3. **Navigator retry:** a tree skipped as implausibly high gets one more chance in cleanup once its published height is
   plausible.
4. **Re-measure:** the same 12-mission comparison, re-run with the fixes, written up next to the old numbers, with a
   recommendation on the default strategy.

### Acceptance (numbers are the designer's call; see section 9)

Replay (section 4.1) is deterministic, so replay criteria have no noise floor. Closed-loop criteria use the noise-floor
rule of the earlier results (a difference smaller than the larger 3-run spread is "no measurable difference").

| # | Criterion | Today |
|---|---|---|
| A1 | Replay, all four datasets: at the lock and at each dose, no published height of a tree that has a truth match is more than 1.5 m above its true top | 21 doses > 1.5 m over (closed loop) |
| A2 | Replay: the under-estimate side does not get worse — the share of matched entries more than 0.3 m below their true top, and the worst under-estimate, are each ≤ today's value on the same dataset | measured in step 4.1 |
| A3 | Replay, 5x5 datasets: entries first published ≥ 5 m across track have mean centre error at their dose ≤ 0.5 m and ≤ 15 % beyond 0.80 m; entries first published < 5 m are not worse than today | 1.04 m, 59 % |
| A4 | Closed loop, 12 missions: total misses ≤ 12 of the visits | 35 |
| A5 | Closed loop: every mission safety row passes in every run (min true clearance > 0, nozzle open outside DOSE 0, sprayed tree re-targeted 0) | all pass |
| A6 | TreeMapper's existing evaluation shows no regression beyond its noise floor (section 6) | — |
| A7 | Navigator: every `target_height_implausible` skip whose height later becomes plausible is retried once in cleanup (counted) | never retried |

If A1–A3 cannot be met after a real diagnosis, stop and report what was found with evidence; do not loosen the
criteria and do not move to the closed-loop re-run with a half fix.

## 3. Non-goals and anti-goals

Non-goals (separate work):
- **Pre-dose re-centring over the crown** (the drone centring itself on the crown it sees at spray altitude). It is the
  fallback if the centre fix (5.2) cannot reach A3; it needs its own design because it is new flight logic on the
  safety path.
- **Making the clearance guard act in Isaac** (published heights biased low). Still required before any real flight;
  not part of this work.
- Orin builds, real flight, ArduPilot, other orchard seeds or layouts, changing SGBM (`num_disparities` stays 64).
- The merged-entry and never-published misses (2 of 35).
- Changing the shipped default `strategy`. The results recommend; the user decides.

Anti-goals (a result with any of these is a failure even if the miss count drops):
- **Heights fixed by pushing them low.** A height below the true top is the unsafe direction for a descent (the
  original spec's main risk). Lowering `height_quantile`, raising the rate limit, or clipping heights to an expected
  canopy are not fixes for A1. A2 guards this.
- **Breaking TreeMapper's non-negotiable rules** (`docs/PROJECT_BRIEF.md` section 7): stable IDs, no silent damage to
  published geometry (outlines never shrink, jump or deform from a few frames), a real height once published, one tree
  one entry, sprayed trees frozen, fail loudly, every drop counted. A fix that needs a rule's wording changed is out.
- Passing by changing a scorer definition, a mission-score threshold, or a navigator safety parameter.
- Regressing the real-flight profile (`config/cand_obj_real.cfg` runs the same code).

## 4. Design

### 4.1 Replay dataset (first; everything else is measured on it)

The 12 comparison bags do not contain the disparity images TreeMapper consumes, so the map cannot be replayed from
them. TreeMapper already has an offline path: `tools/treemapper_tools/bag_frames.py` turns a bag into a TMF1 frame file,
and `treemapper_replay` (`core/apps/replay_main.cpp`) drives `cand_obj` from it and writes the map stream; replay vs the
ROS node was shown equivalent in `results/ros_replay_isaac.md`.

- `scripts/sim_mission.sh` gets `--record-frames`: adds to the bag allow-list exactly the topics `bag_frames.py`
  needs (the downsampled float disparity TreeMapper consumes, plus whatever else it reads), off by default. After the
  run the harness converts the bag to `<run>/frames.bin` and checks it (frame count, pose coverage, no malformed
  disparity).
- Record **one run of each spec × strategy** (4 Isaac runs) with the current branch code. These four frame files are
  the replay dataset. A 5x5 frame file is ~0.75 GB; mind the disk (14 GB free on the work drive, hard floor 8 GB in the
  harness): a bag with disparity is kept only until its frame file is converted and checked.
- **Baseline replay:** run unchanged `cand_obj` on the four frame files and confirm it reproduces the closed-loop
  symptoms for the same trees (the height ramp on the entries named in the results appendix, the far-sighted centre
  bias). Record the baseline metrics; they are the "today" values for A1–A3.
- **Replay metrics script** (extend `sim_compare_report.py` or `score.py`, whichever is cleaner; pytest on a
  synthetic fixture): per matched entry, height error and centre error at the lock (first PLANNING) and at each dose
  of that entry, first-sighting across-track offset, and the A1–A3 summaries. Truth from the run's `truth.json`.
  Replay is open loop (the recorded trajectory is fixed), which is fine: it measures the map, and the closed loop is
  measured again in 4.4.

### 4.2 Height fix (`cand_obj`)

What is known: `refreshHeight` sets `height_target` to the `height_quantile` (0.99) of the latched cells' fused
surface; `stepPublished` walks `height_pub` toward it at `max_height_rate`. A published height that climbs at exactly
0.5 m/s while the tree is 10–25 m behind the drone means `height_target` jumped far above the canopy earlier (to ~9–10 m
on 2–3 m trees) and the rate limiter is chasing it. The rate limiter is not the bug; the target is.

What is not known: which cells carry the high fused heights and from which views. Designer's guesses, to be confirmed
or discarded by the diagnosis, not built on: points accepted up to `z_cap` (drone z minus a margin, ~13 m at scan
altitude) from oblique views at the image edge; mismatched disparity on thin crown edges against the textured ground;
the outlier gate only applying once a cell has weight ≥ `gate_min_weight`.

Required:
1. Diagnose on the replay dataset with per-cell evidence (add counters or a debug dump, not permanent log spam): for the
   ramping entries, which cells drive the quantile up, from which frames, at what range and off-axis angle.
2. Fix the cause in `cand_obj`. The height must still come from what is measured; S8 (no publication from truncated
   crowns only) stays.
3. A test per rule in `core/tests/candidates/cand_obj/test_cand_obj.cpp` (synthetic crowns reproducing the failure:
   fails before, passes after).
4. New tunables follow the existing config rules (key in `cand_obj.cfg` and `cand_obj_real.cfg`, `kConfigKeyCount`,
   range clamp, defaults test). Decision recorded in `docs/DECISIONS.md` (next D-number) with the evidence.

### 4.3 Centre fix (`cand_obj`)

What is known: the published centre is the mean of the latched cells' centres (`refreshHull`), rate-limited to
`max_centre_rate`; latching is permanent and the outline only grows (rule 2). In 43 of 44 bad cases the error points
back toward the track the drone flew when the entry was first published, 89 % of it across track. Closer passes
(3–4 m, the tree's own corridor) do not pull it back far enough. The effect is much weaker on rows_8x12, not yet
explained.

Designer's guess (to be confirmed by diagnosis): cells latched from the far, oblique first view sit offset toward the
drone and, being permanent and equally weighted, keep the centroid there.

Required:
1. Diagnose on the replay dataset which latched cells pull the centre and from which views; say why rows_8x12 shows
   it less.
2. Fix so the centre converges to the crown once near views exist. The centre and the outline are separate published
   quantities: the centre may be estimated differently from the outline (for example weighted by observation quality
   or from near-nadir evidence) as long as the published outline still obeys rule 2 and the centre still moves at no
   more than `max_centre_rate`. Unlatching cells, if the diagnosis points there, must not shrink a published outline
   from a few frames.
3. Tests and DECISIONS.md entry as in 4.2.

### 4.4 Navigator retry of implausible-height skips (`nav/corridor_nav`)

Today a target whose published height exceeds `expected_canopy_top_max + target_height_margin` (above home) is skipped
and counted `target_height_implausible`, and never revisited. Change:
- The skipped id goes onto the cleanup list with reason "implausible height".
- When cleanup reaches it: if the entry still exists, is unsprayed and its current published height passes the same
  plausibility rule, it is targeted once through the normal glide and dose path; else it is dropped. At most one retry
  per tree.
- Counters: `implausible_retried`, `implausible_recovered` (dosed), `implausible_dropped`. NavStatus and the scorecard
  show them.
- Re-resolution of a vanished id follows the existing rules (and the existing plausibility check).
- Core gtests with FakeWorld: a tree whose height is implausible at its turn and plausible later is dosed once in
  cleanup; a tree still implausible is dropped and counted; no tree is ever dosed twice.

This part is independent of 4.2–4.3 and can be built first.

### 4.5 Re-measure the comparison

With 4.2–4.4 in:
1. Re-run the same 12 missions: same specs and seeds, `--runs 3`, same harness settings (`--max-rtf 1.0`,
   `--v-max-xy 4.0 --v-max-z 1.5`, generated cfg with `carrot_lead_s 1.2`), into new run directories (the old ones stay).
2. Regenerate the tables with `sim_compare_report.py`. In `results/corridor_nav_isaac.md` add a section "After the map
   fixes" with before/after per spec × strategy (efficiency, spray, safety, map), the miss classes again, and A4/A5.
3. **Strategy recommendation**, by this rule: if corridor mode's spray outcome is better than field_first's beyond the
   noise floor, recommend corridor and say what it buys; if not, recommend field_first as the default and say what
   corridor mode would need to earn its 10–18 % extra time. The shipped default is not changed in this work.
4. Keep "Not verified on the Orin or on a real flight" in the doc.

## 5. Constraints

- Build and test in one-shot containers as the earlier work did (`scripts/run_tests.sh` header; `TM_ROS_TESTS=1` for
  the ROS block); Isaac runs through `scripts/sim_mission.sh` only. Container names start `tm_`; never touch other
  containers. `/media/alex/waspsim/cloud_build_bundle/{isaac,real}` and the tree asset folder are read-only.
- Disk: work drive ~14 GB free; the harness refuses below 10 GB and aborts below 8 GB. Budget the four frame-recording
  runs (4.1) and the 12 re-runs (4.5, ~1.1 GB) against that.
- Determinism: replay must stay deterministic (same frames → byte-identical map stream), as D65 established.
- Navigator cfg stays equal to the struct defaults (existing test); new navigator counters only, no new safety knobs.

## 6. TreeMapper regression check (A6)

`cand_obj` is the production mapper for real flights too. Before and after 4.2–4.3, run TreeMapper's existing
evaluation and compare beyond its noise floor: the synthetic suite (`scripts/evaluate.sh`, suite_v2), the holdout
(`results/cand_obj_m4_holdout.md`), the Isaac replays (`results/cand_obj_m4_isaac.md`) and the real-flight scorecard
(`results/real_flight_scorecard.md`, profile `cand_obj_real.cfg`). A regression on any of them is either fixed or, if
it is the price of the fix, stated with numbers in DECISIONS.md and the results, and raised to the user before 4.5.

## 7. Order

1. 4.4 navigator retry (independent; small).
2. 4.1 replay dataset + baseline metrics.
3. 4.2 height diagnosis and fix, then 4.3 centre diagnosis and fix (same file; one after the other).
4. Section 6 regression check.
5. 4.5 closed-loop re-run and write-up.

## 8. Left to the builder

- The mechanism of both TreeMapper fixes, after the diagnosis.
- Where the replay metrics live and how they are structured.
- How the harness converts and checks the frame file.
- Whether 4.4's cleanup retry happens per corridor or at the end of the mission, as long as each tree gets at most one
  retry and the counters are exact.

## 9. Calls the designer made (check these)

- **TreeMapper is in scope.** The earlier work kept it untouched; this one changes `cand_obj` because it causes 33 of
  the 35 misses.
- **Acceptance numbers A1–A4** (1.5 m height ceiling, 0.3 m under-estimate guard, 0.5 m / 15 % centre target, ≤ 12
  misses) are the designer's targets from today's numbers, not measured requirements.
- **Pre-dose re-centring is deferred**, as the fallback if the centre fix fails.
- **Same seeds and layouts** for the re-run, so before/after is comparable; more seeds stay out.
- **The default strategy is not changed** by this work; the results recommend.
- **One retry per implausible tree**, in cleanup.
