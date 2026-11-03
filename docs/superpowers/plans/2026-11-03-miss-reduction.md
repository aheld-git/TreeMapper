# Fewer Missed Trees Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cut the corridor-spray misses in Isaac from 35 to ≤ 12 of the visits in the 12-mission comparison. The fixes are TreeMapper's runaway height and image-edge centre bias, plus a navigator retry of trees skipped as implausibly high. Then re-measure corridor vs field_first.

**Architecture:**
- The navigator change is local to `Navigator::advance` and its counters.
- The TreeMapper fixes are developed offline, on frame files recorded from four new Isaac runs and replayed with the existing `treemapper_replay`. A new replay-metrics module scores them against Isaac truth.
- The closed loop is run only twice: once to record the replay dataset and once to re-measure at the end.

**Tech Stack:**
- C++17 (`nav/corridor_nav`, `core/` cand_obj, gtest/ctest)
- Python 3.8 (`tools/treemapper_tools`, pytest)
- ROS Noetic (rosbag)
- Isaac Sim 5.1 through `scripts/sim_mission.sh`

**Spec:** `docs/superpowers/specs/2026-11-03-miss-reduction-design.md` (read it first; acceptance A1–A7 are defined there).

## Global Constraints

**Branch and commits**
- Work on `spec/corridor-spray-nav` (or a branch cut from it). Never push.
- Commits end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- Use explicit `git add` only. Never stage `.claude/settings.json`, `.vscode/` or `results/isaac_5x5_tropical.*`.

**Running builds, tests and missions**
- Every build and test runs in a one-shot container:
  `docker run --rm --name tm_<purpose> --entrypoint bash --cpus 8 --memory 16g --user 1000:1000 -e HOME=/data/work/home -v /home/alex/SUIND/TreeMapper:/ws -v /media/alex/waspsim/cloud_build_bundle/isaac:/data/isaac:ro -v /media/alex/waspsim/cloud_build_bundle/real:/data/real:ro -v /media/alex/waspsim/cloud_build_bundle/work:/data/work treehoppingnavigation-bridge:latest -lc 'source /opt/ros/noetic/setup.bash && cd /ws && <cmd>'`.
  The full suite is `TM_ROS_TESTS=1 ./scripts/run_tests.sh` and must end with `ALL TESTS PASSED`.
- Isaac runs go through `scripts/sim_mission.sh` only.
- Container names start with `tm_`. Never stop, restart or exec into `wasp_sim`, `tree_hopping_sim`, `bridge`, `ros2_docker` or `mlflow`.
- `docker rm`, `kill`, `image`, `volume`, `system` and `compose`, and `rsync`, are not used.
- `/media/alex/waspsim/cloud_build_bundle/{isaac,real}` and the tree asset folder are read-only.
- The old run directories `/data/work/sim_runs/cmp16c_*` are read-only. New runs go to new directories.

**Disk**
- The work drive has about 14 GB free. The harness refuses below 10 GB and aborts below 8 GB.
- A bag with disparity is deleted only after its `frames.bin` has been converted and checked.

**Spec rules that bind every task**
- Never lower heights to pass A1 (spec §3 anti-goals). That means no lowering of `height_quantile`, no raising of `max_height_rate`, and no clipping to an expected canopy.
- Keep all seven rules of `docs/PROJECT_BRIEF.md` §7.
- Do not change a scorer definition, a mission-score threshold or a navigator safety parameter.
- The navigator cfg file stays equal to the struct defaults.
- New TreeMapper tunables follow the config rules:
  - the key goes in both `config/cand_obj.cfg` and `config/cand_obj_real.cfg`;
  - update `kConfigKeyCount`;
  - add a range clamp in `sanitized()`;
  - the defaults test must pass.
- Every TreeMapper behaviour change gets a `docs/DECISIONS.md` entry (next free D-number after D65) with its evidence.
- Replay stays deterministic: the same `frames.bin` and config give a byte-identical map stream, ignoring `stage_ms`.

## Review Focus

1. **Never retrieved:** in field_first nothing updates heights after the single lock, so a tree skipped as implausible usually stays implausible. Cleanup must drop it with `implausible_dropped`, with no loop and no extra glide. Test: Task 1 `ImplausibleStillHighInCleanupIsDropped`.
2. **Vanished before retry:** a retried tree whose id vanished or merged before cleanup must follow the existing re-resolve rules and must never be dosed twice. Test: Task 1 `ImplausibleRetryAfterMergeDosesOnce`.
3. **Truncated edge view:** a tree seen only truncated at the image edge and never from closer must not get a height below its true top after the height fix. The S8 rule stays. Test: Task 5 `EdgeOnlyTreeHeightNotBelowTruth`.
4. **Outline only grows:** a changed centre estimate must never shrink or jump the published outline (rule 2), and the centre still moves at most `max_centre_rate`. Test: Task 6 `CentreFixKeepsOutlineGrowOnly`.
5. **Real-flight profile:** the code also runs with `cand_obj_real.cfg`. New keys must be present there, and the real-flight scorecard must not regress. Checked in Task 7.

---

### Task 1: Navigator retries implausible-height skips in cleanup

**Files:**
- Modify: `nav/corridor_nav/src/navigator.cpp`. Edit `advance()` at about lines 455–496, `plausible()` at about 549, and the `remaining` count in `status()` at about 596–600.
- Modify: `nav/corridor_nav/include/corridor_nav/navigator.hpp`, the doc comments at 149, 169 and 187.
- Modify: `tools/treemapper_tools/mission_score.py`, the navigator counters row at about 797–807.
- Test: `nav/corridor_nav/tests/test_navigator.cpp`.
- Test: `tools/tests/test_mission_score.py`.

**Interfaces:**
- Produces: navigator counters `implausible_retried`, `implausible_recovered` and `implausible_dropped`. They are published through the existing counters map in NavStatus.
- Produces: scorecard JSON key `implausible` = `{"skipped", "retried", "recovered", "dropped"}` (ints) in mission_score's navigator group.

**Behaviour (spec §4.4):**
- When `advance()` passes over a corridor target because `plausible()` is false, push its id onto `assignment_.cleanup` once (no duplicates).
- Cleanup targets it if, at that moment, the tree exists, is unsprayed, is not in `attempted_`, and `support::heightPlausible` passes. Then increment `implausible_retried`.
- A valid dose on a retried id increments `implausible_recovered`.
- Increment `implausible_dropped` when a skipped id gets no retry: it is still implausible when cleanup builds its list, or it has vanished.
- At most one retry per tree. A retry that fails goes through the normal `failAttempt` path and lands in `attempted_`.
- `status().remaining` counts an id in `implausible_` only while its current height is plausible.

- [ ] **Step 1: Write the failing tests** in `test_navigator.cpp`. Use the FakeWorld pattern of `ReresolvedImplausibleTargetIsSkipped` (`grid({6, 12, 18})`, `field30x24()`, `fov90()`, `kDt`, `kMaxSim`).
  - `TEST(Navigator, ImplausibleTreeRetriedInCleanupOnceHeightIsPlausible)`:
    - Tree k=1 starts with `height_bias = 9.0`. A `before_step` sets it back to 0 once the navigator's target is a tree of a later corridor.
    - Expect `w.trees[1].spray_calls == 1`.
    - Expect `counter(nav, "target_height_implausible") == 1`, `"implausible_retried" == 1`, `"implausible_recovered" == 1`, `"implausible_dropped" == 0`.
    - Expect every other tree to have `spray_calls == 1`, and `nav.status().remaining == 0` at the end.
  - `TEST(Navigator, ImplausibleStillHighInCleanupIsDropped)`:
    - The bias stays at 9.0 for the whole run.
    - Expect `spray_calls == 0`, `"implausible_retried" == 0`, `"implausible_dropped" == 1`.
    - Expect the mission to end (`w.run` true) with no request for that id.
  - `TEST(Navigator, ImplausibleRetryAfterMergeDosesOnce)`:
    - As the first test, but before cleanup the tree is republished under `published_id = 50` at the same place, with a plausible height.
    - Expect exactly one spray call on the truth tree, no request for the old id, and `"ids_reresolved" >= 1`.
- [ ] **Step 2: Run the tests and see them fail.** Build `/data/work/build/nav` as `scripts/run_tests.sh` does, then run the navigator gtest binary with `--gtest_filter='Navigator.Implausible*'` in the container. Expected: the 3 new tests FAIL. The retry case has `spray_calls == 0`.
- [ ] **Step 3: Implement the behaviour above.** It goes in `navigator.cpp`, in `advance()`, `plausible()` and `status()`. Update the three header comments.
- [ ] **Step 4: Run the full nav gtests.** Expect all to PASS, including `ReresolvedImplausibleTargetIsSkipped`. If its expectations conflict with the retry, the re-resolved entry is a different id: keep its existing assertions and say why in the report.
- [ ] **Step 5: Scorecard.**
  - Test `test_navigator_implausible_counters_in_scorecard` in `tools/tests/test_mission_score.py`: a synthetic NavStatus with counters `{"target_height_implausible": 2, "implausible_retried": 1, "implausible_recovered": 1, "implausible_dropped": 1}` gives `score["navigator"]["implausible"] == {"skipped": 2, "retried": 1, "recovered": 1, "dropped": 1}`, and the markdown has a row "Implausible height: skipped / retried / recovered / dropped".
  - Run it and see it fail, implement it, then run `PYTHONPATH=tools python3 -m pytest -q tools/tests/test_mission_score.py` and expect a PASS.
- [ ] **Step 6: Full suite.** `TM_ROS_TESTS=1 ./scripts/run_tests.sh` → `ALL TESTS PASSED`.
- [ ] **Step 7: Commit** `fix(nav): retry a tree skipped as implausibly high once in cleanup when its height is plausible`.

---

### Task 2: Record replay frames from an Isaac run

**Files:**
- Modify: `ros/corridor_nav_ros/launch/sim_closed_loop.launch`. Add arg `record_frames` (default `false`). When it is true, the recorder adds `/disparity_image_down_float`, `/cam/bottom/left/half/camera_info` and `/cam/bottom/right/half/camera_info`.
- Modify: `scripts/sim_mission.sh`. Add `--record-frames` (default off) and pass `record_frames:=true`. After the run is scored, convert and check, as in Step 4 below.
- Create: `tools/treemapper_tools/sim_frames.py`, which turns a closed-loop run directory into a TMF1 frame file.
- Test: `tools/tests/test_sim_frames.py`.

**Interfaces:**
- Consumes: `bag_frames.py`, its pairing of detections to disparity by exact stamp and its frame writing. Reuse its functions; refactor shared pieces into importable functions where needed, without changing `bag_frames`' output for its existing inputs.
- Consumes: `frames.py` (TMF1 contract), `sim/specs/camera.json` (`mount`, optical→body, xyzw).
- Consumes from the run dir: `mission.json`, `run.json`, `spec.json`.
- Produces: `python3 -m treemapper_tools.sim_frames --run <run dir> [--out <run dir>/frames.bin]`. It exits 0 and writes `frames.bin` plus `frames.bin.meta.json` with `{"frames", "frames_with_disparity", "pose_invalid", "disparity_rejected", "t_start", "t_end"}`. It exits non-zero if `frames_with_disparity == 0` or `pose_invalid > 1 %` of frames.
- Produces: `sim_frames.check(meta: dict) -> list[str]` returns the failed checks (empty means OK).

**Differences from the old Isaac bags that `bag_frames` was written for:**
- Here `/tf_static` is body→optical, so pass the mount from `camera.json` as `--t-body-cam`. The "spawn pose" derivation does not apply.
- Pose comes from `/tf` world→body in the bag.
- Phase and hop target come from `/mission_state` and `/spray_target` in the bag.
- There is no `world_nwu` conversion.
- Malformed duplicate disparities should not occur. Count them anyway.

- [ ] **Step 1: Write the failing tests** in `test_sim_frames.py`. Build a tiny synthetic bag with `rosbag` inside the test, and skip if `rosbag` is not importable. The bag has 3 detection and disparity pairs, `/tf`, `/mission_state` and `/spray_target`.
  - `test_sim_frames_pairs_and_poses`: the output has 3 frames, each `pose_valid`, with phase taken from `/mission_state`.
  - `test_sim_frames_body_cam_from_camera_json`: the frame's body→camera transform equals `camera.json` `mount`.
  - `test_check_flags_missing_disparity`: `check({"frames": 3, "frames_with_disparity": 0, "pose_invalid": 0, ...})` names the disparity check.
- [ ] **Step 2: Run** `PYTHONPATH=tools python3 -m pytest -q tools/tests/test_sim_frames.py` in the container. Expected: FAIL (module missing).
- [ ] **Step 3: Implement** `sim_frames.py`, the launch arg and the harness flag. In the harness, a failed conversion or check fails that run loudly. The bag is then kept.
- [ ] **Step 4: Harness behaviour with `--record-frames`.**
  - After scoring, run `sim_frames` on the run dir.
  - Only if `check()` is empty, replace `mission.bag` with a copy filtered to the normal allow-list (`rosbag filter`, dropping the disparity and camera_info topics). The directory then looks like a normal run, and `mission_score.py --run` works on it unchanged. Add a unit check for the flag parsing to the existing `sim_inputs` pytest if the harness logic lives there.
- [ ] **Step 5: Run the tests** (pass) and the full suite (`ALL TESTS PASSED`).
- [ ] **Step 6: Commit** `feat(sim): --record-frames records TreeMapper's inputs and converts them to a replay frame file`.

---

### Task 3: Replay metrics against Isaac truth

**Files:**
- Create: `tools/treemapper_tools/replay_metrics.py`.
- Test: `tools/tests/test_replay_metrics.py`.

**Interfaces:**
- Consumes:
  - a map stream JSONL (contract format; the closed loop's `map.jsonl` and `treemapper_replay --out-map` both produce it);
  - `truth.json` of the run;
  - `mission.json` (phases, `/spray_target` timeline);
  - the drone track (`frames.bin.meta.json` is not enough, so read pose from `mission.json` or the frames).
- Produces: `compute(map_jsonl: str, truth_json: str, mission_json: str) -> dict`. Its keys:
  - `entries`: list of `{"id", "truth_id", "first_pub_t", "first_pub_offset_m", "lock": {"dh", "dc"}, "doses": [{"t", "dh", "dc"}]}`. `dh` = published height − true top (m) and `dc` = centre error (m). `first_pub_offset_m` = across-track distance from the drone to the truth centre at first publication.
  - `a1`: `{"max_dh_at_lock", "max_dh_at_dose", "n_over_1p5"}`.
  - `a2`: `{"share_below_0p3", "worst_dh"}`.
  - `a3`: `{"far": {"n", "mean_dc_at_dose", "share_over_0p8"}, "near": {...}}`, split at 5.0 m.
- Produces: CLI `python3 -m treemapper_tools.replay_metrics --map M --truth T --mission J [--json out.json] [--md out.md]`, and `compare(before: dict, after: dict) -> str` for a markdown before/after table.
- Matching: a map entry matches the truth tree whose canopy outline contains its centre, else the nearest truth centre within 2.0 m. These are the same rules as `mission_score.py`; import its matcher rather than copying it. "Lock" is the first PLANNING phase.

- [ ] **Step 1: Write the failing tests** on a synthetic fixture: 2 truth trees, a map stream with 4 snapshots, a mission with SCAN → PLANNING → HOPPING → SPRAYING.
  - `test_heights_and_centres_at_lock_and_dose`: entry A at the lock has `dh == 0.5` and `dc == 0.2`; at its dose, `dh == 0.5`.
  - `test_a1_counts_over_1p5`: an entry at +2.0 m at a dose gives `n_over_1p5 == 1`.
  - `test_a3_far_near_split`: an entry first published 6.0 m across track lands in `far`, one at 3.0 m in `near`.
  - `test_unmatched_entry_excluded`: an entry 3 m from any truth is not in `entries`.
- [ ] **Step 2: Run the tests.** Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Cross-check against the closed loop.** Run `compute()` on `/data/work/sim_runs/cmp16c_5x5_tropical_corridor/run_1/` (its `map.jsonl`, `truth.json`, `mission.json`). Confirm that tree 2 / entry 12 shows `dh` ≈ +6.7 m at the lock (8.99 − 2.31) and that the 5 class (a) trees of that run have `dc` 1.17–2.13 m at their doses (`results/corridor_nav_isaac.md` finding 3). Record that agreement in the commit message.
- [ ] **Step 5: Run the tests** (pass) and the full suite (`ALL TESTS PASSED`).
- [ ] **Step 6: Commit** `tools: replay_metrics, map height and centre errors against Isaac truth at the lock and at each dose`.

---

### Task 4: Record the replay dataset and the baseline

**Files:**
- Create: `results/miss_reduction.md`, with a "Replay dataset" and "Baseline" section.
- Modify: none in code.

**Interfaces:**
- Consumes: Task 2's `--record-frames` and Task 3's `replay_metrics`.
- Produces: four replay datasets at `/data/work/sim_runs/mr_frames_<spec>_<strategy>/run_1/`, each with `frames.bin`, `truth.json`, `mission.json`, `map.jsonl` and the filtered `mission.bag`. Specs are `5x5_tropical` and `rows_8x12`; strategies are `corridor` and `field_first`.
- Produces: baseline numbers (A1–A3 "today" values) in `results/miss_reduction.md` and `/data/work/replay/baseline/<dataset>/metrics.json`.

- [ ] **Step 1: Check disk.** `df -h /media/alex/waspsim` must show ≥ 12 GB free. If not, stop and report.
- [ ] **Step 2: Record the four runs, one at a time.** Each is `scripts/sim_mission.sh sim/specs/<spec>.json --strategy <s> --runs 1 --record-frames --out /data/work/sim_runs/mr_frames_<spec>_<s>`, run in the background with a timeout of 1800 s for 5x5 and 3000 s for rows. Each must exit 0 (mission completed) with a `frames.bin` that passes `check()`. A failed run is reported, not silently retried.
- [ ] **Step 3: Replay the baseline** with unchanged code. For each dataset run `/data/work/build/core/treemapper_replay --frames <run>/frames.bin --system cand_obj --config "$(config file as k=v)" --out-map /data/work/replay/baseline/<ds>/map.jsonl --out-targets /data/work/replay/baseline/<ds>/targets.json --out-counters /data/work/replay/baseline/<ds>/counters.json`. Use `config/cand_obj.cfg`; read `replay_main.cpp` for how a cfg file is passed. Run it twice on one dataset and confirm the map streams are byte-identical without `stage_ms`.
- [ ] **Step 4: Check that replay reproduces the closed loop.**
  - Run `replay_metrics.compute` on the replay map and on the run's closed-loop `map.jsonl`.
  - Over matched entries, for ≥ 90 % of them, the final `dh` and the final `dc` agree within 0.15 m.
  - The ramping entries named in the run's own height traces reproduce their lock-time `dh` within 0.3 m.
  - If replay does not reproduce the closed loop, stop and report the differences. The fixes cannot be developed on a replay that disagrees.
- [ ] **Step 5: Write `results/miss_reduction.md`.**
  - Dataset table: path, frames, duration, size, commit.
  - Equivalence numbers from Step 4.
  - Baseline A1/A2/A3 per dataset, generated with `replay_metrics --md`.
- [ ] **Step 6: Commit** `results: miss reduction replay dataset and baseline (4 Isaac runs, replay = closed loop)`.

---

### Task 5: TreeMapper height fix

**Files:**
- Modify: `core/src/candidates/cand_obj/cand_obj.cpp`. Expected around `updateEvidence` (about 654–710), `refreshHeight` (about 737) and the point weighting (about 280–290). The diagnosis decides.
- Modify: `core/include/treemapper/candidates/cand_obj/cand_obj.hpp` (Config, `kConfigKeyCount`), `config/cand_obj.cfg` and `config/cand_obj_real.cfg`, but only if a tunable is added.
- Modify: `docs/DECISIONS.md` (new D-number) and `results/miss_reduction.md` (section "Height").
- Test: `core/tests/candidates/cand_obj/test_cand_obj.cpp`.

**Interfaces:**
- Consumes: the Task 4 datasets and baseline, and `replay_metrics`.
- Produces: no API change. Any new counters follow rule 7 (counted by reason, published).

- [ ] **Step 1: Diagnose on the replay datasets.**
  - Add temporary instrumentation behind a debug env var or a test-only hook. It must not be permanent log output.
  - For the ramping entries, find which latched cells push `height_target` (the 0.99 quantile) high, in which frames, and at what range, off-axis angle and z relative to `z_cap`.
  - Write the finding into `results/miss_reduction.md` "Height / diagnosis", with the evidence.
  - Remove the instrumentation, or keep it as a counter if it is a real drop reason.
- [ ] **Step 2: Write the failing tests** in `test_cand_obj.cpp`. Synthetic crowns reproduce the diagnosed mechanism, following the existing synthetic-crown tests in that file.
  - `TEST(CandObj, HeightDoesNotRunAwayFrom<diagnosed cause>)`: the published height stays ≤ true top + 0.5 m through the sequence that ran away before the fix. The name reflects the diagnosed cause.
  - `TEST(CandObj, EdgeOnlyTreeHeightNotBelowTruth)`: a tree seen only truncated at the edge, then never again, is either not published (S8) or published at ≥ true top − 0.3 m.
- [ ] **Step 3: Run the tests.** Run `cmake --build /data/work/build/core -j8 && (cd /data/work/build/core && ctest -R cand_obj --output-on-failure)`. Expected: the first new test FAILS.
- [ ] **Step 4: Implement the fix** in `cand_obj.cpp` at the cause, and add a tunable only if the fix needs one, following the config rules in Global Constraints.
- [ ] **Step 5: Run the tests.** All `cand_obj` tests PASS, including the config defaults and key-count tests.
- [ ] **Step 6: Replay all four datasets** with the fix. A1 must hold: `max_dh_at_lock` ≤ 1.5 and `max_dh_at_dose` ≤ 1.5. A2 must hold against the baseline. Replay stays deterministic. Write the before/after table (`replay_metrics.compare`) into `results/miss_reduction.md` "Height / result". If A1 or A2 fails after a real attempt, stop and report (spec §2).
- [ ] **Step 7: DECISIONS.md entry:** what was wrong, the evidence, the change, and its A1/A2 numbers.
- [ ] **Step 8: Full suite** (`ALL TESTS PASSED`), then **commit** `fix(cand_obj): <diagnosed cause>, published heights no longer run away after the pass`.

---

### Task 6: TreeMapper centre fix

**Files:**
- Modify: `core/src/candidates/cand_obj/cand_obj.cpp`. Expected around `refreshHull` (about 712–735), the latching in `updateEvidence` and `stepPublished` (about 757). The diagnosis decides.
- Modify: header, cfg files, `docs/DECISIONS.md` and `results/miss_reduction.md` (section "Centre"), as in Task 5.
- Test: `core/tests/candidates/cand_obj/test_cand_obj.cpp`.

**Interfaces:**
- Consumes: the Task 4 datasets, the Task 5 code, and `replay_metrics` (`a3`).
- Produces: no API change. The published outline and the published centre stay separate quantities.

- [ ] **Step 1: Diagnose on the 5x5 replay datasets.**
  - For the far-sighted entries (first published ≥ 5 m across track), find which latched cells pull the centre toward the first track, and from which views.
  - Say why rows_8x12 shows the effect less.
  - Write this into `results/miss_reduction.md` "Centre / diagnosis".
- [ ] **Step 2: Write the failing tests.**
  - `TEST(CandObj, CentreConvergesAfterNearViews)`: a crown first seen at the far image edge, then seen 3–4 m across track, has its published centre within 0.3 m of the true centre after the near views. Before the fix it stays biased toward the first track.
  - `TEST(CandObj, CentreFixKeepsOutlineGrowOnly)`: through the same sequence the published outline's area never decreases between frames, and the published centre never moves faster than `max_centre_rate`.
- [ ] **Step 3: Run the tests.** The first test FAILS.
- [ ] **Step 4: Implement the fix** in `cand_obj.cpp`. Rule 2 still binds the outline. If the fix unlatches cells, a published outline never shrinks because of a few frames.
- [ ] **Step 5: Run the tests** (PASS) and replay all four datasets.
  - A3 must hold on the 5x5 datasets: far `mean_dc_at_dose` ≤ 0.5 and `share_over_0p8` ≤ 0.15.
  - near must be no worse than the baseline.
  - A1 and A2 must still hold.
  - Write the before/after table into "Centre / result". If A3 fails after a real attempt, stop and report: the spec's fallback (pre-dose re-centring) is separate work.
- [ ] **Step 6: DECISIONS.md entry, full suite, commit** `fix(cand_obj): <diagnosed cause>, centres of trees first seen at the image edge converge once seen closer`.

---

### Task 7: TreeMapper regression check

**Files:**
- Modify: `results/miss_reduction.md` (section "TreeMapper regression check").
- Modify: `docs/DECISIONS.md`, only if a regression is accepted.

**Interfaces:**
- Consumes: the code before Task 5 (the commit before it, built in a separate worktree under the scratchpad or `/data/work`) and HEAD.

- [ ] **Step 1: Synthetic suite.** Run it on both builds: `TM_OUT=/data/work/eval/mr_<before|after> TM_REPEAT=2 ./scripts/evaluate.sh cand_obj suite_v2`. `TM_REPEAT=2` gives the noise floor.
- [ ] **Step 2: Holdout, Isaac replays and real-flight scorecard.** Re-run each on both builds with the commands their results files name: `results/cand_obj_m4_holdout.md`, `results/cand_obj_m4_isaac.md` and `results/real_flight_scorecard.md`. The real-flight scorecard uses `config/cand_obj_real.cfg`.
- [ ] **Step 3: Compare each metric** before and after, against its noise floor. Write the table into `results/miss_reduction.md`.
- [ ] **Step 4: Handle any regression beyond the noise floor.**
  - Fix it in a new commit, with a test, and re-run the affected checks.
  - Or, if it is the price of the fix, write the numbers into DECISIONS.md and the results, then stop and report to the controller before Task 8. The user decides.
- [ ] **Step 5: Commit** `results: TreeMapper regression check of the height and centre fixes`.

---

### Task 8: Re-measure corridor vs field_first

**Files:**
- Modify: `results/corridor_nav_isaac.md`, new section "After the map fixes". The old sections stay.
- Modify: `results/miss_reduction.md` (closing summary of A1–A7), `PROGRESS.md` and the `README.md` "Isaac-only closed loop" section (the `--record-frames` flag).

**Interfaces:**
- Consumes: `scripts/sim_mission.sh`, `tools/treemapper_tools/sim_compare_report.py` and the old batches `/data/work/sim_runs/cmp16c_*`.
- Produces: new batches `/data/work/sim_runs/mr_cmp_<spec>_<strategy>/` (3 runs each).

- [ ] **Step 1: Check disk.** At least 12 GB must be free.
- [ ] **Step 2: Run the 12 missions** sequentially, in the background. Use the same specs and seeds as `cmp16c_*`, `--runs 3`, the timeouts from Task 4, and `--no-build` after the first batch. Do not use `--record-frames`.
- [ ] **Step 3: Generate the tables.** Run `sim_compare_report.py` on the four new batches. Put its tables, verbatim, into "After the map fixes".
  - Side by side with the old batches: misses by class, per spec × strategy, from both sets of batches.
  - A4: total misses ≤ 12.
  - A5: every safety row passes in every run.
  - A7: `implausible_retried` / `recovered` / `dropped`.
- [ ] **Step 4: Strategy recommendation**, by the spec §4.5 rule.
  - If corridor's spray outcome is better than field_first's beyond the noise floor, recommend corridor and say what it buys.
  - Otherwise recommend field_first as the default, and say what corridor would need to earn its extra time.
  - Do not change `config/corridor_nav.cfg`'s `strategy`.
  - Keep the line `Not verified on the Orin or on a real flight`.
- [ ] **Step 5: README and PROGRESS.** README gets one paragraph on `--record-frames` and replay. PROGRESS gets the state and the open items: the guard-in-Isaac experiment, Orin, and the pre-dose re-centring fallback if A3 needed it.
- [ ] **Step 6: Commit** `results: corridor_nav after the TreeMapper fixes, corridor vs field_first re-measured`.
