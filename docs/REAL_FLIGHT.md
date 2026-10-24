# Real flight: input path, camera-chain check, no-ground-truth scoring

Data (read-only): `/data/real/BB_treehopping_farm.bag` (stereo, calibration, disparity, `/tf`, `/tf_static`;
511 s, orchard near 18.27 N 74.49 E) and `/data/real/BB_treehopping_farm_companion_2026-10-23-18-01-20.bag`
(the existing pipeline replayed on it: detections, drone position, the existing map). There is no ground truth,
no mission phase and no hop target.

## Commands (all inside tm_dev, at most 2 cores)

```
D='docker exec tm_dev bash -lc "source /opt/ros/noetic/setup.bash && cd /ws/tools && '
# 1. frames.bin + mission.json (pose only) + camera_info.json, ~30 s
python3 -m treemapper_tools.real_frames --bag /data/real/BB_treehopping_farm.bag \
    --companion /data/real/BB_treehopping_farm_companion_2026-10-23-18-01-20.bag --out /data/work/extracted/real_farm
# 2. the existing system's map (MarkerArray + fused_bboxes -> map stream JSONL), ~20 s
python3 -m treemapper_tools.real_existing --companion /data/real/BB_treehopping_farm_companion_2026-10-23-18-01-20.bag \
    --out /data/work/extracted/real_farm/map_existing.jsonl
# 3. camera-chain check without ground truth (hypotheses A..F + pitch sweep), ~25 min on 2 cores at --every 2
taskset -c 0,1 python3 -m treemapper_tools.real_geometry --frames /data/work/extracted/real_farm/frames.bin \
    --every 2 --extra-pitch 78 80 82 84 --out-json /ws/results/real_flight_geometry.json
# 4. build the replay from a clean export of a commit, run + score (no GT) + plots
git -C /ws archive <commit> core | tar -x -C /data/work/eval/m4c/src_<commit>
cmake -S /data/work/eval/m4c/src_<commit>/core -B /data/work/eval/m4c/build_<commit> -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /data/work/eval/m4c/build_<commit> -j2 --target treemapper_replay
taskset -c 0,1 python3 -m treemapper_tools.real_score --dataset /data/work/extracted/real_farm \
    --replay /data/work/eval/m4c/build_<commit>/treemapper_replay --systems simple,cand_obj,cand_grid,existing \
    --runs /data/work/eval/m4c/runs_<commit> --label <commit> --out-json /ws/results/real_flight_scorecard.json \
    --out-md-table /data/work/eval/m4c/table.md --plots /ws/results/real_flight_plots
```
Tests: `python3 -m pytest -q tests/test_real_frames.py` (units, offset, chain, pairing, converter).
Stereo checks that need the images (rectification and the motion-based depth scale), ~10 min:
`taskset -c 0 python3 -m treemapper_tools.real_stereo_check --bag /data/real/BB_treehopping_farm.bag
--dataset /data/work/extracted/real_farm --out-json /ws/results/real_flight_stereo_check.json`.

## What the reader does (details: `real_frames.py` docstring)

- **Frames**: one per companion detection message.
  - `t` = companion bag time. This clock equals the flight bag's bag time, and the map stream and mission.json
    use it too.
  - `stamp` = header stamp + a constant clock offset. Header stamps are flight time, Oct 2025; the bags were
    re-recorded in Jan 2026.
- **Pairing**: detection to disparity by exact header stamp, else nearest within 20 ms, else no disparity
  (counted). Pose comes from `/tf` `world_nwu -> body` at the disparity stamp. A missing pose gives
  `pose_valid = false` and a counter, never a crash. Heartbeats fill gaps in the detection stream.
- **Camera chain (inferred from one flight, not verified on hardware; `results/real_flight_geometry.md`)**:
  - lever arm from tf_static `body -> sensor_down`;
  - mount pitch **82 deg** (tf_static says 75; `--mount-pitch-deg tf` restores it);
  - optical-to-ROS axis swap;
  - left rectified P intrinsics;
  - baseline 0.1606 m from right `P[3]` in **mm**;
  - disparity offset **4.30 px** subtracted (`--disparity-offset calib` = cx_l - cx_r = 4.918).
- **Phase**: constant `SCAN`, target -1. mission.json carries the pose only. **The descent, lock, spray and
  hop-target rules cannot be exercised on this flight.** The scorer reports them as n/a.
- **Existing map**: per MarkerArray, the `ns=polygon` LINE_STRIPs become outlines (ID = marker ID). Height = the
  `fused_bboxes` median height of the same ID. sprayed = false. Frame world_nwu.

## Results

- `results/real_flight_geometry.{md,json}`, `results/real_flight_stereo_check.json`: the chain and its evidence.
- `results/real_flight_scorecard.{md,json}`, `results/real_flight_crowns.json`, `results/real_flight_plots/`: the
  no-GT scorecard (simple, cand_obj M3, cand_obj M4, cand_grid, existing) and the qualitative comparison.
- `results/real_flight_profile.md`: the cand_obj configuration for real data (`config/cand_obj_real.cfg`).

## Unverified (summary)

- Whether the detector boxes (raw left image) and the disparity share a pixel frame. Up to ~13 px apart at the
  image edges.
- Roll of the mount (+-2 deg); the lever arm.
- Pose-z drift of the flight controller over the flight.
- The crown-top-z vs altitude residual (-0.3 m/m).
- The existing system's own chain and configuration in the companion replay.
- Everything that needs mission phases: locked-phase stability, descent freeze, spraying.
- No ground truth: all "accuracy" statements are relative to stereo crown peaks and to agreement between systems.

## cand_obj profiles

- `config/cand_obj_real.cfg` is the real-flight profile, used by `treemapper_real.launch`.
- `config/cand_obj.cfg` is the sim and default profile.
- On the farm log the default profile over-fragments (102 trees for ~63 crowns). The real profile gives 63.
- The real profile regresses on the synthetic held-out suite (recall 0.93 -> 0.85, more merges), so both profiles
  are kept.
- Sweep and decision: `results/real_flight_profile.md`. Re-run:
  `python3 -m treemapper_tools.real_profile_sweep --round 1|2 ...` (command in that file).
