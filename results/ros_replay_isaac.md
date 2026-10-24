# ROS path vs offline replay: recorded Isaac mission (M4b)

**Question:** does the ROS node, fed from the bag through `rosbag play`, produce the same map as the offline replay
(`treemapper_replay` on frames.bin) that every scorecard so far was computed with?
**Answer:** yes, within transport timing: same 9 trees with the same 9 IDs, per-tree outline IoU 0.98-1.00, centres
within 0.023 m, heights within 0.033 m, identical sprayed flags and hop-target sequence, identical safety and
stability rows; per-frame recall 0.738 / 0.738, mean IoU 0.865 / 0.867, scan latency 2.34 / 2.32 s.

## Setup

- Core: main as of 960ccb7 (cand_obj after M4a hardening), exported with `git archive` to
  `/data/work/catkin_m4b/snap` and built once into the catkin profile `snap`; the node and `treemapper_replay`
  come from **the same build**. Mapper config = `config/cand_obj.cfg` of that export (node) / built-in defaults
  (offline), equal by the core's config test.
- ROS: `ros/treemapper_ros/scripts/ros_replay_check.sh /data/work/m4b/main_q3 45 0 0.00390625 1.0` in tm_dev (ROS
  master inside the container): roscore + `replay_isaac.launch` (node + `nav_emulator.py` + `rosbag play --clock
  --hz=1000 -s 45 -r 1.0` of `/data/isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag`, bag t = 55 s .. 288 s, 239 s
  wall) + `rosbag record` of `/treemapper/{map,hop_target,health,counters,stage_timing}`; live probes (health,
  get_trees all / by id / unknown id) while it ran; node alive at the end.
- `disparity_quantum_px = 1/256`: the node rounds disparity like the frames.bin cache, so both sides see the same
  depth values; without it the inputs differ by < 1/512 px.
- Offline: `treemapper_replay --frames /data/work/extracted/isaac_2026-10-23-15-34-29/frames.bin --system cand_obj`.
- Hop targets: the bag's `/spray_target` (recorded system's IDs) is remapped away; `nav_emulator.py` resolves each
  recorded target by its descent point exactly like the offline runner and calls `set_tree_sprayed` when SPRAYING ends.
  Each stream is scored against its own resolved target timeline.
- Converter `scripts/treemapper_bag_to_jsonl.py` (TreeMap topic -> scorer JSONL), comparison
  `scripts/compare_map_streams.py` (Hungarian match on outline IoU + `treemapper_tools.score` for both, snapshots
  t >= 55 s). Raw: `results/ros_replay_isaac.json`, run dir `/data/work/m4b/main_q3`.

## Hop-target timeline (mapper IDs)

| t offline (s) | t ROS (s) | target |
|---|---|---|
| 87.444 | 87.464 | 8 |
| 107.545 | 107.564 | 9 |
| 125.339 | 125.363 | 6 |
| 145.846 | 145.863 | -1 |
| 151.339 | 151.370 | 4 |
| 167.654 | 167.667 | -1 |
| 186.552 | 186.563 | 15 |
| 206.849 | 206.862 | 14 |
| 224.139 | 224.163 | 12 |
| 241.640 | 241.662 | -1 |

Same targets, 11-31 ms later in ROS (the emulator polls the mission timeline at 20 Hz).

## Final map, trees matched by outline IoU

| id ros | id offline | IoU | centre dist m | height diff m (ros - offline) | sprayed ros / offline |
|---|---|---|---|---|---|
| 1 | 1 | 0.983 | 0.023 | -0.033 | False / False |
| 4 | 4 | 1.000 | 0.000 | -0.005 | True / True |
| 6 | 6 | 0.989 | 0.020 | +0.000 | True / True |
| 7 | 7 | 1.000 | 0.000 | -0.000 | False / False |
| 8 | 8 | 1.000 | 0.000 | -0.001 | True / True |
| 9 | 9 | 1.000 | 0.000 | -0.000 | True / True |
| 12 | 12 | 1.000 | 0.000 | -0.004 | True / True |
| 14 | 14 | 0.993 | 0.020 | -0.003 | True / True |
| 15 | 15 | 1.000 | 0.000 | -0.003 | True / True |

Trees: ros 9, offline 9; matched 9, same ID 9; unmatched ros [], offline [].

## Scorecards (headline)

| metric | ros | offline |
|---|---|---|
| recall (final) | 1 | 1 |
| precision (final) | 1 | 1 |
| published / truth (final) | 9 | 9 |
| recall (per-frame mean) | 0.738 | 0.738 |
| mean IoU (final) | 0.865 | 0.867 |
| min IoU (final) | 0.819 | 0.817 |
| merged entries (final) | 0 | 0 |
| neighbours swallowed (final) | 0 | 0 |
| split trees (final) | 0 | 0 |
| false trees (final) | 0 | 0 |
| centre RMSE m (final) | 0.19 | 0.191 |
| height bias m (final) | n/a | n/a |
| height RMSE m (final) | n/a | n/a |
| spill (final) | 0.065 | 0.064 |
| neighbour spill max (final) | 0.056 | 0.056 |
| ID switches after lock | 0 | 0 |
| geometry retreats | 0 | 0 |
| centre jumps | 0 | 0 |
| height jumps | 0 | 0 |
| IDs vanished while held by nav | 0 | 0 |
| SAFETY total violations | 0 | 0 |
|   undefined-height frames | 0 | 0 |
|   new trees after scan | 0 | 0 |
|   non-target changes in descent | 0 | 0 |
|   sprayed-tree changes | 0 | 0 |
|   new tree inside sprayed | 0 | 0 |
|   invalid-entry frames | 0 | 0 |
|   duplicate-ID frames | 0 | 0 |
|   implausible-height frames | 0 | 0 |
|   locked ID vanished | 0 | 0 |
| truth trees sprayed | 7 | 7 |
| truth trees sprayed twice | 0 | 0 |
| scan latency mean s | 2.335 | 2.319 |

(Height error rows are n/a by design: the Isaac truth heights are a constant fallback, R13.)

## What it took to get here (and why the remaining differences are small)

| run | change | final IDs equal | scan latency ROS / offline |
|---|---|---|---|
| M3 core, first version | — | 9 / 9 | 2.58 / 2.23 s |
| M3 core | early pairing resolve | 6 / 9 (allocator offset on the last 3) | 2.23 / 2.23 s |
| main core | — | 9 / 9 | 2.80 / 2.32 s |
| main core | + monotonic frame t, phase heartbeat after pending frames, 1 kHz replay clock | 9 / 9 | 2.35 / 2.32 s |
| main core (final) | + heartbeat stamps on the header clock | 9 / 9 | 2.34 / 2.32 s |

- The first latency gap came from in-order frame release: a detection whose exact-stamp disparity never arrives
  (1010 of the recording's frames) held every later frame for `sync_wait` = 0.5 s. With in-order disparity stamps it
  is now resolved as soon as a newer disparity arrives (same pairing result).
- The second came from cand_obj's new rate limits (they integrate over frame `t`): `rosbag play`'s default 100 Hz
  clock quantised `t`, and a phase-change heartbeat could be fed before still-pending older frames (t stepping back).
- Run-to-run, the ROS path is not bit-deterministic (message timing, 20 Hz emulator, frames released at receive
  time): mapper counters differ by 0.1-1 % (e.g. crowns_total 13946 vs 13954) and in one run the ID allocator ended
  one tentative instance apart (IDs 10/12/13 vs 11/13/14 for the last three trees, same trees). IDs never changed
  meaning within a run (ID switches after lock 0, no vanished IDs).
- Cost on x86 (`~stage_timing`): `node_frame_total` mean 0.84 ms, p99 1.5 ms, max 6.7 ms per frame (cand_obj's chm
  stage ~1.0 ms); disparity sampling 0.04 ms. Not measured on the Orin.
