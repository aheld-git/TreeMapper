# TreeMapper vs SpotSpraying on a new Isaac scene (5 x 5, tropical)

> First held-out sim mission with **per-tree truth heights**. Never used for tuning. One mission, so there is no seed
> spread: every difference below is one sample. Machine-readable numbers: `results/isaac_5x5_tropical.json`.
> Video (1080p, 1 min 51 s at 6x speed): `/media/alex/waspsim/cloud_build_bundle/work/video/isaac_5x5/treemapper_isaac_5x5_tropical.mp4`,
> 720p copy `treemapper_isaac_5x5_tropical_720p.mp4` next to it.

## Setup

| | |
|---|---|
| Scene | 5 x 5 grid, 7 m spacing, +/- 1.5 m jitter, seed 1001, field 42 x 44 m, tropical biome, 50 % orange / 50 % lemon (25 trees) |
| Mission | travel 15 m, 4 m/s, 0.3 L per tree over 4 s; spec `work/isaac_5x5_tropical/spec.json` |
| Recorded | 2026-10-29, TreeHoppingNavigation `7f3fbca` (per-tree canopy tops in the ground truth), Isaac Sim 5.1 in stream mode |
| SpotSpraying | ran **live** on the Orin and drove the mission; commit not recorded by the run (last confirmed: `5f608dc`, 2026-10-28) |
| TreeMapper | `cand_obj`, built-in defaults (= `config/cand_obj.cfg`), commit `ed7200b`, **replayed offline** on the recorded frames |
| Bag | `isaac_5x5_tropical/cloud_bundle_isaac_5x5_2026-10-29-13-35-35.bag`, 42.4 GB, 608 s sim time, 25 383 frames |

The sweep judged the mission a pass: completed, 0 errors, 26 spray passes over 25 truth trees, 0 revisits, 1 orphan
spray (SpotSpraying's false tree #11), real-time factor 0.13 (86 min wall clock). Phases: three scan passes
(SCAN 64 s), HOPPING 355 s, SPRAYING 104 s.

How the comparison works: both systems get the same camera, disparity and detector boxes. TreeMapper's hop targets
come from the replay's navigation emulator, which follows the recorded navigator (flying on SpotSpraying's map) and
resolves each descent point to a TreeMapper tree.

## Scorecard (final map unless noted)

| Metric | TreeMapper | SpotSpraying |
|---|---|---|
| trees found (recall) | 25 / 25 | 25 / 25 |
| published entries / precision | 26 / 0.96 | 31 / 0.81 |
| false trees | 1 | 6 |
| neighbours swallowed (merges) | 1 | 0 |
| mean / min outline IoU | **0.84** / 0.63 | 0.70 / 0.48 |
| centre RMSE | **0.29 m** | 0.66 m |
| spill outside true canopies | 0.05 | 0.14 |
| height bias / RMSE | -0.69 / 0.81 m | **-0.15 / 0.16 m** |
| per-frame recall (mean) | 0.59 | **0.65** |
| centre jumps / height jumps | **0 / 0** | 16 / 57 |
| safety violations | **0** | 20 (one event, see below) |
| truth trees sprayed | 21 / 25 (replay artefact, see below) | 25 / 25 |
| scan latency (mean) | 0.66 s | 0.43 s |
| cost (x86, one core) | 0.89 ms per mapped frame (2 192 frames) | n/a |

## What the numbers mean

**Geometry: TreeMapper is clearly better.** Outlines fit better (IoU 0.84 vs 0.70), centres are less than half as far
off (0.29 vs 0.66 m), and it publishes 1 false tree against SpotSpraying's 6. It swallowed one neighbour in a merge,
which SpotSpraying did not.

**Stability and safety: TreeMapper is clean, SpotSpraying is not.** TreeMapper has no centre or height jumps and no
safety violations. SpotSpraying's 20 violation frames are one event: tree #67 was published with a height of about
0 m (ground level) for 4 s around t = 276 s, during the second scan pass. It also moved centres 16 times and heights
57 times by more than 1 m.

**Height: TreeMapper is worse, and in the unsafe direction.** It puts every canopy top too low: median -0.58 m,
range -0.30 to -1.90 m over the 25 trees. SpotSpraying is within -0.06 to -0.37 m. A low top means the descent
reference sits closer to the canopy than intended. This is the first mission with real per-tree truth heights (the
3 x 3 recording had a constant 4.0 m fallback), so this weakness was not visible before. A likely cause, **not yet
verified**: TreeMapper freezes heights outside its creation phases (scans at 15 m), while SpotSpraying keeps refining
them during descent, which would also explain its 57 height jumps.

**Trees sprayed, 21 vs 25: a limit of the replay, not of the map.** The mission scanned the field in three passes, and
TreeMapper's map grew 8 -> 16 -> 24 -> 26 entries. Four of the navigator's targets (t = 105, 144, 453 and 522 s) were
trees SpotSpraying had published but TreeMapper had not yet; the nearest TreeMapper tree was 4.6-6.9 m away, so the
emulator could not resolve them. TreeMapper's final map contains all 25 trees. Whether a navigator flying on
TreeMapper's map would have sprayed them in a later pass is not tested here. The lower per-frame recall (0.59 vs
0.65) is the same effect: TreeMapper publishes a tree later.

## Limits

- One mission, one seed; no noise floor.
- TreeMapper did not fly. A closed-loop run (TreeMapper live on the Orin in place of `object_fuser`) is needed to test
  spray coverage and latency effects for real.
- The SpotSpraying commit on the Orin was not recorded.
- Truth heights take two values only (5.49 m lemon, 6.08 m orange): the per-species asset height, not a measured
  per-instance top.

## Reproduce

All paths are under `/media/alex/waspsim/cloud_build_bundle/`. Containers: `treehoppingnavigation-bridge:latest` with
the mounts from `docs/LOCAL_RUN.md` (`work` -> `/data/work`, `isaac_5x5_tropical` -> `/data/isaac_5x5_tropical:ro`).

1. Mission and bag: `./sweep.sh work/isaac_5x5_tropical/spec.json --sim-mode stream` in TreeHoppingNavigation, with
   `work/isaac_5x5_tropical/record_bundle.sh` alongside. Headless mode crashes Isaac on start-up on this machine.
2. Extract: `python3 -m treemapper_tools.extract_bag` then `bag_frames` -> `work/extracted/isaac_5x5_2026-10-29-13-35-35`.
3. Score: `python3 -m treemapper_tools.evaluate --system cand_obj,existing --dataset <extracted>` -> `work/eval_5x5/`.
4. Video: `work/isaac_5x5_tropical/render_5x5.py` with `TM_E`, `TM_R`, `TM_BAG`, `TM_OUT`, `TM_SCORE`, `TM_SPEED=6`.
