# Real flight: no-ground-truth scorecard (BB_treehopping_farm)

Input:
- `/data/work/extracted/real_farm/frames.bin`, with the verified chain: 82 deg mount and disparity offset 4.30 px
  (`real_flight_geometry.md`);
- the existing system's map, converted from the companion bag's `/object_fusion/polygon_marker` +
  `/object_fusion/fused_bboxes` by `real_existing.py`.

Builds and versions:
- **cand_obj** = M3 version: core exported at 89317b2; the cand_obj sources are last changed at 1af8563.
- **cand_obj_m4** = M4a hardened version: core 960ccb7. Both use their frozen default configuration.
- simple and cand_grid: core 89317b2.

Every build is a clean `git archive` export (uncommitted work does not leak in). The frame is world_nwu.

Command:
```
python3 -m treemapper_tools.real_score --dataset /data/work/extracted/real_farm \
  --replay /data/work/eval/m4c/build_89317b2/treemapper_replay \
  --systems "simple,cand_obj,cand_obj_m4=cand_obj@/data/work/eval/m4c/build_960ccb7/treemapper_replay,cand_grid,existing" \
  --runs /data/work/eval/m4c/runs_final --out-json results/real_flight_scorecard.json --plots results/real_flight_plots
```
Raw: `real_flight_scorecard.json` (per system: summary, counters, final tree list; the scorer output is in
`runs_final/<system>/score.json`) and `real_flight_crowns.json` (stereo crown peaks, missed and merged per system).

**What cannot be scored without ground truth or mission phases:**
- recall, precision, IoU, centre and height error, spray outcome, timeliness;
- every locked-phase and descent rule: new trees after the scan, locked ID vanished, non-target changes in descent,
  sprayed-tree changes (all "n/a", or 0 by construction).

The real flight has no `/mission_state` and no hop target. Systems run with a constant SCAN phase.

| metric | simple | cand_obj | cand_obj_m4 | cand_grid | existing |
|---|---|---|---|---|---|
| final trees | 59 | 100 | 102 | 55 | 54 |
| IDs ever published | 67 | 115 | 115 | 80 | 77 |
| outline area m2 p10/p50/p90 | 35.03 / 67.57 / 92.17 | 6.49 / 31.76 / 45.74 | 8.88 / 30.64 / 44.86 | 24.31 / 51.47 / 110.56 | 20.63 / 41.13 / 73.02 |
| total area m2 | 3851.40 | 2873.40 | 2882.60 | 3662.70 | 2629.30 |
| equiv. diameter m p10/p50/p90 | 6.68 / 9.28 / 10.83 | 2.87 / 6.36 / 7.63 | 3.36 / 6.25 / 7.56 | 5.56 / 8.10 / 11.85 | 5.12 / 7.24 / 9.64 |
| height (world z) p10/p50/p90 | 4.19 / 5.15 / 6.39 | 3.00 / 4.94 / 6.38 | 3.51 / 5.13 / 6.80 | 3.56 / 5.33 / 7.11 | 1.66 / 3.57 / 5.27 |
| height above stereo ground p10/p50/p90 | 5.26 / 6.54 / 7.45 | 4.66 / 6.24 / 7.44 | 4.85 / 6.39 / 8.08 | 4.87 / 6.57 / 8.14 | 2.34 / 4.52 / 6.32 |
| stereo crowns (>= 2 m) inside an outline | 61 / 63 | 60 / 63 | 62 / 63 | 60 / 63 | 60 / 63 |
| crown peaks in exactly 1 / no / 2+ outlines | 59 / 2 / 2 | 53 / 4 / 6 | 55 / 3 / 5 | 56 / 5 / 2 | 57 / 5 / 1 |
| outlines with 0 peaks (fragment/false) / 1 / 2+ (merged) | 1 / 53 / 5 | 37 / 61 / 2 | 39 / 61 / 2 | 6 / 42 / 7 | 4 / 42 / 8 |
| nearest-neighbour centre distance m p10/p50 | 7.08 / 7.89 | 2.75 / 3.89 | 3.15 / 4.12 | 5.76 / 7.95 | 7.02 / 8.06 |
| IDs vanished | 8 | 15 | 13 | 25 | 23 |
| ID reappearances / reuse | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| geometry retreats / max lost fraction | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 | 0 / 0.00 |
| centre jumps / height jumps (> 1 m) | 58 / 6 | 42 / 39 | 0 / 0 | 34 / 2 | 75 / 296 |
| max centre / height drift from first m | 4.32 / 2.63 | 3.09 / 3.29 | 3.34 / 4.37 | 4.27 / 5.13 | 7.17 / 5.75 |
| **safety** undefined-height frames | 0 | 0 | 0 | 0 | 0 |
| invalid-entry frames | 0 | 0 | 0 | 0 | 0 |
| duplicate-ID frames | 0 | 0 | 0 | 0 | 0 |
| implausible-height frames (all) | 56 | 57 | 59 | 58 | 3232 |
|   of which height <= 0 (world_nwu z) | 0 | 0 | 0 | 0 | 3232 |
|   of which above the drone | 57 | 58 | 59 | 59 | 0 |
| new trees after scan / locked ID vanished | n/a / n/a | n/a / n/a | n/a / n/a | n/a / n/a | n/a / n/a |
| non-target changes in descent / sprayed changes | n/a / 0 | n/a / 0 | n/a / 0 | n/a / 0 | n/a / 0 |
| cost ms/frame (mean, p95) per stage | extract 0.547/0.708; integrate 0.33/0.704 | chm 0.932/1.192; instances 0.112/0.209 | chm 0.981/1.215; instances 0.15/0.287 | extract 0.846/1.816; integrate 0.668/0.741 | n/a |
| replay wall time s | 9.60 | 10.60 | 11.10 | 13.10 | n/a |

Reading the table:
- **Stereo crown peaks** stand in for truth:
  - definition: local maxima of the stereo canopy-height model, CHM > 1.5 m, at least 3 m apart; there are 63;
  - "Outlines with 0 peaks" = fragments or false trees; "2+ peaks" = outlines that merge crowns.
  - It is a proxy: a large crown can have two peaks, and a small tree can have none.
- **Height above stereo ground** uses the CHM's local ground (the 15th percentile of cell lows over a 9 m window).
  That ground is biased about 1.5 m low by far-range noise, so read this row only for comparisons between systems.
  The per-frame canopy tops in the geometry check are p50 4.6 m.
- **Existing heights** are the fuser's *median* feature height (the only height in the companion bag). Ours are
  canopy tops. The heights are therefore not comparable in absolute terms.
- **Implausible heights:**
  - *Above the drone* (all our systems, about 57 frames): every case is the landing, t > 500 s, with the drone
    below 7.5 m over 7-8 m crowns. That is an artefact of a stream without phases, where the scorer's mission
    window runs to the end. It is not a mapping error.
  - *Height <= 0* (existing, 3232 of 6056 snapshots, from t = 72 s): the existing system publishes trees whose
    median height is at or below the take-off point. Navigation would descend to that height. The ground under
    those trees is lower in world_nwu than the take-off point, so this is not necessarily physically wrong, but
    the median height is not a safe descent reference.
- **Cost**: ms per frame on x86 (tm_dev), from `stage_ms`. The existing system has no timing in the bag.
- **Stability**:
  - no retreats, no ID reuse and no reappearances for any system;
  - the existing system has the most centre jumps (75) and height jumps (296);
  - cand_obj_m4's rate limits remove every jump (0 / 0, against 42 / 39 for the M3 cand_obj);
  - IDs vanished: cand_grid 25 and existing 23 are merges; simple has 8.

## Qualitative comparison (plots in `results/real_flight_plots/`: `final_<system>.png`, `final_overlay.png`)

The stereo CHM shows a regular orchard:
- about 60 crowns, roughly 7 m in diameter, in rows about 8 m apart;
- many crowns touch;
- an open strip at x ~ 0, y ~ -25 (no trees).

- **simple** (59 trees): about one outline per crown, but grow-only hulls of raw boxes make them the largest:
  median 68 m2, a 9.3 m equivalent diameter against roughly 7 m crowns.
  - 5 outlines hold two crown peaks.
  - It misses the crowns at (-11, -20) and (-5, -22), inside merged blobs.
- **cand_obj (M3)** (100 trees) and **cand_obj_m4** (102 trees): the best per-crown geometry. Outlines are the
  tightest (median 31 m2, 6.3 m) and only 2 merge two peaks. But they **over-fragment**:
  - 37 / 39 outlines contain no crown peak;
  - the nearest-neighbour centre distance p10 is 2.8 / 3.2 m, against about 7 m for every other system;
  - the fragments are 6-9 m2 pieces at crown edges and in the gaps between touching crowns.
  - This is the one real-data failure that the synthetic and Isaac results did not show.
  - Missed crowns: M3 4 ((-58, 10), (-43, 6), (-5, -22), (4, -49)); M4 3 ((-43, 6), (-31, -3), (-5, -22)).
- **cand_grid** (55 trees): one outline per crown in the dense left half. On the right half it merges large
  areas: outline 1 holds 6 crown peaks, and 7 outlines hold 2 or more. Its p90 area is 111 m2.
  - It misses 5 crowns ((-53, -35), (-43, 6), (-39, -46), (-11, -20), (4, -49)).
  - It has 6 small outlines without a peak; one (ID 76 at (-6, -27), 18 m2) appears in no other system.
- **existing** (54 trees):
  - 8 outlines merge two or three crowns (for example IDs 20, 4, 22 and 194 along the rows);
  - 5 crowns missed ((-58, 10), (-48, -34), (-11, -20), (-5, -22), (-1, -21));
  - median-height references, some <= 0.
- **Agreement**: almost every final tree of one system overlaps a tree of each other system (overlap >= 20 % of the
  smaller outline). For example, existing trees in cand_obj 53/54 and cand_grid 53/54; cand_obj trees in existing
  93/100. No system finds a crown the others all miss.
  - The crown at (-5, -22), next to the open strip, is missed by four of the five maps; cand_grid covers it
    within a merged outline.
  - Systems differ in how crowns are split or merged, not in which trees they see.

**Tree count against the stereo points:**
- about 60-63 crowns are visible;
- simple 59, cand_grid 55 and existing 54 are close, because they merge about as often as they split;
- cand_obj (100-102) has about 40 surplus fragments.

A config-only fix for cand_obj on real data is in `real_flight_profile.md`.

## Unverified

- See `real_flight_geometry.md`: box pixels in raw vs rectified coordinates, roll, pose-z drift, and the
  crown-top vs altitude residual.
- The crown peaks are a proxy, not truth. The fragment count depends on the 3 m peak separation.
- The existing system ran in the companion replay, at an unknown commit and configuration. Its chain is presumably
  tf_static's 75 deg with the calibration offset of 4.918 px; we cannot verify that from its outputs.
