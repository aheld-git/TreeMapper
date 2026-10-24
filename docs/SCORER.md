# Scorer and contract files

The scorer (`tools/treemapper_tools/score.py`) judges **any** system that writes the output
contract as files. It never looks at how a map was made.

## Commands (inside tm_dev)

```
docker exec tm_dev bash -lc 'cd /ws && ./scripts/run_tests.sh'          # all tests (ctest + pytest)
docker exec tm_dev bash -lc 'cd /ws && ./scripts/score_baseline.sh'     # extract (once) + baseline scorecard
docker exec tm_dev bash -lc 'cd /ws/tools && python3 -m treemapper_tools.score \
    --map M.jsonl --gt ground_truth.json [--mission mission.json] [--camera camera_info.json] \
    --title "..." --out-json X.json --out-md X.md'
```

Python API: `from treemapper_tools.score import score, ScoreConfig, to_markdown`;
`score(snapshots, gt, mission=None, camera=None, cfg=None) -> dict`.
Readers/writers for every file: `treemapper_tools/contract.py`.

## Files

| File | Content |
|---|---|
| map stream `*.jsonl` | one line per published snapshot: `{"t", "frame", "trees": [{"id", "outline": [[x,y]..], "centre": [x,y], "height" (null = undefined), "sprayed"}], "stage_ms"?: {stage: ms}}` |
| `ground_truth.json` | `{"frame", "trees": [{"id", "outline", "centre", "radius", "height"}]}` |
| `mission.json` | `{"phases": [[t, "SCAN"]..], "targets": [[t, id]..], "pose": [[t, x, y, z]..]}` (all optional) |
| `camera_info.json` | `{"left": {"K", "width", "height", ...}, "right": {...}}` (only for timeliness) |

Emit a snapshot at a steady rate (the existing system: ~5 Hz) **including empty ones from the start
of the stream**; the per-frame "all" window averages over every snapshot. The final map is the last
snapshot. Times must share the clock of `mission.json`.

## Metric definitions

See the module docstring of `score.py` (authoritative) and `ScoreConfig` (all thresholds).
Key choices, with reasons in `docs/DECISIONS.md`:

- One-to-one matching = Hungarian on IoU, minimum IoU 0.10. Recall/precision use it.
- An entry *covers* a truth tree if it holds >= 30 % of the tree's area; covering >= 2 trees = merged entry.
- An entry is *owned* by the truth tree holding the largest share (>= 20 %) of the entry's area;
  >= 2 entries owned by one tree = split (extra entry with IoU >= 0.3 = duplicate, else fragment);
  unowned entries = false trees.
- Spill = published area outside every true canopy / published area; neighbour spill = a matched
  entry's area inside *other* true canopies / its area.
- Stability compares consecutive publications of the same ID: retreat = > 2 % of the previous outline
  lost, jump = centre moved > 1 m, height jump = > 1 m. ID switches are reported total and after the
  list first locks (first PLANNING) — only the latter matter to navigation.
- Safety: undefined height frames; IDs first published in PLANNING/HOPPING/SPRAYING; non-target
  changes (outline/height change, vanish, appear) in the descent window = SPRAYING frames plus HOPPING
  frames within 1.5 m (xy) of the point where the next SPRAYING phase starts; sprayed trees changing;
  new trees centred inside a sprayed outline.
- Spray outcome: a truth tree is sprayed iff a sprayed entry's centre lies inside its canopy.
- Timeliness: first snapshot covering the tree minus the first time its centre came within the image
  half-diagonal footprint (yaw-free upper bound), from any phase and from SCAN only.
