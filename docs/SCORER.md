# Scorer and contract files

The scorer (`tools/treemapper_tools/score.py`) judges **any** system that writes the output contract as files.
It never looks at how a map was made. The module docstring is the authoritative definition list, and `ScoreConfig`
holds every threshold. This page is the summary, with reasons. Decisions R1-R13 in `docs/DECISIONS.md` (scorer review series) record
the 2026-10-24 review fixes; R14 the truth-aware implausible-height test (M3).

## Commands (inside tm_dev)

```
docker exec tm_dev bash -lc 'cd /ws && ./scripts/run_tests.sh'          # all tests (ctest + pytest)
docker exec tm_dev bash -lc 'cd /ws && ./scripts/score_baseline.sh'     # extract (once) + baseline scorecard
docker exec tm_dev bash -lc 'cd /ws/tools && python3 -m treemapper_tools.score \
    --map M.jsonl [--gt ground_truth.json] [--mission mission.json] [--camera camera_info.json] \
    --title "..." --out-json X.json --out-md X.md'
```

If you omit `--gt`, the scorer runs in **no-ground-truth mode**: stability + safety + cost only, for the real
flight. Python API: `score(snapshots, gt=None, mission=None, camera=None, cfg=None, gt_frame=None) -> dict`.
`tools/tests/test_score_baseline.py` pins the existing-system scorecard. If a scorer change moves it, update the
pin, `results/baseline_existing_system*` and DECISIONS together.

## Files

| File | Content |
|---|---|
| map stream `*.jsonl` | one line per published snapshot: `{"t", "frame", "trees": [{"id", "outline": [[x,y]..], "centre": [x,y], "height" (null = undefined), "sprayed"}], "stage_ms"?: {stage: ms}}` |
| `ground_truth.json` | `{"frame", "trees": [{"id", "outline", "centre", "radius", "height"}], "height_reliable"?: bool}` |
| `mission.json` | `{"phases": [[t, "SCAN"]..], "targets": [[t, id]..], "pose": [[t, x, y, z]..]}` (all optional; sorted on read) |
| `camera_info.json` | `{"left": {"K", "width", "height", ...}, "right": {...}}` (timeliness, recall-in-view) |

Times must share the clock of `mission.json`. Publish rate does not matter: every stream metric is time-based or
counted per onset.

## Input hygiene (fails loudly)

- Every snapshot's `frame` must equal the ground truth's `frame`, or `score()` raises `ValueError`. Without
  ground truth, the stream must use one frame.
- If a ground-truth outline is degenerate, it is replaced by a circle of its `radius` (or of its mean vertex
  distance) and listed in `gt_repaired_ids`. If neither exists, `ValueError`.
- **Invalid entry** = non-finite centre or outline coordinate, < 3 vertices, or zero area. It counts as
  published (lowers precision), never matches, and is a safety count. A non-finite height is rule 3's own count.
- **Duplicate IDs** in one snapshot are a safety count. Per-ID change checks compare the whole multiset of
  entries with that ID; drift uses the largest entry; retreat uses the union.

## Detection and geometry (with ground truth)

- One-to-one matching = Hungarian on IoU **after zeroing every IoU < 0.10**, so sub-threshold pairs cannot steer
  the assignment. Recall and precision use it. `recall_in_view` = matched truth trees among those whose centre
  has already entered the camera footprint (the "ever in view" denominator; needs pose + camera).
- An entry *covers* a truth tree if it holds >= 30 % of the tree's area; covering >= 2 trees = merged entry.
  *Owner* = truth tree holding the largest share (>= 20 %) of the entry; >= 2 entries owned = split tree
  (duplicate if IoU >= 0.3, else fragment). Unowned valid entries = false trees.
- Spill = published area outside every true canopy / published area. Neighbour spill = a matched entry's area
  inside *other* true canopies / its area.
- **Undefined heights are never rewarded.** A matched entry with an undefined height counts in
  `n_undefined_height` and enters height RMSE / max-abs as `undefined_height_error_m` = 15 m (about the scan
  altitude, worse than any finite answer in the flight envelope). Bias uses finite heights only.
- **GT height reliability.** If every truth height is identical (>= 2 trees) or the file says
  `"height_reliable": false`, `gt_height_reliable` is false. The scorecard then labels height bias / RMSE /
  errors "GT height unreliable (constant fallback)": reported, but not a headline and not pass/fail. The
  recorded Isaac mission is such a case: a constant 4.0 m fallback, while real canopy tops sit about +1.4 to
  +1.9 m higher (`results/geometry_check.md`). Height *stability* metrics do not use ground truth and keep full
  weight. Timeliness uses the truth height only for the camera range (a 1-2 m error moves the footprint radius
  by about 10 %).

## Per-frame means (time-weighted)

The latest published snapshot is sampled on a fixed 0.2 s grid over the **mission window**: first SCAN to the
first RETURN after it, or the end of the stream; without phases, first to last snapshot. Before the first
snapshot the map counts as empty. A system cannot improve its mean by publishing faster when the map is good.
Second variant: from the first non-empty snapshot to the window end.

## Stability (no ground truth needed unless noted)

- **Retreat:** lost = area(U - current) / area(U), where U = union of every outline the ID has ever published.
  Lost > 2 % is a retreat, counted once per onset. An outline going invalid loses 1.0. Max lost fraction is
  reported overall and per ID. Slow shrink can no longer hide between consecutive snapshots.
- **Drift:** per ID, max centre / height distance from its first published value and from its value when first
  seen in a locked phase (`*_since_lock`).
- **Jumps:** centre (height) differs by > 1 m from any value of the same ID in the last 1 s, or from its last
  value. For height, the last *finite* value. Counted once per onset.
- **Reuse (geometric):** an ID that disappears and reappears with an outline overlapping its pre-gap outline by
  < 10 % (intersection / smaller area). With ground truth, `id_reuse_by_owner` is also reported. A reused ID gets
  fresh references (it is a different tree).
- With ground truth: ID switches (a tree's matched ID changes) and meaning changes (an ID's owner changes),
  total and after the list first locks.

## Safety (all must be 0; `n/a` = not judgeable from the inputs given)

| Count | Definition |
|---|---|
| `undefined_height_frames` | snapshots with any non-finite / missing height |
| `invalid_entry_frames` | snapshots with an invalid entry (see above) |
| `duplicate_id_frames` | snapshots with an ID used twice |
| `implausible_height_frames` | a finite height <= 0, or (pose given, inside the mission window) above the drone's altitude. With **reliable** truth heights (R14) an above-the-drone entry counts only if it is also above its truth top + 0.5 m (`implausible_truth_tol_m`); truth top = the tallest truth tree holding >= 20 % of the entry, and an entry no truth tree owns keeps the plain test. Without truth, or with unreliable truth heights (the Isaac recording), the plain definition applies. Excused entries: `implausible_height_excused_entries` |
| `new_trees_after_scan` | IDs first published, **or reused**, while the phase is PLANNING / HOPPING / SPRAYING |
| `locked_id_vanished` | an ID present in the previous snapshot is gone while the phase is locked (`_targets`: it was the hop target) |
| `nontarget_changes_in_descent` | in a descent-window snapshot, any tree other than the current target changes outline, centre or height, vanishes, or appears |
| `sprayed_tree_changes` | the entries of an ID are recorded when it is first sprayed; afterwards it vanishing, losing the flag, or differing from the record (outline, centre, height) is a violation, once per onset of each kind |
| `new_tree_inside_sprayed` | a new or reused ID whose centre lies inside, or whose area lies >= 30 % inside, the union of every outline ever published as sprayed |

**Descent window** (`descent_kinds`), judged per snapshot:
- *approach*: HOPPING with a target, drone xy within 1.5 m of where the next SPRAYING starts;
- *spraying*: every SPRAYING snapshot;
- *climb*: after each SPRAYING, whatever phase follows, until the drone first leaves 1.5 m of that hover point.

The climb is defined by distance, not altitude, symmetric with the approach. Over the hover point the drone is
low, whatever `/mission_state` says. In the recording the drone stays within 0.1 m while it climbs from about
8.4 m to 10.8 m.

If the mission has phases but no target series, the descent count is `n/a`. Without pose, every HOPPING snapshot
with a target is judged, and the climb is not.

Informational: the target's height range during its own approach / spraying window.

## Spray outcome (ground truth, whole stream)

An ID's spray point is its centre when its sprayed flag first appeared. A truth tree is sprayed by every distinct
ID whose spray point lies inside its canopy: >= 2 IDs = sprayed twice; inside no canopy = orphan. A sprayed tree
that vanishes and is re-created and re-sprayed under a new ID therefore counts as sprayed twice.

## Timeliness and cost

First publish of a truth tree = first snapshot where it is one-to-one **matched** to a valid entry with a
**finite height**, minus the first time its centre entered the yaw-free image half-diagonal footprint. This is
measured from any phase, and from SCAN only. Negative latencies (e.g. a giant polygon published before the tree
was in view) are listed and clamped to 0. Cost = per-stage `stage_ms` statistics if the stream carries them.
