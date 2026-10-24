"""Convert the existing system's real-flight map (companion bag) into the scorer's map-stream JSONL contract.

    python3 -m treemapper_tools.real_existing \
        --companion /data/real/BB_treehopping_farm_companion_2026-10-23-18-01-20.bag \
        --out /data/work/extracted/real_farm/map_existing.jsonl

Source topics (one message each per fuser callback, same bag time):

* ``/object_fusion/polygon_marker`` (MarkerArray): a DELETEALL marker, then per published tree a
  ``ns="polygon"`` LINE_STRIP (closed: last point = first) with ``id`` = the fusion ID, and a
  ``ns="polygon_labels"`` TEXT marker at the label position. Frame ``world_nwu``.
* ``/object_fusion/fused_bboxes`` (Float64MultiArray): per tree ``[id, min_x, max_x, min_y, max_y,
  median_height]`` (``ObjectFusionWrapper::publishFusedBboxes``). It is the only height the companion bag
  carries: the **median** feature height (world z), not the ``max_height`` the Isaac snapshot used.

One snapshot per MarkerArray message: ``t`` = bag time (the flight bag's clock, shared with frames.bin and
mission.json), ``frame`` = ``world_nwu`` (no conversion: the real flight has no sim ``world`` frame),
``centre`` = area centroid of the outline, ``height`` = the fused_bboxes median height of that ID in the
message with the same bag time (else the nearest message within 0.2 s that lists the ID: the two topics
are published from one callback but are not always recorded with equal bag times; else undefined),
``sprayed`` = false (the
real flight sprayed nothing; the marker colour is not a sprayed flag here).
"""
from __future__ import annotations

import argparse
import bisect
import collections
import json
from typing import Dict, List, Tuple

from .contract import Snapshot, Tree, polygon_centroid, write_snapshots

MARKERS = "/object_fusion/polygon_marker"
BBOXES = "/object_fusion/fused_bboxes"
LINE_STRIP, DELETEALL = 4, 3


def bboxes_to_heights(data) -> Dict[int, float]:
    """Float64MultiArray data [id, minx, maxx, miny, maxy, h] * n -> {id: h}."""
    if len(data) % 6:
        raise ValueError("fused_bboxes length %d is not a multiple of 6" % len(data))
    return {int(round(data[i])): float(data[i + 5]) for i in range(0, len(data), 6)}


def markers_to_trees(markers, heights, fallback=()) -> Tuple[List[Tree], str, collections.Counter]:
    """``heights`` = {id: h} of the matching bbox message; ``fallback`` = further {id: h} dicts tried in
    order for IDs the first lacks."""
    cnt = collections.Counter()
    trees, frame = [], ""
    for m in markers:
        if m.action == DELETEALL:
            continue
        if m.ns != "polygon" or m.type != LINE_STRIP:
            continue
        frame = m.header.frame_id or frame
        pts = [(float(p.x), float(p.y)) for p in m.points]
        if len(pts) >= 2 and pts[0] == pts[-1]:
            pts = pts[:-1]
        if len(pts) < 3:
            cnt["degenerate_outline"] += 1
        h = heights.get(int(m.id))
        if h is None:
            for fb in fallback:
                if int(m.id) in fb:
                    h = fb[int(m.id)]
                    cnt["height_from_neighbour_message"] += 1
                    break
        if h is None:
            cnt["no_height"] += 1
        trees.append(Tree(int(m.id), pts, polygon_centroid(pts), h, False))
    return trees, frame, cnt


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--companion", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tol", type=float, default=0.2, help="max |dt| between a marker and its bbox message")
    a = ap.parse_args(argv)
    import rosbag

    marks, boxes = [], []
    with rosbag.Bag(a.companion) as b:
        for topic, msg, bt in b.read_messages(topics=[MARKERS, BBOXES]):
            if topic == MARKERS:
                marks.append((bt.to_sec(), msg))
            else:
                boxes.append((bt.to_sec(), bboxes_to_heights(msg.data)))
    bt_ = [t for t, _ in boxes]
    cnt = collections.Counter()
    frames = collections.Counter()
    snaps = []
    for t, msg in marks:
        k = bisect.bisect_left(bt_, t)
        near = sorted((j for j in (k - 2, k - 1, k, k + 1) if 0 <= j < len(bt_) and abs(bt_[j] - t) <= a.tol),
                      key=lambda j: abs(bt_[j] - t))
        best = near[0] if near else None
        heights = boxes[best][1] if best is not None else {}
        fallback = [boxes[j][1] for j in near[1:]]
        if best is None:
            cnt["snapshots_without_bbox_message"] += 1
        elif bt_[best] != t:
            cnt["snapshots_bbox_nearest"] += 1
        trees, fr, c = markers_to_trees(msg.markers, heights, fallback)
        cnt.update(c)
        frames[fr or "(none)"] += 1
        snaps.append(Snapshot(t, trees, "world_nwu"))
    snaps.sort(key=lambda s: s.t)
    n = write_snapshots(a.out, snaps)
    ids = {tr.id for s in snaps for tr in s.trees}
    meta = {"companion": a.companion, "snapshots": n, "marker_frames": dict(frames), "counts": dict(cnt),
            "distinct_ids": len(ids), "final_trees": len(snaps[-1].trees) if snaps else 0,
            "t_first": snaps[0].t if snaps else None, "t_last": snaps[-1].t if snaps else None,
            "height_field": "fused_bboxes median height (world z)"}
    with open(a.out + ".meta.json", "w") as f:
        json.dump(meta, f, indent=1)
    print(json.dumps(meta, indent=1))


if __name__ == "__main__":
    main()
