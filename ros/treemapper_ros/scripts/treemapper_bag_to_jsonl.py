#!/usr/bin/env python3
"""Convert a recorded treemapper_msgs/TreeMap topic into the scorer's map-stream JSONL contract.

    treemapper_bag_to_jsonl.py --bag rec.bag --out-map map.jsonl [--out-targets targets.json]
                               [--map-topic /treemapper/map] [--target-topic /treemapper/hop_target]
                               [--timing-topic /treemapper/stage_timing]

One JSONL line per TreeMap message: {"t", "frame", "trees": [{id, outline, centre, height, sprayed}], "stage_ms"}
(the same form treemapper_replay writes; docs/SCORER.md). t = header.stamp (the node stamps every snapshot with the
ROS clock = bag time under rosbag play --clock, the clock of mission.json). Stamps are made strictly increasing.
stage_ms = the stage_timing message with the same stamp, if recorded. --out-targets writes the hop-target timeline
(the IDs this mapper received, [[t, id], ...], first entry [t_first_map, -1]) for the scorer's mission_resolved.json.
"""
from __future__ import annotations

import argparse
import json
import math


def entry(e):
    return {"id": int(e.id), "outline": [[round(p.x, 4), round(p.y, 4)] for p in e.outline],
            "centre": [round(e.centre.x, 4), round(e.centre.y, 4)],
            "height": round(e.height, 4) if math.isfinite(e.height) else None, "sprayed": bool(e.sprayed)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out-map", required=True)
    ap.add_argument("--out-targets", default="")
    ap.add_argument("--map-topic", default="/treemapper/map")
    ap.add_argument("--target-topic", default="/treemapper/hop_target")
    ap.add_argument("--timing-topic", default="/treemapper/stage_timing")
    a = ap.parse_args(argv)
    import rosbag
    timing = {}
    maps, targets = [], []
    with rosbag.Bag(a.bag) as bag:
        for topic, msg, bt in bag.read_messages(topics=[a.map_topic, a.target_topic, a.timing_topic]):
            if topic == a.map_topic:
                maps.append((msg.header.stamp.to_sec(), msg.header.frame_id, [entry(e) for e in msg.trees]))
            elif topic == a.timing_topic:
                timing[round(msg.header.stamp.to_sec(), 6)] = dict(zip(msg.stages, msg.ms))
            else:
                targets.append([bt.to_sec(), int(msg.data)])
    maps.sort(key=lambda m: m[0])
    last = -math.inf
    n = 0
    with open(a.out_map, "w") as f:
        for t, frame, trees in maps:
            if t <= last:
                continue  # latched re-delivery / duplicate stamp
            last = t
            rec = {"t": round(t, 6), "frame": frame or "world", "trees": trees}
            st = timing.get(round(t, 6))
            if st:
                rec["stage_ms"] = {k: round(v, 4) for k, v in st.items()}
            f.write(json.dumps(rec, separators=(",", ":")) + "\n")
            n += 1
    if a.out_targets:
        t0 = maps[0][0] if maps else (targets[0][0] if targets else 0.0)
        tl = [[t0, -1]]
        for t, tid in sorted(targets):
            if tid != tl[-1][1]:
                tl.append([max(t, tl[-1][0]), tid])
        with open(a.out_targets, "w") as f:
            json.dump(tl, f)
    print("wrote %d snapshots (%d trees in the last), %d target changes" % (n, len(maps[-1][2]) if maps else 0, len(targets)))


if __name__ == "__main__":
    main()
