"""churn.py map.jsonl : per-snapshot centre steps of entries (same id in consecutive snapshots): max step, number of entry-snapshots where
the centre moved by more than 1 mm (the score.py change threshold), and the share of snapshots with at least one such move."""
import json, sys, math
prev = {}; mx = 0.0; moves = 0; snaps = 0; msnaps = 0; n = 0
for l in open(sys.argv[1]):
    d = json.loads(l); snaps += 1; any_ = False
    for e in d['trees']:
        c = e['centre']
        if e['id'] in prev:
            s = math.hypot(c[0] - prev[e['id']][0], c[1] - prev[e['id']][1]); n += 1
            mx = max(mx, s)
            if s > 1e-3: moves += 1; any_ = True
        prev[e['id']] = c
    msnaps += any_
print(f"max step {mx:.3f} m; entry-snapshots with centre move > 1 mm: {moves} of {n} ({100.0*moves/max(n,1):.2f} %); snapshots with at least one: {msnaps} of {snaps}")
