"""Every published snapshot of a map.jsonl: how often a centre lies outside its own outline (and by how much), and how
many (entry, other) pairs have the entry's centre inside ANOTHER entry's outline (pairs = distinct, episodes = runs of
consecutive snapshots).  usage: centre_outline.py map.jsonl [counters.json]"""
import json, sys, math
def pip(x, y, poly):
    ins = False; j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]; xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi: ins = not ins
        j = i
    return ins
def outside(poly, p):
    b = 1e9
    for i in range(len(poly)):
        a, c = poly[i], poly[(i + 1) % len(poly)]
        ex, ey = c[0] - a[0], c[1] - a[1]; l2 = ex * ex + ey * ey
        s = 0 if l2 == 0 else max(0, min(1, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / l2))
        b = min(b, math.hypot(p[0] - a[0] - s * ex, p[1] - a[1] - s * ey))
    return b
nsnap = 0; nent = 0; nout = 0; outs = []; pairs = {}; prev = set(); episodes = 0; snaps_with_pair = 0
ent_ids = set(); out_ids = set()
for line in open(sys.argv[1]):
    d = json.loads(line); trees = [e for e in d['trees'] if len(e['outline']) >= 3]
    nsnap += 1
    bb = {e['id']: (min(p[0] for p in e['outline']), max(p[0] for p in e['outline']),
                    min(p[1] for p in e['outline']), max(p[1] for p in e['outline'])) for e in trees}
    cur = set()
    for e in trees:
        c = e['centre']; nent += 1; ent_ids.add(e['id'])
        if not pip(c[0], c[1], e['outline']):
            o = outside(e['outline'], c); nout += 1; outs.append(o); out_ids.add(e['id'])
        for f in trees:
            if f['id'] == e['id']: continue
            b = bb[f['id']]
            if c[0] < b[0] or c[0] > b[1] or c[1] < b[2] or c[1] > b[3]: continue
            if pip(c[0], c[1], f['outline']):
                k = (e['id'], f['id']); cur.add(k)
                pairs[k] = pairs.get(k, 0) + 1
    episodes += len(cur - prev); prev = cur
    if cur: snaps_with_pair += 1
outs.sort()
def q(p): return outs[int(p * (len(outs) - 1))] if outs else 0.0
print(f"snapshots {nsnap}, entry-snapshots {nent}, entries ever {len(ent_ids)}")
print(f"centre outside own outline: {nout} entry-snapshots ({100.0*nout/max(nent,1):.3f} %), {len(out_ids)} distinct entries; "
      f"outside distance median {q(.5):.3f} p90 {q(.9):.3f} max {outs[-1] if outs else 0:.3f} m")
print(f"centre inside ANOTHER entry's outline: {len(pairs)} distinct (entry, other) pairs, {episodes} episodes, {snaps_with_pair} snapshots with at least one; "
      f"entry-snapshots in a pair {sum(pairs.values())}")
if len(sys.argv) > 2:
    c = json.load(open(sys.argv[2]))
    print("counters: centre_clamped_to_outline =", c.get("centre_clamped_to_outline", 0), "; centre_support_fallback_latched =", c.get("centre_support_fallback_latched", 0))
