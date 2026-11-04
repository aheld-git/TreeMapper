"""For every SPRAYING end: the phases seen until the drone first leaves 1.5 m (scorer R3 window) and 2.0 m (cand_obj hold)
of the hover point, and whether SCAN starts inside the window."""
import json, sys, math, bisect, collections
m = json.load(open(sys.argv[1]))
ph = m['phases']; P = m['pose']; pt = [p[0] for p in P]
def phase_at(t):
    k = max(0, bisect.bisect_right([x[0] for x in ph], t) - 1); return ph[k][1]
ends = [(ph[i][0], ph[i+1][0]) for i in range(len(ph) - 1) if ph[i][1] == 'SPRAYING' and ph[i+1][1] != 'SPRAYING']
cnt = collections.Counter(); scan_in = 0; scan_starts = []
for ts, te in ends:
    k = bisect.bisect_right(pt, te) - 1; hx, hy = P[k][1], P[k][2]
    seen = set(); left = None
    for p in P[k:]:
        if math.hypot(p[1] - hx, p[2] - hy) > 1.5: left = p[0]; break
        seen.add(phase_at(p[0]))
    for s in seen: cnt[s] += 1
    if 'SCAN' in seen: scan_in += 1
    # distance from the hover point when the next SCAN starts
    nxt = [x[0] for x in ph if x[1] == 'SCAN' and x[0] > te]
    if nxt:
        q = P[bisect.bisect_right(pt, nxt[0]) - 1]; scan_starts.append(math.hypot(q[1] - hx, q[2] - hy))
print(f"sprays {len(ends)}; phases seen inside the 1.5 m climb-out window (sprays): {dict(cnt)}; SCAN inside: {scan_in}; "
      f"min distance from the hover point at the next SCAN start: {min(scan_starts) if scan_starts else None}")
