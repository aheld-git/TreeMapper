"""same_maps.py a.jsonl b.jsonl : equal after dropping the timing field "stage_ms" (wall-clock, differs between any two runs)."""
import re, sys
def norm(p): return [re.sub(r',"stage_ms":\{[^}]*\}', '', l) for l in open(p)]
a, b = norm(sys.argv[1]), norm(sys.argv[2])
if a == b: print("IDENTICAL (%d snapshots)" % len(a)); sys.exit(0)
n = next((i for i in range(min(len(a), len(b))) if a[i] != b[i]), min(len(a), len(b)))
print("DIFFERENT at snapshot %d (of %d / %d)" % (n, len(a), len(b))); sys.exit(1)
