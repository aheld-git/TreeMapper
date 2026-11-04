"""compare.py before.json after.json out.md [system]  -- paired before/after over the datasets of two evaluate.py JSON outputs
(synthetic.json or isaac.json), using evaluate.paired_comparison (difference = after - before)."""
import json, sys
sys.path.insert(0, '/ws/tools')
from treemapper_tools import evaluate as ev
b, a, out = sys.argv[1:4]; sysn = sys.argv[4] if len(sys.argv) > 4 else 'cand_obj'
B = json.load(open(b)); A = json.load(open(a))
rb = B['results'][sysn]; ra = A['results'][sysn]
P = ev.paired_comparison(rb, ra)
md = ev.paired_markdown('before', 'after', P)
md += ['', 'Run-to-run max diff (TM_REPEAT) before / after, per metric:', '']
for k in rb.get('run_to_run_max_diff', {}):
    md.append('- %s: %s / %s' % (k, rb['run_to_run_max_diff'][k], ra['run_to_run_max_diff'].get(k)))
open(out, 'w').write('\n'.join(md) + '\n')
json.dump(P['summary'], open(out.replace('.md', '.json'), 'w'), indent=1)
print('\n'.join(md[:40]))
