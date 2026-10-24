#!/usr/bin/env bash
# Mutation check of the cand_obj rule tests (final review S6): every mutant in mutants.py breaks one rule in
# core/src/candidates/cand_obj/cand_obj.cpp; the cand_obj gtests must fail for it ("KILLED").
# Run inside tm_dev:  docker exec tm_dev bash -lc 'cd /ws && scripts/mutation/run_mutants.sh'
# Scratch goes to $TM_MUT (default /data/work/mutation), never into the repo. 4 parallel compiles.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
D="${TM_MUT:-/data/work/mutation}"
mkdir -p "$D/src" "$D/bin" "$D/out"
python3 - "$ROOT" "$D" <<'PY'
import sys
root, d = sys.argv[1], sys.argv[2]
sys.path.insert(0, root + '/scripts/mutation')
from mutants import M
src = open(root + '/core/src/candidates/cand_obj/cand_obj.cpp').read()
names = ['baseline']
open(d + '/src/baseline.cpp', 'w').write(src)
for name, pairs in M:
    s = src
    for old, new in pairs:
        assert s.count(old) == 1, (name, s.count(old), old)
        s = s.replace(old, new)
    open('%s/src/%s.cpp' % (d, name), 'w').write(s)
    names.append(name)
open(d + '/names.txt', 'w').write('\n'.join(names) + '\n')
PY
build() {  # $1 = mutant name
  g++ -std=c++17 -O1 -I"$ROOT/core/include" "$ROOT/core/tests/candidates/cand_obj/test_cand_obj.cpp" "$D/src/$1.cpp" \
    "$ROOT/core/src/geometry.cpp" "$ROOT/core/src/types.cpp" "$ROOT/core/src/simple_mapper.cpp" "$ROOT/core/src/tree_map.cpp" \
    "$ROOT/core/src/mapper_registry.cpp" -lgtest -lgtest_main -pthread -o "$D/bin/$1" > "$D/out/$1.build" 2>&1 \
    && "$D/bin/$1" > "$D/out/$1.log" 2>&1 || true
}
export -f build; export ROOT D
xargs -P4 -I{} bash -c 'build {}' < "$D/names.txt"
rc=0
for n in $(cat "$D/names.txt"); do
  if ! grep -qE "test suites? ran" "$D/out/$n.log" 2>/dev/null; then echo "BUILD/RUN ERROR  $n (see $D/out/$n.build)"; rc=1; continue; fi
  f=$( (grep -E "^\[  FAILED  \] [A-Za-z]" "$D/out/$n.log" || true) | awk '{print $4}' | sort -u | tr '\n' ' ')
  if [ "$n" = baseline ]; then
    [ -z "$f" ] && echo "baseline passes" || { echo "BASELINE FAILS: $f"; rc=1; }
  elif [ -z "$f" ]; then echo "SURVIVED  $n"; else echo "KILLED    $n  by: $f"; fi
done
exit $rc
