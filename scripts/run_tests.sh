#!/usr/bin/env bash
# One command for every test: C++ core (CMake + gtest/ctest) and Python tools (pytest).
# Run inside the tm_dev container:
#   docker exec tm_dev bash -lc 'cd /ws && ./scripts/run_tests.sh'
# Build output goes to $TM_BUILD (default /data/work/build), never into the repo.
# TM_CANDIDATES="cand_a" builds only that candidate (default: all).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${TM_BUILD:-/data/work/build}"
JOBS="${TM_JOBS:-8}"
rc=0

echo "== C++ core: configure + build ($BUILD/core, -j$JOBS)"
cmake -S "$ROOT/core" -B "$BUILD/core" -DCMAKE_BUILD_TYPE=RelWithDebInfo ${TM_CANDIDATES:+-DTM_CANDIDATES="$TM_CANDIDATES"} > "$BUILD.core.cmake.log" 2>&1 \
  || { cat "$BUILD.core.cmake.log"; exit 1; }
cmake --build "$BUILD/core" -j"$JOBS" 2>&1 | grep -E "error|warning|Error" || true
echo "== C++ core: ctest"
(cd "$BUILD/core" && ctest --output-on-failure) || rc=1

echo "== Python tools: pytest"
(cd "$ROOT/tools" && python3 -m pytest -q -p no:cacheprovider tests) || rc=1

if [ "$rc" -eq 0 ]; then echo "ALL TESTS PASSED"; else echo "TESTS FAILED"; fi
exit "$rc"
