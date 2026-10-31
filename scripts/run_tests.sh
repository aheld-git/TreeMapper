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
# A failed build must fail the run. Piping the build into grep used to swallow its exit status, so a
# broken tree printed "ALL TESTS PASSED" after running stale test binaries from an earlier build.
if ! cmake --build "$BUILD/core" -j"$JOBS" > "$BUILD.core.build.log" 2>&1; then
  grep -E "error|Error" "$BUILD.core.build.log" | head -40 || true
  echo "C++ BUILD FAILED (full log: $BUILD.core.build.log)"
  echo "TESTS FAILED"
  exit 1
fi
grep -E "warning" "$BUILD.core.build.log" | head -20 || true
echo "== C++ core: ctest"
(cd "$BUILD/core" && ctest --output-on-failure) || rc=1

echo "== C++ nav (corridor_nav): configure + build ($BUILD/nav, -j$JOBS)"
cmake -S "$ROOT/nav/corridor_nav" -B "$BUILD/nav" -DCMAKE_BUILD_TYPE=RelWithDebInfo > "$BUILD.nav.cmake.log" 2>&1 \
  || { cat "$BUILD.nav.cmake.log"; echo "TESTS FAILED"; exit 1; }
if ! cmake --build "$BUILD/nav" -j"$JOBS" > "$BUILD.nav.build.log" 2>&1; then
  grep -E "error|Error" "$BUILD.nav.build.log" | head -40 || true
  echo "NAV BUILD FAILED (full log: $BUILD.nav.build.log)"
  echo "TESTS FAILED"
  exit 1
fi
grep -E "warning" "$BUILD.nav.build.log" | head -20 || true
echo "== C++ nav: ctest"
(cd "$BUILD/nav" && ctest --output-on-failure) || rc=1

echo "== Python tools: pytest"
(cd "$ROOT/tools" && python3 -m pytest -q -p no:cacheprovider tests) || rc=1

echo "== Sim layout/truth (pure Python): pytest"
(cd "$ROOT" && PYTHONPATH="$ROOT/tools:$ROOT/sim${PYTHONPATH:+:$PYTHONPATH}" python3 -m pytest -q -p no:cacheprovider sim/tests) || rc=1

if [ "$rc" -eq 0 ]; then echo "ALL TESTS PASSED"; else echo "TESTS FAILED"; fi
exit "$rc"
