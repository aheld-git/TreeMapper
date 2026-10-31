#!/usr/bin/env bash
# One command for every test: C++ core (CMake + gtest/ctest) and Python tools (pytest).
# Run inside the tm_dev container:
#   docker exec tm_dev bash -lc 'cd /ws && ./scripts/run_tests.sh'
# Build output goes to $TM_BUILD (default /data/work/build), never into the repo.
# TM_CANDIDATES="cand_a" builds only that candidate (default: all).
# TM_ROS_TESTS=1 also builds the ROS packages (treemapper_msgs, treemapper_ros, corridor_nav_msgs, corridor_nav_ros,
# sim_perception) in the catkin workspace $TM_CATKIN_WS (default /data/work/catkin_nav; the packages are symlinked into
# its src/) and runs `catkin run_tests corridor_nav_ros` (gtests + the nav_sim_lite closed loop) + catkin_test_results.
# Needs ROS Noetic (/opt/ros/noetic). Off by default: the default run is unchanged.
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

echo "== ROS perception (sim_perception, pure Python, no onnxruntime): pytest"
(cd "$ROOT" && PYTHONPATH="$ROOT/ros/sim_perception/src${PYTHONPATH:+:$PYTHONPATH}" python3 -m pytest -q -p no:cacheprovider ros/sim_perception/test) || rc=1

if [ "${TM_ROS_TESTS:-0}" = "1" ]; then
  WS="${TM_CATKIN_WS:-/data/work/catkin_nav}"
  ROS_PKGS="treemapper_msgs treemapper_ros corridor_nav_msgs corridor_nav_ros sim_perception"
  ROS_JOBS="${TM_ROS_JOBS:-2}"
  echo "== ROS (catkin, $WS): build $ROS_PKGS (-j$ROS_JOBS)"
  if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "TM_ROS_TESTS=1 needs ROS Noetic (/opt/ros/noetic/setup.bash not found)"
    echo "TESTS FAILED"
    exit 1
  fi
  ros_fail() { echo "$1"; echo "TESTS FAILED"; exit 1; }
  # ROS setup scripts read unset variables: no `set -u` while sourcing them
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  set -u
  mkdir -p "$WS/src"
  for p in $ROS_PKGS; do ln -sfn "$ROOT/ros/$p" "$WS/src/$p"; done
  if [ ! -d "$WS/.catkin_tools" ]; then
    (cd "$WS" && catkin init > /dev/null && catkin config --extend /opt/ros/noetic \
       --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo > /dev/null) || ros_fail "catkin init of $WS failed"
  fi
  # shellcheck disable=SC2086
  if ! (cd "$WS" && catkin build $ROS_PKGS -j"$ROS_JOBS" --no-status) > "$WS.build.log" 2>&1; then
    grep -E "error|Error" "$WS.build.log" | head -40 || true
    ros_fail "ROS BUILD FAILED (full log: $WS.build.log)"
  fi
  grep -E "warning" "$WS.build.log" | head -20 || true
  set +u
  # shellcheck disable=SC1091
  source "$WS/devel/setup.bash"
  set -u
  if ! (cd "$WS" && catkin build corridor_nav_ros --no-deps --no-status --make-args tests --) > "$WS.tests.build.log" 2>&1; then
    grep -E "error|Error" "$WS.tests.build.log" | head -40 || true
    ros_fail "ROS TEST BUILD FAILED (full log: $WS.tests.build.log)"
  fi
  grep -E "warning" "$WS.tests.build.log" | head -20 || true
  echo "== ROS: catkin run_tests corridor_nav_ros (gtests + nav_sim_lite)"
  rm -rf "$WS/build/corridor_nav_ros/test_results"   # no stale verdicts from an earlier run
  (cd "$WS" && catkin run_tests corridor_nav_ros --no-deps --no-status) > "$WS.run_tests.log" 2>&1 || true
  # catkin_test_results is the verdict (run_tests' exit status does not reflect every failure). It also exits 0 on an
  # empty results directory ("Summary: 0 tests"), so a run that produced no results at all must fail here.
  ros_verdict() {   # $1 = results directory; fails on failed tests, a missing directory, or a total of 0 tests
    local out total
    if [ ! -d "$1" ]; then echo "ROS test results missing: $1"; return 1; fi
    out="$(catkin_test_results "$1" 2>&1)" || { echo "$out"; return 1; }
    echo "$out"
    total="$(printf '%s\n' "$out" | sed -n 's/^Summary: \([0-9][0-9]*\) tests.*/\1/p' | tail -n 1)"
    if [ -z "$total" ] || [ "$total" -eq 0 ]; then echo "ROS tests: no tests ran (total ${total:-unparsed})"; return 1; fi
  }
  if ! (cd "$WS" && ros_verdict build/corridor_nav_ros); then
    grep -E "FAIL|Failure|Error|error" "$WS.run_tests.log" | head -40 || true
    echo "ROS tests failed (full log: $WS.run_tests.log)"
    rc=1
  fi
fi

if [ "$rc" -eq 0 ]; then echo "ALL TESTS PASSED"; else echo "TESTS FAILED"; fi
exit "$rc"
