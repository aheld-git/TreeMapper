#!/bin/bash
# tm_sim_ros entrypoint.
#   bridge   roscore + ros1_bridge parameter_bridge (topics from bridge_topics.yaml)
#   <cmd>    run <cmd> with ROS 1 Noetic sourced (never mix in ROS 2: rospy and
#            the Humble rosgraph_msgs shadow each other)
#
# `set -euo pipefail` is on for this script's own logic. The ROS setup scripts are not
# nounset-clean, so they are sourced through `source_ros`, which drops -u around the
# source and restores it right after (a failing setup script still aborts via -e).
set -euo pipefail

source_ros() {
    set +u
    # shellcheck disable=SC1090
    source "$1"
    set -u
}

if [ "${1:-}" != "bridge" ]; then
    source_ros /opt/ros/noetic/setup.bash
    exec "$@"
fi

BRIDGE_PID=""
CORE_PID=""
cleanup() {
    trap '' TERM INT
    [ -n "$BRIDGE_PID" ] && kill "$BRIDGE_PID" 2>/dev/null || true
    [ -n "$CORE_PID" ] && kill "$CORE_PID" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap 'cleanup; exit 0' TERM INT

source_ros /opt/ros/noetic/setup.bash
roscore &
CORE_PID=$!

# Fail loud if the master never comes up (30 s).
for _ in $(seq 1 60); do
    rosparam list >/dev/null 2>&1 && break
    kill -0 "$CORE_PID" 2>/dev/null || { echo "FATAL: roscore exited during startup" >&2; exit 1; }
    sleep 0.5
done
if ! rosparam list >/dev/null 2>&1; then
    echo "FATAL: ROS master not reachable after 30 s" >&2
    cleanup
    exit 1
fi

# Isaac is the time authority: every ROS 1 node must run on /clock.
rosparam set /use_sim_time true
rosparam load "${BRIDGE_TOPICS:-/tm/bridge_topics.yaml}"

# ros1_bridge needs ROS 1 first, then ROS 2, in this one process only.
source_ros /ros2_humble/install/setup.bash
ros2 run ros1_bridge parameter_bridge &
BRIDGE_PID=$!

# parameter_bridge dies at once on a bad topic list or type; do not report a healthy container then.
sleep 5
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "FATAL: parameter_bridge exited within 5 s of start" >&2
    wait "$BRIDGE_PID" || true
    cleanup
    exit 1
fi
echo "tm_sim_ros: roscore (pid $CORE_PID) and parameter_bridge (pid $BRIDGE_PID) up"

# Stay up while the bridge lives; a dead bridge ends the container non-zero.
set +e
wait "$BRIDGE_PID"
rc=$?
set -e
echo "FATAL: parameter_bridge exited (rc=$rc)" >&2
cleanup
exit 1
