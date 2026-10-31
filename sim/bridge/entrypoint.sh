#!/bin/bash
# tm_sim_ros entrypoint.
#   bridge   roscore + ros1_bridge parameter_bridge (topics from bridge_topics.yaml)
#   <cmd>    run <cmd> with ROS 1 Noetic sourced (never mix in ROS 2: rospy and
#            the Humble rosgraph_msgs shadow each other)
set -e

ros1_env() { set +u; source /opt/ros/noetic/setup.bash; set -u; }

if [ "${1:-}" = "bridge" ]; then
    set +u
    source /opt/ros/noetic/setup.bash
    roscore &
    CORE_PID=$!
    trap 'kill $BRIDGE_PID $CORE_PID 2>/dev/null; wait; exit 0' TERM INT
    until rosparam list >/dev/null 2>&1; do sleep 0.5; done
    # Isaac is the time authority: every ROS 1 node must run on /clock.
    rosparam set /use_sim_time true
    rosparam load "${BRIDGE_TOPICS:-/tm/bridge_topics.yaml}"
    # ros1_bridge needs ROS 1 first, then ROS 2, in this one process only.
    source /ros2_humble/install/setup.bash
    ros2 run ros1_bridge parameter_bridge &
    BRIDGE_PID=$!
    wait $BRIDGE_PID
    kill $CORE_PID 2>/dev/null || true
    exit 0
fi

ros1_env
exec "$@"
