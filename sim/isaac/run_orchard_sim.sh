#!/usr/bin/env bash
# Runs orchard_sim.py inside the Isaac Sim 5.1 image with the ROS 2 Humble environment of the bridge spike,
# and forwards SIGTERM / SIGINT to the Python process (python.sh does not exec it, so `docker stop` would
# otherwise never reach the handler that writes deposition.json and rtf.json).
#
#   docker run -d --rm --name tm_isaac --gpus all --network tm_sim_net --entrypoint bash ... \
#     treehoppingnavigation-treehoppingsim:latest /ws/sim/isaac/run_orchard_sim.sh --spec ... --out ...
set -u
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
# Isaac 5.1 ships the Humble libs under isaacsim.ros2.bridge; 6.2 moved them to isaacsim.ros2.core (humble/ and jazzy/).
ROS2_LIB=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
[ -d /isaac-sim/exts/isaacsim.ros2.core/humble/lib ] && ROS2_LIB=/isaac-sim/exts/isaacsim.ros2.core/humble/lib
export LD_LIBRARY_PATH="$ROS2_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd /isaac-sim
./python.sh -u "$HERE/orchard_sim.py" "$@" &
PY=$!
forward() { pkill -TERM -P "$PY" 2>/dev/null || true; }
trap forward TERM INT
rc=0
while true; do
  wait "$PY"
  rc=$?                                   # a trapped signal also ends `wait` (rc > 128): loop until the process is gone
  kill -0 "$PY" 2>/dev/null || break
done
exit "$rc"
