#!/bin/bash
# End-to-end check of the ROS path on the recorded Isaac mission, inside tm_dev (ROS master local to the container):
#   roscore + replay_isaac.launch (treemapper_node + nav_emulator + rosbag play --clock + rosbag record),
#   live probes while it runs (heartbeat, get_trees all / by id), then: recorded map -> scorer JSONL,
#   the offline replay of the SAME build on frames.bin, and compare_map_streams.py (final trees/IDs + scorecards).
#
#   source <ws>/devel/setup.bash
#   ros_replay_check.sh OUT_DIR [START_S=45] [DURATION_S=0 (to the end)] [QUANTUM_PX=0.0] [RATE=1.0]
#
# Env: TM_EXTRACTED (M1 extraction dir), TM_TOOLS (repo tools/), TM_CFG (mapper .cfg), TM_REPLAY (treemapper_replay).
set -uo pipefail
OUT=${1:?usage: ros_replay_check.sh OUT_DIR [start] [duration] [quantum] [rate]}
START=${2:-45}; DUR=${3:-0}; Q=${4:-0.0}; RATE=${5:-1.0}
PKG=$(rospack find treemapper_ros)
EXTRACTED=${TM_EXTRACTED:-/data/work/extracted/isaac_2026-10-23-15-34-29}
TOOLS=${TM_TOOLS:-$PKG/../../tools}
CFG=${TM_CFG:-}
REPLAY=${TM_REPLAY:-$(catkin_find --first-only treemapper_ros treemapper_replay 2>/dev/null || true)}
[ -x "$REPLAY" ] || REPLAY=$(dirname "$(rospack find treemapper_ros)")/../devel/lib/treemapper_ros/treemapper_replay
mkdir -p "$OUT/ros" "$OUT/offline"
rm -f "$OUT/ros/rec.bag" "$OUT/ros/rec.bag.active"
log() { echo "[ros_replay_check $(date +%H:%M:%S)] $*"; }

if rostopic list > /dev/null 2>&1; then
  echo "a ROS master is already running at $ROS_MASTER_URI: stop it first (this script owns its master)" >&2; exit 1
fi
roscore -p 11311 > "$OUT/ros/roscore.log" 2>&1 &
CORE=$!
for _ in $(seq 1 30); do rostopic list > /dev/null 2>&1 && break; sleep 0.5; done
ARGS=(start:="$START" duration:="$DUR" rate:="$RATE" disparity_quantum_px:="$Q" record:="$OUT/ros/rec.bag" output:=log
      extracted:="$EXTRACTED" tools_path:="$TOOLS")
[ -n "$CFG" ] && ARGS+=(mapper_config_file:="$CFG")
log "roslaunch replay_isaac.launch ${ARGS[*]}"
T0=$(date +%s)
roslaunch treemapper_ros replay_isaac.launch "${ARGS[@]}" > "$OUT/ros/launch.log" 2>&1 &
LAUNCH=$!

# wait for the player to appear, probe once the map is non-empty, then wait for the player to finish
for _ in $(seq 1 60); do rosnode list 2>/dev/null | grep -q '^/player$' && break; sleep 1; done
PROBED=0
while rosnode list 2>/dev/null | grep -q '^/player$'; do
  if ! kill -0 $LAUNCH 2>/dev/null; then log "roslaunch died"; break; fi
  if [ $PROBED = 0 ] && timeout 5 rostopic echo -n1 /treemapper/map/trees[0]/id > /dev/null 2>&1; then
    { echo "== health"; timeout 5 rostopic echo -n1 /treemapper/health
      echo "== get_trees all (ids)"; timeout 5 rosservice call /treemapper/get_trees "id: -1" | grep -E "^found|^message|^  -\s*$|^    id:" | head -40
      ID=$(timeout 5 rostopic echo -n1 /treemapper/map/trees[0]/id 2>/dev/null | head -1)
      echo "== get_trees id $ID"; timeout 5 rosservice call /treemapper/get_trees "id: $ID" | grep -E "found|message|  id:"
      echo "== get_trees id 987654"; timeout 5 rosservice call /treemapper/get_trees "id: 987654" | grep -E "found|message"
    } > "$OUT/ros/probe.txt" 2>&1
    PROBED=1; log "probed: $(grep -c '    id:' "$OUT/ros/probe.txt") ids listed"
  fi
  sleep 2
done
log "player finished after $(( $(date +%s) - T0 )) s"
sleep 3
{ echo "== health (end)"; timeout 5 rostopic echo -n1 /treemapper/health; } >> "$OUT/ros/probe.txt" 2>&1
NODE_ALIVE=$(rosnode list 2>/dev/null | grep -c '^/treemapper$')
kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH 2>/dev/null
kill -INT $CORE 2>/dev/null; wait $CORE 2>/dev/null
log "node alive at end: $NODE_ALIVE"
grep -E "FATAL|ERROR|terminate|Segmentation|process has died" "$OUT/ros/launch.log" "$OUT"/ros/../ros/*.log 2>/dev/null | head -5

python3 "$PKG/scripts/treemapper_bag_to_jsonl.py" --bag "$OUT/ros/rec.bag" --out-map "$OUT/ros/map.jsonl" \
  --out-targets "$OUT/ros/targets.json" || exit 1

log "offline replay: $REPLAY"
"$REPLAY" --frames "$EXTRACTED/frames.bin" --out-map "$OUT/offline/map.jsonl" --out-targets "$OUT/offline/targets.json" \
  --out-counters "$OUT/offline/counters.json" --system cand_obj ${CFG:+--config "$(grep -v '^\s*#' "$CFG" | sed 's/#.*//' | tr -d ' ' | grep = | paste -sd';')"} \
  2> "$OUT/offline/replay.log"; tail -1 "$OUT/offline/replay.log"

python3 "$PKG/scripts/compare_map_streams.py" --a "$OUT/ros/map.jsonl" --a-targets "$OUT/ros/targets.json" --a-name ros \
  --b "$OUT/offline/map.jsonl" --b-targets "$OUT/offline/targets.json" --b-name offline --extracted "$EXTRACTED" \
  --tools-path "$TOOLS" --out-md "$OUT/compare.md" --out-json "$OUT/compare.json" --t-from "$(python3 -c "print(10 + $START)")"
echo "NODE_ALIVE_AT_END=$NODE_ALIVE"
