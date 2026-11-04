#!/usr/bin/env bash
# One-command Isaac-only closed loop: Isaac Sim orchard (tm_isaac) <-> ros1_bridge <-> Noetic perception, TreeMapper
# and the corridor navigator (tm_sim_ros); records the mission and scores it.
#
#   scripts/sim_mission.sh <spec.json> [--strategy corridor|field_first] [--runs N] [--out DIR]
#                          [--detector-model PATH] [--timeout 1800] [--record-images] [--record-frames] [--no-build]
#                          [--force-clean]
#
#   spec.json        sim/specs/*.json (host path)
#   --strategy       navigator strategy written into the run's cfg (default corridor)
#   --runs N         N runs, run_1 .. run_N, each with a fresh Isaac and tm_sim_ros (default 1)
#   --out DIR        output directory (default /data/work/sim_runs/<spec>_<strategy>_<UTC time>); /data/work/... or the
#                    host path of the work drive ($TM_WORK_HOST)
#   --detector-model ONNX model, container path (default /data/work/models/yolox_s_isaac/yolox_s_isaac_epoch190_416_static.onnx)
#   --timeout S      sim seconds from ~start to DONE before ~abort (default 1800)
#   --record-images  also record the left image at 0.2 Hz on /rec/left_0p2hz (default off: disk)
#   --record-frames  also record what TreeMapper consumes (/disparity_image_down_float, the left / right half camera_info;
#                    ~1 MB per disparity image at 10 Hz, default off: disk). After the run is scored it is converted to
#                    <run>/frames.bin + frames.bin.meta.json (tools/treemapper_tools/sim_frames.py, the replay input of
#                    treemapper_replay) and checked (a frame was paired with a disparity, pose coverage). Only if the
#                    check passes is mission.bag replaced by a copy without the disparity / camera_info topics (the run
#                    then looks like a normal one; mission_score --run works on it). A failed conversion or check fails
#                    that run (exit 1) and keeps the full bag. Needs the work drive to stay above the 8 GB floor.
#   --no-build       skip the catkin build of the Noetic packages
#   --force-clean    an --out that already holds run_* directories (or compare.md) is refused unless this is given;
#                    then only the files a run writes (RUN_FILES in this script) are deleted, in a container (they belong to
#                    root and to the Isaac uid 1234), and a run directory with anything else in it is still refused
#
# Per run directory: run.json (spec, strategy, git commit, seed, ...), spec.json, field.yaml (polygon, heading_rad from
# rows.heading_rad, home from drone.home), corridor_nav.cfg (config/corridor_nav.cfg with the strategy and the sim-only
# carrot_lead_s 1.2; all four from tools/treemapper_tools/sim_inputs.py), ground_texture.png, truth.json,
# deposition.json, rtf.json (Isaac), mission.bag, [frames.bin, frames.bin.meta.json with --record-frames], driver.json, isaac.log, ros.log, ros_log/, scorecard.md + mission_score.json (tools/treemapper_tools/mission_score.py). After several runs:
# <out>/compare.md (mission_score --compare).
#
# Noetic packages: built by this script (unless --no-build) in a dedicated catkin workspace $TM_SIM_CATKIN_WS
# (default /data/work/catkin_sim; src/ symlinks to /ws/ros/<pkg>) with the tm_sim_ros image itself, in a one-shot
# container tm_sim_build, as the calling user. tm_sim_ros mounts /data/work and sources <ws>/devel/setup.bash.
# It is separate from the test workspace (/data/work/catkin_nav of run_tests.sh) so a test build never races a mission.
#
# Exit status: 0 every run scored all_pass; 2 a run did not complete or failed a scorecard row (mission_score's 2);
# 1 setup / infrastructure failure (refusal, build, a container or input that never came up, disk). A failure inside a
# run fails that run only (containers stopped, the next run proceeds); the exit status reports it.
# Refuses to start if a container tm_isaac or tm_sim_ros already runs, or the work drive has < 10 GB free; aborts a
# run (fail loud) when it drops below 8 GB.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK_HOST="${TM_WORK_HOST:-/media/alex/waspsim/cloud_build_bundle/work}"   # mounted as /data/work in the containers
WORK_C=/data/work
SIM_ROS_IMAGE="${TM_SIM_ROS_IMAGE:-tm_sim_ros:latest}"
ISAAC_IMAGE="${TM_ISAAC_IMAGE:-treehoppingnavigation-treehoppingsim:latest}"
TREE_ASSETS="${TM_TREE_ASSETS:-/home/alex/SUIND/TreeHoppingNavigation/Nvidia-Isaac-Sim-Tree-Generator/models/Tree}"
ISAAC_CACHE="${TM_ISAAC_CACHE:-$HOME/.cache/tm_isaac}"
CATKIN_WS="${TM_SIM_CATKIN_WS:-$WORK_C/catkin_sim}"
NET=tm_sim_net
DOMAIN=77
ROS_PKGS="treemapper_msgs treemapper_ros corridor_nav_msgs corridor_nav_ros sim_perception"
MIN_FREE_START_GB=10
MIN_FREE_RUN_GB=8
CLOCK_BUDGET_S=400        # Isaac startup ~140 s (results/sim_isaac_checks.md); >= 300 s by ruling
GONE_BUDGET_S=120         # wall s for leftover tm_isaac / tm_sim_ros of a previous run to disappear
STOP_LAUNCH_S=60          # wall s for the launch to exit after SIGINT (rosbag closes the bag)
LOGS_BUDGET_S=15          # wall s for the docker logs follower to end after tm_isaac stopped
# what a run writes into its directory (sim_inputs, Isaac, the launch, the driver, the scorer); --force-clean deletes these
RUN_FILES="run.json spec.json field.yaml corridor_nav.cfg ground_texture.png truth.json ground_truth.json deposition.json
 rtf.json mission.bag mission.bag.active mission.bag.filtered frames.bin frames.bin.meta.json driver.json isaac.log ros.log ros_log scorecard.md mission_score.json map.jsonl
 mission.json"

die() { echo "sim_mission: FATAL: $*" >&2; exit 1; }
log() { echo "[sim_mission $(date -u +%H:%M:%S)] $*"; }

usage() { awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"; exit "${1:-1}"; }

# ------------------------------------------------------------------------------------------------- arguments
[ $# -ge 1 ] || usage
case "$1" in -h|--help) usage 0 ;; esac
SPEC="$1"; shift
STRATEGY=corridor; RUNS=1; OUT=""; TIMEOUT=1800; RECORD_IMAGES=false; RECORD_FRAMES=false; BUILD=1; FORCE_CLEAN=0
DETECTOR_MODEL="$WORK_C/models/yolox_s_isaac/yolox_s_isaac_epoch190_416_static.onnx"
while [ $# -gt 0 ]; do
  case "$1" in
    --strategy) STRATEGY="${2:?}"; shift 2 ;;
    --runs) RUNS="${2:?}"; shift 2 ;;
    --out) OUT="${2:?}"; shift 2 ;;
    --detector-model) DETECTOR_MODEL="${2:?}"; shift 2 ;;
    --timeout) TIMEOUT="${2:?}"; shift 2 ;;
    --record-images) RECORD_IMAGES=true; shift ;;
    --record-frames) RECORD_FRAMES=true; shift ;;
    --no-build) BUILD=0; shift ;;
    --force-clean) FORCE_CLEAN=1; shift ;;
    -h|--help) usage 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
case "$STRATEGY" in corridor|field_first) ;; *) die "--strategy must be corridor or field_first, got '$STRATEGY'" ;; esac
[[ "$RUNS" =~ ^[1-9][0-9]*$ ]] || die "--runs must be a positive integer"
[[ "$TIMEOUT" =~ ^[0-9]+(\.[0-9]+)?$ ]] || die "--timeout must be a number of sim seconds"
[ -f "$SPEC" ] || die "spec not found: $SPEC"
SPEC="$(cd "$(dirname "$SPEC")" && pwd)/$(basename "$SPEC")"
SPEC_NAME="$(basename "$SPEC" .json)"
[ -d "$WORK_HOST" ] || die "work drive not found: $WORK_HOST (set TM_WORK_HOST)"

# host <-> container paths of the work drive
to_host() { case "$1" in "$WORK_C"/*) echo "$WORK_HOST${1#"$WORK_C"}" ;; "$WORK_HOST"/*) echo "$1" ;; *) return 1 ;; esac; }
to_cont() { case "$1" in "$WORK_HOST"/*) echo "$WORK_C${1#"$WORK_HOST"}" ;; "$WORK_C"/*) echo "$1" ;; *) return 1 ;; esac; }

[ -n "$OUT" ] || OUT="$WORK_C/sim_runs/${SPEC_NAME}_${STRATEGY}_$(date -u +%Y%m%dT%H%M%SZ)"
OUT_HOST="$(to_host "$OUT")" || die "--out must be under $WORK_C or $WORK_HOST (the containers see only the work drive)"
OUT_C="$(to_cont "$OUT")"
case "$OUT_C" in "$WORK_C"/sim_runs/*) ;; *) die "--out must be under $WORK_C/sim_runs (the only directory tm_isaac mounts)" ;; esac
MODEL_HOST="$(to_host "$DETECTOR_MODEL")" || die "--detector-model must be under $WORK_C or $WORK_HOST"
DETECTOR_MODEL="$(to_cont "$DETECTOR_MODEL")"
[ -f "$MODEL_HOST" ] || die "detector model not found: $MODEL_HOST"
[ -d "$TREE_ASSETS" ] || die "tree assets not found: $TREE_ASSETS (set TM_TREE_ASSETS)"

# ------------------------------------------------------------------------------------------------- refusals
exists() { local n; n="$(docker ps -a --filter "name=^/$1\$" --format '{{.Names}}')"; [ "$n" = "$1" ]; }
running() { local n; n="$(docker ps --filter "name=^/$1\$" --format '{{.Names}}')"; [ "$n" = "$1" ]; }
for c in tm_isaac tm_sim_ros; do
  if running "$c"; then die "container $c is already running: another mission? (docker stop $c if it is stale)"; fi
done
MIN_FREE_START_B=$((MIN_FREE_START_GB * 1000000000))
MIN_FREE_RUN_B=$((MIN_FREE_RUN_GB * 1000000000))
free_bytes() { df -P -B1 "$1" | awk 'NR == 2 { print $4 }'; }   # exact bytes: no rounding up to a whole GB
FREE="$(free_bytes "$WORK_HOST")"
[ "$FREE" -ge "$MIN_FREE_START_B" ] || die "only $((FREE / 1000000)) MB free on $WORK_HOST (< $MIN_FREE_START_GB GB)"

# an --out with earlier runs: refused, or (--force-clean) emptied of the known run files only
shopt -s nullglob
OLD_RUNS=("$OUT_HOST"/run_*)
shopt -u nullglob
if [ "${#OLD_RUNS[@]}" -gt 0 ] || [ -e "$OUT_HOST/compare.md" ]; then
  [ "$FORCE_CLEAN" = 1 ] || die "$OUT_HOST already holds run directories (${#OLD_RUNS[@]}) or compare.md: choose another --out or pass --force-clean"
  log "--force-clean: deleting the known run files in $OUT_HOST (tm_sim_clean)"
  docker run --rm --name tm_sim_clean -v "$WORK_HOST:$WORK_C" "$SIM_ROS_IMAGE" bash -c "
    set -e
    cd '$OUT_C'
    known=' $(echo $RUN_FILES) '
    bad=0
    for d in run_*; do                    # check everything first: delete nothing if anything unknown is there
      [ -d \"\$d\" ] || { echo \"sim_mission: \$d is not a directory\" >&2; bad=1; continue; }
      for f in \$(ls -A \"\$d\"); do
        case \"\$known\" in *\" \$f \"*) ;; *) echo \"sim_mission: \$d/\$f is not a run output\" >&2; bad=1 ;; esac
      done
    done
    [ \"\$bad\" = 0 ] || exit 1
    rm -f compare.md
    for d in run_*; do
      for f in \$known; do rm -rf \"\$d/\$f\"; done
      rmdir \"\$d\"
    done" || die "--force-clean refused: $OUT_HOST holds files that are not run outputs (listed above); nothing was deleted"
fi
docker network inspect "$NET" > /dev/null 2>&1 || docker network create "$NET" > /dev/null
mkdir -p "$ISAAC_CACHE"/{main,compute,ovdata}
chmod -R a+rwX "$ISAAC_CACHE" 2> /dev/null || true

# ------------------------------------------------------------------------------------------------- build
if [ "$BUILD" = 1 ]; then
  log "catkin build ($CATKIN_WS, tm_sim_build): $ROS_PKGS"
  BUILD_LOG_HOST="$(to_host "$CATKIN_WS").build.log"
  if ! docker run --rm --name tm_sim_build --user "$(id -u):$(id -g)" -e HOME="$WORK_C/home" --cpus 8 \
      -v "$ROOT:/ws:ro" -v "$WORK_HOST:$WORK_C" "$SIM_ROS_IMAGE" bash -c "
        set -e
        mkdir -p '$CATKIN_WS/src'
        for p in $ROS_PKGS; do ln -sfn /ws/ros/\$p '$CATKIN_WS/src/'\$p; done
        cd '$CATKIN_WS'
        if [ ! -d .catkin_tools ]; then
          catkin init > /dev/null
          catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release > /dev/null
        fi
        catkin build $ROS_PKGS -j4 --no-status" > "$BUILD_LOG_HOST" 2>&1; then
    grep -E "error|Error" "$BUILD_LOG_HOST" | head -40 || true
    die "catkin build failed (log: $BUILD_LOG_HOST)"
  fi
  log "build ok"
fi
# (devel/setup.bash is a symlink to a container path: test the link on the host)
WS_SETUP="$(to_host "$CATKIN_WS")/devel/setup.bash"
[ -f "$WS_SETUP" ] || [ -L "$WS_SETUP" ] || die "no built workspace at $CATKIN_WS (run without --no-build)"

# ------------------------------------------------------------------------------------------------- one run
GIT_COMMIT="$(git -C "$ROOT" rev-parse HEAD)"
# modified tracked files anywhere + untracked files in the code directories
GIT_DIRTY="$( { git -C "$ROOT" status --porcelain --untracked-files=no
                git -C "$ROOT" status --porcelain --untracked-files=all -- scripts sim ros tools nav | grep '^??' || true; } | wc -l)"
LOGS_PID=""
# outer wall cap of the driver's `run`: its own budgets (inputs 7 x 180 s, mission timeout / 0.25, abort 60 / 0.25) + slack
DRIVER_CAP_S="$(awk -v t="$TIMEOUT" 'BEGIN { printf "%d", 7 * 180 + t / 0.25 + 60 / 0.25 + 300 }')"

stop_all() {   # stop the launch cleanly (closes the bag), then the two containers (Isaac writes deposition.json)
  if running tm_sim_ros; then
    if docker exec tm_sim_ros pgrep -f sim_closed_loop.launch > /dev/null 2>&1; then
      log "stopping the Noetic launch (SIGINT: closes the bag)"
      docker exec tm_sim_ros pkill -INT -f sim_closed_loop.launch || true
      local k
      for k in $(seq 1 "$STOP_LAUNCH_S"); do
        docker exec tm_sim_ros pgrep -f sim_closed_loop.launch > /dev/null 2>&1 || break
        sleep 1
      done
    fi
  fi
  local c
  for c in tm_isaac tm_sim_ros; do
    if running "$c"; then log "docker stop $c"; timeout 120 docker stop -t 60 "$c" > /dev/null || true; fi
  done
  if [ -n "$LOGS_PID" ]; then   # the follower ends with tm_isaac; never wait for it unbounded
    local w
    for w in $(seq 1 "$LOGS_BUDGET_S"); do kill -0 "$LOGS_PID" 2> /dev/null || break; sleep 1; done
    kill "$LOGS_PID" 2> /dev/null || true
    wait "$LOGS_PID" 2> /dev/null || true
    LOGS_PID=""
  fi
}
trap 'stop_all' EXIT
trap 'log "interrupted"; exit 130' INT TERM

fail_run() {   # $1 = message: the run fails, its containers are stopped, the caller returns and the next run proceeds
  RUN_RC=1
  log "run ${RUN_I:-?}: FAILED: $1"
  stop_all
}

wait_gone() {   # leftovers of a previous run (a stop still in progress) must disappear before the next run starts
  local c k
  for c in tm_isaac tm_sim_ros; do
    for k in $(seq 1 "$GONE_BUDGET_S"); do exists "$c" || break; sleep 1; done
    if exists "$c"; then return 1; fi
  done
}

convert_frames() {   # $1 = run index: <run>/frames.bin from the bag, then the bag without the TreeMapper-input topics
  # returns 0 converted + checked + bag replaced, 1 failed (the full bag stays; frames.bin may be left for inspection)
  local i="$1" run_c="$OUT_C/run_$1" run_h="$OUT_HOST/run_$1" free rc
  free="$(free_bytes "$WORK_HOST")"
  if [ "$free" -lt "$MIN_FREE_RUN_B" ]; then
    log "run $i: FRAMES FAILED: only $((free / 1000000)) MB free on $WORK_HOST (< $MIN_FREE_RUN_GB GB): bag kept"; return 1
  fi
  log "run $i: converting the bag to frames.bin (sim_frames)"
  set +e
  timeout 3600 docker run --rm --name "tm_frames_$i" --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$ROOT:/ws:ro" -v "$WORK_HOST:$WORK_C" "$SIM_ROS_IMAGE" bash -c \
    "cd /ws/tools && python3 -m treemapper_tools.sim_frames --run '$run_c' --filter-bag"
  rc=$?
  set -e
  if [ "$rc" != 0 ] || [ ! -s "$run_h/frames.bin" ] || [ -f "$run_h/mission.bag.filtered" ]; then
    log "run $i: FRAMES FAILED (sim_frames exit $rc, see above): the full bag $run_h/mission.bag is kept"; return 1
  fi
  log "run $i: frames.bin $(du -h "$run_h/frames.bin" | cut -f1), bag now $(du -h "$run_h/mission.bag" | cut -f1)"
}

run_one() {   # $1 = run index; sets RUN_RC (0 scored all_pass, 2 not, 1 failed) and FRAMES_FAIL (1 = --record-frames conversion failed)
  local i="$1" run_h run_c rc t0 t1 up logs
  RUN_I="$i"
  run_h="$OUT_HOST/run_$i"; run_c="$OUT_C/run_$i"
  if ! wait_gone; then fail_run "tm_isaac / tm_sim_ros still exist after ${GONE_BUDGET_S} s"; return; fi
  if ! { mkdir -p -m 777 "$run_h" && chmod 777 "$run_h"; }; then fail_run "cannot create $run_h"; return; fi   # Isaac runs as uid 1234
  if ! PYTHONPATH="$ROOT/tools" python3 -m treemapper_tools.sim_inputs --spec "$SPEC" --run-dir "$run_h" \
      --cfg "$ROOT/config/corridor_nav.cfg" --strategy "$STRATEGY" --git-commit "$GIT_COMMIT" --git-dirty "$GIT_DIRTY" \
      --detector-model "$DETECTOR_MODEL" --timeout "$TIMEOUT" $([ "$RECORD_IMAGES" = true ] && echo --record-images) \
      $([ "$RECORD_FRAMES" = true ] && echo --record-frames); then
    fail_run "cannot write the run inputs"; return
  fi
  t0=$(date +%s)
  log "run $i: $run_h"

  if ! docker run -d --rm --name tm_sim_ros --network "$NET" --gpus all \
      -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      -v "$ROOT:/ws:ro" -v "$WORK_HOST:$WORK_C" "$SIM_ROS_IMAGE" bridge > /dev/null; then
    fail_run "docker run tm_sim_ros failed"; return
  fi
  up=0
  for _ in $(seq 1 90); do
    logs="$(docker logs tm_sim_ros 2>&1 || true)"   # captured: grep -q in a pipe under pipefail can fail on SIGPIPE
    if grep -q "parameter_bridge (pid" <<< "$logs"; then up=1; break; fi
    running tm_sim_ros || break
    sleep 1
  done
  if [ "$up" != 1 ]; then tail -20 <<< "$logs"; fail_run "tm_sim_ros did not come up"; return; fi

  if ! docker run -d --rm --name tm_isaac --gpus all --network "$NET" --entrypoint bash \
      -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e ROS_DOMAIN_ID="$DOMAIN" \
      -v "$ROOT:/ws:ro" -v "$TREE_ASSETS:/assets/trees:ro" \
      -v "$WORK_HOST/sim_runs:$WORK_C/sim_runs" \
      -v "$ISAAC_CACHE/main:/isaac-sim/.cache" -v "$ISAAC_CACHE/compute:/isaac-sim/.nv/ComputeCache" \
      -v "$ISAAC_CACHE/ovdata:/isaac-sim/.local/share/ov/data" \
      "$ISAAC_IMAGE" /ws/sim/isaac/run_orchard_sim.sh --spec "$run_c/spec.json" --out "$run_c" \
      --v-max-xy 4.0 --v-max-z 1.5 --max-rtf 1.0 > /dev/null; then
    fail_run "docker run tm_isaac failed"; return
  fi
  docker logs -f tm_isaac > "$run_h/isaac.log" 2>&1 &
  LOGS_PID=$!

  local dexec=(docker exec -e ROS_LOG_DIR="$run_c/ros_log" tm_sim_ros /tm_entrypoint.sh bash -c)
  local src="source '$CATKIN_WS/devel/setup.bash'"
  log "waiting for /clock (Isaac startup, budget ${CLOCK_BUDGET_S} s wall)"
  local clock=0 tc=$SECONDS
  while [ $((SECONDS - tc)) -lt "$CLOCK_BUDGET_S" ]; do
    running tm_isaac || break                       # Isaac exited during startup: fail now, not after the budget
    if timeout 60 "${dexec[@]}" "$src && python3 /ws/scripts/sim_mission_driver.py wait-clock --budget 10" > /dev/null 2>&1; then
      clock=1; break
    fi
  done
  if [ "$clock" != 1 ]; then
    tail -20 "$run_h/isaac.log"
    if running tm_isaac; then fail_run "no /clock within ${CLOCK_BUDGET_S} s"; else fail_run "tm_isaac exited before /clock"; fi
    return
  fi
  log "/clock is running after $((SECONDS - tc)) s; starting sim_closed_loop.launch"
  if ! docker exec -d -e ROS_LOG_DIR="$run_c/ros_log" tm_sim_ros /tm_entrypoint.sh bash -c \
      "$src && exec roslaunch corridor_nav_ros sim_closed_loop.launch run_dir:='$run_c' detector_model:='$DETECTOR_MODEL' \
       record_images:=$RECORD_IMAGES record_frames:=$RECORD_FRAMES > '$run_c/ros.log' 2>&1"; then
    fail_run "docker exec roslaunch failed"; return
  fi
  set +e
  timeout --kill-after=30 "$DRIVER_CAP_S" "${dexec[@]}" "$src && python3 /ws/scripts/sim_mission_driver.py run \
      --run-dir '$run_c' --timeout $TIMEOUT --min-free-gb $MIN_FREE_RUN_GB"
  rc=$?
  set -e
  stop_all
  t1=$(date +%s)
  log "run $i: driver exit $rc, $((t1 - t0)) s wall in total"
  if [ "$rc" = 124 ] || [ "$rc" = 137 ]; then RUN_RC=1; log "run $i: driver hit the outer wall cap (${DRIVER_CAP_S} s)"; return; fi
  if [ -f "$run_h/mission.bag.active" ] || [ ! -f "$run_h/mission.bag" ]; then
    RUN_RC=1; log "run $i: no closed mission.bag"; return
  fi
  case "$rc" in
    0|3) ;;                              # DONE, or aborted on timeout (scored: the scorer reports it not completed)
    *) RUN_RC=1; log "run $i: driver failed (exit $rc), not scored"; return ;;
  esac
  log "run $i: bag $(du -h "$run_h/mission.bag" | cut -f1), scoring"
  set +e
  timeout 1800 docker run --rm --name "tm_score_$i" --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$ROOT:/ws:ro" -v "$WORK_HOST:$WORK_C" "$SIM_ROS_IMAGE" bash -c \
    "cd /ws/tools && python3 -m treemapper_tools.mission_score --run '$run_c' --out '$run_c/scorecard.md'"
  RUN_RC=$?
  set -e
  [ "$RUN_RC" = 0 ] || [ "$RUN_RC" = 2 ] || RUN_RC=1
  log "run $i: scorer exit $RUN_RC (scorecard $run_h/scorecard.md)"
  if [ "$RECORD_FRAMES" = true ] && { [ "$RUN_RC" = 0 ] || [ "$RUN_RC" = 2 ]; }; then
    convert_frames "$i" || FRAMES_FAIL=1
  fi
}

mkdir -p -m 777 "$OUT_HOST"
log "spec $SPEC_NAME, strategy $STRATEGY, $RUNS run(s) -> $OUT_HOST (commit ${GIT_COMMIT:0:7}, $GIT_DIRTY dirty files)"
FINAL=0
SCORED=()
for i in $(seq 1 "$RUNS"); do
  FREE="$(free_bytes "$WORK_HOST")"
  if [ "$FREE" -lt "$MIN_FREE_START_B" ]; then log "only $((FREE / 1000000)) MB free: not starting run $i"; FINAL=1; break; fi
  RUN_RC=1; FRAMES_FAIL=0
  run_one "$i" || RUN_RC=1
  [ "$FRAMES_FAIL" = 0 ] || FINAL=1
  case "$RUN_RC" in
    0) SCORED+=("$OUT_C/run_$i") ;;
    2) SCORED+=("$OUT_C/run_$i"); [ "$FINAL" = 1 ] || FINAL=2 ;;
    *) FINAL=1 ;;
  esac
done
if [ "${#SCORED[@]}" -gt 1 ]; then
  log "comparing ${#SCORED[@]} runs -> $OUT_HOST/compare.md"
  timeout 1800 docker run --rm --name tm_score_cmp --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$ROOT:/ws:ro" -v "$WORK_HOST:$WORK_C" "$SIM_ROS_IMAGE" bash -c \
    "cd /ws/tools && python3 -m treemapper_tools.mission_score --run ${SCORED[0]} --compare ${SCORED[*]:1} \
     --out '$OUT_C/compare.md'" || true
fi
log "finished: exit $FINAL"
exit "$FINAL"
