#!/usr/bin/env bash
# One-command Isaac-only closed loop: Isaac Sim orchard (tm_isaac) <-> ros1_bridge <-> Noetic perception, TreeMapper
# and the corridor navigator (tm_sim_ros); records the mission and scores it.
#
#   scripts/sim_mission.sh <spec.json> [--strategy corridor|field_first] [--runs N] [--out DIR]
#                          [--detector-model PATH] [--timeout 1800] [--record-images] [--no-build]
#
#   spec.json        sim/specs/*.json (host path)
#   --strategy       navigator strategy written into the run's cfg (default corridor)
#   --runs N         N runs, run_1 .. run_N, each with a fresh Isaac and tm_sim_ros (default 1)
#   --out DIR        output directory (default /data/work/sim_runs/<spec>_<strategy>_<UTC time>); /data/work/... or the
#                    host path of the work drive ($TM_WORK_HOST)
#   --detector-model ONNX model, container path (default /data/work/models/yolox_s_isaac/yolox_s_isaac_epoch190_416_static.onnx)
#   --timeout S      sim seconds from ~start to DONE before ~abort (default 1800)
#   --record-images  also record the left image at 0.2 Hz on /rec/left_0p2hz (default off: disk)
#   --no-build       skip the catkin build of the Noetic packages
#
# Per run directory: run.json (spec, strategy, git commit, seed, ...), spec.json, field.yaml (polygon, heading_rad from
# rows.heading_rad, home from drone.home), corridor_nav.cfg (config/corridor_nav.cfg with the strategy and the sim-only
# carrot_lead_s 1.2), truth.json / deposition.json / rtf.json (Isaac), mission.bag, driver.json, isaac.log, ros.log,
# ros_log/, scorecard.md + mission_score.json (tools/treemapper_tools/mission_score.py). After several runs:
# <out>/compare.md (mission_score --compare).
#
# Noetic packages: built by this script (unless --no-build) in a dedicated catkin workspace $TM_SIM_CATKIN_WS
# (default /data/work/catkin_sim; src/ symlinks to /ws/ros/<pkg>) with the tm_sim_ros image itself, in a one-shot
# container tm_sim_build, as the calling user. tm_sim_ros mounts /data/work and sources <ws>/devel/setup.bash.
# It is separate from the test workspace (/data/work/catkin_nav of run_tests.sh) so a test build never races a mission.
#
# Exit status: 0 every run scored all_pass; 2 a run did not complete or failed a scorecard row (mission_score's 2);
# 1 setup / infrastructure failure (refusal, build, a container or input that never came up, disk).
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

die() { echo "sim_mission: FATAL: $*" >&2; exit 1; }
log() { echo "[sim_mission $(date -u +%H:%M:%S)] $*"; }

usage() { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-1}"; }

# ------------------------------------------------------------------------------------------------- arguments
[ $# -ge 1 ] || usage
case "$1" in -h|--help) usage 0 ;; esac
SPEC="$1"; shift
STRATEGY=corridor; RUNS=1; OUT=""; TIMEOUT=1800; RECORD_IMAGES=false; BUILD=1
DETECTOR_MODEL="$WORK_C/models/yolox_s_isaac/yolox_s_isaac_epoch190_416_static.onnx"
while [ $# -gt 0 ]; do
  case "$1" in
    --strategy) STRATEGY="${2:?}"; shift 2 ;;
    --runs) RUNS="${2:?}"; shift 2 ;;
    --out) OUT="${2:?}"; shift 2 ;;
    --detector-model) DETECTOR_MODEL="${2:?}"; shift 2 ;;
    --timeout) TIMEOUT="${2:?}"; shift 2 ;;
    --record-images) RECORD_IMAGES=true; shift ;;
    --no-build) BUILD=0; shift ;;
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
running() { docker ps --filter "name=^/$1\$" --format '{{.Names}}' | grep -qx "$1"; }
for c in tm_isaac tm_sim_ros; do
  if running "$c"; then die "container $c is already running: another mission? (docker stop $c if it is stale)"; fi
done
free_gb() { df -P --block-size=1G "$1" | awk 'NR == 2 { print $4 }'; }
FREE="$(free_gb "$WORK_HOST")"
[ "$FREE" -ge "$MIN_FREE_START_GB" ] || die "only ${FREE} GB free on $WORK_HOST (< $MIN_FREE_START_GB GB)"
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
GIT_DIRTY="$(git -C "$ROOT" status --porcelain --untracked-files=no | wc -l)"
LOGS_PID=""

stop_all() {   # stop the launch cleanly (closes the bag), then the two containers (Isaac writes deposition.json)
  if running tm_sim_ros; then
    if docker exec tm_sim_ros pgrep -f sim_closed_loop.launch > /dev/null 2>&1; then
      log "stopping the Noetic launch (SIGINT: closes the bag)"
      docker exec tm_sim_ros pkill -INT -f sim_closed_loop.launch || true
      for _ in $(seq 1 60); do
        docker exec tm_sim_ros pgrep -f sim_closed_loop.launch > /dev/null 2>&1 || break
        sleep 1
      done
    fi
  fi
  local c
  for c in tm_isaac tm_sim_ros; do
    if running "$c"; then log "docker stop $c"; docker stop -t 60 "$c" > /dev/null || true; fi
  done
  if [ -n "$LOGS_PID" ]; then wait "$LOGS_PID" 2> /dev/null || true; LOGS_PID=""; fi
}
trap 'stop_all' EXIT
trap 'log "interrupted"; exit 130' INT TERM

write_inputs() {   # $1 = host run dir
  python3 - "$SPEC" "$1" "$ROOT/config/corridor_nav.cfg" "$STRATEGY" "$GIT_COMMIT" "$GIT_DIRTY" "$DETECTOR_MODEL" \
      "$TIMEOUT" "$RECORD_IMAGES" <<'PY'
import json, os, sys, time
spec_path, run_dir, cfg_path, strategy, commit, dirty, model, timeout, rec_img = sys.argv[1:10]
spec = json.load(open(spec_path))
with open(os.path.join(run_dir, "spec.json"), "w") as f:
    json.dump(spec, f, indent=1)
poly = spec["field"]["polygon"]
home = spec["drone"]["home"]
heading = float(spec["rows"]["heading_rad"])
with open(os.path.join(run_dir, "field.yaml"), "w") as f:
    f.write("# generated by scripts/sim_mission.sh from %s\n" % os.path.basename(spec_path))
    f.write("polygon: [%s]\n" % ", ".join("[%r, %r]" % (float(x), float(y)) for x, y in poly))
    f.write("heading_rad: %r\n" % heading)
    f.write("home: [%r, %r]\n" % (float(home[0]), float(home[1])))
# the navigator cfg: config/corridor_nav.cfg with two values replaced (the reader rejects duplicate keys)
override = {"strategy": strategy, "carrot_lead_s": "1.2"}
seen = set()
out = ["# generated by scripts/sim_mission.sh: config/corridor_nav.cfg with strategy = %s and the sim-only"
       " carrot_lead_s = 1.2 (Isaac drone v_max = the navigator's max speeds, 4 / 1.5 m/s)" % strategy]
for line in open(cfg_path):
    key = line.split("#", 1)[0].split("=", 1)[0].strip()
    if "=" in line.split("#", 1)[0] and key in override:
        out.append("%s = %s" % (key, override[key]))
        seen.add(key)
    else:
        out.append(line.rstrip("\n"))
missing = set(override) - seen
if missing:
    sys.exit("config/corridor_nav.cfg has no key %s" % sorted(missing))
with open(os.path.join(run_dir, "corridor_nav.cfg"), "w") as f:
    f.write("\n".join(out) + "\n")
run = {"spec": spec, "spec_file": spec_path, "strategy": strategy, "git_commit": commit,
       "git_dirty_files": int(dirty), "seed": spec.get("seed"), "detector_model": model,
       "timeout_sim_s": float(timeout), "record_images": rec_img == "true",
       "isaac": {"v_max_xy": 4.0, "v_max_z": 1.5, "max_rtf": 1.0},
       "cfg_overrides": override, "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
with open(os.path.join(run_dir, "run.json"), "w") as f:
    json.dump(run, f, indent=1)
PY
}

run_one() {   # $1 = run index; sets RUN_RC (0 scored all_pass, 2 not, 1 infrastructure)
  local i="$1" run_h run_c rc t0 t1
  run_h="$OUT_HOST/run_$i"; run_c="$OUT_C/run_$i"
  mkdir -p -m 777 "$run_h"; chmod 777 "$run_h"          # the Isaac image runs as uid 1234
  write_inputs "$run_h"
  t0=$(date +%s)
  log "run $i: $run_h"

  docker run -d --rm --name tm_sim_ros --network "$NET" --gpus all \
    -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v "$ROOT:/ws:ro" -v "$WORK_HOST:$WORK_C" "$SIM_ROS_IMAGE" bridge > /dev/null
  local up=0
  for _ in $(seq 1 90); do
    if docker logs tm_sim_ros 2>&1 | grep -q "parameter_bridge (pid"; then up=1; break; fi
    running tm_sim_ros || break
    sleep 1
  done
  if [ "$up" != 1 ]; then docker logs tm_sim_ros 2>&1 | tail -20; RUN_RC=1; log "tm_sim_ros did not come up"; return; fi

  docker run -d --rm --name tm_isaac --gpus all --network "$NET" --entrypoint bash \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e ROS_DOMAIN_ID="$DOMAIN" \
    -v "$ROOT:/ws:ro" -v "$TREE_ASSETS:/assets/trees:ro" \
    -v "$WORK_HOST/sim_runs:$WORK_C/sim_runs" \
    -v "$ISAAC_CACHE/main:/isaac-sim/.cache" -v "$ISAAC_CACHE/compute:/isaac-sim/.nv/ComputeCache" \
    -v "$ISAAC_CACHE/ovdata:/isaac-sim/.local/share/ov/data" \
    "$ISAAC_IMAGE" /ws/sim/isaac/run_orchard_sim.sh --spec "$run_c/spec.json" --out "$run_c" \
    --v-max-xy 4.0 --v-max-z 1.5 --max-rtf 1.0 > /dev/null
  docker logs -f tm_isaac > "$run_h/isaac.log" 2>&1 &
  LOGS_PID=$!

  local dexec=(docker exec -e ROS_LOG_DIR="$run_c/ros_log" tm_sim_ros /tm_entrypoint.sh bash -c)
  local src="source '$CATKIN_WS/devel/setup.bash'"
  log "waiting for /clock (Isaac startup, budget ${CLOCK_BUDGET_S} s wall)"
  if ! "${dexec[@]}" "$src && python3 /ws/scripts/sim_mission_driver.py wait-clock --budget $CLOCK_BUDGET_S"; then
    tail -20 "$run_h/isaac.log"; RUN_RC=1; log "run $i: Isaac never started the clock"; stop_all; return
  fi
  log "starting sim_closed_loop.launch"
  docker exec -d -e ROS_LOG_DIR="$run_c/ros_log" tm_sim_ros /tm_entrypoint.sh bash -c \
    "$src && exec roslaunch corridor_nav_ros sim_closed_loop.launch run_dir:='$run_c' detector_model:='$DETECTOR_MODEL' \
     record_images:=$RECORD_IMAGES > '$run_c/ros.log' 2>&1"
  set +e
  "${dexec[@]}" "$src && python3 /ws/scripts/sim_mission_driver.py run --run-dir '$run_c' --timeout $TIMEOUT \
                 --min-free-gb $MIN_FREE_RUN_GB"
  rc=$?
  set -e
  stop_all
  t1=$(date +%s)
  log "run $i: driver exit $rc, $((t1 - t0)) s wall in total"
  if [ -f "$run_h/mission.bag.active" ] || [ ! -f "$run_h/mission.bag" ]; then
    log "run $i: no closed mission.bag"; RUN_RC=1; return
  fi
  case "$rc" in
    0|3) ;;                              # DONE, or aborted on timeout (scored: the scorer reports it not completed)
    *) RUN_RC=1; log "run $i: driver failed (exit $rc), not scored"; return ;;
  esac
  log "run $i: bag $(du -h "$run_h/mission.bag" | cut -f1), scoring"
  set +e
  docker run --rm --name "tm_score_$i" --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$ROOT:/ws:ro" -v "$WORK_HOST:$WORK_C" "$SIM_ROS_IMAGE" bash -c \
    "cd /ws/tools && python3 -m treemapper_tools.mission_score --run '$run_c' --out '$run_c/scorecard.md'"
  RUN_RC=$?
  set -e
  [ "$RUN_RC" = 0 ] || [ "$RUN_RC" = 2 ] || RUN_RC=1
  log "run $i: scorer exit $RUN_RC (scorecard $run_h/scorecard.md)"
}

mkdir -p -m 777 "$OUT_HOST"
log "spec $SPEC_NAME, strategy $STRATEGY, $RUNS run(s) -> $OUT_HOST (commit ${GIT_COMMIT:0:7}, $GIT_DIRTY dirty files)"
FINAL=0
SCORED=()
for i in $(seq 1 "$RUNS"); do
  FREE="$(free_gb "$WORK_HOST")"
  if [ "$FREE" -lt "$MIN_FREE_START_GB" ]; then log "only ${FREE} GB free: not starting run $i"; FINAL=1; break; fi
  RUN_RC=1
  run_one "$i"
  case "$RUN_RC" in
    0) SCORED+=("$OUT_C/run_$i") ;;
    2) SCORED+=("$OUT_C/run_$i"); [ "$FINAL" = 1 ] || FINAL=2 ;;
    *) FINAL=1 ;;
  esac
done
if [ "${#SCORED[@]}" -gt 1 ]; then
  log "comparing ${#SCORED[@]} runs -> $OUT_HOST/compare.md"
  docker run --rm --name tm_score_cmp --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$ROOT:/ws:ro" -v "$WORK_HOST:$WORK_C" "$SIM_ROS_IMAGE" bash -c \
    "cd /ws/tools && python3 -m treemapper_tools.mission_score --run ${SCORED[0]} --compare ${SCORED[*]:1} \
     --out '$OUT_C/compare.md'" || true
fi
log "finished: exit $FINAL"
exit "$FINAL"
