#!/usr/bin/env bash
# One-command evaluation of a mapping system on the recorded Isaac mission and on the synthetic suite.
# Run inside tm_dev:
#   docker exec tm_dev bash -lc 'cd /ws && ./scripts/evaluate.sh [system] [suite]'
#     system  registered system name, or a comma list "simple,cand_a,cand_b" (default: simple).
#             Names = `treemapper_replay --list-systems`. With 2+ systems the scorecards end with a paired
#             comparison (per seed, against the reference = first system, or TM_REF).
#     suite   synthetic suite name under /data/work/synthetic (default: suite_v1)
# Env: TM_OUT (default /data/work/eval/<system>, commas become '+') gets the scorecards (md + json);
#      TM_CANDIDATES="cand_a" build only that candidate (default: all);
#      TM_REF=<name> reference system of the paired comparison (default: first in the list);
#      TM_SET="cand_a.alpha=0.3 cand_a.k=5" configuration entries, space separated (SYSTEM.key=value);
#      TM_REPEAT=2 also reruns every synthetic dataset to measure the run-to-run noise floor;
#      TM_SKIP_ISAAC=1 / TM_SKIP_SYNTH=1 skip a part.
# Builds the core (-j8), extracts the recorded frames once (~1 min) and generates the suite once
# (~5 min, seeded and reproducible) if they are missing. Prints the scorecards.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SYSTEM="${1:-simple}"
SUITE="${2:-suite_v1}"
OUT="${TM_OUT:-/data/work/eval/${SYSTEM//,/+}}"
REF=()
[ -n "${TM_REF:-}" ] && REF=(--reference "$TM_REF")
SETS=()
for kv in ${TM_SET:-}; do SETS+=(--set "$kv"); done
BUILD="${TM_BUILD:-/data/work/build}"
BAG="${TM_ISAAC_BAG:-/data/isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag}"
E="${TM_EXTRACTED:-/data/work/extracted/isaac_2026-10-23-15-34-29}"
SYN="/data/work/synthetic/$SUITE"
mkdir -p "$OUT"
set +u; source /opt/ros/noetic/setup.bash; set -u  # ROS setup scripts use unset variables

echo "== build core ($BUILD/core)"
cmake -S "$ROOT/core" -B "$BUILD/core" -DCMAKE_BUILD_TYPE=RelWithDebInfo ${TM_CANDIDATES:+-DTM_CANDIDATES="$TM_CANDIDATES"} > "$BUILD.core.cmake.log" 2>&1 \
  || { cat "$BUILD.core.cmake.log"; exit 1; }
cmake --build "$BUILD/core" -j8 --target treemapper_replay 2>&1 | grep -E "error|Error" && exit 1 || true
export TM_REPLAY="$BUILD/core/treemapper_replay"
cd "$ROOT/tools"

if [ "${TM_SKIP_ISAAC:-0}" != 1 ]; then
  if [ ! -f "$E/meta.json" ]; then python3 -m treemapper_tools.extract_bag --bag "$BAG" --out "$E"; fi
  if [ ! -f "$E/frames.bin" ]; then
    python3 -m treemapper_tools.bag_frames --bag "$BAG" --extracted "$E" --out "$E/frames.bin" > /dev/null
  fi
  SYS="$SYSTEM"; [ -f "$E/map_existing.jsonl" ] && [[ ",$SYSTEM," != *,existing,* ]] && SYS="$SYSTEM,existing"
  python3 -m treemapper_tools.evaluate --system "$SYS" --dataset "$E" --runs "$OUT/runs" "${REF[@]}" "${SETS[@]}" \
    --title "$SYSTEM vs existing system: recorded Isaac mission" \
    --out-md "$OUT/isaac.md" --out-json "$OUT/isaac.json"
fi
if [ "${TM_SKIP_SYNTH:-0}" != 1 ]; then
  if [ ! -d "$SYN" ]; then python3 -m treemapper_tools.synth --out "$SYN" --levels easy,medium,hard --seeds 0-4 -j 6; fi
  python3 -m treemapper_tools.evaluate --system "$SYSTEM" --suite "$SYN" --runs "$OUT/runs" "${REF[@]}" "${SETS[@]}" \
    --repeat "${TM_REPEAT:-1}" --title "$SYSTEM on synthetic suite $SUITE" \
    --out-md "$OUT/synthetic.md" --out-json "$OUT/synthetic.json"
fi
echo "scorecards in $OUT"
