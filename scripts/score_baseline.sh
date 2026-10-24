#!/usr/bin/env bash
# Baseline scorecard: the existing system's recorded map on the recorded Isaac sim mission.
# Run inside tm_dev:  docker exec tm_dev bash -lc 'cd /ws && ./scripts/score_baseline.sh'
# Extracts the bag once (~1 min) into /data/work/extracted/, then scores (~4 s).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BAG="${TM_ISAAC_BAG:-/data/isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag}"
E="${TM_EXTRACTED:-/data/work/extracted/isaac_2026-10-23-15-34-29}"
cd "$ROOT/tools"
if [ ! -f "$E/meta.json" ]; then
  source /opt/ros/noetic/setup.bash
  python3 -m treemapper_tools.extract_bag --bag "$BAG" --out "$E"
fi
python3 -m treemapper_tools.score \
  --map "$E/map_existing.jsonl" --gt "$E/ground_truth.json" \
  --mission "$E/mission.json" --camera "$E/camera_info.json" \
  --title "Baseline: existing system's recorded map, Isaac sim mission 2026-10-23-15-34-29" \
  --append-md "$ROOT/results/baseline_existing_system_notes.md" \
  --out-json "$ROOT/results/baseline_existing_system.json" \
  --out-md "$ROOT/results/baseline_existing_system.md" > /dev/null
echo "wrote $ROOT/results/baseline_existing_system.{md,json}"
