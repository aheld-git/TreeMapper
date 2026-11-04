#!/bin/bash
# inside container, cwd /ws. 4 parallel jobs (4 cpus).
M=/data/work/mr_t7; O=/data/work/eval/mr_t7; export PYTHONPATH=/ws/tools
cfgset() { # label cfgfile -> --set label.kv ...
  sed -e 's/#.*//' -e 's/[[:space:]]//g' $2 | grep -v '^$' | while read kv; do echo "--set $1.$kv"; done | paste -sd' '; }
# job 1: holdout before
( cd /ws/tools && TM_REPLAY=$M/build_before/core/treemapper_replay python3 -m treemapper_tools.evaluate --system cand_obj --suite /data/work/holdout/suite_holdout --runs $O/holdout_before/runs --title "cand_obj holdout, before (be0e5bd)" --out-md $O/holdout_before.md --out-json $O/holdout_before.json > $O/holdout_before.log 2>&1; echo holdout_before rc=$? ) &
# job 2: holdout after
( cd /ws/tools && TM_REPLAY=$M/build_after/core/treemapper_replay python3 -m treemapper_tools.evaluate --system cand_obj --suite /data/work/holdout/suite_holdout --runs $O/holdout_after/runs --title "cand_obj holdout, after (1cab792)" --out-md $O/holdout_after.md --out-json $O/holdout_after.json > $O/holdout_after.log 2>&1; echo holdout_after rc=$? ) &
# job 3: real flight scorecard (7 systems: sim / real x before, after, after_off, plus existing)
( cd /ws/tools && SETS="$(cfgset real_before $M/before/config/cand_obj_real.cfg) $(cfgset real_after $M/after/config/cand_obj_real.cfg) $(cfgset real_after_off $M/after/config/cand_obj_real.cfg) --set real_after_off.centre_from_support=false --set sim_after_off.centre_from_support=false"
  python3 -m treemapper_tools.real_score --dataset /data/work/extracted/real_farm --replay $M/build_after/core/treemapper_replay \
   --systems "sim_before=cand_obj@$M/build_before/core/treemapper_replay,sim_after=cand_obj@$M/build_after/core/treemapper_replay,sim_after_off=cand_obj@$M/build_after/core/treemapper_replay,real_before=cand_obj@$M/build_before/core/treemapper_replay,real_after=cand_obj@$M/build_after/core/treemapper_replay,real_after_off=cand_obj@$M/build_after/core/treemapper_replay,existing" \
   $SETS --runs $O/real/runs --label "before be0e5bd / after 1cab792" --out-json $O/real/real_score.json > $O/real/real_score.log 2>&1; echo real rc=$? ) &
# job 4: rollback on all of suite_v2. The cmp line below is what ran; it reports DIFFERENT because map.jsonl carries the
# wall-clock stage_ms, so the Task 7 rollback.txt was written afterwards with same_maps.py (map without stage_ms) + cmp of targets.json, i.e.
#   echo "$n $(python3 /ws/results/miss_reduction/regression/same_maps.py $b/map.jsonl $O/rollback/$n/map.jsonl) ..."
# The committed rollback.txt is now the fix-round-1 (D67) one, written by analyse_d67.sh.
( mkdir -p $O/rollback; for d in /data/work/synthetic/suite_v2/*/; do n=$(basename $d); mkdir -p $O/rollback/$n
  $M/build_after/core/treemapper_replay --frames $d/frames.bin --system cand_obj --config "centre_from_support=false" --out-map $O/rollback/$n/map.jsonl --out-targets $O/rollback/$n/targets.json --out-counters $O/rollback/$n/counters.json 2> $O/rollback/$n/replay.log
  b=$O/suite_v2_before/runs/cand_obj/${n}_r0
  cmp $b/map.jsonl $O/rollback/$n/map.jsonl && cmp $b/targets.json $O/rollback/$n/targets.json && echo "$n IDENTICAL" || echo "$n DIFFERENT"; done > $O/rollback.txt 2>&1; echo rollback done ) &
wait
