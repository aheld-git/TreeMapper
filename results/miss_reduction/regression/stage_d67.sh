#!/bin/bash
# Task 7 fix round 1 (inside a tm_t7f_ container, cwd /ws). before = be0e5bd, hold = be0e5bd + the D67 one-line hold
# change, after = the D67 working tree (git stash create, see after.stash_sha). 4 parallel jobs.
M=/data/work/mr_t7f; O=/data/work/eval/mr_t7f; export PYTHONPATH=/ws/tools
cfgset() { sed -e 's/#.*//' -e 's/[[:space:]]//g' $2 | grep -v '^$' | while read kv; do echo "--set $1.$kv"; done | paste -sd' '; }
suite() { ( cd $M/$1 && TM_BUILD=$M/build_$1 TM_OUT=$O/suite_v2_$1 TM_REPEAT=$2 ./scripts/evaluate.sh cand_obj suite_v2 > $O/suite_v2_$1.log 2>&1; echo "suite $1 rc=$?" ); }
holdout() { ( cd /ws/tools && TM_REPLAY=$M/build_$1/core/treemapper_replay python3 -m treemapper_tools.evaluate --system cand_obj --suite /data/work/holdout/suite_holdout --runs $O/holdout_$1/runs --title "cand_obj holdout, $1" --out-md $O/holdout_$1.md --out-json $O/holdout_$1.json > $O/holdout_$1.log 2>&1; echo "holdout $1 rc=$?" ); }
mr() { # build label cfg
  for ds in mr_frames_5x5_tropical_corridor mr_frames_5x5_tropical_field_first mr_frames_rows_8x12_corridor mr_frames_rows_8x12_field_first; do
    CFGFILE=$3 /data/work/replay/t6/run.sh $M/build_$1/core t7f_$2 $ds; done; }
case "$1" in
  suite_before) suite before 2 ;;
  suite_after) suite after 2 ;;
  suite_hold) suite hold 1 ;;   # run by hand after the others (stage.log: "suite hold rc=0"); one repeat
  holdout_before) holdout before ;;
  holdout_after) holdout after ;;
  mr) mr after after /ws/config/cand_obj.cfg; mr hold hold $M/before/config/cand_obj.cfg; mr after after_off /data/work/replay/t6/off.cfg ;;
  real) mkdir -p $O/real; cd /ws/tools && SETS="$(cfgset real_before $M/before/config/cand_obj_real.cfg) $(cfgset real_hold $M/before/config/cand_obj_real.cfg) $(cfgset real_after $M/after/config/cand_obj_real.cfg) $(cfgset real_after_off $M/after/config/cand_obj_real.cfg) --set real_after_off.centre_from_support=false --set sim_after_off.centre_from_support=false"
    python3 -m treemapper_tools.real_score --dataset /data/work/extracted/real_farm --replay $M/build_after/core/treemapper_replay \
     --systems "sim_before=cand_obj@$M/build_before/core/treemapper_replay,sim_hold=cand_obj@$M/build_hold/core/treemapper_replay,sim_after=cand_obj@$M/build_after/core/treemapper_replay,sim_after_off=cand_obj@$M/build_after/core/treemapper_replay,real_before=cand_obj@$M/build_before/core/treemapper_replay,real_hold=cand_obj@$M/build_hold/core/treemapper_replay,real_after=cand_obj@$M/build_after/core/treemapper_replay,real_after_off=cand_obj@$M/build_after/core/treemapper_replay,existing" \
     $SETS --runs $O/real/runs --label "before be0e5bd / hold / after (D67)" --out-json $O/real/real_score.json > $O/real/real_score.log 2>&1; echo "real rc=$?" ;;
  rollback) mkdir -p $O/rollback
    for d in /data/work/synthetic/suite_v2/*/ /data/work/extracted/isaac_2026-10-23-15-34-29/; do n=$(basename $d); mkdir -p $O/rollback/$n
      $M/build_after/core/treemapper_replay --frames $d/frames.bin --system cand_obj --config "centre_from_support=false" --out-map $O/rollback/$n/map.jsonl --out-targets $O/rollback/$n/targets.json --out-counters $O/rollback/$n/counters.json 2> $O/rollback/$n/replay.log; done
    echo "rollback runs done" ;;
esac
