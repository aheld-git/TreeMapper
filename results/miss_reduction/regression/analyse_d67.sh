#!/bin/bash
# inside container, cwd /ws: paired comparisons, rollback / hold map equality, real-log centre/outline stats
O=/data/work/eval/mr_t7f; R=/ws/results/miss_reduction/regression; export PYTHONPATH=/ws/tools
python3 $R/compare.py $O/suite_v2_before/synthetic.json $O/suite_v2_after/synthetic.json $O/compare_suite_v2.md > /dev/null
python3 $R/compare.py $O/suite_v2_before/isaac.json $O/suite_v2_after/isaac.json $O/compare_isaac.md > /dev/null
python3 $R/compare.py $O/holdout_before.json $O/holdout_after.json $O/compare_holdout.md > /dev/null
python3 $R/compare.py $O/suite_v2_before/synthetic.json $O/suite_v2_hold/synthetic.json $O/compare_suite_v2_hold.md > /dev/null
python3 $R/compare.py $O/suite_v2_before/isaac.json $O/suite_v2_hold/isaac.json $O/compare_isaac_hold.md > /dev/null
# rollback: after binary with centre_from_support=false vs the hold build (be0e5bd + the D67 line); hold vs before; after r0 vs r1
for d in /data/work/synthetic/suite_v2/*/ /data/work/extracted/isaac_2026-10-23-15-34-29/; do n=$(basename $d)
  r=${n}_r0; [ -d $O/suite_v2_before/runs/cand_obj/$r ] || r=$n
  h=$O/suite_v2_hold/runs/cand_obj/$n; [ -d $h ] || h=$O/suite_v2_hold/runs/cand_obj/$r
  b=$O/suite_v2_before/runs/cand_obj/$r; a=$O/suite_v2_after/runs/cand_obj/$r
  line="$n off-vs-hold: $(python3 $R/same_maps.py $h/map.jsonl $O/rollback/$n/map.jsonl) $(cmp -s $h/targets.json $O/rollback/$n/targets.json && echo targets-same || echo targets-DIFFER) | hold-vs-before: $(python3 $R/same_maps.py $b/map.jsonl $h/map.jsonl)"
  [ -d $O/suite_v2_after/runs/cand_obj/${n}_r1 ] && line="$line | after r0-vs-r1: $(python3 $R/same_maps.py $a/map.jsonl $O/suite_v2_after/runs/cand_obj/${n}_r1/map.jsonl)"
  echo "$line"
done > $O/rollback.txt
for p in sim real; do for x in "before after_off" "hold after_off" "before hold" "before after"; do set -- $x
  echo "${p}_$1 vs ${p}_$2: $(python3 $R/same_maps.py $O/real/runs/${p}_$1/map.jsonl $O/real/runs/${p}_$2/map.jsonl)"; done; done > $O/maps_equal.txt
for s in sim_before sim_after real_before real_after; do echo "== $s"; python3 $R/centre_outline.py $O/real/runs/$s/map.jsonl $O/real/runs/$s/counters.json; done > $O/centre_outline_real_log.txt
for f in $O/real/runs/sim_before $O/real/runs/sim_after $O/real/runs/real_before $O/real/runs/real_after $O/suite_v2_before/runs/cand_obj/isaac_2026-10-23-15-34-29 $O/suite_v2_after/runs/cand_obj/isaac_2026-10-23-15-34-29; do echo "$f: $(python3 $R/churn.py $f/map.jsonl)"; done > $O/churn.txt
echo analyse done
