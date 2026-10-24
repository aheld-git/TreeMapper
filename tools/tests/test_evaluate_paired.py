"""Paired comparison of two systems in evaluate.py: differences are per dataset (= per seed), signed S - ref,
wins respect each metric's direction."""
import math

import pytest

from treemapper_tools import evaluate as ev


def _system(values_by_ds):
    """values_by_ds: {dataset: {label: value}}; unspecified headline labels are None."""
    per = {}
    for ds, vals in values_by_ds.items():
        h = {label: None for label, _, _ in ev.HEADLINE}
        h.update(vals)
        per[ds] = [{"headline": h}]
    return {"per_dataset": per}


R, F, S = "recall (final)", "false trees (final)", "SAFETY total violations"


def test_differences_are_per_dataset_and_signed_other_minus_ref():
    ref = _system({"easy_s0": {R: 0.8, F: 2}, "easy_s1": {R: 0.6, F: 4}, "medium_s0": {R: 0.5, F: 9}})
    oth = _system({"easy_s0": {R: 0.9, F: 1}, "easy_s1": {R: 0.6, F: 5}, "medium_s0": {R: 0.4, F: 9}})
    P = ev.paired_comparison(ref, oth)
    assert P["n_pairs"] == 3
    assert P["per_dataset"]["easy_s0"][R] == pytest.approx(0.1)
    assert P["per_dataset"]["medium_s0"][R] == pytest.approx(-0.1)
    assert P["per_dataset"]["easy_s1"][F] == 1
    st = P["summary"][R]
    # recall: higher is better -> one win, one loss, one tie
    assert (st["wins"], st["losses"], st["ties"]) == (1, 1, 1)
    # false trees: lower is better -> easy_s0 win, easy_s1 loss, medium_s0 tie
    sf = P["summary"][F]
    assert (sf["wins"], sf["losses"], sf["ties"]) == (1, 1, 1)
    assert st["mean_diff"] == pytest.approx(0.0)


def test_missing_values_and_unpaired_datasets_are_skipped():
    ref = _system({"a": {R: 0.5}, "b": {R: None}, "only_ref": {R: 1.0}})
    oth = _system({"a": {R: 0.7}, "b": {R: 0.2}, "only_other": {R: 0.0}})
    P = ev.paired_comparison(ref, oth)
    assert list(P["per_dataset"]) == ["a", "b"]
    assert P["per_dataset"]["b"][R] is None
    assert P["summary"][R]["n"] == 1
    assert P["summary"][R]["std_diff"] is None and P["summary"][R]["t"] is None
    assert P["summary"]["height RMSE m (final)"]["n"] == 0


def test_t_statistic_and_identical_systems():
    ref = _system({"d%d" % i: {S: 0} for i in range(4)})
    same = ev.paired_comparison(ref, ref)
    assert same["summary"][S]["t"] == 0.0 and same["summary"][S]["ties"] == 4
    oth = _system({"d0": {S: 2}, "d1": {S: 4}, "d2": {S: 2}, "d3": {S: 4}})
    st = ev.paired_comparison(ref, oth)["summary"][S]
    sd = math.sqrt(sum((x - 3) ** 2 for x in (2, 4, 2, 4)) / 3)
    assert st["t"] == pytest.approx(3 / (sd / 2))
    assert (st["wins"], st["losses"]) == (0, 4)  # more violations = worse


def test_markdown_has_summary_group_and_per_seed_tables():
    ref = _system({"easy_s0": {R: 0.8}, "medium_s0": {R: 0.5}})
    oth = _system({"easy_s0": {R: 0.9}, "medium_s0": {R: 0.4}})
    md = "\n".join(ev.paired_markdown("simple", "cand_a", ev.paired_comparison(ref, oth)))
    assert "## Paired: cand_a vs simple (difference = cand_a - simple" in md
    assert "Mean paired difference per group" in md
    assert "| easy_s0 |" in md and "| medium_s0 |" in md
    assert "+0.100" in md and "-0.100" in md
