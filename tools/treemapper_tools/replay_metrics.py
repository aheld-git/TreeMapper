"""Replay metrics: TreeMapper's published height and centre against Isaac truth, per tree, at the lock and at every dose.

    python3 -m treemapper_tools.replay_metrics --map M.jsonl --truth truth.json --mission mission.json
        [--replay] [--targets targets.json] [--heading-rad H] [--before before.json] [--json out.json] [--md out.md]

``M.jsonl`` is a map stream in the contract format (``contract.py``): a closed loop's ``map.jsonl`` and the stream
``treemapper_replay --out-map`` writes are the same format (same keys; ``treemapper_replay`` writes ``"height":null``
for an undefined height, the contract's ``null``), so ``compute()`` takes either. ``truth.json`` is the run's Isaac
truth (``top_z`` per tree; a contract ``ground_truth.json`` with ``height`` also works). ``mission.json`` is the
recorded run's (``phases``, ``targets`` = ``/spray_target`` timeline, ``pose`` = the drone track), written by
``mission_score.py``.

A replay has no mission of its own: the phases and the drone track come from the recorded run's ``mission.json``. The
``/spray_target`` ids in it, however, are the recorded navigation's, not the replayed mapper's: ``treemapper_replay``
resolves each recorded target to *its own* id (navigation emulator) and writes that timeline to ``--out-targets``.
For a replay map pass that file as ``--targets`` (``targets_json``); it replaces ``mission.json``'s ``targets``.
``--replay`` (``replay=True``) makes ``--targets`` mandatory (CLI usage error / ``ValueError``). Whatever the flag,
every dose episode is accounted for (``n_dose_episodes`` = attached + without a target id + unattached, the latter
broken down in ``unattached_breakdown``) and ``problems()`` flags the result when the unattached share of the episodes
with a target id exceeds ``UNATTACHED_MAX_SHARE``: the CLI then prints it on stderr, writes it into the output
(``problems``) and exits 3, so a replay scored with the recorded navigation's ids cannot pass unnoticed.

Definitions (controller rulings for the miss-reduction work, Task 3):
* ``dh`` = published height - true top (m, + = too high); ``dc`` = distance published centre - truth centre (m).
  A value at time t comes from the latest map snapshot with stamp <= t that holds the entry.
* an entry's truth tree = the match ``mission_score.match_target_to_truth`` gives (truth outline containing the entry's
  centre, else the nearest truth centre within 2.0 m) at the *first snapshot where it matches at all*. The
  identity is sticky: a centre that later drifts beyond 2 m (the far-sighted centre bias this work is about) is still
  that tree's entry, with the full error. Entries that never match are left out of ``entries`` (listed in
  ``unmatched_entries``).
* doses of an entry = the SPRAYING dose episodes (``mission_score.dose_episodes``, imported, not copied) whose
  ``/spray_target`` id is the entry; the dose time is the episode start.
* LOCK RULE (corridor mode locks every corridor at its own PLANNING, trees of later corridors are not yet seen at
  the first one): the lock of an entry = the start of the last PLANNING phase before the entry's first dose; an
  entry never dosed uses the first PLANNING start at or after its first publication; an entry published after the
  last PLANNING and never dosed has no lock (``lock`` values null) and is left out of the lock summaries. In
  field_first there is one PLANNING, so this is the single lock.
* ``first_pub_t`` = stamp of the first snapshot with the entry; ``first_pub_offset_m`` = signed across-track distance
  from the drone (latest pose at that stamp) to the truth centre. The across axis is (-sin h, cos h) for the row
  heading h: ``--heading-rad``, else ``run.json`` (``spec.rows.heading_rad``) next to ``mission.json``, else the
  drone's direction of motion around that stamp (+- 2 s, then +- 5 s). Far / near split at ``|offset|`` >= 5.0 m.
* acceptance summaries (spec 2026-11-03-miss-reduction-design.md, section 2):
  A1 ``max_dh_at_lock`` / ``max_dh_at_dose`` = largest dh over the entries' lock / dose values; ``n_over_1p5`` =
  number of lock and dose values with dh > 1.5 m (``_lock`` / ``_dose`` parts alongside).
  A2 ``share_below_0p3`` = share of entries (with a defined dh at the lock or a dose) that have any lock / dose dh
  < -0.3 m; ``worst_dh`` = the most negative lock / dose dh.
  A3 per far / near group: ``n`` = entries with a known offset and at least one dose, ``mean_dc_at_dose`` = mean over
  them of the entry's mean dc over its doses, ``share_over_0p8`` = share of them whose dose dc > 0.80 m.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from treemapper_tools import contract
from treemapper_tools import mission_score as ms

FAR_M = 5.0            # first-publication across-track offset that splits far from near (m)
OVER_M = 1.5           # A1: height more than this above the true top
UNDER_M = 0.3          # A2: height more than this below the true top
FOOTPRINT_M = 0.8      # A3: spray footprint radius (m)
UNATTACHED_MAX_SHARE = 0.10   # unattached share of dose episodes with a target id; the 12 cmp16c closed-loop runs give 0-4 % (phantom entries)
MOTION_WINDOWS_S = (2.0, 5.0)
MOTION_MIN_M = 0.5


# ----------------------------------------------------------------------------- inputs

def _load_truth(path: str) -> List[dict]:
    with open(path) as f:
        return list(json.load(f)["trees"])


def _load_targets(path: str) -> List[Tuple[float, int]]:
    with open(path) as f:
        return sorted([(float(t), int(i)) for t, i in json.load(f)], key=lambda r: r[0])


def _run_heading(mission_json: str) -> Optional[float]:
    p = os.path.join(os.path.dirname(os.path.abspath(mission_json)), "run.json")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        h = ((json.load(f).get("spec") or {}).get("rows") or {}).get("heading_rad")
    return None if h is None else float(h)


def _pose_at(poses: Sequence[Sequence[float]], times: Sequence[float], t: float) -> Optional[Sequence[float]]:
    if not poses:
        return None
    return poses[max(0, bisect.bisect_right(times, t) - 1)]


def _motion_axis(poses, times, t: float) -> Optional[Tuple[float, float]]:
    """Unit across-track vector from the drone's direction of motion around t, or None when it barely moved."""
    for w in MOTION_WINDOWS_S:
        a, b = _pose_at(poses, times, t - w), _pose_at(poses, times, t + w)
        if a is None or b is None:
            return None
        dx, dy = b[1] - a[1], b[2] - a[2]
        n = math.hypot(dx, dy)
        if n >= MOTION_MIN_M:
            return (-dy / n, dx / n)
    return None


def _planning_starts(phases: Sequence[Tuple[float, str]]) -> List[float]:
    return [t for i, (t, p) in enumerate(phases) if p == "PLANNING" and (i == 0 or phases[i - 1][1] != "PLANNING")]


# ----------------------------------------------------------------------------- compute

def compute(map_jsonl: str, truth_json: str, mission_json: str, targets_json: Optional[str] = None,
            heading_rad: Optional[float] = None, replay: bool = False) -> dict:
    if replay and not targets_json:
        raise ValueError("a replay map must be scored with its own --out-targets (targets_json): mission.json's "
                         "/spray_target ids belong to the recorded navigation, not to the replayed mapper")
    snaps = contract.read_snapshots(map_jsonl)
    truth = _load_truth(truth_json)
    by_truth = {ms._tree_id(t): t for t in truth}
    mission = contract.read_mission(mission_json)
    targets = _load_targets(targets_json) if targets_json else list(mission.targets)
    poses = list(mission.pose)
    pose_t = [p[0] for p in poses]
    if heading_rad is None:
        heading_rad = _run_heading(mission_json)
    across_fixed = None if heading_rad is None else (-math.sin(heading_rad), math.cos(heading_rad))

    times = [s.t for s in snaps]
    state: List[Dict[int, Tuple[Tuple[float, float], Optional[float]]]] = []
    first_pub: Dict[int, Tuple[float, int]] = {}      # id -> (stamp, snapshot index)
    identity: Dict[int, int] = {}                     # id -> truth id of its first matching snapshot
    last_centre: Dict[int, Tuple[float, float]] = {}
    seen_unmatched = set()
    for k, s in enumerate(snaps):
        cur = {}
        for tr in s.trees:
            cur[tr.id] = (tr.centre, tr.height)
            first_pub.setdefault(tr.id, (s.t, k))
            seen_unmatched.add(tr.id)
            if tr.id in identity or last_centre.get(tr.id) == tr.centre:
                continue
            last_centre[tr.id] = tr.centre
            m = ms.match_target_to_truth(tr.id, [tr], truth)
            if m is not None:
                identity[tr.id] = m
        state.append(cur)

    t_end = max([times[-1] if times else 0.0, mission.phases[-1][0] if mission.phases else 0.0,
                 pose_t[-1] if pose_t else 0.0])
    episodes = ms.dose_episodes(mission.phases, targets, [], t_end)
    doses_of: Dict[int, List[ms.Episode]] = {}
    for e in episodes:
        if e.tm_id is not None and not e.orphan:
            doses_of.setdefault(e.tm_id, []).append(e)
    planning = _planning_starts(mission.phases)

    def errors_at(i: int, t: float, tt: dict) -> Tuple[Optional[float], Optional[float], bool]:
        """(dh, dc, present) of entry i in the latest snapshot with stamp <= t."""
        k = bisect.bisect_right(times, t) - 1
        ent = state[k].get(i) if k >= 0 else None
        if ent is None:
            return None, None, False
        (cx, cy), h = ent
        tc = ms._tree_centre(tt)
        return (None if h is None else h - ms._tree_top(tt)), math.hypot(cx - tc[0], cy - tc[1]), True

    entries = []
    not_in_snapshot = 0
    for i in sorted(identity):
        tt = by_truth[identity[i]]
        t_first = first_pub[i][0]
        doses = []
        for e in sorted(doses_of.get(i, []), key=lambda e: e.start):
            dh, dc, present = errors_at(i, e.start, tt)
            if not present:
                not_in_snapshot += 1
                continue
            doses.append({"t": e.start, "dh": dh, "dc": dc})
        if doses_of.get(i):
            first_dose = min(e.start for e in doses_of[i])
            before = [p for p in planning if p <= first_dose]
            lock_t = max(before) if before else None
        else:
            after = [p for p in planning if p >= t_first]
            lock_t = min(after) if after else None
        lock = {"t": lock_t, "dh": None, "dc": None}
        if lock_t is not None:
            lock["dh"], lock["dc"], _ = errors_at(i, lock_t, tt)
        offset = None
        p = _pose_at(poses, pose_t, t_first)
        if p is not None:
            ax = across_fixed or _motion_axis(poses, pose_t, t_first)
            if ax is not None:
                tc = ms._tree_centre(tt)
                offset = (tc[0] - p[1]) * ax[0] + (tc[1] - p[2]) * ax[1]
        entries.append({"id": i, "truth_id": identity[i], "first_pub_t": t_first, "first_pub_offset_m": offset,
                        "lock": lock, "doses": doses})

    # every dose episode is accounted for: no target id, attached to a matched entry, or unattached (and why)
    eps = [e for e in episodes if not e.orphan]
    n_attached = sum(len(e["doses"]) for e in entries)
    n_no_target = sum(1 for e in eps if e.tm_id is None)
    breakdown = {"id_not_in_map": sum(1 for e in eps if e.tm_id is not None and e.tm_id not in first_pub),
                 "entry_unmatched": sum(1 for e in eps if e.tm_id in first_pub and e.tm_id not in identity),
                 "entry_not_in_snapshot": not_in_snapshot}
    n_unattached = sum(breakdown.values())
    n_with_target = len(eps) - n_no_target
    assert n_attached + n_unattached == n_with_target
    res = {"entries": entries,
           "unmatched_entries": sorted(i for i in seen_unmatched if i not in identity),
           "n_dose_episodes": len(eps), "n_dose_episodes_attached": n_attached,
           "n_dose_episodes_without_target": n_no_target, "n_dose_episodes_unattached": n_unattached,
           "unattached_breakdown": breakdown,
           "unattached_share": (n_unattached / n_with_target) if n_with_target else None}
    res.update(_summaries(entries))
    return res


def problems(res: dict, replay: bool = False) -> List[str]:
    """Reasons the metrics cannot be trusted (empty = fine): the share of dose episodes with a target id that could not
    be attached to a truth-matched entry of the map exceeds ``UNATTACHED_MAX_SHARE``. Scoring a replay map with the
    recorded navigation's target ids (instead of the replay's own ``--out-targets``) shows up here."""
    share = res.get("unattached_share")
    if share is None or share <= UNATTACHED_MAX_SHARE:
        return []
    b = res["unattached_breakdown"]
    return ["%d of %d dose episodes with a target id (%.0f %% > %.0f %%) are not attached to a matched map entry "
            "(id not in map %d, entry unmatched to truth %d, entry not in the snapshot at the dose %d)%s" % (
                res["n_dose_episodes_unattached"], res["n_dose_episodes"] - res["n_dose_episodes_without_target"],
                100 * share, 100 * UNATTACHED_MAX_SHARE, b["id_not_in_map"], b["entry_unmatched"],
                b["entry_not_in_snapshot"],
                "; is --targets the replay's own --out-targets?" if replay else "")]


def _summaries(entries: Sequence[dict]) -> dict:
    lock = [e["lock"]["dh"] for e in entries if e["lock"]["dh"] is not None]
    dose = [d["dh"] for e in entries for d in e["doses"] if d["dh"] is not None]
    a1 = {"max_dh_at_lock": max(lock, default=None), "max_dh_at_dose": max(dose, default=None),
          "n_over_1p5": sum(v > OVER_M for v in lock) + sum(v > OVER_M for v in dose),
          "n_over_1p5_lock": sum(v > OVER_M for v in lock), "n_over_1p5_dose": sum(v > OVER_M for v in dose)}
    per_entry = [[e["lock"]["dh"]] + [d["dh"] for d in e["doses"]] for e in entries]
    per_entry = [[v for v in vs if v is not None] for vs in per_entry]
    per_entry = [vs for vs in per_entry if vs]
    a2 = {"share_below_0p3": (sum(any(v < -UNDER_M for v in vs) for vs in per_entry) / len(per_entry)
                              if per_entry else None),
          "worst_dh": min(lock + dose, default=None), "n_entries": len(per_entry)}

    def group(sel: Sequence[dict]) -> dict:
        mean_dc = [sum(d["dc"] for d in e["doses"]) / len(e["doses"]) for e in sel]
        return {"n": len(sel), "mean_dc_at_dose": (sum(mean_dc) / len(mean_dc) if mean_dc else None),
                "share_over_0p8": (sum(v > FOOTPRINT_M for v in mean_dc) / len(mean_dc) if mean_dc else None)}

    known = [e for e in entries if e["first_pub_offset_m"] is not None and e["doses"]]
    a3 = {"far": group([e for e in known if abs(e["first_pub_offset_m"]) >= FAR_M]),
          "near": group([e for e in known if abs(e["first_pub_offset_m"]) < FAR_M])}
    return {"a1": a1, "a2": a2, "a3": a3}


# ----------------------------------------------------------------------------- markdown

def _f(v, nd: int = 3) -> str:
    if v is None:
        return "n/a"
    if isinstance(v, int) and not isinstance(v, bool):
        return str(v)
    return "%.*f" % (nd, v)


def _rows(res: dict) -> List[Tuple[str, Optional[float]]]:
    a1, a2, a3 = res["a1"], res["a2"], res["a3"]
    out = [("A1 max dh at lock (m)", a1["max_dh_at_lock"]), ("A1 max dh at dose (m)", a1["max_dh_at_dose"]),
           ("A1 n over 1.5 m (lock + dose values)", a1["n_over_1p5"]),
           ("A2 share of entries below -0.3 m", a2["share_below_0p3"]), ("A2 entries (denominator)", a2["n_entries"]),
           ("A2 worst dh (m)", a2["worst_dh"])]
    for g in ("far", "near"):
        out += [("A3 %s: entries with a dose" % g, a3[g]["n"]),
                ("A3 %s: mean dc at dose (m)" % g, a3[g]["mean_dc_at_dose"]),
                ("A3 %s: share dc > 0.8 m" % g, a3[g]["share_over_0p8"])]
    return out


def _delta(b, a) -> str:
    if b is None or a is None:
        return "n/a"
    return ("%+d" % (a - b)) if isinstance(b, int) and isinstance(a, int) else ("%+.3f" % (a - b))


def compare(before: dict, after: dict) -> str:
    """Markdown before / after table of the A1-A3 summaries, then per truth tree (its first entry) the lock and dose
    errors before and after."""
    lines = ["| Metric | Before | After | Change |", "|---|---|---|---|"]
    for (name, b), (_, a) in zip(_rows(before), _rows(after)):
        lines.append("| %s | %s | %s | %s |" % (name, _f(b), _f(a), _delta(b, a)))

    def per_tree(res: dict) -> Dict[int, dict]:
        out: Dict[int, dict] = {}
        for e in res["entries"]:
            if e["truth_id"] not in out or (e["doses"] and not out[e["truth_id"]]["doses"]):
                out[e["truth_id"]] = e
        return out

    pb, pa = per_tree(before), per_tree(after)

    def dose(e: Optional[dict], key: str) -> Optional[float]:
        vs = [d[key] for d in e["doses"] if d[key] is not None] if e else []
        return max(vs, key=abs) if vs else None

    lines += ["", "| Truth tree | Lock dh before | after | Dose dh before | after | Dose dc before | after |",
              "|---|---|---|---|---|---|---|"]
    for tid in sorted(set(pb) | set(pa)):
        eb, ea = pb.get(tid), pa.get(tid)
        lines.append("| %d | %s | %s | %s | %s | %s | %s |" % (
            tid, _f(eb["lock"]["dh"] if eb else None, 2), _f(ea["lock"]["dh"] if ea else None, 2),
            _f(dose(eb, "dh"), 2), _f(dose(ea, "dh"), 2), _f(dose(eb, "dc"), 2), _f(dose(ea, "dc"), 2)))
    return "\n".join(lines) + "\n"


def to_markdown(res: dict) -> str:
    lines = ["# Replay metrics against truth", "", "| Metric | Value |", "|---|---|"]
    lines += ["| %s | %s |" % (name, _f(v)) for name, v in _rows(res)]
    b = res["unattached_breakdown"]
    lines += ["", "%d entries matched to truth; unmatched entries (left out): %s. Dose episodes: %d, attached %d, "
              "without a target id %d, unattached %d (id not in map %d, entry unmatched to truth %d, entry not in the "
              "snapshot at the dose %d)." % (
                  len(res["entries"]), res["unmatched_entries"] or "none", res["n_dose_episodes"],
                  res["n_dose_episodes_attached"], res["n_dose_episodes_without_target"],
                  res["n_dose_episodes_unattached"], b["id_not_in_map"], b["entry_unmatched"],
                  b["entry_not_in_snapshot"]),
              "", "| Entry | Truth | First published (s) | Offset (m) | Lock dh | Lock dc | Doses (t: dh / dc) |",
              "|---|---|---|---|---|---|---|"]
    for e in res["entries"]:
        ds = "; ".join("%.1f: %s / %s" % (d["t"], _f(d["dh"], 2), _f(d["dc"], 2)) for d in e["doses"]) or "-"
        lines.append("| %d | %d | %.1f | %s | %s | %s | %s |" % (
            e["id"], e["truth_id"], e["first_pub_t"], _f(e["first_pub_offset_m"], 2), _f(e["lock"]["dh"], 2),
            _f(e["lock"]["dc"], 2), ds))
    return "\n".join(lines) + "\n"


# ----------------------------------------------------------------------------- CLI

def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--map", required=True, help="map stream (contract JSONL): map.jsonl or treemapper_replay --out-map")
    ap.add_argument("--truth", required=True, help="truth.json (top_z) or a contract ground_truth.json (height)")
    ap.add_argument("--mission", required=True, help="the recorded run's mission.json (phases, targets, pose)")
    ap.add_argument("--targets", help="treemapper_replay --out-targets file: replaces mission.json's /spray_target timeline")
    ap.add_argument("--replay", action="store_true",
                    help="the map is a treemapper_replay output: --targets (its --out-targets) is then required")
    ap.add_argument("--heading-rad", type=float, help="row heading for the across-track axis (default: run.json, else motion)")
    ap.add_argument("--before", help="earlier --json output: also write the before / after table")
    ap.add_argument("--json", help="write the metrics here")
    ap.add_argument("--md", help="write the markdown here (default: stdout)")
    a = ap.parse_args(argv)
    if a.replay and not a.targets:
        ap.error("--replay needs --targets (the replay's own --out-targets): mission.json's /spray_target ids are "
                 "the recorded navigation's, not the replayed mapper's")
    res = compute(a.map, a.truth, a.mission, a.targets, a.heading_rad, a.replay)
    res["problems"] = problems(res, a.replay)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1)
    md = to_markdown(res)
    if res["problems"]:
        md += "\n**NOT TRUSTWORTHY:** " + "; ".join(res["problems"]) + "\n"
    if a.before:
        with open(a.before) as f:
            md += "\n## Against " + a.before + "\n\n" + compare(json.load(f), res)
    if a.md:
        with open(a.md, "w") as f:
            f.write(md)
    else:
        sys.stdout.write(md)
    for msg in res["problems"]:
        print("replay_metrics: " + msg, file=sys.stderr)
    return 3 if res["problems"] else 0


if __name__ == "__main__":
    sys.exit(main())
