"""Closed-loop mission scorer for the Isaac corridor-spray loop (spec section 8.2).

    python3 -m treemapper_tools.mission_score --run <run dir> [--compare <run dir> ...] [--out <md>]

A run directory holds (written by sim_mission.sh / orchard_sim.py): ``run.json`` (spec, strategy, git_commit, seed),
``truth.json`` (sim/isaac/truth.py), ``ground_truth.json`` (contract format; written here if missing),
``deposition.json`` (final cumulative nozzle deposition), ``rtf.json`` and ``mission.bag``.

Per run it writes ``mission_score.json`` and, into the run directory, ``map.jsonl``, ``mission.json`` and
``ground_truth.json`` (if missing), then calls the existing map scorer (``score.score``). The markdown scorecard goes
to ``--out`` or stdout. With ``--compare`` the runs are grouped by strategy and each metric gets mean, worst case
(in the bad direction) and run-to-run spread.

Definitions (controller rulings, Task 15 and the interim whole-branch review, I1 / I2):
* dose episode = one contiguous ``/mission_state`` SPRAYING interval with a single ``/spray_target`` id (a target
  change inside SPRAYING starts a new episode). The nozzle-open intervals of ``/corridor_nav/spray`` (rising ..
  falling edge) inside it, within ``EPISODE_SLACK_S`` of its ends, belong to it: a guard climb that closes the nozzle
  mid-dose does not make a second episode. A nozzle-open interval outside every SPRAYING interval is an *orphan*
  unit of its own (and a failed nozzle safety row).
* intended dose of an episode = ``flow_lps`` x its SPRAYING duration (orphan: x its open time); emitted = ``flow_lps``
  x the nozzle-open time.
* an episode's deposit = the cumulative ``/sim/deposition`` at the first sample at or after its window end, minus the
  latest sample at or before its window start (window = the SPRAYING interval widened to its nozzle intervals). The
  window end is clipped at the next unit's start (no shared deposit); with no sample after the end of the last unit
  the final ``deposition.json`` is used (any other unit: error).
* missing or inconsistent inputs are listed in the card's ``errors`` (no deposition samples, flow <= 0, no TF poses,
  no nozzle-open interval, deposition.json missing or disagreeing with the episodes by > 5 % of the litres emitted);
  any error, a failed safety row, or a mission that did not complete makes ``all_pass`` false and the CLI exit with 2.
* *completed* = the first NavStatus end state is DONE, the last NavStatus counters have ``aborts`` == 0 and
  ``hold_timeouts`` == 0, and ``/mission_state`` was never ERROR (an airborne ABORT shows ERROR for one tick, then
  flies home and lands in DONE; the 2 Hz NavStatus can miss the ABORT state).
* a truth tree is *sprayed* if it received >= 50 % of some episode's intended dose; *double-sprayed* if two separate
  episodes each delivered >= 20 % of their intended dose to it (a spray metric, not a safety row); *missed* if never
  sprayed.
* *targeted again* (safety row) = an episode whose TreeMapper id was already ``sprayed`` in the latest map before the
  episode, or had an earlier episode after which a map showed it ``sprayed`` (the flag flipping is the evidence of a
  completed set_tree_sprayed mark). A retry on an id that was never marked is not targeted again.
* an episode's truth target = its TreeMapper id mapped with the latest map snapshot before the episode (truth outline
  containing the TreeMapper centre, else nearest truth centre within 2 m, else unmatched). on-target = litres on the
  matched truth tree / litres emitted; an unmatched episode counts 0 on-target and is counted separately.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from treemapper_tools import contract

GROUND = -1                    # key of the ground in a per-event deposit / cumulative deposition dict
SPRAYING = "SPRAYING"
SPRAYED_FRAC = 0.5
DOUBLE_FRAC = 0.2
CONSISTENCY_TOL = 0.05         # event deposits must match deposition.json within this fraction of the emitted litres
EPISODE_SLACK_S = 0.15         # s: /corridor_nav/spray and /mission_state are separate topics, a control tick apart
MATCH_NEAR_M = 2.0

# NavStatus.state -> time-split group (spec 8.2: scan / settle / transit / glide / dose / hold)
STATE_GROUP = {"SCAN_OUT": "scan", "SCAN_SETTLE": "settle", "TAKEOFF": "transit", "TRANSIT": "transit",
               "LOCK": "transit", "RETURN": "transit", "LAND": "transit", "GLIDE": "glide", "DOSE": "dose",
               "HOLD": "hold"}
GROUPS = ("scan", "settle", "transit", "glide", "dose", "hold", "other")
END_STATES = ("DONE", "ABORT")

TIMING_STREAMS = (("step", "/corridor_nav/timing", "step"), ("sgbm", "/stereo_sgbm/timing", "sgbm"),
                  ("detector", "/detector/timing", "detector"))


# ----------------------------------------------------------------------------- small geometry

def _as_outline(tree) -> List[Tuple[float, float]]:
    o = tree["outline"] if isinstance(tree, dict) else tree.outline
    return [(float(p[0]), float(p[1])) for p in o]


def _tree_id(tree) -> int:
    return int(tree["id"] if isinstance(tree, dict) else tree.id)


def _tree_top(tree) -> float:
    if isinstance(tree, dict):
        return float(tree["top_z"] if "top_z" in tree else tree["height"])
    return float(tree.height)


def _tree_centre(tree) -> Tuple[float, float]:
    c = tree["centre"] if isinstance(tree, dict) else tree.centre
    return (float(c[0]), float(c[1]))


def point_in_polygon(x: float, y: float, poly: Sequence[Tuple[float, float]]) -> bool:
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def dist_to_polygon(x: float, y: float, poly: Sequence[Tuple[float, float]]) -> float:
    """0 inside the polygon, else the distance to its boundary."""
    if point_in_polygon(x, y, poly):
        return 0.0
    best = math.inf
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + u * dx), y - (ay + u * dy)))
    return best


# ----------------------------------------------------------------------------- pure metric functions

def dose_events(spray: Sequence[Tuple[float, bool]]) -> List[Tuple[float, float]]:
    """Nozzle-open intervals: (time of the first true sample, time of the next false sample). An interval still
    open at the last sample is closed at the last sample time."""
    events: List[Tuple[float, float]] = []
    t0 = None
    last = None
    for t, v in sorted(spray, key=lambda r: r[0]):
        last = t
        if v and t0 is None:
            t0 = t
        elif not v and t0 is not None:
            events.append((t0, t))
            t0 = None
    if t0 is not None and last is not None:
        events.append((t0, last))
    return events


@dataclass
class Episode:
    """One dose episode (or an orphan nozzle-open interval outside every SPRAYING interval)."""
    start: float
    end: float
    tm_id: Optional[int]                      # TreeMapper id on /spray_target (None: -1 / unknown)
    intervals: List[Tuple[float, float]] = field(default_factory=list)   # nozzle-open intervals in it
    orphan: bool = False

    @property
    def spraying_s(self) -> float:
        """Duration that sets the intended dose: the SPRAYING interval (orphan: its open time)."""
        return self.open_s if self.orphan else self.end - self.start

    @property
    def open_s(self) -> float:
        return float(sum(b - a for a, b in self.intervals))

    @property
    def window(self) -> Tuple[float, float]:
        """Deposit window: the episode widened to its nozzle intervals."""
        return (min([self.start] + [a for a, _ in self.intervals]), max([self.end] + [b for _, b in self.intervals]))


def _tm(v) -> Optional[int]:
    return int(v) if v is not None and int(v) >= 0 else None


def dose_episodes(phases: Sequence[Tuple[float, str]], targets: Sequence[Tuple[float, int]],
                  spray: Sequence[Tuple[float, bool]], t_end: float,
                  slack: float = EPISODE_SLACK_S) -> List[Episode]:
    """Dose episodes, sorted by start: each contiguous SPRAYING interval of ``phases`` (change list), split where
    ``targets`` changes strictly inside it (more than ``slack`` from both ends), with the nozzle-open intervals of
    ``spray`` whose start lies in [start - slack, end + slack); a SPRAYING interval still open at the last phase change
    closes at ``t_end``. Nozzle intervals in no episode become orphan episodes (target = /spray_target at their
    start)."""
    ph = sorted(phases, key=lambda r: r[0])
    tg = sorted(targets, key=lambda r: r[0])
    eps: List[Episode] = []
    for i, (t, p) in enumerate(ph):
        if p != SPRAYING or (i > 0 and ph[i - 1][1] == SPRAYING):
            continue
        j = i
        while j + 1 < len(ph) and ph[j + 1][1] == SPRAYING:
            j += 1
        end = ph[j + 1][0] if j + 1 < len(ph) else max(t_end, t)
        cuts = [tc for tc, _ in tg if t + slack < tc < end - slack]
        bounds = [t] + cuts + [end]
        for a, b in zip(bounds, bounds[1:]):
            eps.append(Episode(a, b, _tm(contract.step_lookup(tg, a + (slack if a == t else 0.0), -1))))
    for t0, t1 in dose_events(spray):
        home = next((e for e in eps if e.start - slack <= t0 < e.end + slack), None)
        if home is not None:
            home.intervals.append((t0, t1))
        else:
            eps.append(Episode(t0, t1, _tm(contract.step_lookup(tg, t0, -1)), [(t0, t1)], orphan=True))
    eps.sort(key=lambda e: e.window[0])
    return eps


def _marked(snap, tm_id: int) -> bool:
    return any(int(tr.id) == tm_id and bool(tr.sprayed) for tr in snap.trees)


def targeted_again(episodes: Sequence[Episode], snaps: Sequence[contract.Snapshot]) -> List[int]:
    """Indices of the episodes that aimed at a TreeMapper id already sprayed: ``sprayed`` in the latest snapshot
    before the episode start, or in any snapshot between the end of an earlier episode on the same id and this
    start (a completed set_tree_sprayed mark). Episodes without an id are not judged."""
    ss = sorted(snaps, key=lambda s: s.t)
    st = [s.t for s in ss]
    out = []
    for k, e in enumerate(episodes):
        if e.tm_id is None:
            continue
        before = bisect.bisect_left(st, e.start)          # snapshots strictly before the start
        if before > 0 and _marked(ss[before - 1], e.tm_id):
            out.append(k)
            continue
        for p in episodes[:k]:
            if p.tm_id != e.tm_id or p.end > e.start:
                continue
            lo = bisect.bisect_right(st, p.end)
            if any(_marked(ss[j], e.tm_id) for j in range(lo, before)):
                out.append(k)
                break
    return out


def _cum_at(series: Sequence[Tuple[float, Dict[int, float]]], times: Sequence[float], t: float) -> Dict[int, float]:
    i = bisect.bisect_right(times, t)
    return series[i - 1][1] if i > 0 else {}


def per_event_deposit(events: Sequence[Tuple[float, float]], deposition_series: Sequence[Tuple[float, Dict[int, float]]],
                      final: Optional[Dict[int, float]] = None, errors: Optional[List[str]] = None) -> List[Dict[int, float]]:
    """Litres per tree id (``GROUND`` = -1 for the ground) deposited in each event: the cumulative deposition at the
    first series sample at or after the event end, minus the latest sample at or before its start. If that sample
    comes after the next event's start the window end is clipped to the latest sample at or before that start, so
    two close events cannot claim the same deposit. With no sample after the end: the last event uses ``final``
    (the final cumulative deposition.json); any other event (or no ``final``) appends a message to ``errors``."""
    series = sorted(deposition_series, key=lambda r: r[0])
    times = [r[0] for r in series]
    if not series:
        return [{} for _ in events]
    out = []
    for k, (t0, t1) in enumerate(events):
        nxt = events[k + 1][0] if k + 1 < len(events) else math.inf
        a = _cum_at(series, times, t0)
        i = bisect.bisect_left(times, t1)
        if i < len(times):
            b = series[i][1] if times[i] <= nxt else _cum_at(series, times, nxt)
        elif k + 1 == len(events) and final is not None:
            b = final
        else:
            b = a
            if errors is not None:
                errors.append("no /sim/deposition sample at or after the end of dose event %d (t = %.2f s)" % (k, t1))
        d = {}
        for key in set(a) | set(b):
            v = b.get(key, 0.0) - a.get(key, 0.0)
            if v > 1e-12:
                d[key] = v
        out.append(d)
    return out


def deposit_consistency(deposits: Sequence[Dict[int, float]], final: Dict[int, float], emitted: float,
                        tol: float = CONSISTENCY_TOL) -> List[str]:
    """Messages for every key (tree or ground) whose summed per-event deposit differs from the final deposition by
    more than ``tol`` x emitted, and for the total."""
    tot: Dict[int, float] = {}
    for d in deposits:
        for k, v in d.items():
            tot[k] = tot.get(k, 0.0) + v
    lim = tol * emitted
    msgs = []
    for k in sorted(set(tot) | set(final)):
        if abs(tot.get(k, 0.0) - final.get(k, 0.0)) > lim:
            msgs.append("%s: events sum to %.4f L, deposition.json has %.4f L" % (
                "ground" if k == GROUND else "tree %d" % k, tot.get(k, 0.0), final.get(k, 0.0)))
    s_ev, s_fin = sum(tot.values()), sum(final.values())
    if abs(s_ev - s_fin) > lim:
        msgs.append("total: events sum to %.4f L, deposition.json has %.4f L" % (s_ev, s_fin))
    return msgs


def _truth_ids(truth) -> List[int]:
    return [int(t) if isinstance(t, (int, np.integer)) else _tree_id(t) for t in truth]


def spray_metrics(events_deposits: Sequence[Dict[int, float]], truth, intended_l_per_event,
                  targets: Optional[Sequence[Optional[int]]] = None,
                  emitted_l_per_event: Optional[Sequence[float]] = None) -> dict:
    """Spray outcome per dose episode. ``truth``: truth trees (or ids). ``intended_l_per_event``: intended dose of
    each episode (a sequence, or one number for all); sprayed / double-sprayed are fractions of it.
    ``targets``: truth id each episode aimed at (None = unmatched); optional. ``emitted_l_per_event``: litres the
    nozzle emitted in each episode, the base of the on-target fraction (default: the intended dose)."""
    n = len(events_deposits)
    ids = _truth_ids(truth)
    if isinstance(intended_l_per_event, (int, float)):
        want = [float(intended_l_per_event)] * n
    else:
        want = [float(v) for v in intended_l_per_event]
    if len(want) != n:
        raise ValueError("intended_l_per_event has %d entries for %d events" % (len(want), n))
    out_l = list(want) if emitted_l_per_event is None else [float(v) for v in emitted_l_per_event]
    if len(out_l) != n:
        raise ValueError("emitted_l_per_event has %d entries for %d events" % (len(out_l), n))
    targets = list(targets) if targets is not None else [None] * n
    if len(targets) != n:
        raise ValueError("targets has %d entries for %d events" % (len(targets), n))
    eps = 1e-9
    sprayed, big_events = set(), {i: 0 for i in ids}
    on_target = 0.0
    unmatched = 0
    rows = []
    for e, (dep, w, em, tg) in enumerate(zip(events_deposits, want, out_l, targets)):
        got_target = 0.0
        if tg is None:
            unmatched += 1
        else:
            got_target = dep.get(tg, 0.0)
            on_target += got_target
        newly = []
        for i in ids:
            frac = dep.get(i, 0.0) / w if w > 0 else 0.0
            if frac >= SPRAYED_FRAC - eps:
                newly.append(i)
            if frac >= DOUBLE_FRAC - eps:
                big_events[i] += 1
        sprayed.update(newly)
        rows.append({"event": e, "target": tg, "intended_l": w, "emitted_l": em, "on_target_l": got_target,
                     "ground_l": dep.get(GROUND, 0.0), "trees_l": {str(k): round(v, 6) for k, v in sorted(dep.items())
                                                                   if k != GROUND}})
    emitted = sum(out_l)
    return {
        "n_events": n,
        "intended_l_total": sum(want),
        "emitted_l_total": emitted,
        "sprayed_ids": sorted(sprayed), "n_sprayed": len(sprayed),
        "missed_ids": [i for i in ids if i not in sprayed], "n_missed": len([i for i in ids if i not in sprayed]),
        "double_sprayed_ids": [i for i in ids if big_events[i] >= 2],
        "n_double_sprayed": len([i for i in ids if big_events[i] >= 2]),
        "on_target_l": on_target,
        "on_target_fraction": (on_target / emitted) if emitted > 0 else None,
        "off_target_l": max(0.0, emitted - on_target),
        "unmatched_events": unmatched,
        "ground_l": sum(r["ground_l"] for r in rows),
        "events": rows,
    }


def clearance_min_at(poses: Iterable[Sequence[float]], truth, drone_radius: float) -> Tuple[float, Optional[float]]:
    """(minimum of ``drone z - top_z`` over every pose and every truth tree whose outline lies within
    ``drone_radius`` of the drone xy, time of that minimum). ``(inf, None)`` if the drone was never near a tree."""
    trees = [(_as_outline(t), _tree_top(t)) for t in truth]
    boxes = [(min(p[0] for p in o) - drone_radius, max(p[0] for p in o) + drone_radius,
              min(p[1] for p in o) - drone_radius, max(p[1] for p in o) + drone_radius) for o, _ in trees]
    best, best_t = math.inf, None
    for t, x, y, z in poses:
        for (o, top), (x0, x1, y0, y1) in zip(trees, boxes):
            if x < x0 or x > x1 or y < y0 or y > y1:
                continue
            if z - top < best and dist_to_polygon(x, y, o) <= drone_radius:
                best, best_t = z - top, t
    return best, best_t


def clearance_min(poses: Iterable[Sequence[float]], truth, drone_radius: float) -> float:
    return clearance_min_at(poses, truth, drone_radius)[0]


def _durations(series: Sequence[Tuple[float, str]], t0: float, t1: float) -> Dict[str, float]:
    """Seconds spent in each value of a piecewise-constant series within [t0, t1]."""
    out: Dict[str, float] = {}
    s = sorted(series, key=lambda r: r[0])
    for i, (t, v) in enumerate(s):
        a = max(t, t0)
        b = min(s[i + 1][0] if i + 1 < len(s) else t1, t1)
        if b > a:
            out[v] = out.get(v, 0.0) + (b - a)
    return out


def time_split(phases: Sequence[Tuple[float, str]], status_states: Sequence[Tuple[float, str]],
               t_end: Optional[float] = None) -> dict:
    """Seconds per ``/mission_state`` phase, per NavStatus state, and per group (scan / settle / transit / glide /
    dose / hold) over the mission window: first non-IDLE state .. first DONE/ABORT (else ``t_end``, else the last
    sample)."""
    st = sorted(status_states, key=lambda r: r[0])
    ph = sorted(phases, key=lambda r: r[0])
    main = st or ph
    if t_end is None:
        t_end = max([r[0] for r in st + ph], default=0.0)
    t_start = next((t for t, s in main if s != "IDLE"), None)
    if t_start is None:
        return {"t_start": None, "t_end": None, "mission_time_s": 0.0, "by_phase": {}, "by_state": {},
                "groups": {g: 0.0 for g in GROUPS}}
    t_stop = next((t for t, s in main if t > t_start and s in END_STATES), t_end)
    t_stop = max(t_stop, t_start)
    by_state = _durations(st, t_start, t_stop)
    groups = {g: 0.0 for g in GROUPS}
    for s, d in by_state.items():
        if s in ("IDLE",) + END_STATES:
            continue
        groups[STATE_GROUP.get(s, "other")] += d
    return {"t_start": t_start, "t_end": t_stop, "mission_time_s": t_stop - t_start,
            "by_phase": _durations(ph, t_start, t_stop), "by_state": by_state, "groups": groups}


def nozzle_outside_dose(spray: Sequence[Tuple[float, bool]], phases: Sequence[Tuple[float, str]]) -> int:
    """Samples with the nozzle commanded open while ``/mission_state`` is not SPRAYING."""
    ph = sorted(phases, key=lambda r: r[0])
    times = [r[0] for r in ph]
    n = 0
    for t, v in spray:
        if not v:
            continue
        i = bisect.bisect_right(times, t)
        if i == 0 or ph[i - 1][1] != SPRAYING:
            n += 1
    return n


def cost_metrics(timing_series: Dict[str, Sequence[float]]) -> dict:
    """{stream: {n, mean_ms, p99_ms, max_ms}} from {stream: [ms, ...]}."""
    out = {}
    for name, vals in timing_series.items():
        a = np.asarray([v for v in vals if v is not None and math.isfinite(v)], dtype=float)
        if len(a) == 0:
            out[name] = {"n": 0, "mean_ms": None, "p99_ms": None, "max_ms": None}
        else:
            out[name] = {"n": int(len(a)), "mean_ms": float(a.mean()), "p99_ms": float(np.percentile(a, 99)),
                         "max_ms": float(a.max())}
    return out


def match_target_to_truth(tm_id: int, map_trees: Sequence[contract.Tree], truth,
                          near_m: float = MATCH_NEAR_M) -> Optional[int]:
    """Truth id of a TreeMapper tree: the truth outline containing its centre, else the nearest truth centre within
    ``near_m``, else None."""
    tm = next((t for t in map_trees if t.id == tm_id), None)
    if tm is None or not all(math.isfinite(v) for v in tm.centre):
        return None
    cx, cy = tm.centre
    for t in truth:
        if point_in_polygon(cx, cy, _as_outline(t)):
            return _tree_id(t)
    best, best_d = None, near_m
    for t in truth:
        tx, ty = _tree_centre(t)
        d = math.hypot(tx - cx, ty - cy)
        if d <= best_d:
            best, best_d = _tree_id(t), d
    return best


# ----------------------------------------------------------------------------- bag

@dataclass
class BagData:
    poses: List[Tuple[float, float, float, float]] = field(default_factory=list)
    phases: List[Tuple[float, str]] = field(default_factory=list)
    targets: List[Tuple[float, int]] = field(default_factory=list)
    spray: List[Tuple[float, bool]] = field(default_factory=list)
    status_states: List[Tuple[float, str]] = field(default_factory=list)
    status_last: Dict[str, Any] = field(default_factory=dict)
    timing: Dict[str, List[float]] = field(default_factory=dict)
    maps: List[Tuple[float, str, List[dict]]] = field(default_factory=list)
    map_timing: Dict[float, Dict[str, float]] = field(default_factory=dict)
    deposition: List[Tuple[float, Dict[int, float]]] = field(default_factory=list)
    deposition_unparsed: int = 0


def _entry(e) -> dict:
    """TreeEntry -> contract tree dict (same conversion as treemapper_bag_to_jsonl.entry)."""
    return {"id": int(e.id), "outline": [[round(p.x, 4), round(p.y, 4)] for p in e.outline],
            "centre": [round(e.centre.x, 4), round(e.centre.y, 4)],
            "height": round(e.height, 4) if math.isfinite(e.height) else None, "sprayed": bool(e.sprayed)}


def parse_deposition(doc: dict) -> Dict[int, float]:
    d = {int(k): float(v) for k, v in (doc.get("trees") or {}).items()}
    d[GROUND] = float(doc.get("ground", 0.0))
    return d


def _stamp_or(msg, bt) -> float:
    h = getattr(msg, "header", None)
    if h is not None and (h.stamp.secs or h.stamp.nsecs):
        return h.stamp.to_sec()
    return bt.to_sec()


def read_bag(path: str) -> BagData:
    import rosbag
    b = BagData()
    wanted = ["/tf", "/mission_state", "/spray_target", "/corridor_nav/spray", "/corridor_nav/status",
              "/treemapper/map", "/treemapper/stage_timing", "/sim/deposition"] + [t for _, t, _ in TIMING_STREAMS]
    stage_of = {t: (k, s) for k, t, s in TIMING_STREAMS}
    with rosbag.Bag(path) as bag:
        for topic, msg, bt in bag.read_messages(topics=wanted):
            if topic == "/tf":
                for tr in msg.transforms:
                    if tr.header.frame_id.lstrip("/") == "world" and tr.child_frame_id.lstrip("/") == "body":
                        v = tr.transform.translation
                        b.poses.append((tr.header.stamp.to_sec() or bt.to_sec(), v.x, v.y, v.z))
            elif topic == "/mission_state":
                if not b.phases or b.phases[-1][1] != msg.data:
                    b.phases.append((bt.to_sec(), msg.data))
            elif topic == "/spray_target":
                if not b.targets or b.targets[-1][1] != int(msg.data):
                    b.targets.append((bt.to_sec(), int(msg.data)))
            elif topic == "/corridor_nav/spray":
                b.spray.append((bt.to_sec(), bool(msg.data)))
            elif topic == "/corridor_nav/status":
                t = _stamp_or(msg, bt)
                if not b.status_states or b.status_states[-1][1] != msg.state:
                    b.status_states.append((t, msg.state))
                b.status_last = {"t": t, "trees_sprayed": int(msg.trees_sprayed),
                                 "trees_skipped": int(msg.trees_skipped),
                                 "trees_remaining": int(msg.trees_remaining),
                                 "counters": {str(r): int(c) for r, c in zip(msg.reasons, msg.counts)}}
            elif topic == "/treemapper/map":
                b.maps.append((msg.header.stamp.to_sec(), msg.header.frame_id, [_entry(e) for e in msg.trees]))
            elif topic == "/treemapper/stage_timing":
                st = dict(zip(msg.stages, msg.ms))
                b.map_timing[round(msg.header.stamp.to_sec(), 6)] = st
                for k, v in st.items():
                    b.timing.setdefault("treemapper." + k, []).append(float(v))
            elif topic == "/sim/deposition":
                try:
                    doc = json.loads(msg.data)
                    if not isinstance(doc, dict):
                        raise ValueError("not an object")
                    b.deposition.append((bt.to_sec(), parse_deposition(doc)))
                except (ValueError, TypeError, AttributeError, KeyError):
                    b.deposition_unparsed += 1
            elif topic in stage_of:
                key, stage = stage_of[topic]
                for s, v in zip(msg.stages, msg.ms):
                    if s == stage:
                        b.timing.setdefault(key, []).append(float(v))
    b.poses.sort(key=lambda r: r[0])
    return b


def snapshots_from_bag(b: BagData) -> List[contract.Snapshot]:
    """One Snapshot per TreeMap message, strictly increasing stamps (as treemapper_bag_to_jsonl.py)."""
    snaps, last = [], -math.inf
    for t, frame, trees in sorted(b.maps, key=lambda m: m[0]):
        if t <= last:
            continue
        last = t
        rec = {"t": round(t, 6), "frame": frame or "world", "trees": trees}
        st = b.map_timing.get(round(t, 6))
        if st:
            rec["stage_ms"] = {k: round(v, 4) for k, v in st.items()}
        snaps.append(contract.Snapshot.from_json(rec))
    return snaps


# ----------------------------------------------------------------------------- run scoring

def _load_json(path: str, default=None):
    if not os.path.exists(path):
        return default
    with open(path) as f:
        return json.load(f)


def _clean(o):
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.generic):
        return _clean(o.item())
    return o


def _load_truth(run_dir: str) -> List[dict]:
    t = _load_json(os.path.join(run_dir, "truth.json"))
    if t is not None:
        return list(t["trees"])
    gt = contract.read_ground_truth(os.path.join(run_dir, "ground_truth.json"))
    return [{"id": g.id, "outline": [list(p) for p in g.outline], "centre": list(g.centre), "radius": g.radius,
             "top_z": g.height} for g in gt]


def _ensure_ground_truth(run_dir: str, truth: List[dict]) -> str:
    p = os.path.join(run_dir, "ground_truth.json")
    if not os.path.exists(p):
        contract.write_ground_truth(p, [contract.GroundTruthTree(t["id"], [tuple(q) for q in t["outline"]],
                                                                 tuple(t["centre"]), t.get("radius", 0.0), t["top_z"])
                                        for t in truth], frame="world")
        with open(p) as f:
            doc = json.load(f)
        doc["height_reliable"] = True
        with open(p, "w") as f:
            json.dump(doc, f, indent=1)
    return p


def _downsample_poses(poses, dt=0.1):
    out, last = [], -math.inf
    for p in poses:
        if p[0] - last >= dt:
            out.append(p)
            last = p[0]
    return out


def _path_length(poses) -> float:
    return float(sum(math.dist(a[1:], b[1:]) for a, b in zip(poses, poses[1:])))


def episode_targets(episodes: Sequence[Episode], snaps, truth) -> List[Optional[int]]:
    """Truth id each episode aimed at (None = unmatched): its TreeMapper id in the latest snapshot before it."""
    snap_t = [s.t for s in snaps]
    out = []
    for e in episodes:
        i = bisect.bisect_right(snap_t, e.start)
        if e.tm_id is None or i == 0:
            out.append(None)
        else:
            out.append(match_target_to_truth(e.tm_id, snaps[i - 1].trees, truth))
    return out


def score_run(run_dir: str) -> dict:
    run = _load_json(os.path.join(run_dir, "run.json"), {})
    spec = run.get("spec", {})
    errors: List[str] = []
    try:
        flow = float((spec.get("nozzle") or {}).get("flow_lps"))
    except (TypeError, ValueError):
        flow = float("nan")
    if not (math.isfinite(flow) and flow > 0):
        errors.append("run.json: spec.nozzle.flow_lps is missing or not > 0 (no intended dose can be computed)")
        flow = 0.0
    radius = float(spec.get("drone", {}).get("radius", 0.8))
    truth = _load_truth(run_dir)
    gt_path = _ensure_ground_truth(run_dir, truth)
    depo = _load_json(os.path.join(run_dir, "deposition.json"))
    rtf = _load_json(os.path.join(run_dir, "rtf.json"), {})
    b = read_bag(os.path.join(run_dir, "mission.bag"))

    snaps = snapshots_from_bag(b)
    contract.write_snapshots(os.path.join(run_dir, "map.jsonl"), snaps)
    mission = contract.Mission(list(b.phases), list(b.targets), _downsample_poses(b.poses))
    contract.write_mission(os.path.join(run_dir, "mission.json"), mission)

    t_end = max([r[0] for r in (b.poses[-1:] + b.phases[-1:] + b.status_states[-1:] + b.spray[-1:])], default=0.0)
    split = time_split(b.phases, b.status_states, t_end)
    events = dose_events(b.spray)
    episodes = dose_episodes(b.phases, b.targets, b.spray, t_end)
    final = parse_deposition(depo) if isinstance(depo, dict) and depo else None
    if not b.deposition:
        errors.append("no /sim/deposition samples in the bag")
    if b.deposition_unparsed:
        errors.append("%d unparseable /sim/deposition samples" % b.deposition_unparsed)
    if final is None:
        errors.append("deposition.json is missing or empty")
    if not b.poses:
        errors.append("no TF pose samples (no world -> body transform in /tf): clearance cannot be judged")
    if not events:
        errors.append("no dose events (/corridor_nav/spray was never true)")
    dep_errors: List[str] = []
    deposits = per_event_deposit([e.window for e in episodes], b.deposition, final,
                                 dep_errors if b.deposition else None)
    errors += dep_errors
    if events and b.deposition and final is not None:
        emitted_final = float(depo["emitted"]) if "emitted" in depo else sum(final.values())
        errors += ["deposition inconsistent: " + m for m in deposit_consistency(deposits, final, emitted_final)]
    intended = [flow * e.spraying_s for e in episodes]
    emitted_ep = [flow * e.open_s for e in episodes]
    targets = episode_targets(episodes, snaps, truth)
    sp = spray_metrics(deposits, truth, intended, targets, emitted_ep)
    again = targeted_again(episodes, snaps)
    cmin, cmin_t = clearance_min_at(b.poses, truth, radius)
    outside = nozzle_outside_dose(b.spray, b.phases)
    counters = dict(b.status_last.get("counters", {}))
    cost = cost_metrics(b.timing)
    for k, _, _ in TIMING_STREAMS:
        cost.setdefault(k, {"n": 0, "mean_ms": None, "p99_ms": None, "max_ms": None})
    cost["rtf"] = rtf.get("rtf")
    cost["sim_s"], cost["wall_s"] = rtf.get("sim_s"), rtf.get("wall_s")

    emitted = depo.get("emitted") if isinstance(depo, dict) else None
    if emitted is None:
        emitted = sum(emitted_ep)
    mt = split["mission_time_s"]
    n_sp = sp["n_sprayed"]
    end_state = next((st for _, st in b.status_states if st in END_STATES), "none")
    aborts, hold_timeouts = int(counters.get("aborts", 0)), int(counters.get("hold_timeouts", 0))
    error_phase = any(p == "ERROR" for _, p in b.phases)
    completed = end_state == "DONE" and aborts == 0 and hold_timeouts == 0 and not error_phase
    safety = {"clearance_min_m": cmin if math.isfinite(cmin) else None, "clearance_min_t": cmin_t,
              "clearance_pass": bool(b.poses) and cmin > 0, "n_pose_samples": len(b.poses),
              "nozzle_outside_dose": outside, "nozzle_pass": outside == 0,
              "sprayed_tree_targeted_again": len(again), "targeted_again_episodes": again,
              "retarget_pass": not again}
    safety["completed"] = completed
    safety["all_pass"] = (not errors and completed and safety["clearance_pass"] and safety["nozzle_pass"]
                          and safety["retarget_pass"])
    fail = list(errors)
    if not completed:
        why = []
        if end_state != "DONE":
            why.append("end state %s (DONE required)" % end_state)
        if aborts or hold_timeouts:
            why.append("NavStatus counters aborts %d, hold_timeouts %d (0 required)" % (aborts, hold_timeouts))
        if error_phase:
            why.append("/mission_state was ERROR")
        fail.append("mission did not complete: " + "; ".join(why))
    if not safety["clearance_pass"] and b.poses:
        fail.append("clearance: minimum %.3f m is not > 0" % cmin)
    if not safety["nozzle_pass"]:
        fail.append("nozzle open outside DOSE in %d samples" % outside)
    if not safety["retarget_pass"]:
        fail.append("sprayed tree targeted again in %d episodes" % len(again))
    card: Dict[str, Any] = {
        "errors": errors, "end_state": end_state, "completed": completed, "all_pass": safety["all_pass"],
        "aborts": aborts, "hold_timeouts": hold_timeouts, "error_phase": error_phase,
        "fail_reasons": fail,
        "run": {"dir": os.path.abspath(run_dir), "strategy": run.get("strategy"), "seed": run.get("seed"),
                "git_commit": run.get("git_commit"), "flow_lps": flow, "drone_radius": radius,
                "n_truth_trees": len(truth)},
        "efficiency": {"mission_time_s": mt, "path_length_m": _path_length(b.poses), "time_split": split,
                       "seconds_per_sprayed_tree": (mt / n_sp) if n_sp else None,
                       "glide_climbs_inserted": counters.get("glide_climbs_inserted", 0),
                       "litres_emitted": emitted, "dose_events": len(events), "dose_episodes": len(episodes)},
        "spray": dict(sp, nav_trees_sprayed=b.status_last.get("trees_sprayed"),
                      nav_trees_skipped=b.status_last.get("trees_skipped"),
                      nav_trees_remaining=b.status_last.get("trees_remaining")),
        "safety": safety,
        "guards": {"guard_triggers": counters.get("guard_triggers", 0), "guard_blind": counters.get("guard_blind", 0),
                   "holds": {k: v for k, v in counters.items() if k.startswith("hold_")},
                   "ids_reresolved": counters.get("ids_reresolved", 0),
                   "ids_dropped_vanished": counters.get("ids_dropped_vanished", 0),
                   "late_trees": counters.get("late_trees", 0), "counters": counters},
        "cost": cost,
    }
    if snaps:
        from treemapper_tools import score as scorer
        card["map"] = scorer.score(snaps, contract.read_ground_truth(gt_path), mission)
    else:
        card["map"] = None
        card["map_note"] = "no /treemapper/map messages in the bag"
    card["flat"] = flatten(card)
    with open(os.path.join(run_dir, "mission_score.json"), "w") as f:
        json.dump(_clean(card), f, indent=1)
    return card


# ----------------------------------------------------------------------------- comparison

# metric -> (label, bad direction: "max" = larger is worse, "min" = smaller is worse)
COMPARE_METRICS = {
    "mission_time_s": ("Mission time (s)", "max"),
    "path_length_m": ("Path length (m)", "max"),
    "seconds_per_sprayed_tree": ("Seconds per sprayed tree", "max"),
    "litres_emitted": ("Litres emitted", "max"),
    "n_sprayed": ("Truth trees sprayed", "min"),
    "n_missed": ("Truth trees missed", "max"),
    "n_double_sprayed": ("Double-sprayed", "max"),
    "on_target_fraction": ("On-target fraction", "min"),
    "off_target_l": ("Off-target litres", "max"),
    "clearance_min_m": ("Minimum clearance (m)", "min"),
    "nozzle_outside_dose": ("Nozzle open outside DOSE", "max"),
    "step_mean_ms": ("Navigator step mean (ms)", "max"),
    "step_p99_ms": ("Navigator step p99 (ms)", "max"),
    "sgbm_mean_ms": ("SGBM mean (ms)", "max"),
    "detector_mean_ms": ("Detector mean (ms)", "max"),
    "rtf": ("Real-time factor", "min"),
}


def flatten(card: dict) -> Dict[str, Optional[float]]:
    e, s, sf, c = card["efficiency"], card["spray"], card["safety"], card["cost"]
    return {"mission_time_s": e["mission_time_s"], "path_length_m": e["path_length_m"],
            "seconds_per_sprayed_tree": e["seconds_per_sprayed_tree"], "litres_emitted": e["litres_emitted"],
            "n_sprayed": s["n_sprayed"], "n_missed": s["n_missed"], "n_double_sprayed": s["n_double_sprayed"],
            "on_target_fraction": s["on_target_fraction"], "off_target_l": s["off_target_l"],
            "clearance_min_m": sf["clearance_min_m"], "nozzle_outside_dose": sf["nozzle_outside_dose"],
            "step_mean_ms": c["step"]["mean_ms"], "step_p99_ms": c["step"]["p99_ms"],
            "sgbm_mean_ms": c["sgbm"]["mean_ms"], "detector_mean_ms": c["detector"]["mean_ms"], "rtf": c.get("rtf")}


# metrics of the mission as a whole: an aborted or unfinished run would flatter them, so they skip incomplete runs
MISSION_METRICS = ("mission_time_s", "path_length_m", "seconds_per_sprayed_tree")


def compare_runs(cards: Sequence[dict]) -> Dict[str, Dict[str, dict]]:
    """{strategy: {metric: {n, mean, worst, spread, values}, "_runs": {n, completed, incomplete}}}; worst is in
    the bad direction. Runs that did not complete are left out of ``MISSION_METRICS`` and listed in ``_runs``."""
    groups: Dict[str, List[dict]] = {}
    for c in cards:
        groups.setdefault(str(c["run"].get("strategy")), []).append(c)
    out: Dict[str, Dict[str, dict]] = {}
    for strat, cs in sorted(groups.items()):
        out[strat] = {"_runs": {"n": len(cs), "completed": sum(1 for c in cs if c.get("completed", True)),
                                "incomplete": [os.path.basename(str(c["run"].get("dir", "?")).rstrip("/"))
                                               for c in cs if not c.get("completed", True)]}}
        for key, (_, bad) in COMPARE_METRICS.items():
            use = [c for c in cs if c.get("completed", True) or key not in MISSION_METRICS]
            vals = [c["flat"][key] for c in use if c["flat"].get(key) is not None]
            if not vals:
                out[strat][key] = {"n": 0, "mean": None, "worst": None, "spread": None, "values": []}
                continue
            out[strat][key] = {"n": len(vals), "mean": float(np.mean(vals)),
                               "worst": max(vals) if bad == "max" else min(vals),
                               "spread": max(vals) - min(vals), "values": vals}
    return out


# ----------------------------------------------------------------------------- markdown

def _f(v, nd=3) -> str:
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, int):
        return str(v)
    return "n/a" if not math.isfinite(v) else ("%.*f" % (nd, v))


def _pf(v) -> str:
    return "PASS" if v else "FAIL"


def card_markdown(card: dict) -> str:
    r, e, s, sf, g, c = (card[k] for k in ("run", "efficiency", "spray", "safety", "guards", "cost"))
    sp = e["time_split"]
    L = ["# Mission scorecard: %s" % os.path.basename(r["dir"].rstrip("/")), ""]
    if card.get("errors"):
        L += ["## ERRORS: this scorecard is not valid", ""] + ["- " + m for m in card["errors"]] + [""]
    L += [
         "Mission end state `%s`, %s; overall %s." % (card.get("end_state"), "completed" if card.get("completed")
                                                     else "NOT completed", _pf(card.get("all_pass"))), "",
         "Strategy `%s`, seed %s, commit `%s`, %d truth trees, flow %s L/s, drone radius %s m." % (
             r["strategy"], r["seed"], r["git_commit"], r["n_truth_trees"], _f(r["flow_lps"]), _f(r["drone_radius"])), "",
         "## Efficiency", "", "| Metric | Value |", "|---|---|",
         "| Mission time (s) | %s |" % _f(e["mission_time_s"], 1),
         "| Path length (m) | %s |" % _f(e["path_length_m"], 1),
         "| Seconds per sprayed tree | %s |" % _f(e["seconds_per_sprayed_tree"], 1),
         "| Glide climbs inserted | %s |" % _f(e["glide_climbs_inserted"]),
         "| Litres emitted | %s (%d dose episodes, %d nozzle openings) |" % (
             _f(e["litres_emitted"], 4), e.get("dose_episodes", e["dose_events"]), e["dose_events"]), "",
         "Time split (s): " + ", ".join("%s %s" % (k, _f(v, 1)) for k, v in sp["groups"].items()) + ".",
         "By mission phase (s): " + (", ".join("%s %s" % (k, _f(v, 1)) for k, v in sp["by_phase"].items()) or "n/a") + ".",
         "", "## Spray", "", "| Metric | Value |", "|---|---|",
         "| Truth trees sprayed (>= 50 %% of an episode's intended dose) | %d of %d |" % (s["n_sprayed"], r["n_truth_trees"]),
         "| Missed | %d %s |" % (s["n_missed"], s["missed_ids"]),
         "| Double-sprayed (>= 20 %% in two episodes) | %d %s |" % (s["n_double_sprayed"], s["double_sprayed_ids"]),
         "| On-target fraction | %s |" % _f(s["on_target_fraction"]),
         "| Off-target litres (on the ground: %s) | %s |" % (_f(s["ground_l"], 4), _f(s["off_target_l"], 4)),
         "| Episodes with no matched truth target | %d of %d |" % (s["unmatched_events"], s["n_events"]),
         "| Skipped trees (navigator) | %s |" % _f(s.get("nav_trees_skipped")),
         "| Trees remaining (navigator) | %s |" % _f(s.get("nav_trees_remaining")), "",
         "## Safety (must pass)", "", "| Check | Value | Result |", "|---|---|---|",
         "| Mission completed (DONE, no abort or hold timeout, never ERROR) | %s, aborts %s, hold timeouts %s%s | %s |" % (
             card.get("end_state"), card.get("aborts", 0), card.get("hold_timeouts", 0),
             ", ERROR phase" if card.get("error_phase") else "", _pf(card.get("completed"))),
         "| Minimum clearance over truth canopy (m) at t = %s | %s | %s |" % (
             _f(sf["clearance_min_t"], 2), _f(sf["clearance_min_m"]), _pf(sf["clearance_pass"])),
         "| Nozzle open outside DOSE (samples) | %d | %s |" % (sf["nozzle_outside_dose"], _pf(sf["nozzle_pass"])),
         "| Sprayed tree targeted again (episodes) | %d | %s |" % (sf["sprayed_tree_targeted_again"], _pf(sf["retarget_pass"])),
         "", "## Guards", "",
         "Clearance-guard triggers %s; guard-blind counter %s; re-resolved IDs %s; dropped IDs %s; late trees %s." % (
             g["guard_triggers"], g["guard_blind"], g["ids_reresolved"], g["ids_dropped_vanished"], g["late_trees"]),
         "Holds by reason: %s." % (", ".join("%s %d" % kv for kv in sorted(g["holds"].items())) or "none"), "",
         "## Cost", "", "| Stream | n | mean (ms) | p99 (ms) |", "|---|---|---|---|"]
    for k, v in sorted(c.items()):
        if isinstance(v, dict) and "mean_ms" in v:
            L.append("| %s | %d | %s | %s |" % (k, v["n"], _f(v["mean_ms"], 2), _f(v["p99_ms"], 2)))
    L += ["", "Real-time factor %s (sim %s s / wall %s s)." % (_f(c.get("rtf")), _f(c.get("sim_s"), 1), _f(c.get("wall_s"), 1)), ""]
    if card.get("map") is not None:
        from treemapper_tools import score as scorer
        md = scorer.to_markdown(card["map"], "Map (treemapper scorer)")
        L += ["## " + md.split("\n", 1)[0][2:], ""] + [("#" + ln if ln.startswith("#") else ln)
                                                      for ln in md.split("\n", 1)[1].split("\n")]
    else:
        L += ["## Map", "", card.get("map_note", "not scored"), ""]
    return "\n".join(L)


def compare_markdown(cmp: Dict[str, Dict[str, dict]], cards: Sequence[dict]) -> str:
    L = ["# Mission comparison", "", "Runs: " + ", ".join(
        "`%s` (%s)" % (os.path.basename(c["run"]["dir"].rstrip("/")), c["run"].get("strategy")) for c in cards), "",
         "Per strategy: mean over runs, worst case (max for times and failures, min for clearance, sprayed, "
         "on-target and RTF) and spread (max - min).", ""]
    for strat, metrics in cmp.items():
        ru = metrics["_runs"]
        L += ["## %s (%d runs)" % (strat, ru["n"]), "",
              "Completed (DONE, no abort or hold timeout, never ERROR): %d of %d.%s" % (ru["completed"], ru["n"], (
                  " Incomplete, excluded from mission time, path length and seconds per tree: %s." %
                  ", ".join("`%s`" % x for x in ru["incomplete"])) if ru["incomplete"] else ""), "",
              "| Metric | Mean | Worst | Spread |", "|---|---|---|---|"]
        for key, (label, _) in COMPARE_METRICS.items():
            m = metrics[key]
            L.append("| %s | %s | %s | %s |" % (label, _f(m["mean"]), _f(m["worst"]), _f(m["spread"])))
        L.append("")
    return "\n".join(L)


# ----------------------------------------------------------------------------- CLI

def main(argv=None):
    ap = argparse.ArgumentParser(description="Closed-loop mission scorer (Isaac corridor-spray loop).")
    ap.add_argument("--run", required=True, help="run directory (run.json, truth.json, deposition.json, mission.bag)")
    ap.add_argument("--compare", nargs="*", default=[], help="further run directories; grouped by strategy")
    ap.add_argument("--out", help="write the markdown here instead of stdout")
    a = ap.parse_args(argv)
    dirs = [a.run] + list(a.compare)
    cards = [score_run(d) for d in dirs]
    parts = []
    if len(cards) > 1:
        parts.append(compare_markdown(compare_runs(cards), cards))
    parts += [card_markdown(c) for c in cards]
    md = "\n\n".join(parts)
    if a.out:
        with open(a.out, "w") as f:
            f.write(md)
    else:
        print(md)
    bad = [c for c in cards if not c["all_pass"] or c["errors"]]
    for c in bad:
        print("mission_score: FAIL %s: %s" % (c["run"]["dir"], "; ".join(c["fail_reasons"]) or "all_pass is false"),
              file=sys.stderr)
    return 2 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
