"""Scorer: judges ANY system that emits the output contract.

Inputs are the contract files of ``contract.py`` (map stream, optional ground truth,
optional mission context and camera intrinsics). Nothing here knows how the map
was produced, so the existing system, our baselines and future candidates are
scored identically. docs/SCORER.md holds the rationale; every threshold lives in
``ScoreConfig``.

Modes. With ground truth: everything below. Without ground truth (``gt=None``,
``--gt`` omitted, e.g. the real flight): stability, safety and cost only; every
truth-dependent number is ``None``.

Input hygiene (fail loudly):
  * every snapshot's ``frame`` must equal the ground truth's frame (without truth:
    one frame for the whole stream), else ``ValueError``;
  * snapshots and mission series are sorted by time;
  * a ground-truth tree with a degenerate outline is replaced by a circle of its
    radius (or of its mean vertex distance) and listed in ``gt_repaired_ids``; with
    neither, ``ValueError``.
  * an entry is *invalid* if its centre or an outline coordinate is non-finite, it
    has < 3 vertices, or its outline has zero area. Invalid entries count as
    published (precision) but never match a truth tree. (A non-finite height is
    rule 3's own counter, ``undefined_height_frames``.)
  * duplicate IDs in one snapshot: per-ID checks compare the whole multiset of
    entries carrying that ID; drift uses the largest entry; retreat the union.

Detection, per map (one snapshot):
  * one-to-one matching: Hungarian on IoU after zeroing every IoU < match_min_iou
    (so sub-threshold pairs cannot steer the assignment). recall = matched / n_truth,
    precision = matched / n_published. ``recall_in_view`` = matched truth trees among
    those already in the camera footprint (needs pose + camera) / their number.
  * cov(P,G) = |P∩G|/|G|; P "covers" G if cov >= cover_min.
  * owner(P) = truth tree holding the largest share of P, if that share >= own_min.
  * missed = truth trees with no match (``absorbed`` = covered by some entry, else ``unseen``).
  * merged_entries = entries covering >= 2 truth trees; wrong_merges = neighbours swallowed.
  * extra entries owned by an already-owned tree: duplicates (IoU >= dup_min_iou) or fragments.
  * false_trees = valid entries with no owner.
Geometry (matched pairs): IoU, centre error, height error (published - truth). An
  undefined height on a matched entry counts in ``n_undefined_height`` and enters
  height RMSE / max-abs as ``undefined_height_error_m`` (worse than any finite answer
  in the flight envelope), never as "no error". Bias uses finite heights only.
  spill = published area outside every true canopy / published area;
  neighbour_spill = matched entry's area inside OTHER true canopies / its area.
  ``gt_height_reliable`` is False when every truth height is identical (n >= 2) or
  the truth file says ``"height_reliable": false``: height errors are then still
  reported but flagged and kept out of the headline.
Per-frame means: the latest published snapshot is sampled on a fixed grid of
  ``sample_dt_s`` over the mission window (first SCAN .. first RETURN after it, or the
  end of the stream; without phases: first .. last snapshot). Before the first
  snapshot the map is empty. Means are therefore time-weighted, not snapshot-weighted.
  Second variant: from the first non-empty snapshot to the window end.
Stability (whole stream, per ID, independent of publish rate):
  * retreat: lost = area(U - current)/area(U), U = union of every outline the ID has
    ever published; > retreat_tol is a retreat (counted once per onset). An outline
    going invalid loses 1.0. ``max_lost_fraction`` overall and per ID.
  * drift: per ID, max centre / height distance from its first published value and
    from its value when it was first seen in a locked phase (``*_since_lock``).
  * jumps: centre (height) differs by > jump_m (height_jump_m) from any value of the
    same ID within the last jump_window_s, or from its last value (height: last
    FINITE value); counted once per onset.
  * reuse: an ID that disappears and reappears with an outline overlapping its
    pre-gap outline by < reuse_overlap_max (intersection / smaller area) is a reused
    ID (geometry only, works without truth). With truth also ``id_reuse_by_owner``.
  * id_switches / id_meaning_changes (truth): a tree's matched ID changes / an ID's
    owner changes; both total and after the list first locks.
Safety (must all be zero; ``None`` = not judgeable from the inputs given):
  * undefined_height_frames, invalid_entry_frames, duplicate_id_frames.
  * implausible_height_frames: a finite height <= 0, or (pose given, inside the
    mission window) above the drone's altitude. With reliable truth heights an entry
    above the drone is flagged only if it is also above the truth top + 
    implausible_truth_tol_m, where the truth top is the highest truth tree holding
    >= own_min of the entry (an entry owned by no truth tree keeps the plain test):
    a descent hover at or below a tall tree's canopy top is not a height error.
  * new_trees_after_scan: IDs first published -- or reused -- in a locked phase.
  * locked_id_vanished: an ID present in the previous snapshot is gone while the
    phase is locked (``*_targets`` = it was the hop target).
  * nontarget_changes_in_descent: in a descent-window snapshot any tree other than
    the current target changes outline / centre / height, vanishes or appears.
    Window (``descent_kinds``): SPRAYING snapshots; HOPPING snapshots with a target
    whose drone xy is within descent_radius_m of where the next SPRAYING starts
    (arrival + descent); and after each SPRAYING, whatever the phase, every
    snapshot until the drone first leaves descent_radius_m of that hover point (the
    climb out). No target series at all -> None. Without pose: every HOPPING
    snapshot with a target is judged, the climb out is not.
  * sprayed_tree_changes: each ID's entries are recorded when it is first sprayed;
    afterwards it vanishing, losing the flag, or differing from the record
    (outline, centre, height) is a violation, once per onset of each kind.
  * new_tree_inside_sprayed: a new or reused ID whose centre lies inside -- or with
    >= inside_sprayed_min of its area inside -- the union of every outline ever
    published as sprayed.
  Informational: target height range during its own approach/spraying window.
Spray outcome (truth, whole stream): an ID's spray point = its centre when the
  flag first appeared; a truth tree is sprayed by every distinct ID whose spray
  point lies inside its canopy; >= 2 IDs = sprayed twice; no tree = orphan.
Timeliness: first publish of a truth tree = first snapshot where it is one-to-one
  matched to a valid entry with a finite height, minus the first time its centre
  entered the yaw-free camera footprint. Negative latencies are listed and clamped
  to 0 in the statistics.
Cost: per-stage processing time if the stream carries ``stage_ms``.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
from collections import deque
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment
from shapely.geometry import Point as SPoint
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid

from .contract import (GroundTruthTree, Mission, Snapshot, Tree, read_ground_truth, read_mission,
                       read_snapshots, step_lookup)

MAX_EVENTS = 50


@dataclass
class ScoreConfig:
    match_min_iou: float = 0.10      # one-to-one match needs at least this IoU
    cover_min: float = 0.30          # entry covers a truth tree if it holds >= 30% of it
    own_min: float = 0.20            # entry is owned by a tree holding >= 20% of the entry
    dup_min_iou: float = 0.30        # an extra entry this similar to its tree is a duplicate
    retreat_tol: float = 0.02        # > 2% of the ID's historic outline union lost = retreat
    jump_m: float = 1.0              # centre change > this within jump_window_s = jump
    height_jump_m: float = 1.0       # height change > this within jump_window_s = jump
    jump_window_s: float = 1.0
    change_eps_area: float = 1e-3    # m^2 symmetric difference that counts as a change
    change_eps_h: float = 1e-3       # m height change that counts as a change
    change_eps_m: float = 1e-3       # m centre move that counts as a change
    reuse_overlap_max: float = 0.10  # reappearing ID overlapping its pre-gap outline less = reused
    inside_sprayed_min: float = 0.30  # new entry with >= this share inside sprayed outlines
    undefined_height_error_m: float = 15.0  # height error charged for an undefined height (~scan altitude)
    implausible_truth_tol_m: float = 0.5  # above-the-drone heights within this of the truth top are not flagged
    sample_dt_s: float = 0.2         # per-frame sampling grid (5 Hz)
    locked_phases: Tuple[str, ...] = ("PLANNING", "HOPPING", "SPRAYING")
    approach_phases: Tuple[str, ...] = ("HOPPING",)
    spray_phases: Tuple[str, ...] = ("SPRAYING",)
    descent_radius_m: float = 1.5    # xy distance from the spray hover point that counts as "over it"
    scan_phases: Tuple[str, ...] = ("SCAN",)
    window_end_phases: Tuple[str, ...] = ("RETURN",)


# ----------------------------------------------------------------------------- geometry

def _finite(v) -> bool:
    return v is not None and isinstance(v, (int, float)) and math.isfinite(v)


def _finite_pt(p) -> bool:
    return p is not None and len(p) == 2 and all(_finite(c) for c in p)


def to_poly(outline: Sequence[Tuple[float, float]]):
    if len(outline) < 3 or not all(_finite_pt(p) for p in outline):
        return None
    p = Polygon(outline)
    if not p.is_valid:
        p = make_valid(p)
        parts = [g for g in getattr(p, "geoms", [p]) if g.area > 0]
        p = unary_union(parts) if parts else None
    if p is None or p.is_empty or p.area <= 0:
        return None
    return p


def _area(g) -> float:
    return 0.0 if g is None else float(g.area)


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def entry_problems(tr: Tree) -> List[str]:
    out = []
    if not _finite_pt(tr.centre):
        out.append("nonfinite_centre")
    if not all(_finite_pt(p) for p in tr.outline):
        out.append("nonfinite_outline")
    elif len(tr.outline) < 3:
        out.append("too_few_vertices")
    elif to_poly(tr.outline) is None:
        out.append("zero_area")
    return out


def gt_polygon(g: GroundTruthTree):
    """Truth polygon; a degenerate outline falls back to a circle (returns (poly, repaired))."""
    p = to_poly(g.outline)
    if p is not None:
        return p, False
    r = g.radius if _finite(g.radius) and g.radius > 0 else 0.0
    if r <= 0 and _finite_pt(g.centre):
        ds = [_dist(v, g.centre) for v in g.outline if _finite_pt(v)]
        r = float(np.mean(ds)) if ds else 0.0
    if r > 0 and _finite_pt(g.centre):
        return SPoint(g.centre).buffer(r, 16), True
    raise ValueError("ground-truth tree %d has no usable outline (degenerate outline and no radius)" % g.id)


def _overlap_frac(a, b) -> float:
    m = min(a.area, b.area)
    return a.intersection(b).area / m if m > 0 else 0.0


def match_one_to_one(iou: np.ndarray, min_iou: float) -> Dict[int, int]:
    """Hungarian on IoU with every sub-threshold IoU zeroed first."""
    if iou.size == 0:
        return {}
    z = np.where(iou >= min_iou, iou, 0.0)
    r, c = linear_sum_assignment(-z)
    return {int(i): int(j) for i, j in zip(r, c) if z[i, j] > 0 and z[i, j] >= min_iou}


class SnapView:
    """Truth-independent view of one snapshot: validity, polygons and entries grouped by ID."""

    def __init__(self, snap: Snapshot):
        self.snap, self.t, self.trees = snap, snap.t, snap.trees
        self.problems = [entry_problems(tr) for tr in self.trees]
        self.polys = [to_poly(tr.outline) for tr in self.trees]
        self.valid = [not p for p in self.problems]
        self.by_id: Dict[int, List[int]] = {}
        for i, tr in enumerate(self.trees):
            self.by_id.setdefault(tr.id, []).append(i)
        self.dup_ids = sorted(k for k, v in self.by_id.items() if len(v) > 1)

    def entries(self, tid) -> List[Tuple[Tree, object]]:
        return [(self.trees[i], self.polys[i]) for i in self.by_id.get(tid, [])]

    def id_poly(self, tid):
        ps = [self.polys[i] for i in self.by_id.get(tid, []) if self.polys[i] is not None]
        if not ps:
            return None
        return ps[0] if len(ps) == 1 else unary_union(ps)

    def rep(self, tid) -> Tree:
        idx = self.by_id[tid]
        return self.trees[max(idx, key=lambda i: _area(self.polys[i]))]


def _tree_changed(a: Tree, b: Tree, pa, pb, cfg: ScoreConfig) -> bool:
    ha, hb = _finite(a.height), _finite(b.height)
    if ha != hb or (ha and abs(a.height - b.height) > cfg.change_eps_h):
        return True
    ca, cb = _finite_pt(a.centre), _finite_pt(b.centre)
    if ca != cb or (ca and _dist(a.centre, b.centre) > cfg.change_eps_m):
        return True
    if (pa is None) != (pb is None):
        return True
    if pa is None:
        return repr(a.outline) != repr(b.outline)
    return pa.symmetric_difference(pb).area > cfg.change_eps_area


def _entries_differ(a: List[Tuple[Tree, object]], b: List[Tuple[Tree, object]], cfg: ScoreConfig) -> bool:
    if len(a) != len(b):
        return True
    key = lambda e: tuple(round(c, 3) if _finite(c) else 1e18 for c in e[0].centre)
    for (ta, pa), (tb, pb) in zip(sorted(a, key=key), sorted(b, key=key)):
        if _tree_changed(ta, tb, pa, pb, cfg):
            return True
    return False


class Frame:
    """Pairwise geometry between one published snapshot and the ground truth."""

    def __init__(self, view: SnapView, gt: List[GroundTruthTree], gt_polys, cfg: ScoreConfig):
        self.view, self.snap, self.gt, self.cfg = view, view.snap, gt, cfg
        self.trees = view.trees
        self.polys = [p if ok else None for p, ok in zip(view.polys, view.valid)]
        n, m = len(self.trees), len(gt)
        self.inter = np.zeros((n, m))
        for i, p in enumerate(self.polys):
            if p is None:
                continue
            for j, g in enumerate(gt_polys):
                if p.intersects(g):
                    self.inter[i, j] = p.intersection(g).area
        self.parea = np.array([_area(p) for p in self.polys]) if n else np.zeros(0)
        self.garea = np.array([g.area for g in gt_polys])
        union = self.parea[:, None] + self.garea[None, :] - self.inter if n else np.zeros((0, m))
        self.iou = np.divide(self.inter, union, out=np.zeros_like(self.inter), where=union > 0)
        self.cov = self.inter / self.garea[None, :] if n else np.zeros((0, m))
        self.share = np.divide(self.inter, self.parea[:, None], out=np.zeros_like(self.inter),
                               where=self.parea[:, None] > 0) if n else np.zeros((0, m))
        self.match_p2g = match_one_to_one(self.iou, cfg.match_min_iou) if n and m else {}
        self.match_g2p = {j: i for i, j in self.match_p2g.items()}
        self.owner: List[Optional[int]] = []
        for i in range(n):
            j = int(np.argmax(self.share[i])) if m else -1
            self.owner.append(j if m and self.share[i, j] >= cfg.own_min else None)

    def owner_gt_id(self, i: int) -> Optional[int]:
        j = self.owner[i]
        return None if j is None else self.gt[j].id


def detection_metrics(fr: Frame, seen: Optional[set] = None) -> dict:
    """``seen``: truth indices already in the camera footprint (None = unknown)."""
    cfg = fr.cfg
    n, m = len(fr.trees), len(fr.gt)
    matched = len(fr.match_p2g)
    covers = fr.cov >= cfg.cover_min if n else np.zeros((0, m), bool)
    missed = [j for j in range(m) if j not in fr.match_g2p]
    absorbed = [j for j in missed if n and covers[:, j].any()]
    merged_entries = [i for i in range(n) if covers[i].sum() >= 2]
    wrong_merges = int(sum(covers[i].sum() - 1 for i in merged_entries))
    owned: Dict[int, List[int]] = {}
    for i, j in enumerate(fr.owner):
        if j is not None:
            owned.setdefault(j, []).append(i)
    duplicates = fragments = 0
    split_trees = []
    for j, ents in owned.items():
        if len(ents) < 2:
            continue
        split_trees.append(j)
        keep = fr.match_g2p.get(j, max(ents, key=lambda i: fr.iou[i, j]))
        for i in ents:
            if i == keep:
                continue
            if fr.iou[i, j] >= cfg.dup_min_iou:
                duplicates += 1
            else:
                fragments += 1
    invalid = [i for i in range(n) if not fr.view.valid[i]]
    false_trees = [i for i in range(n) if fr.view.valid[i] and fr.owner[i] is None]
    out = {
        "n_truth": m, "n_published": n, "matched": matched,
        "recall": matched / m if m else float("nan"),
        "precision": matched / n if n else float("nan"),
        "missed": len(missed), "missed_absorbed": len(absorbed), "missed_unseen": len(missed) - len(absorbed),
        "merged_entries": len(merged_entries), "wrong_merges": wrong_merges,
        "trees_in_merged_entries": int(sum(covers[i].sum() for i in merged_entries)),
        "split_trees": len(split_trees), "duplicates": duplicates, "fragments": fragments,
        "false_trees": len(false_trees), "invalid_entries": len(invalid),
        "missed_ids": [fr.gt[j].id for j in missed],
        "split_tree_ids": [fr.gt[j].id for j in split_trees],
        "merged_tree_ids": sorted({fr.gt[j].id for i in merged_entries for j in np.nonzero(covers[i])[0]}),
        "merged_entry_ids": {str(fr.trees[i].id): [fr.gt[j].id for j in np.nonzero(covers[i])[0]]
                             for i in merged_entries},
        "false_tree_ids": [fr.trees[i].id for i in false_trees],
    }
    if seen is not None:
        out["n_in_view"] = len(seen)
        out["recall_in_view"] = (sum(1 for j in seen if j in fr.match_g2p) / len(seen)) if seen else float("nan")
    return out


def geometry_metrics(fr: Frame, gt_union, cfg: ScoreConfig) -> dict:
    ious, cerr, herr, herr_fin, nspill = [], [], [], [], []
    per_tree = []
    n_undef = 0
    for i, j in sorted(fr.match_p2g.items(), key=lambda kv: fr.gt[kv[1]].id):
        t, g = fr.trees[i], fr.gt[j]
        ious.append(fr.iou[i, j])
        ce = _dist(t.centre, g.centre)
        cerr.append(ce)
        if _finite(t.height):
            he = t.height - g.height
            herr.append(abs(he))
            herr_fin.append(he)
        else:
            he = None
            n_undef += 1
            herr.append(cfg.undefined_height_error_m)
        other = fr.inter[i].sum() - fr.inter[i, j]
        ns = other / fr.parea[i] if fr.parea[i] > 0 else 0.0
        nspill.append(ns)
        per_tree.append({"truth_id": g.id, "map_id": t.id, "iou": round(float(fr.iou[i, j]), 3),
                         "centre_err_m": round(ce, 3), "height_err_m": None if he is None else round(he, 3),
                         "neighbour_spill": round(float(ns), 3)})
    tot = float(fr.parea.sum()) if len(fr.parea) else 0.0
    outside = 0.0
    for p in fr.polys:
        if p is not None:
            outside += p.difference(gt_union).area
    arr = lambda v: np.asarray(v, float)
    return {
        "mean_iou": float(np.mean(ious)) if ious else float("nan"),
        "min_iou": float(np.min(ious)) if ious else float("nan"),
        "centre_mean_m": float(np.mean(cerr)) if cerr else float("nan"),
        "centre_rmse_m": float(np.sqrt(np.mean(arr(cerr) ** 2))) if cerr else float("nan"),
        "centre_max_m": float(np.max(cerr)) if cerr else float("nan"),
        "n_undefined_height": n_undef,
        "undefined_height_entries": sum(1 for t in fr.trees if not _finite(t.height)),
        "height_bias_m": float(np.mean(herr_fin)) if herr_fin else float("nan"),
        "height_rmse_m": float(np.sqrt(np.mean(arr(herr) ** 2))) if herr else float("nan"),
        "height_max_abs_m": float(np.max(herr)) if herr else float("nan"),
        "spill": outside / tot if tot > 0 else float("nan"),
        "neighbour_spill_mean": float(np.mean(nspill)) if nspill else float("nan"),
        "neighbour_spill_max": float(np.max(nspill)) if nspill else float("nan"),
        "per_tree": per_tree,
    }


# ----------------------------------------------------------------------------- mission helpers

class PoseLookup:
    def __init__(self, pose):
        self.pose = pose
        self.t = [p[0] for p in pose]

    def at(self, t: float):
        """Last pose sample at or before t (the first one before the pose starts)."""
        if not self.pose:
            return None
        return self.pose[max(bisect.bisect_right(self.t, t) - 1, 0)]


def cruise_altitude(mission: Mission, cfg: ScoreConfig) -> Optional[float]:
    if not mission.pose or not mission.phases:
        return None
    zs = [p[3] for p in mission.pose if step_lookup(mission.phases, p[0]) in cfg.scan_phases]
    return float(np.median(zs)) if zs else None


def mission_window(snaps: List[Snapshot], mission: Mission, cfg: ScoreConfig) -> Tuple[Optional[float], Optional[float], str]:
    """(t0, t1, description): first SCAN .. first RETURN after it (or the stream end)."""
    if not snaps:
        return None, None, "empty stream"
    t0 = next((t for t, ph in mission.phases if ph in cfg.scan_phases), None)
    d0 = "first %s" % "/".join(cfg.scan_phases)
    if t0 is None:
        t0, d0 = snaps[0].t, "first snapshot"
    t1 = next((t for t, ph in mission.phases if ph in cfg.window_end_phases and t > t0), None)
    d1 = "first %s" % "/".join(cfg.window_end_phases)
    if t1 is None:
        t1, d1 = snaps[-1].t, "last snapshot"
    return t0, t1, "%s (%.2f s) .. %s (%.2f s)" % (d0, t0, d1, t1)


def descent_kinds(snaps: List[Snapshot], mission: Mission, cfg: ScoreConfig) -> Optional[List[str]]:
    """Per snapshot: '' (not judged), 'approach', 'spraying' or 'climb'. None = no target series.

    approach: an approach-phase (HOPPING) snapshot with a target >= 0 whose drone xy is
              within descent_radius_m of the point where the following SPRAYING starts
              (arrival, descent, pre-spray hover). Without pose: every such snapshot.
    spraying: every SPRAYING snapshot.
    climb:    after a SPRAYING ends, whatever the phase (HOPPING, SCAN_APPROACH, RETURN),
              every snapshot until the drone first leaves descent_radius_m of that hover
              point. The drone is still low over the sprayed tree. Needs pose.
    """
    if not mission.phases or not mission.targets:
        return None
    phases = mission.phases
    ptimes = [t for t, _ in phases]
    pl = PoseLookup(mission.pose)
    hover: Dict[int, Tuple[float, float]] = {}
    climbs = []
    for k, (t, ph) in enumerate(phases):
        if ph not in cfg.spray_phases or not mission.pose:
            continue
        q = min(bisect.bisect_left(pl.t, t), len(pl.t) - 1)
        hover[k] = (mission.pose[q][1], mission.pose[q][2])
        if k + 1 < len(phases):
            te = phases[k + 1][0]
            ce = math.inf
            for p in mission.pose[bisect.bisect_left(pl.t, te):]:
                if math.hypot(p[1] - hover[k][0], p[2] - hover[k][1]) > cfg.descent_radius_m:
                    ce = p[0]
                    break
            climbs.append((te, ce))
    out = []
    for s in snaps:
        kp = bisect.bisect_right(ptimes, s.t) - 1
        ph = phases[kp][1] if kp >= 0 else None
        tgt = step_lookup(mission.targets, s.t, -1)
        kind = ""
        if ph in cfg.spray_phases:
            kind = "spraying"
        elif ph in cfg.approach_phases and tgt is not None and tgt >= 0:
            if not mission.pose:
                kind = "approach"
            elif kp + 1 < len(phases) and (kp + 1) in hover:
                p = pl.at(s.t)
                if math.hypot(p[1] - hover[kp + 1][0], p[2] - hover[kp + 1][1]) <= cfg.descent_radius_m:
                    kind = "approach"
        if not kind:
            for te, ce in climbs:
                if te <= s.t < ce:
                    kind = "climb"
                    break
        out.append(kind)
    return out


def descent_mask(snaps: List[Snapshot], mission: Mission, cfg: ScoreConfig) -> Optional[List[bool]]:
    k = descent_kinds(snaps, mission, cfg)
    return None if k is None else [bool(x) for x in k]


def id_births(views: List[SnapView], cfg: ScoreConfig) -> Dict[Tuple[int, int], str]:
    """(snapshot index, id) -> 'first' (first publication), 'reuse' (reappears after a gap with
    an outline overlapping its pre-gap outline < reuse_overlap_max) or 'return' (same place)."""
    out: Dict[Tuple[int, int], str] = {}
    last_poly: Dict[int, object] = {}
    seen: set = set()
    gone: set = set()
    prev: set = set()
    for k, v in enumerate(views):
        present = set(v.by_id)
        for tid in v.by_id:
            cur = v.id_poly(tid)
            if tid not in seen:
                out[(k, tid)] = "first"
            elif tid in gone:
                ref = last_poly.get(tid)
                reuse = ref is not None and cur is not None and _overlap_frac(ref, cur) < cfg.reuse_overlap_max
                out[(k, tid)] = "reuse" if reuse else "return"
            gone.discard(tid)
            seen.add(tid)
            if cur is not None:
                last_poly[tid] = cur
        gone |= prev - present
        prev = present
    return out


# ----------------------------------------------------------------------------- main scorer

def check_frames(snaps: List[Snapshot], frame: Optional[str]) -> str:
    ref = frame if frame is not None else (snaps[0].frame if snaps else "world")
    bad = [s for s in snaps if s.frame != ref]
    if bad:
        raise ValueError("map stream frame %r differs from the %s frame %r at t=%.3f (%d snapshots)"
                         % (bad[0].frame, "ground-truth" if frame is not None else "stream's first",
                            ref, bad[0].t, len(bad)))
    return ref


def gt_height_reliability(gt) -> Tuple[bool, str]:
    flag = getattr(gt, "height_reliable", None)
    if flag is False:
        return False, "ground-truth file marks heights unreliable"
    hs = {round(g.height, 6) for g in gt}
    if len(gt) >= 2 and len(hs) == 1:
        return False, "GT height unreliable (constant fallback): every truth height is %.3f m" % next(iter(hs))
    return True, "per-tree truth heights"


def score(snaps: List[Snapshot], gt: Optional[List[GroundTruthTree]] = None, mission: Optional[Mission] = None,
          camera: Optional[dict] = None, cfg: Optional[ScoreConfig] = None, gt_frame: Optional[str] = None) -> dict:
    cfg = cfg or ScoreConfig()
    mission = mission or Mission()
    mission = Mission(sorted(mission.phases, key=lambda r: r[0]), sorted(mission.targets, key=lambda r: r[0]),
                      sorted(mission.pose, key=lambda r: r[0]))
    snaps = sorted(snaps, key=lambda s: s.t)
    has_gt = gt is not None
    frame = check_frames(snaps, (gt_frame or getattr(gt, "frame", None) or "world") if has_gt else gt_frame)
    views = [SnapView(s) for s in snaps]
    t0, t1, wdesc = mission_window(snaps, mission, cfg)
    res: dict = {"config": asdict(cfg), "mode": "ground truth" if has_gt else "no ground truth",
                 "frame": frame, "n_snapshots": len(snaps),
                 "t_start": snaps[0].t if snaps else None, "t_end": snaps[-1].t if snaps else None,
                 "mission_window": {"t0": t0, "t1": t1, "definition": wdesc}}
    births = id_births(views, cfg)
    frames = None
    if has_gt:
        rel, why = gt_height_reliability(gt)
        gt = list(gt)
        gp = [gt_polygon(g) for g in gt]
        gt_polys = [p for p, _ in gp]
        res["gt_repaired_ids"] = [g.id for g, (_, r) in zip(gt, gp) if r]
        res["gt_height_reliable"], res["gt_height_note"] = rel, why
        gt_union = unary_union(gt_polys) if gt_polys else Polygon()
        frames = [Frame(v, gt, gt_polys, cfg) for v in views]
        view_t, view_scan_t = first_view_times(gt, mission, camera, cfg)
        have_view = bool(view_t) or (footprint_radius_per_m(camera) is not None and bool(mission.pose))
        seen_at = (lambda t: {j for j, tv in view_t.items() if tv <= t}) if have_view else (lambda t: None)
        if frames:
            fin = frames[-1]
            res["final"] = {"t": fin.snap.t, "detection": detection_metrics(fin, seen_at(fin.snap.t)),
                            "geometry": geometry_metrics(fin, gt_union, cfg)}
        res["per_frame"] = per_frame(frames, gt_union, cfg, t0, t1, seen_at, "mission window: " + wdesc)
        fp = next((f.snap.t for f in frames if f.trees), None)
        res["per_frame_published"] = per_frame(frames, gt_union, cfg, max(t0, fp) if fp is not None else None, t1,
                                               seen_at, "first non-empty snapshot .. mission window end")
        res["spray"] = spray_outcome(views, gt, gt_polys)
        if "final" in res:
            res["final"]["spray"] = res["spray"]
        res["timeliness"] = timeliness_metrics(frames, gt, view_t, view_scan_t, camera, mission)
    else:
        res["final"] = {"t": snaps[-1].t if snaps else None, "n_published": len(snaps[-1].trees) if snaps else 0}
        res["note"] = "no ground truth: detection, geometry, spray outcome and timeliness not scored"
    res["stability"] = stability_metrics(views, frames, births, mission, cfg)
    if frames is not None:
        ever_split, ever_merged = set(), set()
        for f in frames:
            d = detection_metrics(f)
            ever_split |= set(d["split_tree_ids"])
            ever_merged |= set(d["merged_tree_ids"])
        res["stability"]["truth_trees_ever_split"] = sorted(ever_split)
        res["stability"]["truth_trees_ever_in_a_merged_entry"] = sorted(ever_merged)
    truth_tops = None
    if frames is not None and res.get("gt_height_reliable"):
        truth_tops = [[max((f.gt[j].height for j in range(len(f.gt)) if f.share[i, j] >= cfg.own_min), default=None)
                       for i in range(len(f.trees))] for f in frames]
    res["safety"] = safety_metrics(views, births, mission, cfg, t0, t1, truth_tops)
    res["cost"] = cost_metrics(snaps)
    res["ids_ever_published"] = sorted({t.id for s in snaps for t in s.trees})
    return res


def per_frame(frames: List[Frame], gt_union, cfg: ScoreConfig, t0: Optional[float], t1: Optional[float],
              seen_at, window: str) -> dict:
    """Time-weighted means: the latest snapshot sampled every sample_dt_s over [t0, t1)."""
    pf: dict = {"window": window, "sample_dt_s": cfg.sample_dt_s, "n_samples": 0}
    if not frames or t0 is None or t1 is None:
        return pf
    grid = np.arange(t0, t1, cfg.sample_dt_s) if t1 > t0 else np.array([t0])
    times = [f.snap.t for f in frames]
    idx = [bisect.bisect_right(times, float(t)) - 1 for t in grid]
    cache: Dict[int, Tuple[dict, dict, set]] = {}
    det, geo = [], []
    for t, k in zip(grid, idx):
        if k not in cache:
            fr = frames[k] if k >= 0 else None
            if fr is None:
                m = len(frames[0].gt)
                d = {"n_truth": m, "n_published": 0, "matched": 0, "recall": 0.0 if m else float("nan"),
                     "missed": m, "false_trees": 0, "wrong_merges": 0, "split_trees": 0}
                cache[k] = (d, {}, set())
            else:
                cache[k] = (detection_metrics(fr), geometry_metrics(fr, gt_union, cfg), set(fr.match_g2p))
        d, g, matched = cache[k]
        d = dict(d)
        seen = seen_at(float(t))
        if seen is not None:
            d["recall_in_view"] = (sum(1 for j in seen if j in matched) / len(seen)) if seen else float("nan")
        det.append(d)
        geo.append(g)
    pf["n_samples"] = len(grid)
    pf["n_distinct_snapshots"] = len({k for k in idx if k >= 0})
    for key, rows in (("detection", det), ("geometry", geo)):
        agg = {}
        keys = []
        for r in rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
        for k in keys:
            vals = [r[k] for r in rows if k in r and isinstance(r[k], (int, float)) and not isinstance(r[k], bool)
                    and not (isinstance(r[k], float) and math.isnan(r[k]))]
            if vals:
                agg[k] = {"mean": float(np.mean(vals)), "min": float(np.min(vals)), "max": float(np.max(vals))}
        pf[key] = agg
    return pf


def spray_outcome(views: List[SnapView], gt: List[GroundTruthTree], gt_polys) -> dict:
    """Over the whole stream: each ID's spray point = its centre when the flag first appeared."""
    spray_pt: Dict[int, Tuple[float, Tuple[float, float]]] = {}
    for v in views:
        for tid, idx in v.by_id.items():
            if tid in spray_pt:
                continue
            sp = [i for i in idx if v.trees[i].sprayed]
            if sp:
                spray_pt[tid] = (v.t, v.trees[sp[0]].centre)
    per_gt: Dict[int, List[int]] = {}
    orphans = []
    for tid, (t, c) in sorted(spray_pt.items()):
        hits = [j for j, gp in enumerate(gt_polys) if _finite_pt(c) and gp.contains(SPoint(c))]
        for j in hits:
            per_gt.setdefault(j, []).append(tid)
        if not hits:
            orphans.append(tid)
    return {"definition": ("over the whole stream: a truth tree is sprayed by every distinct ID whose centre, "
                           "when its sprayed flag first appeared, lies inside the canopy"),
            "sprayed_ids": len(spray_pt), "truth_trees_sprayed": len(per_gt),
            "truth_trees_sprayed_twice": sum(1 for v in per_gt.values() if len(v) >= 2),
            "sprayed_by": {str(gt[j].id): v for j, v in sorted(per_gt.items())},
            "spray_times": {str(k): round(v[0], 3) for k, v in sorted(spray_pt.items())},
            "orphan_sprayed_entries": len(orphans), "orphan_sprayed_ids": orphans,
            "truth_trees_not_sprayed": [g.id for j, g in enumerate(gt) if j not in per_gt]}


def stability_metrics(views: List[SnapView], frames: Optional[List[Frame]], births, mission: Mission,
                      cfg: ScoreConfig) -> dict:
    locked_t = lambda t: bool(mission.phases) and step_lookup(mission.phases, t) in cfg.locked_phases
    t_lock = next((t for t, ph in mission.phases if ph in cfg.locked_phases), None)
    after_lock = lambda t: t_lock is not None and t >= t_lock
    W = cfg.jump_window_s
    st: Dict[int, dict] = {}
    retreats, jumps, hjumps, reuse_ev = [], [], [], []
    reappear = 0
    lost_max = 0.0

    def new_state(t):
        return {"first_t": t, "union": None, "first_c": None, "first_h": None, "locked": False,
                "lock_c": None, "lock_h": None, "max_lost": 0.0, "retreating": False,
                "cbuf": deque(), "hbuf": deque(), "last_c": None, "last_h": None, "cjump": False, "hjump": False,
                "dc_first": 0.0, "dc_lock": 0.0, "dh_first": 0.0, "dh_lock": 0.0}

    for k, v in enumerate(views):
        t = v.t
        for tid in v.by_id:
            b = births.get((k, tid))
            if b in ("return", "reuse"):
                reappear += 1
            if b == "reuse":
                reuse_ev.append({"t": round(t, 3), "id": tid, "locked_phase": locked_t(t)})
            if tid not in st or b == "reuse":   # a reused ID is a different tree: fresh references
                st[tid] = new_state(t)
            s = st[tid]
            cur = v.id_poly(tid)
            tr = v.rep(tid)
            c = tr.centre if _finite_pt(tr.centre) else None
            h = tr.height if _finite(tr.height) else None
            U = s["union"]
            if U is not None:
                lost = 1.0 if cur is None else U.difference(cur).area / U.area
                s["max_lost"] = max(s["max_lost"], lost)
                lost_max = max(lost_max, lost)
                r = lost > cfg.retreat_tol
                if r and not s["retreating"]:
                    retreats.append({"t": round(t, 3), "id": tid, "lost_fraction": round(lost, 3)})
                s["retreating"] = r
            if cur is not None:
                s["union"] = cur if U is None else U.union(cur)
            if locked_t(t):
                s["locked"] = True
            if c is not None:
                if s["first_c"] is None:
                    s["first_c"] = c
                if s["locked"] and s["lock_c"] is None:
                    s["lock_c"] = c
                s["dc_first"] = max(s["dc_first"], _dist(c, s["first_c"]))
                if s["lock_c"] is not None:
                    s["dc_lock"] = max(s["dc_lock"], _dist(c, s["lock_c"]))
                buf = s["cbuf"]
                while buf and t - buf[0][0] > W:
                    buf.popleft()
                refs = [p for _, p in buf] + ([s["last_c"]] if s["last_c"] is not None else [])
                dmax = max((_dist(c, p) for p in refs), default=0.0)
                j = dmax > cfg.jump_m
                if j and not s["cjump"]:
                    jumps.append({"t": round(t, 3), "id": tid, "moved_m": round(dmax, 3)})
                s["cjump"] = j
                buf.append((t, c))
                s["last_c"] = c
            if h is not None:
                if s["first_h"] is None:
                    s["first_h"] = h
                if s["locked"] and s["lock_h"] is None:
                    s["lock_h"] = h
                s["dh_first"] = max(s["dh_first"], abs(h - s["first_h"]))
                if s["lock_h"] is not None:
                    s["dh_lock"] = max(s["dh_lock"], abs(h - s["lock_h"]))
                buf = s["hbuf"]
                while buf and t - buf[0][0] > W:
                    buf.popleft()
                refs = [x for _, x in buf] + ([s["last_h"]] if s["last_h"] is not None else [])
                dh = max(refs, key=lambda x: abs(h - x)) if refs else h
                j = abs(h - dh) > cfg.height_jump_m
                if j and not s["hjump"]:
                    hjumps.append({"t": round(t, 3), "id": tid, "dh_m": round(h - dh, 3)})
                s["hjump"] = j
                buf.append((t, h))
                s["last_h"] = h

    # truth-based identity metrics
    id_switches, switch_events, meaning_changes, reuse_owner = 0, [], [], 0
    if frames is not None:
        last_match: Dict[int, int] = {}
        owner_hist: Dict[int, int] = {}
        owner_before: Dict[int, Optional[int]] = {}
        for k, f in enumerate(frames):
            t = f.snap.t
            for j, i in f.match_g2p.items():
                gid, mid = f.gt[j].id, f.trees[i].id
                if gid in last_match and last_match[gid] != mid:
                    id_switches += 1
                    switch_events.append({"t": round(t, 3), "truth_id": gid, "from": last_match[gid], "to": mid,
                                          "after_lock": after_lock(t)})
                last_match[gid] = mid
            for i, tr in enumerate(f.trees):
                own = f.owner_gt_id(i)
                if births.get((k, tr.id)) in ("return", "reuse"):
                    b = owner_before.get(tr.id)
                    if b is not None and own is not None and b != own:
                        reuse_owner += 1
                if own is not None:
                    if tr.id in owner_hist and owner_hist[tr.id] != own:
                        meaning_changes.append({"t": round(t, 3), "id": tr.id, "from": owner_hist[tr.id], "to": own,
                                                "after_lock": after_lock(t)})
                    owner_hist[tr.id] = own
                    owner_before[tr.id] = own
    seen = set(st)
    final_ids = set(views[-1].by_id) if views else set()
    targeted = {i for _, i in mission.targets if i >= 0}
    sprayed_ever = {tr.id for v in views for tr in v.trees if tr.sprayed}
    vanished = sorted(seen - final_ids)
    r3 = lambda x: round(float(x), 3)
    per_id = {str(tid): {"first_t": r3(s["first_t"]), "max_lost_fraction": round(s["max_lost"], 4),
                         "centre_drift_from_first_m": r3(s["dc_first"]),
                         "centre_drift_since_lock_m": r3(s["dc_lock"]) if s["lock_c"] is not None else None,
                         "height_drift_from_first_m": r3(s["dh_first"]),
                         "height_drift_since_lock_m": r3(s["dh_lock"]) if s["lock_h"] is not None else None}
              for tid, s in sorted(st.items())}
    mx = lambda key: max((s[key] for s in st.values()), default=0.0)
    return {
        "first_lock_t": t_lock,
        "id_switches": id_switches if frames is not None else None,
        "id_switches_after_lock": sum(1 for e in switch_events if e["after_lock"]) if frames is not None else None,
        "id_switch_events": switch_events[:MAX_EVENTS],
        "id_meaning_changes": len(meaning_changes) if frames is not None else None,
        "id_meaning_changes_after_lock": (sum(1 for e in meaning_changes if e["after_lock"])
                                          if frames is not None else None),
        "id_meaning_change_events": meaning_changes[:MAX_EVENTS],
        "id_reappearances": reappear, "id_reuse": len(reuse_ev),
        "id_reuse_by_owner": reuse_owner if frames is not None else None,
        "id_reuse_events": reuse_ev[:MAX_EVENTS],
        "geometry_retreats": len(retreats), "max_lost_fraction": round(lost_max, 4),
        "retreat_events": retreats[:MAX_EVENTS],
        "centre_jumps": len(jumps), "centre_jump_events": jumps[:MAX_EVENTS],
        "height_jumps": len(hjumps), "height_jump_events": hjumps[:MAX_EVENTS],
        "max_centre_drift_from_first_m": r3(mx("dc_first")), "max_centre_drift_since_lock_m": r3(mx("dc_lock")),
        "max_height_drift_from_first_m": r3(mx("dh_first")), "max_height_drift_since_lock_m": r3(mx("dh_lock")),
        "per_id": per_id,
        "ids_vanished": vanished,
        "ids_vanished_while_held_by_navigation": sorted(set(vanished) & (targeted | sprayed_ever)),
    }


def safety_metrics(views: List[SnapView], births, mission: Mission, cfg: ScoreConfig,
                   t0: Optional[float], t1: Optional[float],
                   truth_tops: Optional[List[List[Optional[float]]]] = None) -> dict:
    """``truth_tops[k][i]``: the truth top under entry i of snapshot k (None = no owning truth tree);
    ``truth_tops`` None = no reliable truth heights (plain above-the-drone test)."""
    out: dict = {}
    # ---- per-snapshot hygiene: undefined heights, invalid entries, duplicate IDs, implausible heights
    pl = PoseLookup(mission.pose)
    # the window end is exclusive when it is a RETURN, inclusive when it is the stream's last snapshot
    t_end = t1 if (t1 is not None and views and t1 < views[-1].t) else math.inf
    und_f = und_i = inv_f = dup_f = imp_f = 0
    und_ev, inv_ev, dup_ev, imp_ev = [], [], [], []
    inv_reasons: Dict[str, int] = {}
    imp_excused = 0
    for k, v in enumerate(views):
        bad = [tr.id for tr in v.trees if not _finite(tr.height)]
        if bad:
            und_f += 1
            und_i += len(bad)
            if len(und_ev) < MAX_EVENTS:
                und_ev.append({"t": round(v.t, 3), "ids": bad})
        inv = [(tr.id, pr) for tr, pr in zip(v.trees, v.problems) if pr]
        if inv:
            inv_f += 1
            for _, pr in inv:
                for r in pr:
                    inv_reasons[r] = inv_reasons.get(r, 0) + 1
            if len(inv_ev) < MAX_EVENTS:
                inv_ev.append({"t": round(v.t, 3), "entries": [{"id": i, "reasons": pr} for i, pr in inv]})
        if v.dup_ids:
            dup_f += 1
            if len(dup_ev) < MAX_EVENTS:
                dup_ev.append({"t": round(v.t, 3), "ids": v.dup_ids})
        p = pl.at(v.t) if (mission.pose and t0 is not None and t0 <= v.t < t_end) else None
        imp = []
        for i, tr in enumerate(v.trees):
            if not _finite(tr.height):
                continue
            if tr.height <= 0:
                imp.append({"id": tr.id, "height": round(tr.height, 3)})
            elif p is not None and tr.height > p[3]:
                top = truth_tops[k][i] if truth_tops is not None else None
                if top is not None and tr.height <= top + cfg.implausible_truth_tol_m:
                    imp_excused += 1           # accurate height of a tree whose top is above the hover
                    continue
                e = {"id": tr.id, "height": round(tr.height, 3)}
                if top is not None:
                    e["truth_top"] = round(top, 3)
                imp.append(e)
        if imp:
            imp_f += 1
            if len(imp_ev) < MAX_EVENTS:
                imp_ev.append({"t": round(v.t, 3), "drone_z": None if p is None else round(p[3], 3), "entries": imp})
    out.update({"undefined_height_frames": und_f, "undefined_height_instances": und_i,
                "undefined_height_events": und_ev,
                "invalid_entry_frames": inv_f, "invalid_entry_reasons": inv_reasons, "invalid_entry_events": inv_ev,
                "duplicate_id_frames": dup_f, "duplicate_id_events": dup_ev,
                "implausible_height_frames": imp_f, "implausible_height_events": imp_ev,
                "implausible_height_excused_entries": imp_excused if truth_tops is not None else None,
                "implausible_height_definition": (
                    "finite height <= 0 (no pose: drone-altitude test not run)" if not mission.pose else
                    "finite height <= 0, or above the drone altitude inside the mission window" +
                    ("" if truth_tops is None else
                     " and above the owning truth top + %.2f m (no owning truth tree: drone test only)"
                     % cfg.implausible_truth_tol_m))})

    # ---- sprayed protection (persistent record per sprayed ID) + new trees inside sprayed outlines
    records: Dict[int, dict] = {}
    sp_parts: List[object] = []
    sp_union = None
    sp_last: Dict[int, object] = {}
    sprayed_ev, new_in_ev = [], []
    for k, v in enumerate(views):
        cur_sp = {tid: v.id_poly(tid) for tid, idx in v.by_id.items() if any(v.trees[i].sprayed for i in idx)}
        for tid in v.by_id:
            if births.get((k, tid)) not in ("first", "reuse"):
                continue
            others = [p for sid, p in cur_sp.items() if sid != tid and p is not None]
            region = unary_union(([sp_union] if sp_union is not None else []) + others) if (sp_union is not None or others) else None
            if region is None or region.is_empty:
                continue
            tr = v.rep(tid)
            cp = v.id_poly(tid)
            c_in = _finite_pt(tr.centre) and region.contains(SPoint(tr.centre))
            share = (cp.intersection(region).area / cp.area) if cp is not None else 0.0
            if c_in or share >= cfg.inside_sprayed_min:
                new_in_ev.append({"t": round(v.t, 3), "id": tid, "centre_inside": bool(c_in),
                                  "share_inside": round(share, 3), "kind": births[(k, tid)]})
        for tid, rec in records.items():
            ents = v.entries(tid)
            if not ents:
                kind = "vanished"
            elif not all(tr.sprayed for tr, _ in ents):
                kind = "flag_cleared"
            elif _entries_differ(rec["entries"], ents, cfg):
                kind = "changed"
            else:
                kind = None
            if kind is not None and kind != rec["state"]:
                sprayed_ev.append({"t": round(v.t, 3), "id": tid, "kind": kind})
            rec["state"] = kind
        for tid in cur_sp:
            if tid not in records:
                records[tid] = {"t": v.t, "entries": v.entries(tid), "state": None}
            p = cur_sp[tid]
            if p is not None and (tid not in sp_last or sp_last[tid].symmetric_difference(p).area > cfg.change_eps_area):
                sp_parts.append(p)
                sp_last[tid] = p
                sp_union = unary_union(sp_parts)
    out["sprayed_tree_changes"] = len(sprayed_ev)
    out["sprayed_tree_change_kinds"] = {kd: sum(1 for e in sprayed_ev if e["kind"] == kd)
                                        for kd in ("changed", "vanished", "flag_cleared")}
    out["sprayed_tree_change_events"] = sprayed_ev[:MAX_EVENTS]
    out["new_tree_inside_sprayed"] = len(new_in_ev)
    out["new_tree_inside_sprayed_events"] = new_in_ev[:MAX_EVENTS]

    base = und_f + inv_f + dup_f + imp_f + len(sprayed_ev) + len(new_in_ev)
    if not mission.has_phases:
        out["new_trees_after_scan"] = None
        out["locked_id_vanished"] = None
        out["nontarget_changes_in_descent"] = None
        out["note"] = "no mission phases: phase-dependent safety metrics not applicable"
        out["total_violations"] = base
        return out

    phase_at = lambda t: step_lookup(mission.phases, t)
    # ---- new trees while the list is locked (first publication or a reused ID)
    new_after = []
    for k, v in enumerate(views):
        ph = phase_at(v.t)
        if ph not in cfg.locked_phases:
            continue
        for tid in v.by_id:
            b = births.get((k, tid))
            if b in ("first", "reuse"):
                new_after.append({"t": round(v.t, 3), "id": tid, "phase": ph, "kind": b})
    out["new_trees_after_scan"] = len(new_after)
    out["new_tree_after_scan_events"] = new_after[:MAX_EVENTS]

    # ---- IDs vanishing while the list is locked
    lv = []
    for k in range(1, len(views)):
        t = views[k].t
        if phase_at(t) not in cfg.locked_phases:
            continue
        tg = {step_lookup(mission.targets, t, -1), step_lookup(mission.targets, views[k - 1].t, -1)}
        for tid in views[k - 1].by_id:
            if tid not in views[k].by_id:
                lv.append({"t": round(t, 3), "id": tid, "phase": phase_at(t), "was_target": tid in tg})
    out["locked_id_vanished"] = len(lv)
    out["locked_id_vanished_targets"] = sum(1 for e in lv if e["was_target"])
    out["locked_id_vanished_events"] = lv[:MAX_EVENTS]

    # ---- non-target changes during descent windows
    kinds = descent_kinds([v.snap for v in views], mission, cfg)
    total = base + len(new_after) + len(lv)
    if kinds is None:
        out["nontarget_changes_in_descent"] = None
        out["descent_note"] = "mission has phases but no hop-target series: descent window not judgeable"
        out["total_violations"] = total
        return out
    events = []
    tgt_h: Dict[int, List[float]] = {}
    for k in range(len(views)):
        if not kinds[k]:
            continue
        b = views[k]
        tgt = step_lookup(mission.targets, b.t, -1)
        if kinds[k] in ("approach", "spraying") and tgt is not None and tgt >= 0 and tgt in b.by_id:
            h = b.rep(tgt).height
            if _finite(h):
                tgt_h.setdefault(tgt, []).append(h)
        if k == 0:
            continue
        a = views[k - 1]
        for tid in a.by_id:
            if tid == tgt:
                continue
            if tid not in b.by_id:
                events.append({"t": round(b.t, 3), "id": tid, "target": tgt, "kind": "vanished", "window": kinds[k]})
            elif _entries_differ(a.entries(tid), b.entries(tid), cfg):
                events.append({"t": round(b.t, 3), "id": tid, "target": tgt, "kind": "changed", "window": kinds[k]})
        for tid in b.by_id:
            if tid not in a.by_id and tid != tgt:
                events.append({"t": round(b.t, 3), "id": tid, "target": tgt, "kind": "appeared", "window": kinds[k]})
    out["descent_frames"] = sum(1 for x in kinds if x)
    out["descent_frames_by_kind"] = {kd: sum(1 for x in kinds if x == kd) for kd in ("approach", "spraying", "climb")}
    out["descent_window_note"] = ("approach + spraying + climb-out (pose given)" if mission.pose else
                                  "no pose: every HOPPING snapshot with a target judged; climb-out not judged")
    out["cruise_altitude_m"] = cruise_altitude(mission, cfg)
    out["nontarget_changes_in_descent"] = len(events)
    out["nontarget_change_events"] = events[:MAX_EVENTS]
    rng = {str(t): round(max(v) - min(v), 3) for t, v in sorted(tgt_h.items())}
    out["target_height_range_in_descent_m"] = rng
    out["target_height_range_in_descent_max_m"] = max(rng.values()) if rng else None
    out["total_violations"] = total + len(events)
    return out


def footprint_radius_per_m(camera: Optional[dict]) -> Optional[float]:
    """Half-diagonal of the nadir image footprint, per metre of range. A tree centre
    farther than this cannot be in the image for any yaw; the pose carries no camera
    yaw, so this yaw-free bound is used (latency is therefore an upper bound)."""
    if not camera:
        return None
    c = camera.get("left", camera)
    fx, fy = c["K"][0], c["K"][4]
    return math.hypot(c["width"] / 2.0 / fx, c["height"] / 2.0 / fy)


def first_view_times(gt: List[GroundTruthTree], mission: Mission, camera: Optional[dict],
                     cfg: ScoreConfig) -> Tuple[Dict[int, float], Dict[int, float]]:
    """truth index -> first time its centre was inside the footprint (any phase / SCAN only)."""
    k = footprint_radius_per_m(camera)
    fv: Dict[int, float] = {}
    fvs: Dict[int, float] = {}
    if not k or not mission.pose:
        return fv, fvs
    for t, x, y, z in mission.pose:
        ph = step_lookup(mission.phases, t) if mission.phases else cfg.scan_phases[0]
        for j, g in enumerate(gt):
            rng = z - g.height
            if rng <= 0 or math.hypot(x - g.centre[0], y - g.centre[1]) > k * rng:
                continue
            fv.setdefault(j, t)
            if ph in cfg.scan_phases:
                fvs.setdefault(j, t)
    return fv, fvs


def timeliness_metrics(frames: List[Frame], gt: List[GroundTruthTree], view_t, view_scan_t,
                       camera: Optional[dict], mission: Mission) -> dict:
    first_pub: Dict[int, float] = {}
    for f in frames:
        for j, i in f.match_g2p.items():
            if j not in first_pub and _finite(f.trees[i].height):
                first_pub[j] = f.snap.t
    has_view = footprint_radius_per_m(camera) is not None and bool(mission.pose)

    def lat(ref):
        return {j: first_pub[j] - ref[j] for j in range(len(gt)) if j in first_pub and j in ref}
    la, ls = lat(view_t), lat(view_scan_t)
    neg = sorted({gt[j].id for j, v in list(la.items()) + list(ls.items()) if v < 0})
    per = [{"truth_id": g.id, "first_in_view_t": view_t.get(j), "first_in_view_scan_t": view_scan_t.get(j),
            "first_published_t": first_pub.get(j),
            "latency_s": None if j not in la else round(la[j], 3),
            "latency_scan_s": None if j not in ls else round(ls[j], 3)} for j, g in enumerate(gt)]
    clamp = lambda v: [max(0.0, x) for x in v]
    stat = lambda v: (float(np.mean(v)) if v else None, float(np.max(v)) if v else None)
    m_any, x_any = stat(clamp(la.values()))
    m_scan, x_scan = stat(clamp(ls.values()))
    return {"definition": ("first snapshot where the tree is one-to-one matched to a valid entry with a finite "
                           "height, minus the first time its centre came within the nadir footprint's "
                           "half-diagonal (yaw-free bound); 'scan' variant counts view time only while the phase "
                           "is SCAN; negative latencies are listed and clamped to 0 in mean/max") if has_view else
                          "camera/pose not given: only absolute first-publish times",
            "trees_never_published": [g.id for j, g in enumerate(gt) if j not in first_pub],
            "negative_latency_trees": neg,
            "latency_mean_s": m_any, "latency_max_s": x_any,
            "latency_scan_mean_s": m_scan, "latency_scan_max_s": x_scan,
            "per_tree": per}


def cost_metrics(snaps: List[Snapshot]) -> dict:
    stages: Dict[str, List[float]] = {}
    for s in snaps:
        for k, v in s.stage_ms.items():
            stages.setdefault(k, []).append(float(v))
    if not stages:
        return {"available": False, "note": "map stream carries no stage_ms"}
    return {"available": True, "stages": {k: {"mean_ms": float(np.mean(v)), "p95_ms": float(np.percentile(v, 95)),
                                              "max_ms": float(np.max(v)), "n": len(v)} for k, v in stages.items()}}


# ----------------------------------------------------------------------------- report

def _f(v, nd=2):
    if v is None:
        return "n/a"
    if isinstance(v, float):
        return "n/a" if math.isnan(v) else ("%.*f" % (nd, v))
    return str(v)


def to_markdown(res: dict, title: str) -> str:
    L = ["# %s" % title, ""]
    cfgd = res["config"]
    L.append("Mode: %s; frame `%s`; %d snapshots." % (res["mode"], res["frame"], res["n_snapshots"]))
    if res.get("gt_repaired_ids"):
        L.append("Ground-truth trees with a repaired (degenerate) outline: %s." % res["gt_repaired_ids"])
    L.append("")
    fin = res.get("final") or {}
    pf = res.get("per_frame", {})
    pp = res.get("per_frame_published", {})
    if fin.get("detection"):
        d, g, sp = fin["detection"], fin["geometry"], res["spray"]
        hrel = res.get("gt_height_reliable", True)
        hflag = "" if hrel else " **(GT height unreliable (constant fallback): not a headline, not pass/fail)**"

        def pm(grp, k):
            a = pf.get(grp, {}).get(k)
            b = pp.get(grp, {}).get(k)
            return "%s / %s" % (_f(a["mean"]) if a else "n/a", _f(b["mean"]) if b else "n/a")
        pd, pg = "detection", "geometry"
        L += ["Final map at t = %.2f s. Per-frame column = time-weighted mean of the latest snapshot sampled every "
              "%.2f s over the %s (%d samples) / from the first non-empty snapshot to the window end (%d samples)." %
              (fin["t"], cfgd["sample_dt_s"], pf.get("window", "n/a"), pf.get("n_samples", 0),
               pp.get("n_samples", 0)), "",
              "## Detection", "", "| Metric | Final map | Per-frame mean (window / from first publish) |",
              "|---|---|---|",
              "| Trees in truth / published / matched | %d / %d / %d | — |" % (d["n_truth"], d["n_published"], d["matched"]),
              "| Recall | %s | %s |" % (_f(d["recall"]), pm(pd, "recall")),
              "| Recall over trees already in view | %s (%s trees ever in view) | %s |" % (
                  _f(d.get("recall_in_view")), _f(d.get("n_in_view")), pm(pd, "recall_in_view")),
              "| Precision | %s | %s |" % (_f(d["precision"]), pm(pd, "precision")),
              "| Missed (absorbed by a merge / unseen) | %d (%d / %d) | %s |" % (d["missed"], d["missed_absorbed"], d["missed_unseen"], pm(pd, "missed")),
              "| Merged entries / trees inside them / neighbours swallowed | %d / %d / %d | %s (swallowed) |" % (d["merged_entries"], d["trees_in_merged_entries"], d["wrong_merges"], pm(pd, "wrong_merges")),
              "| Split trees (duplicates / fragments) | %d (%d / %d) | %s |" % (d["split_trees"], d["duplicates"], d["fragments"], pm(pd, "split_trees")),
              "| False trees / invalid entries | %d / %d | %s |" % (d["false_trees"], d["invalid_entries"], pm(pd, "false_trees")),
              "", "Missed truth IDs: %s. Merged entries (map id: truth ids covered): %s." % (d["missed_ids"], d["merged_entry_ids"]), "",
              "## Geometry (matched pairs)", "",
              "Truth heights: %s." % res.get("gt_height_note", "n/a"), "",
              "| Metric | Final map | Per-frame mean (window / from first publish) |", "|---|---|---|",
              "| Mean IoU (min) | %s (%s) | %s |" % (_f(g["mean_iou"]), _f(g["min_iou"]), pm(pg, "mean_iou")),
              "| Centre error mean / RMSE / max (m) | %s / %s / %s | %s RMSE |" % (_f(g["centre_mean_m"]), _f(g["centre_rmse_m"]), _f(g["centre_max_m"]), pm(pg, "centre_rmse_m")),
              "| Height bias / RMSE / max abs (m)%s | %s / %s / %s | %s RMSE |" % (hflag, _f(g["height_bias_m"]), _f(g["height_rmse_m"]), _f(g["height_max_abs_m"]), pm(pg, "height_rmse_m")),
              "| Matched entries with undefined height (charged %.0f m each) | %d | %s |" % (cfgd["undefined_height_error_m"], g["n_undefined_height"], pm(pg, "n_undefined_height")),
              "| Spill outside all true canopies | %s | %s |" % (_f(g["spill"]), pm(pg, "spill")),
              "| Neighbour spill mean / max | %s / %s | %s |" % (_f(g["neighbour_spill_mean"]), _f(g["neighbour_spill_max"]), pm(pg, "neighbour_spill_mean")),
              "", "| Truth id | Map id | IoU | Centre err (m) | Height err (m)%s | Neighbour spill |" % ("" if hrel else " (GT unreliable)"),
              "|---|---|---|---|---|---|"]
        for r in g["per_tree"]:
            L.append("| %d | %d | %s | %s | %s | %s |" % (r["truth_id"], r["map_id"], _f(r["iou"]), _f(r["centre_err_m"]), _f(r["height_err_m"]), _f(r["neighbour_spill"])))
        L += ["", "## Spray outcome (whole stream)", "",
              "Sprayed IDs %d; truth trees sprayed %d of %d; sprayed twice %d; orphan sprayed IDs %d; not sprayed: %s; "
              "sprayed by (truth: ids): %s. (%s.)" %
              (sp["sprayed_ids"], sp["truth_trees_sprayed"], d["n_truth"], sp["truth_trees_sprayed_twice"],
               sp["orphan_sprayed_entries"], sp["truth_trees_not_sprayed"], sp["sprayed_by"], sp["definition"]), ""]
    elif res.get("note"):
        L += [res["note"] + ". Final map: %s entries at t = %s." % (fin.get("n_published"), _f(fin.get("t"))), ""]
    st, sa, co = res["stability"], res["safety"], res["cost"]
    L += ["## Stability", "", "| Metric | Value |", "|---|---|",
          "| ID switches, total / after the list locks (t = %s s) | %s / %s |" % (_f(st["first_lock_t"]), _f(st["id_switches"]), _f(st["id_switches_after_lock"])),
          "| ID meaning changes, total / after lock | %s / %s |" % (_f(st["id_meaning_changes"]), _f(st["id_meaning_changes_after_lock"])),
          "| ID reappearances / geometric reuse / reuse by truth owner | %d / %d / %s |" % (st["id_reappearances"], st["id_reuse"], _f(st["id_reuse_by_owner"])),
          "| Geometry retreats (> %.0f%% of the ID's historic outline union lost; onsets) / max lost fraction | %d / %s |" % (cfgd["retreat_tol"] * 100, st["geometry_retreats"], _f(st["max_lost_fraction"], 3)),
          "| Centre jumps (> %.1f m within %.1f s; onsets) | %d |" % (cfgd["jump_m"], cfgd["jump_window_s"], st["centre_jumps"]),
          "| Height jumps (> %.1f m within %.1f s or vs last finite; onsets) | %d |" % (cfgd["height_jump_m"], cfgd["jump_window_s"], st["height_jumps"]),
          "| Max centre drift per ID from first publish / since lock (m) | %s / %s |" % (_f(st["max_centre_drift_from_first_m"]), _f(st["max_centre_drift_since_lock_m"])),
          "| Max height drift per ID from first publish / since lock (m) | %s / %s |" % (_f(st["max_height_drift_from_first_m"]), _f(st["max_height_drift_since_lock_m"])),
          "| IDs vanished (of which held by navigation) | %s (%s) |" % (st["ids_vanished"], st["ids_vanished_while_held_by_navigation"]),
          "| Truth trees ever split into >= 2 entries | %s |" % _f(st.get("truth_trees_ever_split")),
          "| Truth trees ever inside a merged entry | %s |" % _f(st.get("truth_trees_ever_in_a_merged_entry")),
          "", "Per ID: max lost fraction, centre drift from first / since lock, height drift from first / since lock (m):", "",
          "| ID | First published (s) | Max lost | Centre drift first / lock | Height drift first / lock |", "|---|---|---|---|---|"]
    for tid, p in st["per_id"].items():
        L.append("| %s | %s | %s | %s / %s | %s / %s |" % (tid, _f(p["first_t"]), _f(p["max_lost_fraction"], 3),
                                                       _f(p["centre_drift_from_first_m"]), _f(p["centre_drift_since_lock_m"]),
                                                       _f(p["height_drift_from_first_m"]), _f(p["height_drift_since_lock_m"])))
    dk = sa.get("descent_frames_by_kind") or {}
    L += ["", "## Safety (all must be 0; n/a = not judgeable from the inputs)", "", "| Metric | Value |", "|---|---|",
          "| Frames publishing an undefined height | %d |" % sa["undefined_height_frames"],
          "| Frames with an invalid entry (non-finite centre/outline, < 3 vertices, zero area) | %d |" % sa["invalid_entry_frames"],
          "| Frames with a duplicate ID | %d |" % sa["duplicate_id_frames"],
          "| Frames with an implausible height (<= 0, or above the drone%s) | %d |" % (
              "" if sa.get("implausible_height_excused_entries") is None else
              " and above truth top + %.1f m; %d accurate entries excused" % (
                  cfgd["implausible_truth_tol_m"], sa["implausible_height_excused_entries"]),
              sa["implausible_height_frames"]),
          "| New (or reused) IDs while the list is locked (%s) | %s |" % ("/".join(cfgd["locked_phases"]), _f(sa.get("new_trees_after_scan"))),
          "| IDs vanished while the list is locked (of which the hop target) | %s (%s) |" % (_f(sa.get("locked_id_vanished")), _f(sa.get("locked_id_vanished_targets"))),
          "| Non-target changes during descent (%s snapshots judged: approach %s, spraying %s, climb-out %s) | %s |" % (
              _f(sa.get("descent_frames")), _f(dk.get("approach")), _f(dk.get("spraying")), _f(dk.get("climb")),
              _f(sa.get("nontarget_changes_in_descent"))),
          "| Sprayed tree changed / vanished / lost its flag after spraying | %d |" % sa["sprayed_tree_changes"],
          "| New tree inside a sprayed outline | %d |" % sa["new_tree_inside_sprayed"],
          "| **Total violations** | **%d** |" % sa["total_violations"], "",
          "Informational: target height range during its own approach/spraying window, max %s m; per target %s." % (
              _f(sa.get("target_height_range_in_descent_max_m")), sa.get("target_height_range_in_descent_m")), ""]
    for key in ("new_tree_after_scan_events", "locked_id_vanished_events", "nontarget_change_events",
                "sprayed_tree_change_events", "new_tree_inside_sprayed_events", "undefined_height_events",
                "invalid_entry_events", "duplicate_id_events", "implausible_height_events", "id_switch_events",
                "id_meaning_change_events", "id_reuse_events", "retreat_events", "centre_jump_events",
                "height_jump_events"):
        ev = sa.get(key) if key in sa else st.get(key)
        if ev:
            L.append("- `%s` (first %d of %d listed): %s" % (key, min(len(ev), 12), len(ev), json.dumps(ev[:12])))
    ti = res.get("timeliness")
    if ti:
        L += ["", "## Timeliness", "", ti["definition"] + ".", "",
              "Latency mean / max: %s s / %s s from first view in any phase; %s s / %s s from first view while scanning. "
              "Never published: %s. Negative latencies (clamped to 0): %s." % (
                  _f(ti["latency_mean_s"]), _f(ti["latency_max_s"]), _f(ti.get("latency_scan_mean_s")),
                  _f(ti.get("latency_scan_max_s")), ti["trees_never_published"], ti["negative_latency_trees"]), "",
              "| Truth id | First in view (s) | First in view while scanning (s) | First published (s) | Latency (s) | Latency from scan view (s) |", "|---|---|---|---|---|---|"]
        for r in ti["per_tree"]:
            L.append("| %d | %s | %s | %s | %s | %s |" % (r["truth_id"], _f(r["first_in_view_t"]), _f(r.get("first_in_view_scan_t")), _f(r["first_published_t"]), _f(r["latency_s"]), _f(r.get("latency_scan_s"))))
    L += ["", "## Cost", ""]
    if co.get("available"):
        L += ["| Stage | mean ms | p95 ms | max ms |", "|---|---|---|---|"]
        for k, v in co["stages"].items():
            L.append("| %s | %s | %s | %s |" % (k, _f(v["mean_ms"]), _f(v["p95_ms"]), _f(v["max_ms"])))
    else:
        L.append(co.get("note", "n/a"))
    return "\n".join(L) + "\n"


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, tuple):
        return list(o)
    raise TypeError(type(o))


def _nan_to_none(o):
    if isinstance(o, float) and not math.isfinite(o):
        return None
    if isinstance(o, dict):
        return {k: _nan_to_none(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_nan_to_none(v) for v in o]
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(description="Score a map stream (output contract), with or without ground truth.")
    ap.add_argument("--map", required=True, help="map stream .jsonl (contract format)")
    ap.add_argument("--gt", help="ground_truth.json (omit: stability + safety + cost only)")
    ap.add_argument("--mission", help="mission.json (phases, targets, pose)")
    ap.add_argument("--camera", help="camera_info.json (for timeliness)")
    ap.add_argument("--title", default="Scorecard")
    ap.add_argument("--out-json")
    ap.add_argument("--out-md")
    ap.add_argument("--append-md", help="markdown file appended to the scorecard (hand-written notes)")
    a = ap.parse_args(argv)
    snaps = read_snapshots(a.map)
    gt = read_ground_truth(a.gt) if a.gt else None
    mission = read_mission(a.mission)
    camera = None
    if a.camera:
        with open(a.camera) as f:
            camera = json.load(f) or None
    res = score(snaps, gt, mission, camera)
    res["inputs"] = {"map": a.map, "gt": a.gt, "mission": a.mission, "camera": a.camera}
    md = to_markdown(res, a.title)
    if a.append_md:
        with open(a.append_md) as f:
            md += "\n" + f.read()
    if a.out_json:
        with open(a.out_json, "w") as f:
            json.dump(_nan_to_none(res), f, indent=1, default=_json_default)
    if a.out_md:
        with open(a.out_md, "w") as f:
            f.write(md)
    print(md)


if __name__ == "__main__":
    main()
