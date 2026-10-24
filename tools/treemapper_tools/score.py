"""Scorer: judges ANY system that emits the output contract against ground truth.

Inputs are the contract files of ``contract.py`` (map stream, ground truth,
optional mission context and camera intrinsics). Nothing here knows how the map
was produced, so the existing system, our baselines and future candidates are
scored identically.

Metric definitions (all thresholds live in ``ScoreConfig``; see
docs/SCORER.md for the rationale):

Detection, per snapshot (final map = last snapshot):
  * one-to-one matching: Hungarian on IoU, pairs need IoU >= match_min_iou.
    recall = matched / n_truth, precision = matched / n_published.
  * cov(P,G) = |P∩G|/|G|; P "covers" G if cov >= cover_min.
  * owner(P) = truth tree holding the largest share of P, if that share >= own_min, else none.
  * missed       = truth trees with no match. Split into ``absorbed`` (covered by
                   some published entry, i.e. swallowed by a merge) and ``unseen``.
  * merged_entries = published entries covering >= 2 truth trees;
    wrong_merges   = sum over those of (n_covered - 1) = neighbours swallowed.
  * extra entries owned by an already-owned truth tree: ``duplicates`` if the extra
    entry has IoU >= dup_min_iou with that tree, else ``fragments``;
    split_trees = truth trees owning >= 2 entries.
  * false_trees  = published entries with no owner.
Geometry (matched pairs): IoU, centre error, height error (published - truth),
  spill = published area outside every true canopy / published area,
  neighbour_spill = matched entry's area inside OTHER true canopies / its area.
Stability (whole stream, per ID across consecutive publications):
  * id_switches: a truth tree's matched ID changes (a -> b, a != b).
  * id_meaning_changes: an ID's owner changes from one truth tree to another.
  * id_reappearances: an ID disappears and is later published again
    (id_reuse = reappearances whose owner differs).
  * retreats: fraction of the previous outline no longer covered > retreat_tol
    (shrink or deformation); jumps: centre moves > jump_m in one update;
    height_jumps: |Δheight| > height_jump_m in one update.
  * vanished ids, and vanished ids that navigation held (sprayed or targeted).
Safety (must all be zero):
  * undefined_height_frames: snapshots publishing any non-finite / missing height.
  * new_trees_after_scan: IDs first published while the phase is in locked_phases.
  * nontarget_changes_in_descent: during the descent window (see ``descent_mask``)
    any tree other than the current hop target changes outline/height, vanishes,
    or a new tree appears.
  * sprayed_tree_changes: outline/height of a sprayed tree changes after it was sprayed.
  * new_tree_inside_sprayed: a new ID whose centre lies inside a sprayed outline.
Timeliness: per truth tree, first publish time (first snapshot where some entry
  covers it) minus the first time its centre entered the camera footprint
  (needs pose + camera); also absolute first-publish times.
Cost: per-stage processing time if the stream carries ``stage_ms``.
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment
from shapely.geometry import Point as SPoint
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid

from .contract import (GroundTruthTree, Mission, Snapshot, Tree, read_ground_truth, read_mission,
                       read_snapshots, step_lookup)


@dataclass
class ScoreConfig:
    match_min_iou: float = 0.10      # one-to-one match needs at least this IoU
    cover_min: float = 0.30          # entry covers a truth tree if it holds >= 30% of it
    own_min: float = 0.20            # entry is owned by a tree holding >= 20% of the entry
    dup_min_iou: float = 0.30        # an extra entry this similar to its tree is a duplicate
    retreat_tol: float = 0.02        # > 2% of the previous outline lost in one update = retreat
    jump_m: float = 1.0              # centre moving more than this in one update = jump
    height_jump_m: float = 1.0       # height changing more than this in one update = jump
    change_eps_area: float = 1e-3    # m^2 symmetric difference that counts as a change
    change_eps_h: float = 1e-3       # m height change that counts as a change
    locked_phases: Tuple[str, ...] = ("PLANNING", "HOPPING", "SPRAYING")
    descent_phases: Tuple[str, ...] = ("HOPPING", "SPRAYING")
    descent_radius_m: float = 1.5    # in HOPPING, descent = within this xy distance of the spray hover point
    scan_phases: Tuple[str, ...] = ("SCAN",)


# ----------------------------------------------------------------------------- geometry

def to_poly(outline: Sequence[Tuple[float, float]]):
    if len(outline) < 3:
        return None
    p = Polygon(outline)
    if not p.is_valid:
        p = make_valid(p)
        p = unary_union([g for g in getattr(p, "geoms", [p]) if g.area > 0]) if p.area > 0 else None
    if p is None or p.is_empty or p.area <= 0:
        return None
    return p


def _area(g) -> float:
    return 0.0 if g is None else float(g.area)


def _finite(v) -> bool:
    return v is not None and math.isfinite(v)


class Frame:
    """Pairwise geometry between one published snapshot and the ground truth."""

    def __init__(self, snap: Snapshot, gt: List[GroundTruthTree], gt_polys, cfg: ScoreConfig):
        self.snap, self.gt, self.cfg = snap, gt, cfg
        self.trees = snap.trees
        self.polys = [to_poly(t.outline) for t in self.trees]
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
        # one-to-one matching
        self.match_p2g: Dict[int, int] = {}
        if n and m:
            r, c = linear_sum_assignment(-self.iou)
            for i, j in zip(r, c):
                if self.iou[i, j] >= cfg.match_min_iou:
                    self.match_p2g[i] = j
        self.match_g2p = {j: i for i, j in self.match_p2g.items()}
        self.owner: List[Optional[int]] = []
        for i in range(n):
            j = int(np.argmax(self.share[i])) if m else -1
            self.owner.append(j if m and self.share[i, j] >= cfg.own_min else None)

    def owner_gt_id(self, i: int) -> Optional[int]:
        j = self.owner[i]
        return None if j is None else self.gt[j].id


def detection_metrics(fr: Frame) -> dict:
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
    false_trees = [i for i in range(n) if fr.owner[i] is None]
    return {
        "n_truth": m, "n_published": n, "matched": matched,
        "recall": matched / m if m else float("nan"),
        "precision": matched / n if n else float("nan"),
        "missed": len(missed), "missed_absorbed": len(absorbed), "missed_unseen": len(missed) - len(absorbed),
        "merged_entries": len(merged_entries), "wrong_merges": wrong_merges,
        "trees_in_merged_entries": int(sum(covers[i].sum() for i in merged_entries)),
        "split_trees": len(split_trees), "duplicates": duplicates, "fragments": fragments,
        "false_trees": len(false_trees),
        "missed_ids": [fr.gt[j].id for j in missed],
        "split_tree_ids": [fr.gt[j].id for j in split_trees],
        "merged_tree_ids": sorted({fr.gt[j].id for i in merged_entries for j in np.nonzero(covers[i])[0]}),
        "merged_entry_ids": {str(fr.trees[i].id): [fr.gt[j].id for j in np.nonzero(covers[i])[0]]
                             for i in merged_entries},
        "false_tree_ids": [fr.trees[i].id for i in false_trees],
    }


def geometry_metrics(fr: Frame, gt_union) -> dict:
    ious, cerr, herr, nspill = [], [], [], []
    per_tree = []
    for i, j in sorted(fr.match_p2g.items(), key=lambda kv: fr.gt[kv[1]].id):
        t, g = fr.trees[i], fr.gt[j]
        ious.append(fr.iou[i, j])
        ce = math.hypot(t.centre[0] - g.centre[0], t.centre[1] - g.centre[1])
        cerr.append(ce)
        he = (t.height - g.height) if _finite(t.height) else None
        if he is not None:
            herr.append(he)
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
        "height_bias_m": float(np.mean(herr)) if herr else float("nan"),
        "height_rmse_m": float(np.sqrt(np.mean(arr(herr) ** 2))) if herr else float("nan"),
        "height_max_abs_m": float(np.max(np.abs(herr))) if herr else float("nan"),
        "spill": outside / tot if tot > 0 else float("nan"),
        "neighbour_spill_mean": float(np.mean(nspill)) if nspill else float("nan"),
        "neighbour_spill_max": float(np.max(nspill)) if nspill else float("nan"),
        "per_tree": per_tree,
    }


# ----------------------------------------------------------------------------- mission helpers

def cruise_altitude(mission: Mission, cfg: ScoreConfig) -> Optional[float]:
    if not mission.pose or not mission.phases:
        return None
    zs = [p[3] for p in mission.pose if step_lookup(mission.phases, p[0]) in cfg.scan_phases]
    return float(np.median(zs)) if zs else None


def descent_mask(snaps: List[Snapshot], mission: Mission, cfg: ScoreConfig) -> List[bool]:
    """True for snapshots inside a descent onto the hop target (the freeze window).

    /mission_state has no DESCENT value, and hop transit runs at its own (lower)
    altitude, so altitude alone cannot tell transit from descent. Definition:
    every SPRAYING frame, plus every HOPPING frame whose drone xy is within
    ``descent_radius_m`` of the point where the following SPRAYING phase starts
    (the hover point over the target): arrival, descent and the pre-spray hover.
    Hop transit is not judged: the brief lets existing trees grow while hopping.
    Without pose, every HOPPING frame is judged (stricter). Only frames with a
    valid target (>= 0) are judged.
    """
    phases = mission.phases
    spray_xy: Dict[int, Tuple[float, float]] = {}  # index of a SPRAYING phase change -> hover xy
    if mission.pose:
        for k, (t, ph) in enumerate(phases):
            if ph == "SPRAYING":
                ts = [p[0] for p in mission.pose]
                q = min(max(int(np.searchsorted(ts, t)), 0), len(ts) - 1)
                spray_xy[k] = (mission.pose[q][1], mission.pose[q][2])
    pose_t = [p[0] for p in mission.pose]
    out = []
    for s in snaps:
        ph = step_lookup(phases, s.t)
        tgt = step_lookup(mission.targets, s.t, -1)
        if ph not in cfg.descent_phases or tgt is None or tgt < 0:
            out.append(False)
            continue
        if ph == "HOPPING" and mission.pose:
            k = next((k for k, (t, _) in enumerate(phases) if t > s.t), None)
            if k is None or phases[k][1] != "SPRAYING" or k not in spray_xy:
                out.append(False)
                continue
            q = max(int(np.searchsorted(pose_t, s.t, side="right")) - 1, 0)
            x, y = mission.pose[q][1], mission.pose[q][2]
            hx, hy = spray_xy[k]
            out.append(math.hypot(x - hx, y - hy) <= cfg.descent_radius_m)
        else:
            out.append(True)
    return out


def _tree_changed(a: Tree, b: Tree, pa, pb, cfg: ScoreConfig) -> bool:
    if _finite(a.height) != _finite(b.height):
        return True
    if _finite(a.height) and abs(a.height - b.height) > cfg.change_eps_h:
        return True
    if (pa is None) != (pb is None):
        return True
    if pa is None:
        return False
    return pa.symmetric_difference(pb).area > cfg.change_eps_area


# ----------------------------------------------------------------------------- main scorer

def score(snaps: List[Snapshot], gt: List[GroundTruthTree], mission: Optional[Mission] = None,
          camera: Optional[dict] = None, cfg: Optional[ScoreConfig] = None) -> dict:
    cfg = cfg or ScoreConfig()
    mission = mission or Mission()
    gt_polys = [to_poly(g.outline) for g in gt]
    gt_union = unary_union(gt_polys)
    gt_index = {g.id: j for j, g in enumerate(gt)}
    frames = [Frame(s, gt, gt_polys, cfg) for s in snaps]
    res: dict = {"config": asdict(cfg), "n_snapshots": len(snaps),
                 "t_start": snaps[0].t if snaps else None, "t_end": snaps[-1].t if snaps else None}

    # ---- final map
    if frames:
        fin = frames[-1]
        res["final"] = {"t": fin.snap.t, "detection": detection_metrics(fin),
                        "geometry": geometry_metrics(fin, gt_union)}
        res["final"]["spray"] = spray_outcome(fin, cfg)

    # ---- per-frame: "mission" = every snapshot in the stream (empty ones score recall 0);
    #      "published" = from the first non-empty snapshot on.
    first_pub = next((k for k, f in enumerate(frames) if f.trees), None)
    res["per_frame"] = per_frame(frames, gt_union, "every snapshot of the stream")
    res["per_frame_published"] = per_frame(frames[first_pub:] if first_pub is not None else [], gt_union,
                                           "first non-empty snapshot .. last snapshot")
    res["stability"] = stability_metrics(frames, mission, cfg)
    ever_split, ever_merged = set(), set()
    for f in frames:
        d = detection_metrics(f)
        ever_split |= set(d["split_tree_ids"])
        ever_merged |= set(d["merged_tree_ids"])
    res["stability"]["truth_trees_ever_split"] = sorted(ever_split)
    res["stability"]["truth_trees_ever_in_a_merged_entry"] = sorted(ever_merged)
    res["safety"] = safety_metrics(frames, mission, cfg)
    res["timeliness"] = timeliness_metrics(frames, gt, mission, camera, cfg)
    res["cost"] = cost_metrics(snaps)
    res["ids_ever_published"] = sorted({t.id for s in snaps for t in s.trees})
    return res


def per_frame(win: List[Frame], gt_union, window: str) -> dict:
    pf: dict = {"window": window, "n_frames": len(win)}
    if not win:
        return pf
    det = [detection_metrics(f) for f in win]
    geo = [geometry_metrics(f, gt_union) for f in win]
    for key, rows in (("detection", det), ("geometry", geo)):
        agg = {}
        for k in rows[0]:
            vals = [r[k] for r in rows if isinstance(r[k], (int, float)) and not isinstance(r[k], bool)
                    and not (isinstance(r[k], float) and math.isnan(r[k]))]
            if vals:
                agg[k] = {"mean": float(np.mean(vals)), "min": float(np.min(vals)), "max": float(np.max(vals))}
        pf[key] = agg
    return pf


def spray_outcome(fr: Frame, cfg: ScoreConfig) -> dict:
    """Navigation flies to an entry's centre and sprays there, so a truth tree counts
    as sprayed iff the centre of a sprayed entry lies inside its true canopy."""
    sprayed = [i for i, t in enumerate(fr.trees) if t.sprayed]
    gpolys = [to_poly(g.outline) for g in fr.gt]
    per_gt: Dict[int, int] = {}
    orphans = []
    for i in sprayed:
        c = SPoint(fr.trees[i].centre)
        hits = [j for j, gp in enumerate(gpolys) if gp is not None and gp.contains(c)]
        for j in hits:
            per_gt[j] = per_gt.get(j, 0) + 1
        if not hits:
            orphans.append(fr.trees[i].id)
    return {"definition": "truth tree sprayed iff a sprayed entry's centre lies inside its canopy",
            "sprayed_entries": len(sprayed), "truth_trees_sprayed": len(per_gt),
            "truth_trees_sprayed_twice": sum(1 for v in per_gt.values() if v >= 2),
            "orphan_sprayed_entries": len(orphans), "orphan_sprayed_ids": orphans,
            "truth_trees_not_sprayed": [g.id for j, g in enumerate(fr.gt) if j not in per_gt]}


def stability_metrics(frames: List[Frame], mission: Mission, cfg: ScoreConfig) -> dict:
    id_switches = 0
    switch_events = []
    last_match: Dict[int, int] = {}
    owner_hist: Dict[int, int] = {}
    meaning_changes = []
    present_prev: set = set()
    seen: set = set()
    reappear = 0
    reuse = 0
    owner_before_gap: Dict[int, Optional[int]] = {}
    gone: set = set()
    prev_tree: Dict[int, Tuple[Tree, object]] = {}
    retreats, jumps, hjumps = [], [], []
    max_lost = 0.0
    t_lock = next((t for t, ph in mission.phases if ph in cfg.locked_phases), None)
    after_lock = lambda t: t_lock is not None and t >= t_lock
    for f in frames:
        t = f.snap.t
        for j, i in f.match_g2p.items():
            gid, mid = f.gt[j].id, f.trees[i].id
            if gid in last_match and last_match[gid] != mid:
                id_switches += 1
                switch_events.append({"t": round(t, 3), "truth_id": gid, "from": last_match[gid], "to": mid,
                                      "after_lock": after_lock(t)})
            last_match[gid] = mid
        present = set()
        for i, tr in enumerate(f.trees):
            present.add(tr.id)
            own = f.owner_gt_id(i)
            if own is not None:
                if tr.id in owner_hist and owner_hist[tr.id] != own:
                    meaning_changes.append({"t": round(t, 3), "id": tr.id, "from": owner_hist[tr.id], "to": own,
                                            "after_lock": after_lock(t)})
                owner_hist[tr.id] = own
            if tr.id in gone:
                reappear += 1
                gone.discard(tr.id)
                b = owner_before_gap.get(tr.id)
                if b is not None and own is not None and b != own:
                    reuse += 1
            p = f.polys[i]
            if tr.id in prev_tree:
                a, pa = prev_tree[tr.id]
                if pa is not None and p is not None and pa.area > 0:
                    lost = pa.difference(p).area / pa.area
                    max_lost = max(max_lost, lost)
                    if lost > cfg.retreat_tol:
                        retreats.append({"t": round(t, 3), "id": tr.id, "lost_fraction": round(lost, 3)})
                d = math.hypot(tr.centre[0] - a.centre[0], tr.centre[1] - a.centre[1])
                if d > cfg.jump_m:
                    jumps.append({"t": round(t, 3), "id": tr.id, "moved_m": round(d, 3)})
                if _finite(a.height) and _finite(tr.height) and abs(tr.height - a.height) > cfg.height_jump_m:
                    hjumps.append({"t": round(t, 3), "id": tr.id, "dh_m": round(tr.height - a.height, 3)})
            prev_tree[tr.id] = (tr, p)
        for vid in present_prev - present:
            gone.add(vid)
            owner_before_gap[vid] = owner_hist.get(vid)
        seen |= present
        present_prev = present
    final_ids = present_prev
    targeted = {i for _, i in mission.targets if i >= 0}
    sprayed_ever = {tr.id for f in frames for tr in f.trees if tr.sprayed}
    vanished = sorted(seen - final_ids)
    return {
        "first_lock_t": t_lock,
        "id_switches": id_switches,
        "id_switches_after_lock": sum(1 for e in switch_events if e["after_lock"]),
        "id_switch_events": switch_events[:50],
        "id_meaning_changes": len(meaning_changes),
        "id_meaning_changes_after_lock": sum(1 for e in meaning_changes if e["after_lock"]),
        "id_meaning_change_events": meaning_changes[:50],
        "id_reappearances": reappear, "id_reuse": reuse,
        "geometry_retreats": len(retreats), "max_lost_fraction": round(max_lost, 4),
        "retreat_events": retreats[:50],
        "centre_jumps": len(jumps), "centre_jump_events": jumps[:50],
        "height_jumps": len(hjumps), "height_jump_events": hjumps[:50],
        "ids_vanished": vanished,
        "ids_vanished_while_held_by_navigation": sorted(set(vanished) & (targeted | sprayed_ever)),
    }


def safety_metrics(frames: List[Frame], mission: Mission, cfg: ScoreConfig) -> dict:
    snaps = [f.snap for f in frames]
    undefined_frames = 0
    undefined_instances = 0
    undefined_events = []
    for s in snaps:
        bad = [t.id for t in s.trees if not _finite(t.height)]
        if bad:
            undefined_frames += 1
            undefined_instances += len(bad)
            if len(undefined_events) < 50:
                undefined_events.append({"t": round(s.t, 3), "ids": bad})
    out = {"undefined_height_frames": undefined_frames, "undefined_height_instances": undefined_instances,
           "undefined_height_events": undefined_events}

    # sprayed protection + new trees inside sprayed (phase-independent)
    seen: set = set()
    sprayed_changes, new_in_sprayed = [], []
    prev: Dict[int, Tuple[Tree, object]] = {}
    for f in frames:
        sprayed_polys = [(tr.id, f.polys[i]) for i, tr in enumerate(f.trees)
                         if tr.sprayed and f.polys[i] is not None]
        for i, tr in enumerate(f.trees):
            if tr.id not in seen:
                c = SPoint(tr.centre)
                inside = [sid for sid, sp in sprayed_polys if sid != tr.id and sp.contains(c)]
                # also sprayed in the previous frame (the new tree may appear with the spray flag frame)
                for pid, (pt, pp) in prev.items():
                    if pt.sprayed and pp is not None and pid != tr.id and pp.contains(c) and pid not in inside:
                        inside.append(pid)
                if inside:
                    new_in_sprayed.append({"t": round(f.snap.t, 3), "id": tr.id, "inside": inside})
            if tr.id in prev:
                a, pa = prev[tr.id]
                if a.sprayed and _tree_changed(a, tr, pa, f.polys[i], cfg):
                    sprayed_changes.append({"t": round(f.snap.t, 3), "id": tr.id})
        seen |= {tr.id for tr in f.trees}
        prev = {tr.id: (tr, f.polys[i]) for i, tr in enumerate(f.trees)}
    out["sprayed_tree_changes"] = len(sprayed_changes)
    out["sprayed_tree_change_events"] = sprayed_changes[:50]
    out["new_tree_inside_sprayed"] = len(new_in_sprayed)
    out["new_tree_inside_sprayed_events"] = new_in_sprayed[:50]

    if not mission.has_phases:
        out["new_trees_after_scan"] = None
        out["nontarget_changes_in_descent"] = None
        out["note"] = "no mission phases: phase-dependent safety metrics not applicable"
        out["total_violations"] = undefined_frames + len(sprayed_changes) + len(new_in_sprayed)
        return out

    # new trees while the tree list is locked
    seen = set()
    new_after = []
    for s in snaps:
        ph = step_lookup(mission.phases, s.t)
        for tr in s.trees:
            if tr.id not in seen and ph in cfg.locked_phases:
                new_after.append({"t": round(s.t, 3), "id": tr.id, "phase": ph})
        seen |= {tr.id for tr in s.trees}
    out["new_trees_after_scan"] = len(new_after)
    out["new_tree_after_scan_events"] = new_after[:50]

    # non-target changes during descent
    mask = descent_mask(snaps, mission, cfg)
    events = []
    for k in range(1, len(frames)):
        if not mask[k]:
            continue
        tgt = step_lookup(mission.targets, frames[k].snap.t, -1)
        a_f, b_f = frames[k - 1], frames[k]
        a_map = {tr.id: (tr, a_f.polys[i]) for i, tr in enumerate(a_f.trees)}
        b_map = {tr.id: (tr, b_f.polys[i]) for i, tr in enumerate(b_f.trees)}
        for tid, (a, pa) in a_map.items():
            if tid == tgt:
                continue
            if tid not in b_map:
                events.append({"t": round(b_f.snap.t, 3), "id": tid, "target": tgt, "kind": "vanished"})
            elif _tree_changed(a, b_map[tid][0], pa, b_map[tid][1], cfg):
                events.append({"t": round(b_f.snap.t, 3), "id": tid, "target": tgt, "kind": "changed"})
        for tid in b_map:
            if tid not in a_map and tid != tgt:
                events.append({"t": round(b_f.snap.t, 3), "id": tid, "target": tgt, "kind": "appeared"})
    out["descent_frames"] = int(sum(mask))
    out["cruise_altitude_m"] = cruise_altitude(mission, cfg)
    out["nontarget_changes_in_descent"] = len(events)
    out["nontarget_change_events"] = events[:50]
    out["total_violations"] = (undefined_frames + len(new_after) + len(events) + len(sprayed_changes)
                               + len(new_in_sprayed))
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


def timeliness_metrics(frames: List[Frame], gt: List[GroundTruthTree], mission: Mission,
                       camera: Optional[dict], cfg: ScoreConfig) -> dict:
    first_pub: Dict[int, float] = {}
    for f in frames:
        for j in range(len(gt)):
            if gt[j].id in first_pub:
                continue
            if len(f.trees) and (f.cov[:, j] >= cfg.cover_min).any():
                first_pub[gt[j].id] = f.snap.t
    k = footprint_radius_per_m(camera)
    first_view: Dict[int, float] = {}
    first_view_scan: Dict[int, float] = {}
    if k and mission.pose:
        for t, x, y, z in mission.pose:
            ph = step_lookup(mission.phases, t) if mission.phases else "SCAN"
            for g in gt:
                rng = z - g.height
                if rng <= 0 or math.hypot(x - g.centre[0], y - g.centre[1]) > k * rng:
                    continue
                first_view.setdefault(g.id, t)
                if ph in cfg.scan_phases:
                    first_view_scan.setdefault(g.id, t)

    def lat(ref):
        return {g.id: first_pub[g.id] - ref[g.id] for g in gt if g.id in first_pub and g.id in ref}
    la, ls = lat(first_view), lat(first_view_scan)
    per = [{"truth_id": g.id, "first_in_view_t": first_view.get(g.id),
            "first_in_view_scan_t": first_view_scan.get(g.id), "first_published_t": first_pub.get(g.id),
            "latency_s": None if g.id not in la else round(la[g.id], 3),
            "latency_scan_s": None if g.id not in ls else round(ls[g.id], 3)} for g in gt]
    stat = lambda v: (float(np.mean(v)) if v else None, float(np.max(v)) if v else None)
    m_any, x_any = stat(list(la.values()))
    m_scan, x_scan = stat(list(ls.values()))
    return {"definition": ("first snapshot covering the tree (cov >= cover_min) minus the first time its centre "
                           "came within the nadir footprint's half-diagonal (yaw-free bound); 'scan' variant counts view "
                           "time only while the phase is SCAN") if k else
                          "camera/pose not given: only absolute first-publish times",
            "trees_never_published": [g.id for g in gt if g.id not in first_pub],
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
    fin = res.get("final")
    pf = res.get("per_frame", {})
    pp = res.get("per_frame_published", {})
    if fin:
        d, g, sp = fin["detection"], fin["geometry"], fin["spray"]

        def pm(grp, k):
            a = pf.get(grp, {}).get(k)
            b = pp.get(grp, {}).get(k)
            return "%s / %s" % (_f(a["mean"]) if a else "n/a", _f(b["mean"]) if b else "n/a")
        pd, pg = "detection", "geometry"
        L += ["Final map at t = %.2f s. Per-frame column = mean over every snapshot of the stream (%d) / "
              "mean over snapshots from the first non-empty one (%d)." %
              (fin["t"], pf.get("n_frames", 0), pp.get("n_frames", 0)), "",
              "## Detection", "", "| Metric | Final map | Per-frame mean (all / from first publish) |", "|---|---|---|",
              "| Trees in truth / published / matched | %d / %d / %d | — |" % (d["n_truth"], d["n_published"], d["matched"]),
              "| Recall | %s | %s |" % (_f(d["recall"]), pm(pd, "recall")),
              "| Precision | %s | %s |" % (_f(d["precision"]), pm(pd, "precision")),
              "| Missed (absorbed by a merge / unseen) | %d (%d / %d) | %s |" % (d["missed"], d["missed_absorbed"], d["missed_unseen"], pm(pd, "missed")),
              "| Merged entries / trees inside them / neighbours swallowed | %d / %d / %d | %s (swallowed) |" % (d["merged_entries"], d["trees_in_merged_entries"], d["wrong_merges"], pm(pd, "wrong_merges")),
              "| Split trees (duplicates / fragments) | %d (%d / %d) | %s |" % (d["split_trees"], d["duplicates"], d["fragments"], pm(pd, "split_trees")),
              "| False trees | %d | %s |" % (d["false_trees"], pm(pd, "false_trees")),
              "", "Missed truth IDs: %s. Merged entries (map id: truth ids covered): %s." % (d["missed_ids"], d["merged_entry_ids"]), "",
              "## Geometry (matched pairs)", "", "| Metric | Final map | Per-frame mean (all / from first publish) |", "|---|---|---|",
              "| Mean IoU (min) | %s (%s) | %s |" % (_f(g["mean_iou"]), _f(g["min_iou"]), pm(pg, "mean_iou")),
              "| Centre error mean / RMSE / max (m) | %s / %s / %s | %s RMSE |" % (_f(g["centre_mean_m"]), _f(g["centre_rmse_m"]), _f(g["centre_max_m"]), pm(pg, "centre_rmse_m")),
              "| Height bias / RMSE / max abs (m) | %s / %s / %s | %s RMSE |" % (_f(g["height_bias_m"]), _f(g["height_rmse_m"]), _f(g["height_max_abs_m"]), pm(pg, "height_rmse_m")),
              "| Spill outside all true canopies | %s | %s |" % (_f(g["spill"]), pm(pg, "spill")),
              "| Neighbour spill mean / max | %s / %s | %s |" % (_f(g["neighbour_spill_mean"]), _f(g["neighbour_spill_max"]), pm(pg, "neighbour_spill_mean")),
              "", "| Truth id | Map id | IoU | Centre err (m) | Height err (m) | Neighbour spill |", "|---|---|---|---|---|---|"]
        for r in g["per_tree"]:
            L.append("| %d | %d | %s | %s | %s | %s |" % (r["truth_id"], r["map_id"], _f(r["iou"]), _f(r["centre_err_m"]), _f(r["height_err_m"]), _f(r["neighbour_spill"])))
        L += ["", "## Spray outcome (final map)", "",
              "Sprayed entries %d; truth trees sprayed %d of %d; sprayed twice %d; orphan sprayed entries %d; not sprayed: %s. (%s.)" %
              (sp["sprayed_entries"], sp["truth_trees_sprayed"], d["n_truth"], sp["truth_trees_sprayed_twice"], sp["orphan_sprayed_entries"], sp["truth_trees_not_sprayed"], sp["definition"]), ""]
    st, sa, ti, co = res["stability"], res["safety"], res["timeliness"], res["cost"]
    L += ["## Stability", "", "| Metric | Value |", "|---|---|",
          "| ID switches, total / after the list locks (t = %s s) | %d / %d |" % (_f(st["first_lock_t"]), st["id_switches"], st["id_switches_after_lock"]),
          "| ID meaning changes, total / after lock | %d / %d |" % (st["id_meaning_changes"], st["id_meaning_changes_after_lock"]),
          "| ID reappearances / reuse | %d / %d |" % (st["id_reappearances"], st["id_reuse"]),
          "| Geometry retreats (>%.0f%% of outline lost in one update); max lost fraction | %d; %s |" % (res["config"]["retreat_tol"] * 100, st["geometry_retreats"], _f(st["max_lost_fraction"], 3)),
          "| Centre jumps (> %.1f m) | %d |" % (res["config"]["jump_m"], st["centre_jumps"]),
          "| Height jumps (> %.1f m) | %d |" % (res["config"]["height_jump_m"], st["height_jumps"]),
          "| IDs vanished (of which held by navigation) | %s (%s) |" % (st["ids_vanished"], st["ids_vanished_while_held_by_navigation"]),
          "| Truth trees ever split into >= 2 entries | %s |" % st.get("truth_trees_ever_split"),
          "| Truth trees ever inside a merged entry | %s |" % st.get("truth_trees_ever_in_a_merged_entry"),
          "", "## Safety (all must be 0)", "", "| Metric | Value |", "|---|---|",
          "| Frames publishing an undefined height | %d |" % sa["undefined_height_frames"],
          "| New trees while the list is locked (%s) | %s |" % ("/".join(res["config"]["locked_phases"]), _f(sa.get("new_trees_after_scan"))),
          "| Non-target changes during descent (%s frames judged) | %s |" % (_f(sa.get("descent_frames")), _f(sa.get("nontarget_changes_in_descent"))),
          "| Sprayed tree changed after spraying | %d |" % sa["sprayed_tree_changes"],
          "| New tree centred inside a sprayed tree | %d |" % sa["new_tree_inside_sprayed"],
          "| **Total violations** | **%d** |" % sa["total_violations"], ""]
    for key in ("new_tree_after_scan_events", "nontarget_change_events", "sprayed_tree_change_events",
                "new_tree_inside_sprayed_events", "undefined_height_events", "id_switch_events",
                "id_meaning_change_events", "retreat_events", "centre_jump_events", "height_jump_events"):
        ev = sa.get(key) if key in sa else st.get(key)
        if ev:
            L.append("- `%s` (first %d): %s" % (key, min(len(ev), 12), json.dumps(ev[:12])))
    L += ["", "## Timeliness", "", ti["definition"] + ".", "",
          "Latency mean / max: %s s / %s s from first view in any phase; %s s / %s s from first view while scanning. Never published: %s." % (_f(ti["latency_mean_s"]), _f(ti["latency_max_s"]), _f(ti.get("latency_scan_mean_s")), _f(ti.get("latency_scan_max_s")), ti["trees_never_published"]), "",
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
    ap = argparse.ArgumentParser(description="Score a map stream (output contract) against ground truth.")
    ap.add_argument("--map", required=True, help="map stream .jsonl (contract format)")
    ap.add_argument("--gt", required=True, help="ground_truth.json")
    ap.add_argument("--mission", help="mission.json (phases, targets, pose)")
    ap.add_argument("--camera", help="camera_info.json (for timeliness)")
    ap.add_argument("--title", default="Scorecard")
    ap.add_argument("--out-json")
    ap.add_argument("--out-md")
    a = ap.parse_args(argv)
    snaps = read_snapshots(a.map)
    gt = read_ground_truth(a.gt)
    mission = read_mission(a.mission)
    camera = json.load(open(a.camera)) if a.camera else None
    res = score(snaps, gt, mission, camera)
    res["inputs"] = {"map": a.map, "gt": a.gt, "mission": a.mission, "camera": a.camera}
    md = to_markdown(res, a.title)
    if a.out_json:
        with open(a.out_json, "w") as f:
            json.dump(_nan_to_none(res), f, indent=1, default=_json_default)
    if a.out_md:
        with open(a.out_md, "w") as f:
            f.write(md)
    print(md)


if __name__ == "__main__":
    main()
