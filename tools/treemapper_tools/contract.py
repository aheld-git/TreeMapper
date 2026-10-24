"""File formats shared by every system and by the scorer.

This is the *file* form of the output contract (PROJECT_BRIEF section 4): what
any mapping system must emit so the scorer can judge it. All coordinates are in
the fixed world frame of the ground truth (for the Isaac sim: ``world``, ENU).

Map stream (``*.jsonl``): one JSON object per published snapshot, in time order::

    {"t": 81.77,                      # seconds, bag/sim time of publication
     "frame": "world",
     "trees": [
        {"id": 7,                     # stable integer ID
         "outline": [[x, y], ...],    # ground footprint polygon, >= 3 vertices
         "centre": [x, y],            # point navigation flies to
         "height": 5.35,              # canopy height (descent reference); null = undefined
         "sprayed": false}
     ],
     "stage_ms": {"fuse": 1.2}}       # optional: per-stage processing time for cost metrics

Ground truth (``ground_truth.json``)::

    {"frame": "world", "trees": [{"id": 0, "outline": [[x, y], ...],
                                  "centre": [x, y], "radius": 2.46, "height": 4.0}],
     "height_reliable": false}         # optional: false = heights are a fallback, not measured

Mission context (``mission.json``) -- optional; without it phase-dependent safety
metrics report "n/a" (no phase information = scanning, per brief section 6)::

    {"phases":  [[t, "SCAN"], ...],          # /mission_state changes
     "targets": [[t, 9], ...],               # /spray_target changes (-1 = none)
     "pose":    [[t, x, y, z], ...]}         # drone position, world frame, z = height
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def nwu_to_world(x: float, y: float) -> Point:
    """world_nwu -> sim world (ENU): (x, y) -> (-y, x). See DATA_README."""
    return (-y, x)


def polygon_centroid(pts: Sequence[Point]) -> Point:
    """Area-weighted centroid; falls back to the vertex mean for degenerate input."""
    n = len(pts)
    if n == 0:
        return (float("nan"), float("nan"))
    a = cx = cy = 0.0
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        cr = x0 * y1 - x1 * y0
        a += cr
        cx += (x0 + x1) * cr
        cy += (y0 + y1) * cr
    if abs(a) < 1e-12:
        return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n)
    return (cx / (3.0 * a), cy / (3.0 * a))


def _num(v) -> Optional[float]:
    """JSON-safe float: non-finite -> None (undefined)."""
    if v is None:
        return None
    v = float(v)
    return v if math.isfinite(v) else None


@dataclass
class Tree:
    id: int
    outline: List[Point]
    centre: Point
    height: Optional[float]  # None = undefined (NaN/inf/missing)
    sprayed: bool = False

    def to_json(self) -> dict:
        return {"id": int(self.id),
                "outline": [[round(x, 4), round(y, 4)] for x, y in self.outline],
                "centre": [round(self.centre[0], 4), round(self.centre[1], 4)],
                "height": _num(self.height),
                "sprayed": bool(self.sprayed)}

    @staticmethod
    def from_json(d: dict) -> "Tree":
        outline = [(float(p[0]), float(p[1])) for p in d.get("outline", [])]
        c = d.get("centre")
        centre = (float(c[0]), float(c[1])) if c is not None else polygon_centroid(outline)
        return Tree(int(d["id"]), outline, centre, _num(d.get("height")), bool(d.get("sprayed", False)))


@dataclass
class Snapshot:
    t: float
    trees: List[Tree]
    frame: str = "world"
    stage_ms: Dict[str, float] = field(default_factory=dict)

    def to_json(self) -> dict:
        d = {"t": round(self.t, 6), "frame": self.frame, "trees": [tr.to_json() for tr in self.trees]}
        if self.stage_ms:
            d["stage_ms"] = self.stage_ms
        return d

    @staticmethod
    def from_json(d: dict) -> "Snapshot":
        return Snapshot(float(d["t"]), [Tree.from_json(x) for x in d.get("trees", [])],
                        d.get("frame", "world"), dict(d.get("stage_ms", {}) or {}))


@dataclass
class GroundTruthTree:
    id: int
    outline: List[Point]
    centre: Point
    radius: float
    height: float


class GroundTruthList(list):
    """List of ``GroundTruthTree`` that remembers the frame its file declared (the scorer
    raises when the map stream's frame differs) and the optional ``height_reliable`` flag
    (False = the truth heights are a fallback, not measured canopy heights; None = unknown)."""

    def __init__(self, trees=(), frame: str = "world", height_reliable: Optional[bool] = None):
        super().__init__(trees)
        self.frame = frame
        self.height_reliable = height_reliable


@dataclass
class Mission:
    phases: List[Tuple[float, str]] = field(default_factory=list)
    targets: List[Tuple[float, int]] = field(default_factory=list)
    pose: List[Tuple[float, float, float, float]] = field(default_factory=list)

    @property
    def has_phases(self) -> bool:
        return bool(self.phases)

    @property
    def has_targets(self) -> bool:
        return bool(self.targets)


def write_snapshots(path: str, snaps: Iterable[Snapshot]) -> int:
    n = 0
    with open(path, "w") as f:
        for s in snaps:
            f.write(json.dumps(s.to_json(), separators=(",", ":")) + "\n")
            n += 1
    return n


def read_snapshots(path: str) -> List[Snapshot]:
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(Snapshot.from_json(json.loads(line)))
    out.sort(key=lambda s: s.t)
    return out


def write_ground_truth(path: str, trees: Sequence[GroundTruthTree], frame: str = "world") -> None:
    with open(path, "w") as f:
        json.dump({"frame": frame, "trees": [
            {"id": g.id, "outline": [[round(x, 4), round(y, 4)] for x, y in g.outline],
             "centre": [round(g.centre[0], 4), round(g.centre[1], 4)],
             "radius": round(g.radius, 4), "height": g.height} for g in trees]}, f, indent=1)


def read_ground_truth(path: str) -> "GroundTruthList":
    with open(path) as f:
        d = json.load(f)
    out = []
    for g in d["trees"]:
        outline = [(float(p[0]), float(p[1])) for p in g["outline"]]
        c = g.get("centre") or polygon_centroid(outline)
        out.append(GroundTruthTree(int(g["id"]), outline, (float(c[0]), float(c[1])),
                                   float(g.get("radius", 0.0)), float(g["height"])))
    hr = d.get("height_reliable")
    return GroundTruthList(out, frame=str(d.get("frame", "world")),
                           height_reliable=None if hr is None else bool(hr))


def write_mission(path: str, m: Mission) -> None:
    with open(path, "w") as f:
        json.dump({"phases": [[round(t, 6), s] for t, s in m.phases],
                   "targets": [[round(t, 6), int(i)] for t, i in m.targets],
                   "pose": [[round(v, 4) for v in p] for p in m.pose]}, f, separators=(",", ":"))


def read_mission(path: Optional[str]) -> Mission:
    if not path:
        return Mission()
    with open(path) as f:
        d = json.load(f)
    # Every series is a piecewise-constant / sampled signal looked up by time (step_lookup,
    # bisect): sort by time (stable, so equal stamps keep file order) instead of trusting the file.
    by_t = lambda r: r[0]
    return Mission(sorted([(float(t), str(s)) for t, s in d.get("phases", [])], key=by_t),
                   sorted([(float(t), int(i)) for t, i in d.get("targets", [])], key=by_t),
                   sorted([tuple(float(v) for v in p) for p in d.get("pose", [])], key=by_t))


def step_lookup(series: Sequence[Tuple[float, object]], t: float, default=None):
    """Value of a piecewise-constant (time, value) series at time t (last change <= t)."""
    lo, hi = 0, len(series)
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    return series[lo - 1][1] if lo > 0 else default
