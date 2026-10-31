"""Seeded orchard layout: where every tree stands, before any mesh is loaded.

Pure Python + numpy (no Isaac imports); runs under Python 3.8 and Isaac's 3.11.
Rows run along ``rows.heading_rad``; ``rows.count`` rows of ``rows.trees_per_row``
slots. A dropped (missing) tree keeps its slot, ids are given to existing trees
only, in row-major order, starting at 1.
"""
import json
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

import numpy as np


@dataclass
class Placement:
    id: int
    species: str
    x: float
    y: float
    z_ground: float
    yaw: float
    scale: float


# (dotted key, expected kind). Optional blocks (clutter, nozzle, assets_dir) are not listed.
_REQUIRED = [
    ("seed", "int"),
    ("field", "dict"),
    ("field.polygon", "polygon"),
    ("rows", "dict"),
    ("rows.count", "posint"),
    ("rows.trees_per_row", "posint"),
    ("rows.row_spacing", "pos"),
    ("rows.tree_spacing", "pos"),
    ("rows.heading_rad", "num"),
    ("rows.origin", "pair"),
    ("rows.jitter", "nonneg"),
    ("rows.missing_fraction", "frac"),
    ("species", "weights"),
    ("scale_range", "range"),
    ("terrain", "dict"),
    ("terrain.slope_rad", "num"),
    ("terrain.slope_dir_rad", "num"),
    ("drone", "dict"),
    ("drone.start", "triple"),
    ("drone.home", "pair"),
]


def _is_num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _check(kind: str, v: Any) -> bool:
    if kind == "dict":
        return isinstance(v, dict)
    if kind == "int":
        return isinstance(v, int) and not isinstance(v, bool)
    if kind == "posint":
        return isinstance(v, int) and not isinstance(v, bool) and v >= 1
    if kind == "num":
        return _is_num(v)
    if kind == "pos":
        return _is_num(v) and v > 0
    if kind == "nonneg":
        return _is_num(v) and v >= 0
    if kind == "frac":
        return _is_num(v) and 0 <= v < 1
    if kind in ("pair", "triple"):
        n = 2 if kind == "pair" else 3
        return isinstance(v, (list, tuple)) and len(v) == n and all(_is_num(e) for e in v)
    if kind == "range":
        return (isinstance(v, (list, tuple)) and len(v) == 2 and all(_is_num(e) for e in v)
                and 0 < v[0] <= v[1])
    if kind == "polygon":
        return (isinstance(v, (list, tuple)) and len(v) >= 3 and
                all(isinstance(p, (list, tuple)) and len(p) == 2 and all(_is_num(e) for e in p) for p in v))
    if kind == "weights":
        return (isinstance(v, dict) and len(v) > 0 and all(_is_num(w) and w >= 0 for w in v.values())
                and sum(v.values()) > 0)
    raise AssertionError(kind)


def validate_spec(spec: Dict[str, Any]) -> None:
    """Raise ValueError naming the first missing or malformed key (dotted path)."""
    for key, kind in _REQUIRED:
        v = spec
        for part in key.split("."):
            if not isinstance(v, dict) or part not in v:
                raise ValueError("scene spec: missing key '%s'" % key)
            v = v[part]
        if not _check(kind, v):
            raise ValueError("scene spec: bad value for key '%s': %r" % (key, v))


def load_spec(path: str) -> Dict[str, Any]:
    with open(path) as f:
        spec = json.load(f)
    if not isinstance(spec, dict):
        raise ValueError("scene spec: top level must be an object")
    validate_spec(spec)
    return spec


def ground_z(spec: Dict[str, Any], x: float, y: float) -> float:
    """Terrain height on the sloped plane z = tan(slope) * (x cos(dir) + y sin(dir))."""
    t = spec["terrain"]
    d = float(t["slope_dir_rad"])
    return math.tan(float(t["slope_rad"])) * (x * math.cos(d) + y * math.sin(d))


def plan_layout(spec: Dict[str, Any]) -> List[Placement]:
    validate_spec(spec)
    rows = spec["rows"]
    rng = np.random.default_rng(int(spec["seed"]))
    heading = float(rows["heading_rad"])
    ux, uy = math.cos(heading), math.sin(heading)  # along a row
    vx, vy = -uy, ux                                # across rows
    ox, oy = float(rows["origin"][0]), float(rows["origin"][1])
    jitter = float(rows["jitter"])
    missing = float(rows["missing_fraction"])
    names = sorted(spec["species"])
    weights = np.array([float(spec["species"][n]) for n in names])
    weights = weights / weights.sum()
    lo, hi = float(spec["scale_range"][0]), float(spec["scale_range"][1])

    out = []  # type: List[Placement]
    next_id = 1
    for r in range(int(rows["count"])):
        for c in range(int(rows["trees_per_row"])):
            # Every slot draws the same number of variates, so dropping one tree
            # does not shift the random stream of the others.
            jx, jy = rng.uniform(-jitter, jitter, size=2) if jitter > 0 else (0.0, 0.0)
            dropped = rng.random() < missing
            sp = names[int(rng.choice(len(names), p=weights))]
            scale = float(rng.uniform(lo, hi))
            yaw = float(rng.uniform(0.0, 2.0 * math.pi))
            if dropped:
                continue
            a = c * float(rows["tree_spacing"])
            b = r * float(rows["row_spacing"])
            x = ox + a * ux + b * vx + float(jx)
            y = oy + a * uy + b * vy + float(jy)
            out.append(Placement(next_id, sp, x, y, ground_z(spec, x, y), yaw, scale))
            next_id += 1
    return out


def extent(spec: Dict[str, Any]) -> Tuple[float, float, float, float]:
    """Nominal (jitter-free) slot bounds (xmin, ymin, xmax, ymax) of the tree grid."""
    rows = spec["rows"]
    h = float(rows["heading_rad"])
    ux, uy, vx, vy = math.cos(h), math.sin(h), -math.sin(h), math.cos(h)
    la = (int(rows["trees_per_row"]) - 1) * float(rows["tree_spacing"])
    lb = (int(rows["count"]) - 1) * float(rows["row_spacing"])
    ox, oy = float(rows["origin"][0]), float(rows["origin"][1])
    xs = [ox + a * ux + b * vx for a in (0, la) for b in (0, lb)]
    ys = [oy + a * uy + b * vy for a in (0, la) for b in (0, lb)]
    return (min(xs), min(ys), max(xs), max(ys))
