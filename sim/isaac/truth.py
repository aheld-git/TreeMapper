"""Ground truth of a placed tree, from the world-space points of its canopy mesh.

Pure Python + numpy (no Isaac imports). ``write_truth`` writes ``truth.json``
(the sim's own record) and, next to it, ``ground_truth.json`` in the scorer's
contract format (``treemapper_tools.contract``).
"""
import json
import math
import os
from typing import Any, Dict, List, Sequence

import numpy as np

from treemapper_tools import contract


def _convex_hull(pts: np.ndarray) -> np.ndarray:
    """Andrew's monotone chain; counter-clockwise, collinear points dropped."""
    p = np.unique(np.round(np.asarray(pts, dtype=float), 9), axis=0)  # sorted by x, then y
    if len(p) < 3:
        return p

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []  # type: List[Any]
    for q in p:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    upper = []  # type: List[Any]
    for q in p[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.array(lower[:-1] + upper[:-1])


def tree_truth(placement, canopy_points_world: np.ndarray) -> Dict[str, Any]:
    """{id, species, centre, outline, radius, top_z, ground_z} of one placed tree."""
    pts = np.asarray(canopy_points_world, dtype=float).reshape(-1, 3)
    if len(pts) < 3 or not np.all(np.isfinite(pts)):
        raise ValueError("tree %s: need >= 3 finite canopy points" % placement.id)
    hull = _convex_hull(pts[:, :2])
    if len(hull) < 3:
        raise ValueError("tree %s: canopy points are degenerate (hull has %d vertices)"
                         % (placement.id, len(hull)))
    x, y = hull[:, 0], hull[:, 1]
    area = 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    outline = [(float(a), float(b)) for a, b in hull]
    cx, cy = contract.polygon_centroid(outline)
    return {
        "id": int(placement.id),
        "species": placement.species,
        "centre": [float(cx), float(cy)],
        "outline": [[a, b] for a, b in outline],
        "radius": math.sqrt(abs(area) / math.pi),
        "top_z": float(pts[:, 2].max()),
        "ground_z": float(placement.z_ground),
    }


def write_truth(path: str, trees: Sequence[Dict[str, Any]], frame: str = "world") -> None:
    """Write ``path`` (truth.json) and ``ground_truth.json`` beside it (scorer contract)."""
    d = os.path.dirname(os.path.abspath(path))
    with open(path, "w") as f:
        json.dump({"frame": frame, "trees": list(trees)}, f, indent=1)
    gt = [contract.GroundTruthTree(t["id"], [tuple(p) for p in t["outline"]], tuple(t["centre"]),
                                   t["radius"], t["top_z"]) for t in trees]
    gt_path = os.path.join(d, "ground_truth.json")
    contract.write_ground_truth(gt_path, gt, frame=frame)
    with open(gt_path) as f:
        doc = json.load(f)
    doc["height_reliable"] = True  # heights are measured from the placed mesh
    with open(gt_path, "w") as f:
        json.dump(doc, f, indent=1)
