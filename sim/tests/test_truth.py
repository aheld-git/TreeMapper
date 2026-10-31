import json
import math
import os

import numpy as np
import pytest

from isaac.layout import Placement
from isaac import truth
from treemapper_tools import contract


def _sphere(cx, cy, cz, r, n=3000):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([cx + r * np.cos(th) * np.sin(phi),
                     cy + r * np.sin(th) * np.sin(phi),
                     cz + r * np.cos(phi)], axis=1)


def _placement(i=1):
    return Placement(id=i, species="orange", x=3.0, y=4.0, z_ground=0.2, yaw=0.0, scale=1.0)


def test_truth_from_points():
    t = truth.tree_truth(_placement(), _sphere(3, 4, 5, 2))
    assert t["id"] == 1 and t["species"] == "orange"
    assert t["centre"][0] == pytest.approx(3.0, abs=0.05)
    assert t["centre"][1] == pytest.approx(4.0, abs=0.05)
    assert t["radius"] == pytest.approx(2.0, abs=0.05)
    assert t["top_z"] == pytest.approx(7.0, abs=0.01)
    assert t["ground_z"] == pytest.approx(0.2)
    assert len(t["outline"]) >= 3


def test_hull_ignores_interior_and_collinear():
    pts = np.array([[0, 0, 1], [4, 0, 2], [4, 4, 3], [0, 4, 9], [2, 2, 1], [2, 0, 1], [4, 2, 1.0]])
    t = truth.tree_truth(_placement(), pts)
    assert len(t["outline"]) == 4
    assert t["radius"] == pytest.approx(math.sqrt(16 / math.pi))
    assert t["centre"] == pytest.approx([2.0, 2.0])
    assert t["top_z"] == 9.0


def test_degenerate_points_rejected():
    with pytest.raises(ValueError):
        truth.tree_truth(_placement(), np.array([[0, 0, 1], [1, 1, 1.0]]))


def test_ground_truth_file_reads_with_contract(tmp_path):
    a = truth.tree_truth(_placement(1), _sphere(3, 4, 5, 2))
    b = truth.tree_truth(_placement(2), _sphere(10, 4, 4, 1.5))
    path = str(tmp_path / "truth.json")
    truth.write_truth(path, [a, b])
    assert json.load(open(path))["trees"][0]["top_z"] == a["top_z"]
    gt = contract.read_ground_truth(str(tmp_path / "ground_truth.json"))
    assert gt.height_reliable is True
    assert gt.frame == "world"
    assert [g.id for g in gt] == [1, 2]
    assert gt[0].height == pytest.approx(7.0, abs=0.01)
    assert gt[1].height == pytest.approx(5.5, abs=0.01)
    assert gt[0].radius == pytest.approx(2.0, abs=0.05)
