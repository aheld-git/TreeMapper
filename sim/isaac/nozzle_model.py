"""Nozzle model: spray cone directions, volume bookkeeping, command watchdog.

Pure Python + numpy (no Isaac imports); runs under Python 3.8 and Isaac's 3.11.
The Isaac side (``nozzle.py``) casts the rays and maps hits to tree ids; this module
only decides where the rays go and where the liquid is booked.
"""
import math
from typing import Dict, Optional, Sequence

import numpy as np


def cone_directions(n: int, half_angle_rad: float, seed: int = 0) -> np.ndarray:
    """``n`` unit vectors in the body frame, uniform over the solid angle of a cone about -z.

    cos(theta) is uniform on [cos(half_angle), 1] and phi is uniform on [0, 2*pi).
    """
    rng = np.random.default_rng(seed)
    cos_t = rng.uniform(math.cos(half_angle_rad), 1.0, size=n)
    phi = rng.uniform(0.0, 2.0 * math.pi, size=n)
    sin_t = np.sqrt(np.maximum(0.0, 1.0 - cos_t * cos_t))
    return np.stack([sin_t * np.cos(phi), sin_t * np.sin(phi), -cos_t], axis=1)


class Deposition:
    """Books the emitted liquid to the tree (or the ground) each ray hit."""

    def __init__(self, flow_lps: float):
        self.flow_lps = flow_lps
        self.open_time = 0.0
        self._emitted = 0.0
        self._ground = 0.0
        self._trees = {}  # type: Dict[int, float]

    def add(self, hits: Sequence[Optional[int]], dt: float) -> None:
        """One open nozzle step: ``hits[i]`` is the tree id ray i hit, or None for the ground.

        Every ray carries an equal share of ``flow_lps * dt``. With no rays at all the
        liquid is booked to the ground so the totals still balance.
        """
        volume = self.flow_lps * dt
        self.open_time += dt
        self._emitted += volume
        n = len(hits)
        if n == 0:
            self._ground += volume
            return
        share = volume / n
        for h in hits:
            if h is None:
                self._ground += share
            else:
                tid = int(h)
                self._trees[tid] = self._trees.get(tid, 0.0) + share

    def to_json(self) -> dict:
        return {
            "trees": {str(k): v for k, v in sorted(self._trees.items())},
            "ground": self._ground,
            "emitted": self._emitted,
            "open_time": self.open_time,
        }


class NozzleGate:
    """Open only while the last command said open and is fresh (watchdog against a dead navigator)."""

    def __init__(self, timeout: float = 0.5):
        self.timeout = timeout
        self._open = False
        self._t_cmd = None  # type: Optional[float]

    def on_command(self, open: bool, now: float) -> None:
        self._open = bool(open)
        self._t_cmd = now

    def is_open(self, now: float) -> bool:
        """Open iff the last command said open and its age ``now - t_cmd`` is in [0, timeout]: a command stamped
        after ``now`` (negative age: a clock that jumped back) is not fresh either."""
        if not self._open or self._t_cmd is None:
            return False
        age = now - self._t_cmd
        return 0.0 <= age <= self.timeout
