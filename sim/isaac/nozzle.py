"""Isaac-only: nozzle ray casts against the canopy colliders.

While ``NozzleGate.is_open(now)`` every physics step casts ``rays`` rays (uniform cone from
``nozzle_model.cone_directions``, rotated by the body attitude) from ``body pose o offset_body`` with the
PhysX scene-query interface (max 30 m). Colliders in the scene are the canopy hulls only (no ground or trunk
collider), so a ray that hits a collider hits a canopy; the hit prim path is mapped to the tree id
(``paths.tree_id_from_collider_path``) and anything else, or no hit, is booked to the ground.
"""
import math
from typing import Dict, List, Optional, Sequence

import numpy as np

from isaac import frames, paths
from isaac.nozzle_model import Deposition, NozzleGate, cone_directions

MAX_RANGE_M = 30.0


class NozzleCaster:
    def __init__(self, nozzle_spec: Dict, gate_timeout: float = 0.5, seed: int = 0):
        import carb  # noqa: F401  (Float3 below)
        from omni.physx import get_physx_scene_query_interface
        self.flow_lps = float(nozzle_spec["flow_lps"])
        self.half_angle = math.radians(float(nozzle_spec["cone_half_angle_deg"]))
        self.rays = int(nozzle_spec["rays"])
        self.offset_body = np.asarray(nozzle_spec["offset_body"], dtype=float)
        if self.offset_body.shape != (3,) or self.rays < 1 or self.flow_lps < 0.0:
            raise ValueError("bad nozzle spec: %r" % (nozzle_spec,))
        self.gate = NozzleGate(gate_timeout)
        self.deposition = Deposition(self.flow_lps)
        self._query = get_physx_scene_query_interface()
        self._seed = int(seed)
        self._n_steps = 0
        self.unknown_hits = 0   # hits on a collider that is not a canopy hull (booked to the ground)

    def on_command(self, open_: bool, now: float) -> None:
        self.gate.on_command(open_, now)

    def hit_tree(self, origin: Sequence[float], direction: Sequence[float]) -> Optional[int]:
        """Tree id the ray hits first, or None (no hit, or a non-canopy collider)."""
        from carb import Float3
        hit = self._query.raycast_closest(Float3(*(float(c) for c in origin)),
                                          Float3(*(float(c) for c in direction)), MAX_RANGE_M)
        if not hit or not hit.get("hit", True):
            return None
        path = hit.get("collision") or hit.get("rigidBody") or ""
        tid = paths.tree_id_from_collider_path(str(path))
        if tid is None:
            self.unknown_hits += 1
        return tid

    def step(self, now: float, dt: float, position: Sequence[float], R_body_to_world: np.ndarray) -> List[Optional[int]]:
        """One physics step. Returns the per-ray hits (empty list if the nozzle is closed)."""
        if not self.gate.is_open(now):
            return []
        origin = np.asarray(position, dtype=float) + np.asarray(R_body_to_world) @ self.offset_body
        # a fresh sample each step (seeded by the step counter) so the cone is covered, not one fixed pattern
        dirs = frames.rotate_cone(cone_directions(self.rays, self.half_angle, seed=self._seed + self._n_steps),
                                  R_body_to_world)
        self._n_steps += 1
        hits = [self.hit_tree(origin, d) for d in dirs]
        self.deposition.add(hits, dt)
        return hits
