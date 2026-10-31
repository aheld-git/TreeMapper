"""USD prim paths shared by scene.py (which creates them) and nozzle.py (which reads them back). Pure Python."""
import re
from typing import Optional

COLLIDER_ROOT = "/World/Colliders"
COLLISION_GROUP = "/World/CollisionGroups/canopy"
_COLLIDER_RE = re.compile(r"^" + re.escape(COLLIDER_ROOT) + r"/tree_(\d+)(/|$)")


def canopy_collider_path(tree_id: int) -> str:
    """Prim path of the invisible convex-hull collider of tree ``tree_id``."""
    return "%s/tree_%d/hull" % (COLLIDER_ROOT, int(tree_id))


def tree_id_from_collider_path(path: str) -> Optional[int]:
    """Tree id of a canopy collider prim path (or one of its descendants); None for anything else."""
    m = _COLLIDER_RE.match(path or "")
    return int(m.group(1)) if m else None
