# One tree of the map, as navigation consumes it (PROJECT_BRIEF section 4).
# All geometry is in the map frame given by the enclosing TreeMap header
# (world_nwu on real logs, world in the Isaac sim; a configuration choice).

int32 id                          # Stable tree ID. Never changes meaning, never reused for another tree.
geometry_msgs/Point[] outline     # Ground footprint (canopy seen from above), >= 3 vertices, CCW, z = 0.
geometry_msgs/Point centre        # Point to fly to (x, y; z = 0).
float64 height                    # Canopy top as a WORLD-FRAME z coordinate (m) in the map frame of the TreeMap
                                  # header, NOT a height above ground: the 0.99 quantile of the fused world z of
                                  # the tree's latched canopy cells. Datum = the world frame's origin. On the real
                                  # drone world_nwu re-origins at every boot (origin = power-on position), so the
                                  # value is only comparable with poses of the same session; above-ground height =
                                  # height - local ground z. Descent reference: compare with the drone's world z.
                                  # Always finite: a tree is not published before it has a valid height.
bool sprayed                      # Set by navigation through SetTreeSprayed. Sprayed trees no longer change.

# Diagnostics (navigation may ignore these):
bool locked                       # True once the tree list is fixed (planning/hopping); geometry may still grow.
float32 confidence                # RESERVED, always 0: no mapper provides a confidence (publication itself is the
                                  # confidence decision, see docs/DESIGN.md section 2.4).
uint32 observations               # Frames in which a crown/observation was matched to this tree (cand_obj: frames
                                  # with a matched crown, max of both sides after a merge). 0 = not provided by
                                  # the mapper in use.
time first_published              # When the tree first appeared in the map.
time last_updated                 # Last time its outline or height changed.
