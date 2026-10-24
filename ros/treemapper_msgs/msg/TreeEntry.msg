# One tree of the map, as navigation consumes it (PROJECT_BRIEF section 4).
# All geometry is in the map frame given by the enclosing TreeMap header
# (world_nwu on real logs, world in the Isaac sim; a configuration choice).

int32 id                          # Stable tree ID. Never changes meaning, never reused for another tree.
geometry_msgs/Point[] outline     # Ground footprint (canopy seen from above), >= 3 vertices, CCW, z = 0.
geometry_msgs/Point centre        # Point to fly to (x, y; z = 0).
float64 height                    # Canopy top height above ground (m), the descent reference.
                                  # Always finite: a tree is not published before it has a valid height.
bool sprayed                      # Set by navigation through SetTreeSprayed. Sprayed trees no longer change.

# Diagnostics (navigation may ignore these):
bool locked                       # True once the tree list is fixed (planning/hopping); geometry may still grow.
float32 confidence                # 0..1 confidence that this is one real tree (system-defined).
uint32 observations               # Number of frames that contributed accepted points.
time first_published              # When the tree first appeared in the map.
time last_updated                 # Last time its outline or height changed.
