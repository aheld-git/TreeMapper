// corridor_nav: pure helpers of the navigator state machine (path building, stop ordering, the trees near a path).
// No state, no ROS; the Navigator owns the map and the config and passes them in.
#pragma once

#include <map>
#include <vector>

#include "corridor_nav/config.hpp"
#include "corridor_nav/glide.hpp"
#include "corridor_nav/ordering.hpp"
#include "corridor_nav/types.hpp"
#include "treemapper/types.hpp"

namespace corridor_nav {
namespace support {

/// A tree the planners can use: finite centre, height and outline, at least a triangle.
bool usable(const treemapper::TreeEntry& t);

/// A plausible target height: published height <= home_z + expected_canopy_top_max + target_height_margin. A crown
/// published higher (a stereo phantom: the closed loop once mapped a 12.98 m "tree" in a 3 m orchard) is not visited.
bool heightPlausible(const treemapper::TreeEntry& t, double home_z, const Config& cfg);

/// Yaw (rad) of the pose's quaternion.
double yawOf(const treemapper::DronePose& p);

double distXY(const Point3& a, const Point3& b);
double dist3(const Point3& a, const Point3& b);

/// Polyline without zero-length legs; always at least two points (a lone point is repeated).
Path3 makePath(const std::vector<Point3>& pts);

/// The entry of `map` with this id, or nullptr.
const treemapper::TreeEntry* findTree(const std::vector<treemapper::TreeEntry>& map, treemapper::TreeId id);

/// Climb vertically to `z_scan_abs` (if below it), then fly level to above `to`.
std::vector<Point3> transitPoints(const Point3& from, const Point3& to, double z_scan_abs);

/// True when the xy segment a -> b (sampled every 0.25 m) comes farther than `tol` outside `field`.
bool leavesField(const treemapper::Polygon2& field, const Point3& a, const Point3& b, double tol);

/// planGlide, except that a glide whose xy segment leaves the field (a non-convex field, cleanup and field_first
/// glides between corridors) goes up-over at z_up = max(z_scan_abs, the glide's points): [from, (from.xy, z_up),
/// (to.xy, z_up), spray point], counted `glide_outside_field_climbs` (outside the field nothing was scanned).
/// "Outside" allows xy_tol, the position tolerance. `counters` may be null.
Path3 glideLeg(const Point3& from, treemapper::TreeId from_id, const treemapper::TreeEntry& to,
               const std::vector<treemapper::TreeEntry>& map, const Config& cfg, const treemapper::Polygon2& field,
               double z_scan_abs, Counters* counters);

/// The unsprayed trees of `ids` that are in `map`, as spray-point stops in flying order from `start` to `end`
/// (orderTargets with cost = glideLeg time, and the transit time for the leg to `end`). The order budget runs on a
/// deterministic proxy clock (n^2 pair evaluations per 2-opt pass), so the same inputs always give the same order.
std::vector<Stop> orderStops(const std::vector<treemapper::TreeId>& ids, const std::vector<treemapper::TreeEntry>& map,
                             const Point3& start, const Point3& end, double z_scan_abs, const Config& cfg,
                             const treemapper::Polygon2& field);

/// Published heights of `target` and of every tree whose outline comes within drone_radius + lateral_margin of the
/// path's xy, for the re-plan trigger (needsReplan).
std::map<treemapper::TreeId, double> heightsNear(const Path3& path, treemapper::TreeId target,
                                                 const std::vector<treemapper::TreeEntry>& map, const Config& cfg);

}  // namespace support
}  // namespace corridor_nav
