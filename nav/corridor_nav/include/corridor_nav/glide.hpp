// corridor_nav: terrain-following glide planner. A path between two spray points that never goes below a tree's
// clearance requirement: the straight 3D segment when it is clear, else up-over-down. Pure functions, no ROS.
#pragma once

#include <map>
#include <vector>

#include "corridor_nav/config.hpp"
#include "corridor_nav/types.hpp"
#include "treemapper/types.hpp"

namespace corridor_nav {

/// Polyline in world coordinates. `climbed` = the up-over-down detour was needed.
struct Path3 {
  std::vector<Point3> pts;
  bool climbed{false};
};

/// Distance from `p` to the polygon (either winding): 0 inside or on the boundary, else to the nearest edge.
/// Infinity for an empty polygon.
double distanceToPolygon(const treemapper::Polygon2& poly, const treemapper::Point2& p);

/// Spray point of `target`: (centre, height + spray_clearance), raised to height_k + transit_clearance for every other
/// tree k of `map` whose outline is within drone_radius + lateral_margin of the target centre (a tree the mapper
/// split in two). Counted `spray_point_raised`. `counters` may be null.
Point3 sprayPoint(const treemapper::TreeEntry& target, const std::vector<treemapper::TreeEntry>& map,
                  const Config& config, Counters* counters);

/// Path from `from` (at the spray point of tree `from_id`, or any point) to the spray point of `to`.
/// The xy segment is sampled every 0.05 m; where a tree outline is within drone_radius + lateral_margin the path must
/// be at or above height + transit_clearance (the `to` and `from_id` trees: height + spray_clearance - descent_slack).
/// If the straight segment violates that, the path is [from, (from.xy, z_up), (to.xy, z_up), p_to] with
/// z_up = max(violated requirements, from.z, p_to.z), zero-length legs dropped, counted `glide_climbs_inserted`.
/// `counters` may be null.
Path3 planGlide(const Point3& from, treemapper::TreeId from_id, const treemapper::TreeEntry& to,
                const std::vector<treemapper::TreeEntry>& map, const Config& config, Counters* counters);

/// Flight time of the path: per segment max(dxy / max_speed_xy, |dz| / max_speed_z), summed.
double pathTime(const Path3& path, const Config& config);

/// Setpoint to stream: project `pos` onto the path, advance carrot_lead_s of travel time along it, clamp at the end.
Point3 carrot(const Path3& path, const Point3& pos, const Config& config);

/// True when any planned tree's published height moved by more than replan_dz, or the tree left the map.
bool needsReplan(const std::map<treemapper::TreeId, double>& planned_heights,
                 const std::vector<treemapper::TreeEntry>& map, const Config& config);

}  // namespace corridor_nav
