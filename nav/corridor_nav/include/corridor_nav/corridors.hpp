// corridor_nav: corridor planner. Cuts the field into equal strips across the row heading; each strip's scan
// line is the part of its centre line that lies inside the field. Pure functions, no ROS.
#pragma once

#include <string>
#include <vector>

#include "corridor_nav/config.hpp"
#include "corridor_nav/types.hpp"

namespace corridor_nav {

/// Direction (rad, in [0, pi)) of the long side of the polygon's minimum-area bounding rectangle
/// (rotating calipers over the convex hull's edges). 0 for a degenerate polygon.
double minAreaRectHeading(const treemapper::Polygon2& field);

/// Widest strip that keeps a whole crown in view when its centre is inside the strip:
/// 2 (z_scan - expected_canopy_top_max) tan(min(hfov, vfov) / 2) - 2 expected_canopy_radius_max - w_margin.
double maxCorridorWidth(const CameraFov& fov, const Config& config);

struct CorridorPlan {
  std::vector<Corridor> corridors;
  double width{0.0};   ///< m, the width chosen (strips are field_extent / ceil(field_extent / width) <= width wide)
  double w_max{0.0};   ///< m, maxCorridorWidth
  bool ok{false};
  std::string error;   ///< "coverage impossible" | "field invalid" when !ok
};

/// Boustrophedon corridors over `field`. Heading = field.heading_rad, else minAreaRectHeading(polygon).
/// Corridor k is scanned along the heading for even k and against it for odd k. `counters` may be null.
/// Counts: corridor_width_clamped, corridors_dropped_short.
CorridorPlan planCorridors(const Field& field, const CameraFov& fov, const Config& config, Counters* counters);

}  // namespace corridor_nav
