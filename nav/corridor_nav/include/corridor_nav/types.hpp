// corridor_nav: plain value types shared by the navigator's pure functions and the ROS wrapper.
// Geometry (Point2, Polygon2, TreeEntry, TreeId, DronePose) comes from the TreeMapper core.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include "treemapper/types.hpp"

namespace corridor_nav {

struct Point3 {
  double x{0.0}, y{0.0}, z{0.0};
};

/// The field to spray: boundary, optional row heading, and the take-off / return point.
struct Field {
  treemapper::Polygon2 polygon;        ///< world xy, counter-clockwise
  std::optional<double> heading_rad;   ///< row direction; absent = derived from the polygon (min-width axis)
  treemapper::Point2 home;             ///< world xy of the take-off point (the scan altitude is above it)
};

/// Camera field of view of the scan camera (full angles, rad).
struct CameraFov {
  double hfov_rad{0.0}, vfov_rad{0.0};
};

/// One scan corridor: a strip of the field flown at z_scan, entered at `entry`, left at `exit`.
struct Corridor {
  int index{0};
  treemapper::Polygon2 strip;          ///< the strip outline, world xy
  treemapper::Point2 entry, exit;      ///< ends of the scan line
  double heading_rad{0.0};             ///< direction entry -> exit
};

/// Named event counters (every hold, drop, skip, retry, climb, guard trigger), published as-is.
using Counters = std::map<std::string, std::uint64_t>;

}  // namespace corridor_nav
