// Plain data types of the tree map: the output contract of PROJECT_BRIEF section 4.
// No ROS dependency. All geometry is in the fixed world frame (metres).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace treemapper {

struct Point2 {
  double x{0.0};
  double y{0.0};
};

/// Simple polygon, counter-clockwise, no repeated closing vertex.
using Polygon2 = std::vector<Point2>;

/// Stable tree identifier. Never reused for a different tree (rule 1).
using TreeId = std::int32_t;
constexpr TreeId kNoTree = -1;

/// One map entry as navigation sees it.
struct TreeEntry {
  TreeId id{kNoTree};
  Polygon2 outline;      ///< ground footprint seen from above, world frame
  Point2 centre;         ///< point navigation flies to
  double height{0.0};    ///< canopy TOP as world-frame z (m), not height above ground: the 0.99 quantile of the
                         ///< fused world z of the latched canopy cells. Datum = world-frame origin (world_nwu
                         ///< re-origins at every boot on the real drone). Descent reference; finite once published.
  int observations{0};   ///< frames with a matched observation (diagnostic; 0 = not provided by the mapper)
  bool sprayed{false};   ///< set by navigation via markSprayed()
};

/// Mission phase reported by navigation (/mission_state).
enum class MissionPhase : std::uint8_t {
  kUnknown = 0,  ///< missing or unrecognised: behave as scanning (brief section 6) and warn
  kIdle,
  kScanApproach,
  kScan,
  kPlanning,
  kHopping,
  kSpraying,
  kReturn,
  kError,
};

/// Parse navigation's phase string ("SCAN", "HOPPING", ...). Unknown strings -> kUnknown.
MissionPhase parseMissionPhase(const std::string& s);
const char* toString(MissionPhase p);

/// Drone pose in the world frame.
struct DronePose {
  double t{0.0};
  double x{0.0}, y{0.0}, z{0.0};             ///< world-frame position; z = world z (not height above ground)
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
  bool valid{false};
};

/// One 2D detection box in pixel coordinates of the (left) image.
struct Detection2D {
  double x{0.0}, y{0.0}, width{0.0}, height{0.0};
  double score{0.0};
  int label{0};
};

}  // namespace treemapper
