// Per-frame input of the mapping system (PROJECT_BRIEF section 4), ROS-free.
// File form and field documentation: tools/treemapper_tools/frames.py.
#pragma once

#include <vector>

#include "treemapper/geometry.hpp"
#include "treemapper/types.hpp"

namespace treemapper {

struct FrameInput {
  double t{0.0};       ///< receive time (clock of the mission context and of the map stream)
  double stamp{0.0};   ///< image stamp
  DronePose pose;      ///< world <- body at `stamp`; pose.valid == false -> processing holds
  MissionPhase phase{MissionPhase::kUnknown};
  TreeId hop_target{kNoTree};           ///< navigation's hop target in THIS mapper's ID space
  std::vector<Detection2D> boxes;       ///< pixels of the full left image, (x, y) = top-left corner
  int grid_w{0}, grid_h{0};             ///< disparity grid size (empty disparity = none this frame)
  std::vector<float> disparity;         ///< row-major grid_h x grid_w, full-res pixels, NaN = invalid
};

}  // namespace treemapper
