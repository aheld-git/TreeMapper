// corridor_nav: stereo nadir clearance and the fail-closed clearance guard (spec 4.6). Pure functions, no ROS.
#pragma once

#include <cmath>

#include "corridor_nav/config.hpp"

namespace corridor_nav {

/// A disparity image, row-major, in pixels; NaN, inf and d <= 0 are invalid. `data` is not owned.
struct DisparityView {
  int width{0}, height{0};
  const float* data{nullptr};
  double stamp{0.0};   ///< s
};

struct StereoIntrinsics {
  double fx{0.0};         ///< px
  double baseline{0.0};   ///< m
};

/// Canopy clearance straight below the drone.
struct Clearance {
  bool valid{false};
  double metres{NAN};
  double stamp{0.0};   ///< s, of the disparity image it came from
};

/// Nearest-rank 10th percentile of Z = fx * baseline / d over the central `guard_window` fraction of the width and of
/// the height, times cos(roll) * cos(pitch). valid = false (metres NaN) when fewer than `guard_min_valid` of the
/// window's pixels are valid, or on unusable input (fx or baseline <= 0, empty image).
Clearance nadirClearance(const DisparityView& view, const StereoIntrinsics& stereo, double roll, double pitch,
                         const Config& cfg);

enum class GuardAction { kNone, kClimb, kNoDescent };

/// Checked in this order: invalid or older than `guard_max_age_s` -> kNoDescent (blind: do not go lower);
/// metres < min_clearance -> kClimb; while `climbing`, metres < min_clearance + guard_hysteresis -> kClimb;
/// otherwise kNone.
GuardAction guardDecision(const Clearance& clearance, double now, bool climbing, const Config& cfg);

}  // namespace corridor_nav
