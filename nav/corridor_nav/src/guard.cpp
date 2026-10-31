#include "corridor_nav/guard.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace corridor_nav {

namespace {
constexpr double kPercentile = 0.10;
constexpr double kRankEps = 1e-9;   ///< keeps ceil(0.1 * n) exact when 0.1 * n lands a hair above an integer
}  // namespace

Clearance nadirClearance(const DisparityView& view, const StereoIntrinsics& stereo, double roll, double pitch,
                         const Config& cfg) {
  Clearance out;
  out.stamp = view.stamp;
  if (!(stereo.fx > 0.0) || !(stereo.baseline > 0.0) || !std::isfinite(stereo.fx) || !std::isfinite(stereo.baseline))
    return out;
  if (view.data == nullptr || view.width <= 0 || view.height <= 0) return out;
  if (!std::isfinite(roll) || !std::isfinite(pitch)) return out;

  // Central window: round(frac * size) pixels, centred (100 px at 0.2 -> 40..59).
  const double frac = std::clamp(cfg.guard_window, 0.0, 1.0);
  const int ww = std::clamp(static_cast<int>(std::lround(frac * view.width)), 1, view.width);
  const int wh = std::clamp(static_cast<int>(std::lround(frac * view.height)), 1, view.height);
  const int x0 = (view.width - ww) / 2, y0 = (view.height - wh) / 2;

  const double fb = stereo.fx * stereo.baseline;
  std::vector<double> z;
  z.reserve(static_cast<std::size_t>(ww) * wh);
  for (int y = y0; y < y0 + wh; ++y) {
    const float* row = view.data + static_cast<std::size_t>(y) * view.width;
    for (int x = x0; x < x0 + ww; ++x) {
      const float d = row[x];
      if (std::isfinite(d) && d > 0.0f) z.push_back(fb / static_cast<double>(d));
    }
  }
  const double window_px = static_cast<double>(ww) * wh;
  if (z.empty() || static_cast<double>(z.size()) < cfg.guard_min_valid * window_px) return out;

  const std::size_t n = z.size();
  const std::size_t rank = std::clamp<std::size_t>(
      static_cast<std::size_t>(std::ceil(kPercentile * static_cast<double>(n) - kRankEps)), 1, n);
  std::nth_element(z.begin(), z.begin() + static_cast<std::ptrdiff_t>(rank - 1), z.end());
  out.metres = z[rank - 1] * std::cos(roll) * std::cos(pitch);
  out.valid = true;
  return out;
}

GuardAction guardDecision(const Clearance& clearance, double now, bool climbing, const Config& cfg) {
  if (!clearance.valid || !std::isfinite(clearance.metres) || now - clearance.stamp > cfg.guard_max_age_s)
    return GuardAction::kNoDescent;
  if (clearance.metres < cfg.min_clearance) return GuardAction::kClimb;
  if (climbing && clearance.metres < cfg.min_clearance + cfg.guard_hysteresis) return GuardAction::kClimb;
  return GuardAction::kNone;
}

}  // namespace corridor_nav
