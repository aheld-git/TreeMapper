#include "corridor_nav/spray.hpp"

#include <algorithm>
#include <cmath>

namespace corridor_nav {

double doseTime(double outline_area_m2, const Config& cfg) {
  if (cfg.dose_mode == DoseMode::kFixed) return cfg.dose_fixed_s;
  if (!std::isfinite(outline_area_m2) || outline_area_m2 < 0.0) return cfg.dose_min_s;
  return std::clamp(outline_area_m2 * cfg.dose_s_per_m2, cfg.dose_min_s, cfg.dose_max_s);
}

void DoseTracker::start(double t, double dose_s) {
  t0_ = t;
  dose_s_ = dose_s;
  last_t_ = t;
  last_open_ = false;
  open_s_ = 0.0;
}

void DoseTracker::update(double t, bool nozzle_open) {
  if (last_open_) {
    const double from = std::max(last_t_, t0_);
    const double to = std::min(t, t0_ + dose_s_);
    if (to > from) open_s_ += to - from;
  }
  last_t_ = std::max(last_t_, t);   // a clock step back never re-credits time
  last_open_ = nozzle_open;
}

double DoseTracker::openFraction() const {
  if (!(dose_s_ > 0.0)) return 0.0;
  return std::clamp(open_s_ / dose_s_, 0.0, 1.0);
}

}  // namespace corridor_nav
