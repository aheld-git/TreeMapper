// corridor_nav: dose time and dose validity (spec 4.6). Pure functions / a small value class, no ROS.
#pragma once

#include "corridor_nav/config.hpp"

namespace corridor_nav {

/// Seconds the nozzle must spend open on one tree. Area mode: clamp(area * dose_s_per_m2, dose_min_s, dose_max_s);
/// a non-finite or negative area gives dose_min_s. Fixed mode: dose_fixed_s.
double doseTime(double outline_area_m2, const Config& cfg);

/// Integrates how long the nozzle was open during one dose. `update(t, open)` credits the interval since the previous
/// `update` (or `start`) with the PREVIOUS open flag, then stores `open`; time after the end (t0 + dose_s + the paused
/// time) is not counted.
class DoseTracker {
 public:
  explicit DoseTracker(double valid_frac) : valid_frac_(valid_frac) {}

  /// Begins a dose at time `t` lasting `dose_s`; resets everything. The nozzle is taken as closed until the first
  /// `update`.
  void start(double t, double dose_s);
  void update(double t, bool nozzle_open);
  /// The dose was interrupted for `dt` s with the nozzle closed (a HOLD): its end moves `dt` later, so the
  /// interruption does not count against it. The open time already credited is kept; dt <= 0 changes nothing.
  void pause(double dt);
  /// True once t >= t0 + dose_s + the paused time.
  bool done(double t) const { return t >= end(); }
  /// openFraction() >= valid_frac (never true for an empty dose).
  bool valid() const { return dose_s_ > 0.0 && openFraction() >= valid_frac_; }
  /// Integrated open time / dose_s, in [0, 1]; 0 for an empty dose.
  double openFraction() const;

 private:
  double valid_frac_;
  double end() const { return t0_ + dose_s_ + paused_s_; }

  double t0_{0.0}, dose_s_{0.0}, paused_s_{0.0};
  double last_t_{0.0};
  bool last_open_{false};
  double open_s_{0.0};
};

}  // namespace corridor_nav
