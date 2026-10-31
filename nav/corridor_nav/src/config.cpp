#include "corridor_nav/config.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace corridor_nav {

namespace {
using Log = std::function<void(const std::string&)>;

constexpr double kEps = 1e-3;  ///< smallest allowed length / time / speed ("> 0")
constexpr double kBig = 1e4;   ///< generous upper bound: the clamp is a sanity net, not a policy

template <typename T>
T clampLog(const char* name, T v, T lo, T hi, const Log& log) {
  // NaN compares false with everything and would slip through the range test: reject it (and +-inf) outright
  if (!std::isfinite(static_cast<double>(v))) {
    throw std::invalid_argument("corridor_nav config: " + std::string(name) + " = " + std::to_string(v) + " is not finite");
  }
  if (v < lo || v > hi) {
    const T c = std::clamp(v, lo, hi);
    if (log) log("corridor_nav config: " + std::string(name) + " = " + std::to_string(v) + " clamped to " + std::to_string(c));
    return c;
  }
  return v;
}
}  // namespace

Config Config::sanitized(const Log& log) const {
  Config c = *this;
  c.z_scan = clampLog("z_scan", c.z_scan, kEps, kBig, log);
  c.scan_speed = clampLog("scan_speed", c.scan_speed, kEps, kBig, log);
  c.corridor_width_min = clampLog("corridor_width_min", c.corridor_width_min, kEps, kBig, log);
  c.corridor_width = clampLog("corridor_width", c.corridor_width, 0.0, kBig, log);
  if (c.corridor_width > 0.0 && c.corridor_width < c.corridor_width_min) {   // 0 = automatic, else >= the minimum
    c.corridor_width = clampLog("corridor_width", c.corridor_width, c.corridor_width_min, kBig, log);
  }
  c.corridor_length_min = clampLog("corridor_length_min", c.corridor_length_min, kEps, kBig, log);
  c.w_margin = clampLog("w_margin", c.w_margin, kEps, kBig, log);
  c.expected_canopy_top_max = clampLog("expected_canopy_top_max", c.expected_canopy_top_max, kEps, kBig, log);
  c.expected_canopy_radius_max = clampLog("expected_canopy_radius_max", c.expected_canopy_radius_max, kEps, kBig, log);
  c.target_height_margin = clampLog("target_height_margin", c.target_height_margin, 0.0, kBig, log);
  c.settle_s = clampLog("settle_s", c.settle_s, kEps, kBig, log);
  c.settle_timeout_s = clampLog("settle_timeout_s", c.settle_timeout_s, kEps, kBig, log);
  c.lock_hold_s = clampLog("lock_hold_s", c.lock_hold_s, kEps, kBig, log);
  c.order_budget_ms = clampLog("order_budget_ms", c.order_budget_ms, kEps, kBig, log);
  c.min_clearance = clampLog("min_clearance", c.min_clearance, kEps, kBig, log);
  c.spray_clearance = clampLog("spray_clearance", c.spray_clearance, c.min_clearance, kBig, log);  // never below the guard floor
  c.transit_clearance = clampLog("transit_clearance", c.transit_clearance, kEps, kBig, log);
  c.descent_slack = clampLog("descent_slack", c.descent_slack, kEps, kBig, log);
  c.drone_radius = clampLog("drone_radius", c.drone_radius, kEps, kBig, log);
  c.lateral_margin = clampLog("lateral_margin", c.lateral_margin, kEps, kBig, log);
  c.replan_dz = clampLog("replan_dz", c.replan_dz, kEps, kBig, log);
  c.carrot_lead_s = clampLog("carrot_lead_s", c.carrot_lead_s, kEps, kBig, log);
  c.max_speed_xy = clampLog("max_speed_xy", c.max_speed_xy, kEps, kBig, log);
  c.max_speed_z = clampLog("max_speed_z", c.max_speed_z, kEps, kBig, log);
  c.xy_tol = clampLog("xy_tol", c.xy_tol, kEps, kBig, log);
  c.z_tol = clampLog("z_tol", c.z_tol, kEps, kBig, log);
  c.arrive_speed = clampLog("arrive_speed", c.arrive_speed, kEps, kBig, log);
  c.dose_s_per_m2 = clampLog("dose_s_per_m2", c.dose_s_per_m2, kEps, kBig, log);
  c.dose_min_s = clampLog("dose_min_s", c.dose_min_s, kEps, kBig, log);
  c.dose_max_s = clampLog("dose_max_s", c.dose_max_s, c.dose_min_s, kBig, log);  // raise the max, never lower the min
  c.dose_fixed_s = clampLog("dose_fixed_s", c.dose_fixed_s, kEps, kBig, log);
  c.dose_valid_frac = clampLog("dose_valid_frac", c.dose_valid_frac, kEps, 1.0, log);
  c.guard_hysteresis = clampLog("guard_hysteresis", c.guard_hysteresis, kEps, kBig, log);
  c.guard_window = clampLog("guard_window", c.guard_window, kEps, 1.0, log);
  c.guard_min_valid = clampLog("guard_min_valid", c.guard_min_valid, kEps, 1.0, log);
  c.guard_max_age_s = clampLog("guard_max_age_s", c.guard_max_age_s, kEps, kBig, log);
  c.map_stale_s = clampLog("map_stale_s", c.map_stale_s, kEps, kBig, log);
  c.hold_timeout_s = clampLog("hold_timeout_s", c.hold_timeout_s, kEps, kBig, log);
  c.reresolve_radius = clampLog("reresolve_radius", c.reresolve_radius, kEps, kBig, log);
  c.pose_max_age_s = clampLog("pose_max_age_s", c.pose_max_age_s, kEps, kBig, log);
  c.service_retries = clampLog("service_retries", c.service_retries, 0, 10, log);
  return c;
}

Config configFrom(const treemapper::MapperConfig& config) {
  Config c;
  treemapper::ConfigReader r(config);

  const std::string strategy = r.getString("strategy", c.strategy == Strategy::kCorridor ? "corridor" : "field_first");
  if (strategy == "corridor") c.strategy = Strategy::kCorridor;
  else if (strategy == "field_first") c.strategy = Strategy::kFieldFirst;
  else throw std::invalid_argument("config key 'strategy': '" + strategy + "' is not corridor/field_first");
  const std::string dose = r.getString("dose_mode", c.dose_mode == DoseMode::kArea ? "area" : "fixed");
  if (dose == "area") c.dose_mode = DoseMode::kArea;
  else if (dose == "fixed") c.dose_mode = DoseMode::kFixed;
  else throw std::invalid_argument("config key 'dose_mode': '" + dose + "' is not area/fixed");

  c.z_scan = r.getDouble("z_scan", c.z_scan);
  c.scan_speed = r.getDouble("scan_speed", c.scan_speed);
  c.corridor_width = r.getDouble("corridor_width", c.corridor_width);
  c.corridor_width_min = r.getDouble("corridor_width_min", c.corridor_width_min);
  c.corridor_length_min = r.getDouble("corridor_length_min", c.corridor_length_min);
  c.w_margin = r.getDouble("w_margin", c.w_margin);
  c.expected_canopy_top_max = r.getDouble("expected_canopy_top_max", c.expected_canopy_top_max);
  c.expected_canopy_radius_max = r.getDouble("expected_canopy_radius_max", c.expected_canopy_radius_max);
  c.target_height_margin = r.getDouble("target_height_margin", c.target_height_margin);
  c.settle_s = r.getDouble("settle_s", c.settle_s);
  c.settle_timeout_s = r.getDouble("settle_timeout_s", c.settle_timeout_s);
  c.lock_hold_s = r.getDouble("lock_hold_s", c.lock_hold_s);
  c.order_budget_ms = r.getDouble("order_budget_ms", c.order_budget_ms);
  c.spray_clearance = r.getDouble("spray_clearance", c.spray_clearance);
  c.transit_clearance = r.getDouble("transit_clearance", c.transit_clearance);
  c.descent_slack = r.getDouble("descent_slack", c.descent_slack);
  c.drone_radius = r.getDouble("drone_radius", c.drone_radius);
  c.lateral_margin = r.getDouble("lateral_margin", c.lateral_margin);
  c.replan_dz = r.getDouble("replan_dz", c.replan_dz);
  c.carrot_lead_s = r.getDouble("carrot_lead_s", c.carrot_lead_s);
  c.max_speed_xy = r.getDouble("max_speed_xy", c.max_speed_xy);
  c.max_speed_z = r.getDouble("max_speed_z", c.max_speed_z);
  c.xy_tol = r.getDouble("xy_tol", c.xy_tol);
  c.z_tol = r.getDouble("z_tol", c.z_tol);
  c.arrive_speed = r.getDouble("arrive_speed", c.arrive_speed);
  c.dose_s_per_m2 = r.getDouble("dose_s_per_m2", c.dose_s_per_m2);
  c.dose_min_s = r.getDouble("dose_min_s", c.dose_min_s);
  c.dose_max_s = r.getDouble("dose_max_s", c.dose_max_s);
  c.dose_fixed_s = r.getDouble("dose_fixed_s", c.dose_fixed_s);
  c.dose_valid_frac = r.getDouble("dose_valid_frac", c.dose_valid_frac);
  c.min_clearance = r.getDouble("min_clearance", c.min_clearance);
  c.guard_hysteresis = r.getDouble("guard_hysteresis", c.guard_hysteresis);
  c.guard_window = r.getDouble("guard_window", c.guard_window);
  c.guard_min_valid = r.getDouble("guard_min_valid", c.guard_min_valid);
  c.guard_max_age_s = r.getDouble("guard_max_age_s", c.guard_max_age_s);
  c.map_stale_s = r.getDouble("map_stale_s", c.map_stale_s);
  c.hold_timeout_s = r.getDouble("hold_timeout_s", c.hold_timeout_s);
  c.reresolve_radius = r.getDouble("reresolve_radius", c.reresolve_radius);
  c.pose_max_age_s = r.getDouble("pose_max_age_s", c.pose_max_age_s);
  c.service_retries = r.getInt("service_retries", c.service_retries);
  r.finish("corridor_nav");
  return c;
}

}  // namespace corridor_nav
