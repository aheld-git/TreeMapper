// corridor_nav configuration: every tunable of the navigator in one struct. In-struct defaults equal
// config/corridor_nav.cfg (enforced by a test). Clamping lives in sanitized() only, and every clamp is logged.
// Units: metres, seconds, m/s. `strategy` and `dose_mode` are enums parsed from strings.
#pragma once

#include <functional>
#include <string>

#include "treemapper/mapper.hpp"  // MapperConfig, ConfigReader

namespace corridor_nav {

enum class Strategy { kCorridor, kFieldFirst };  ///< "corridor" | "field_first"
enum class DoseMode { kArea, kFixed };           ///< "area" | "fixed"

struct Config {
  // --- strategy
  Strategy strategy{Strategy::kCorridor};
  // --- corridors
  double z_scan{15.0};                    ///< m above home
  double scan_speed{4.0};                 ///< m/s along a corridor
  double corridor_width{0.0};             ///< m, 0 = the widest strip the camera footprint allows, else >= corridor_width_min
  double corridor_width_min{3.0};         ///< m
  double corridor_length_min{5.0};        ///< m, shorter strips are dropped
  double w_margin{1.0};                   ///< m
  double expected_canopy_top_max{7.0};    ///< m above home
  double expected_canopy_radius_max{2.5}; ///< m
  // --- settle / lock
  double settle_s{2.0};
  double settle_timeout_s{8.0};
  double lock_hold_s{0.5};
  // --- ordering
  double order_budget_ms{5.0};
  // --- glide
  double spray_clearance{2.5};            ///< m above the canopy top while spraying
  double transit_clearance{2.0};          ///< m above any canopy while gliding
  double descent_slack{0.5};              ///< m
  double drone_radius{0.8};               ///< m
  double lateral_margin{0.5};             ///< m
  double replan_dz{0.3};                  ///< m
  double carrot_lead_s{0.5};              ///< s
  double max_speed_xy{4.0};               ///< m/s
  double max_speed_z{1.5};                ///< m/s
  double xy_tol{0.3};                     ///< m
  double z_tol{0.3};                      ///< m
  double arrive_speed{0.3};               ///< m/s
  // --- dose
  DoseMode dose_mode{DoseMode::kArea};
  double dose_s_per_m2{0.3};
  double dose_min_s{2.0};
  double dose_max_s{6.0};
  double dose_fixed_s{4.0};
  double dose_valid_frac{0.8};
  // --- guard
  double min_clearance{1.5};              ///< m
  double guard_hysteresis{0.5};           ///< m
  double guard_window{0.2};               ///< fraction of the image: the central guard_window of its width and of its height
  double guard_min_valid{0.3};
  double guard_max_age_s{0.5};
  // --- inputs
  double map_stale_s{2.0};
  double hold_timeout_s{20.0};
  double reresolve_radius{1.5};           ///< m
  double pose_max_age_s{0.2};
  int service_retries{3};

  /// Range-checked copy; each clamp is reported through `log` (may be empty).
  Config sanitized(const std::function<void(const std::string&)>& log) const;
};

/// Key/value -> Config (keys = field names); unknown keys, bad numbers, non-finite values and unknown enum
/// strings throw std::invalid_argument.
Config configFrom(const treemapper::MapperConfig& config);
/// Number of tunables (numeric fields + strategy + dose_mode), for the config-file test.
constexpr int kConfigKeyCount = 41;

/// Every numeric field (the enums are parsed from strings and checked separately).
#define CORRIDOR_NAV_FIELDS(X)                                                                              \
  X(z_scan) X(scan_speed) X(corridor_width) X(corridor_width_min) X(corridor_length_min) X(w_margin)        \
  X(expected_canopy_top_max) X(expected_canopy_radius_max) X(settle_s) X(settle_timeout_s) X(lock_hold_s)   \
  X(order_budget_ms) X(spray_clearance) X(transit_clearance) X(descent_slack) X(drone_radius)               \
  X(lateral_margin) X(replan_dz) X(carrot_lead_s) X(max_speed_xy) X(max_speed_z) X(xy_tol) X(z_tol)         \
  X(arrive_speed) X(dose_s_per_m2) X(dose_min_s) X(dose_max_s) X(dose_fixed_s) X(dose_valid_frac)           \
  X(min_clearance) X(guard_hysteresis) X(guard_window) X(guard_min_valid) X(guard_max_age_s)                \
  X(map_stale_s) X(hold_timeout_s) X(reresolve_radius) X(pose_max_age_s) X(service_retries)

}  // namespace corridor_nav
