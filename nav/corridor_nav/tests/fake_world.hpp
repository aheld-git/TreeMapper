// Test double for the Navigator: a point drone that flies straight at the setpoint at max_speed_xy / max_speed_z,
// a field of truth trees, a map that publishes a tree once the drone has scanned near it, and set_tree_sprayed.
// Records what the navigator commanded on every tick so tests can check sequences and invariants.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "corridor_nav/navigator.hpp"

namespace corridor_nav {
namespace fake {

struct TruthTree {
  double x{0.0}, y{0.0};
  double r{1.5};                ///< crown radius, m
  double height{4.0};           ///< canopy top, world z
  bool sprayed{false};          ///< pre-marked trees are published with sprayed = true
  bool settle_only{false};      ///< seen only while the navigator hovers in SCAN_SETTLE
  int dwell_needed{3};          ///< ticks in SCAN within see_radius before it is published
  // filled by the world
  int dwell{0};
  bool published{false};
  std::string published_phase;
  NavState published_state{NavState::kIdle};
  int spray_calls{0};
};

/// One tick as the navigator saw it (pose before the drone moved) and what it commanded.
struct Sample {
  double t{0.0};
  NavState state{NavState::kIdle};   ///< after the step
  std::string phase;
  treemapper::TreeId target{treemapper::kNoTree};
  bool nozzle{false};
  double xy_err{NAN};                ///< drone xy to the target's truth centre, NaN without a target
  bool cleanup{false};
  int corridor{-1};
  Point3 pos, setpoint;
};

struct FakeWorld {
  Config cfg;
  std::vector<TruthTree> trees;
  Point3 drone;
  double yaw{0.0};
  double t{0.0};
  double start_at{0.2};             ///< s, start is requested from this time on while the navigator is IDLE
  double see_radius{6.0};           ///< m, xy distance at which a scanned tree counts as seen
  double ground_clearance{15.0};    ///< m, clearance reported over bare ground
  std::optional<SprayResult> pending;
  std::vector<Sample> samples;
  std::vector<treemapper::TreeId> requests;
  double min_clearance{std::numeric_limits<double>::infinity()};

  static treemapper::TreeId idOf(std::size_t i) { return static_cast<treemapper::TreeId>(i + 1); }

  treemapper::TreeEntry entry(std::size_t i) const {
    const TruthTree& tt = trees[i];
    treemapper::TreeEntry e;
    e.id = idOf(i);
    for (int k = 0; k < 16; ++k) {
      const double a = 2.0 * 3.14159265358979323846 * k / 16;
      e.outline.push_back({tt.x + tt.r * std::cos(a), tt.y + tt.r * std::sin(a)});
    }
    e.centre = {tt.x, tt.y};
    e.height = tt.height;
    e.observations = 10;
    e.sprayed = tt.sprayed;
    return e;
  }

  std::vector<treemapper::TreeEntry> map() const {
    std::vector<treemapper::TreeEntry> out;
    for (std::size_t i = 0; i < trees.size(); ++i) {
      if (trees[i].published) out.push_back(entry(i));
    }
    return out;
  }

  bool set_tree_sprayed(treemapper::TreeId id) {
    TruthTree& tt = trees.at(static_cast<std::size_t>(id - 1));
    tt.sprayed = true;
    ++tt.spray_calls;
    return true;
  }

  /// Drone z minus the highest truth crown whose disk holds the drone's xy, else ground_clearance.
  double clearance() const {
    double top = -std::numeric_limits<double>::infinity();
    for (const TruthTree& tt : trees) {
      if (std::hypot(drone.x - tt.x, drone.y - tt.y) <= tt.r) top = std::max(top, tt.height);
    }
    return std::isfinite(top) ? drone.z - top : ground_clearance;
  }

  void tick(Navigator& nav, double dt) {
    Inputs in;
    in.t = t;
    in.pose.t = t;
    in.pose.x = drone.x;
    in.pose.y = drone.y;
    in.pose.z = drone.z;
    in.pose.qz = std::sin(yaw / 2.0);
    in.pose.qw = std::cos(yaw / 2.0);
    in.pose.valid = true;
    in.map = MapSnapshot{t, map()};
    in.clearance.valid = true;
    in.clearance.metres = clearance();
    in.clearance.stamp = t;
    in.start_requested = t >= start_at && nav.state() == NavState::kIdle;
    in.spray_result = pending;
    pending.reset();

    const Outputs out = nav.step(in);
    const Status st = nav.status();

    Sample s;
    s.t = t;
    s.state = nav.state();
    s.phase = out.phase;
    s.target = out.target;
    s.nozzle = out.nozzle;
    if (out.target != treemapper::kNoTree) {
      const TruthTree& tt = trees.at(static_cast<std::size_t>(out.target - 1));
      s.xy_err = std::hypot(drone.x - tt.x, drone.y - tt.y);
    }
    s.cleanup = st.cleanup;
    s.corridor = st.corridor;
    s.pos = drone;
    s.setpoint = out.setpoint;
    samples.push_back(s);
    min_clearance = std::min(min_clearance, clearance());

    if (out.request_sprayed) {
      requests.push_back(*out.request_sprayed);
      pending = SprayResult{*out.request_sprayed, set_tree_sprayed(*out.request_sprayed)};
    }

    // kinematic point drone
    const double dx = out.setpoint.x - drone.x, dy = out.setpoint.y - drone.y;
    const double dxy = std::hypot(dx, dy);
    const double step_xy = std::min(dxy, cfg.max_speed_xy * dt);
    if (dxy > 0.0) {
      drone.x += dx / dxy * step_xy;
      drone.y += dy / dxy * step_xy;
    }
    drone.z += std::clamp(out.setpoint.z - drone.z, -cfg.max_speed_z * dt, cfg.max_speed_z * dt);
    yaw = out.yaw;

    // the map learns trees seen while scanning
    if (out.phase == "SCAN") {
      for (TruthTree& tt : trees) {
        if (tt.published || std::hypot(drone.x - tt.x, drone.y - tt.y) > see_radius) continue;
        if (tt.settle_only && nav.state() != NavState::kScanSettle) continue;
        if (++tt.dwell >= tt.dwell_needed) {
          tt.published = true;
          tt.published_phase = out.phase;
          tt.published_state = nav.state();
        }
      }
    }
    t += dt;
  }

  /// Ticks until the navigator is DONE (true) or `t_max` of simulated time passed (false).
  bool run(Navigator& nav, double dt, double t_max) {
    while (t < t_max) {
      tick(nav, dt);
      if (nav.state() == NavState::kDone) return true;
    }
    return false;
  }
};

}  // namespace fake
}  // namespace corridor_nav
