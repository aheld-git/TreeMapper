// corridor_nav: the Navigator's safety and recovery members (spec 4.4 - 4.6): HOLD on missing inputs, ABORT, the
// clearance guard, set_tree_sprayed with re-resolve and retries, re-resolving vanished targets and re-planning glides.
// The state machine flow is in navigator.cpp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "corridor_nav/navigator.hpp"
#include "corridor_nav/navigator_support.hpp"
#include "treemapper/geometry.hpp"

namespace corridor_nav {

namespace {
using treemapper::kNoTree;
using treemapper::Point2;
using treemapper::TreeEntry;
using treemapper::TreeId;

constexpr double kServiceTimeoutS = 2.0;   ///< s, set_tree_sprayed without an answer counts as a transport failure
constexpr int kGuardClimbsPerAttempt = 3;  ///< climb episodes in one glide + dose attempt before it is given up

void count(Counters& counters, const std::string& name) { ++counters[name]; }

bool flying(NavState s) {
  return s != NavState::kIdle && s != NavState::kHold && s != NavState::kAbort && s != NavState::kDone;
}

/// States whose decisions use the map: a stale map holds them (TAKEOFF, TRANSIT, RETURN and LAND fly at or to scan
/// altitude, or straight down at home, and need no map).
bool needsMap(NavState s) {
  return s == NavState::kScanOut || s == NavState::kScanSettle || s == NavState::kLock || s == NavState::kGlide ||
         s == NavState::kDose;
}

}  // namespace

void Navigator::updateHold(const Inputs& in) {
  const auto map_stale = [&](NavState s) { return needsMap(s) && !(in.t - map_t_ <= cfg_.map_stale_s); };
  if (state_ != NavState::kHold) {
    if (!flying(state_)) return;
    if (!pose_ok_) {
      startHold("no_pose", in.t);
    } else if (map_stale(state_)) {
      startHold("map_stale", in.t);
    }
    return;
  }
  const double held = in.t - hold_t_;
  if (pose_ok_ && !map_stale(held_state_) && !service_hold_) {   // every reason cleared: resume where we were
    if (log_) log_("HOLD cleared after " + std::to_string(held) + " s");
    enter(held_state_, in.t);
    state_t_ = held_state_t_ + held;   // the state's own timers exclude the hold
    settle_gain_t_ += held;
    guard_ok_t_ += held;
    return;
  }
  if (held < cfg_.hold_timeout_s) return;
  count(counters_, "hold_timeouts");
  if (log_) log_("HOLD timed out");
  if (counters_["hold_timeouts"] >= 2) {
    startAbort("repeated hold timeout", in.t);
  } else {
    startReturn(in.t);
  }
}

GuardAction Navigator::guardUpdate(const Inputs& in) {
  const GuardAction a = guardDecision(in.clearance, in.t, climbing_, cfg_);
  const bool was_climbing = climbing_;
  climbing_ = a == GuardAction::kClimb;
  guard_released_ = was_climbing && !climbing_;
  if (a == GuardAction::kNone) guard_ok_t_ = in.t;
  if (a != GuardAction::kNoDescent) blind_ = false;
  if (a == GuardAction::kClimb && !was_climbing) {
    count(counters_, "guard_triggers");
    ++attempt_climbs_;
    const std::uint64_t n = counters_["guard_triggers"];
    if (log_ && (n == 1 || n % 10 == 0)) {
      log_("clearance guard: " + std::to_string(in.clearance.metres) + " m, climbing (trigger " + std::to_string(n) +
           ")");
    }
  } else if (a == GuardAction::kNoDescent && !blind_) {
    blind_ = true;
    count(counters_, "guard_blind");
  }
  // an attempt the guard keeps failing is given up: the third climb, or no clear reading for hold_timeout_s
  guard_failed_ = a != GuardAction::kNone &&
                  (attempt_climbs_ >= kGuardClimbsPerAttempt || in.t - guard_ok_t_ >= cfg_.hold_timeout_s);
  return a;
}

void Navigator::guardApply(GuardAction a, Outputs& out) const {
  if (a == GuardAction::kClimb) {
    // straight up from where we are at max_speed_z (the carrot leads by carrot_lead_s), not above scan altitude
    const double up = std::min(pos_.z + cfg_.max_speed_z * cfg_.carrot_lead_s, z_scan_abs_);
    out.setpoint = {pos_.x, pos_.y, std::max(pos_.z, up)};
  } else if (a == GuardAction::kNoDescent) {
    out.setpoint.z = std::max(out.setpoint.z, pos_.z);
  }
}

void Navigator::sendMark(TreeId id, double t, Outputs& out) {
  mark_.id = id;
  mark_.sent_t = t;
  out.request_sprayed = id;
}

void Navigator::pollMark(const Inputs& in, Outputs& out) {
  if (in.spray_result && in.spray_result->id == mark_.id) {
    service_hold_ = false;
    if (in.spray_result->ok) {
      ++sprayed_;
      mark_.done = true;
      return;
    }
    if (!mark_.reresolved) {   // unknown id (merged away): once, retry on the id that now stands for the tree
      mark_.reresolved = true;
      const TreeId id = reresolve(mark_.centre);
      if (id != kNoTree) {
        count(counters_, "ids_reresolved");
        if (log_) log_("set_tree_sprayed(" + std::to_string(mark_.id) + ") failed, retrying as " + std::to_string(id));
        target_ = id;
        mark_.tries = 0;
        sendMark(id, in.t, out);
        return;
      }
    }
    count(counters_, "spray_mark_failed");
    if (log_) log_("set_tree_sprayed(" + std::to_string(mark_.id) + ") failed, continuing");
    mark_.done = true;
    return;
  }
  if (in.t - mark_.sent_t < kServiceTimeoutS) return;
  if (mark_.tries < cfg_.service_retries) {   // no answer: transport failure
    ++mark_.tries;
    count(counters_, "spray_mark_retries");
    sendMark(mark_.id, in.t, out);
    return;
  }
  sendMark(mark_.id, in.t, out);   // retries exhausted: hold, and keep asking every kServiceTimeoutS
  if (!service_hold_) {
    service_hold_ = true;
    startHold("service", in.t);
  }
}

void Navigator::abandonMark() {
  if (mark_.active && !mark_.done) {
    count(counters_, "spray_mark_failed");
    if (log_) log_("set_tree_sprayed(" + std::to_string(mark_.id) + ") abandoned");
  }
  mark_ = Mark{};
  service_hold_ = false;
}

bool Navigator::keepTarget(double t) {
  if (const TreeEntry* tr = tree(target_)) {
    target_centre_ = tr->centre;
    return true;
  }
  const TreeId id = reresolve(target_centre_);
  count(counters_, id == kNoTree ? "ids_dropped_vanished" : "ids_reresolved");
  if (log_) log_("target " + std::to_string(target_) + " vanished -> " + std::to_string(id));
  if (id == kNoTree) {
    advance(t);
    return false;
  }
  target_ = id;
  planToTarget(t, "target re-resolved");
  return true;
}

TreeId Navigator::reresolve(const Point2& centre) const {
  const TreeId id = reresolveId(centre, map_, cfg_.reresolve_radius);
  return attempted_.count(id) ? kNoTree : id;
}

void Navigator::planToTarget(double t, const char* replan_why) {
  const TreeEntry& to = *tree(target_);
  target_centre_ = to.centre;
  setPath(glideTo(pos_, from_id_, to, &counters_));
  spray_pt_ = path_.pts.back();
  target_area_ = std::fabs(treemapper::polygonArea(to.outline));
  planned_heights_ = support::heightsNear(path_, target_, map_, cfg_);
  if (!replan_why) return;
  count(counters_, "glide_replans");
  if (log_) log_("glide to " + std::to_string(target_) + " re-planned: " + replan_why + " at t=" + std::to_string(t));
}

Path3 Navigator::glideTo(const Point3& from, TreeId from_id, const TreeEntry& to, Counters* counters) const {
  return support::glideLeg(from, from_id, to, map_, cfg_, field_.polygon, z_scan_abs_, counters);
}

void Navigator::stepHold(const Inputs& in, Outputs& out) {
  out.setpoint = hold_pos_;
  if (mark_.active && !mark_.done) pollMark(in, out);   // the mark goes on (and may clear a service hold)
}

void Navigator::stepAbort(const Inputs& in, Outputs& out) {
  out.setpoint = hold_;
  if (!airborne_ || in.t <= state_t_) return;   // on the ground: stay; else ERROR for one tick, then home
  startReturn(in.t);
  out.setpoint = follow(cfg_);
}

void Navigator::startHold(const std::string& reason, double t) {
  count(counters_, "hold_" + reason);
  if (log_) log_("HOLD(" + reason + ")");
  if (state_ == NavState::kHold) return;   // already holding: one more reason
  held_state_ = state_;
  held_state_t_ = state_t_;
  hold_t_ = t;
  hold_pos_ = pos_;
  enter(NavState::kHold, t);
}

void Navigator::startAbort(const std::string& why, double t) {
  count(counters_, "aborts");
  warnings_.push_back("abort: " + why);
  if (log_) log_("ABORT: " + why);
  abandonMark();
  dose_running_ = false;
  cleanup_mode_ = false;
  target_ = kNoTree;
  hold_ = pos_;
  enter(NavState::kAbort, t);
}

}  // namespace corridor_nav
