#include "corridor_nav/navigator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "corridor_nav/corridors.hpp"
#include "corridor_nav/navigator_support.hpp"
#include "treemapper/geometry.hpp"

namespace corridor_nav {

namespace {
using treemapper::kNoTree;
using treemapper::Point2;
using treemapper::TreeEntry;
using treemapper::TreeId;
using support::dist3;
using support::distXY;
using support::makePath;
using support::usable;
using support::yawOf;

void count(Counters& counters, const std::string& name) { ++counters[name]; }

}  // namespace

const char* toString(NavState s) {
  switch (s) {
    case NavState::kIdle: return "IDLE";
    case NavState::kTakeoff: return "TAKEOFF";
    case NavState::kTransit: return "TRANSIT";
    case NavState::kScanOut: return "SCAN_OUT";
    case NavState::kScanSettle: return "SCAN_SETTLE";
    case NavState::kLock: return "LOCK";
    case NavState::kGlide: return "GLIDE";
    case NavState::kDose: return "DOSE";
    case NavState::kReturn: return "RETURN";
    case NavState::kLand: return "LAND";
    case NavState::kHold: return "HOLD";
    case NavState::kAbort: return "ABORT";
    case NavState::kDone: return "DONE";
  }
  return "UNKNOWN";
}

const char* statusName(const Status& s) { return s.cleanup ? "CLEANUP" : toString(s.state); }

Navigator::Navigator(Config config, Field field, CameraFov fov, std::function<void(const std::string&)> log)
    : cfg_(config.sanitized(log)), scan_cfg_(cfg_), field_(std::move(field)), fov_(fov), log_(std::move(log)),
      dose_(cfg_.dose_valid_frac) {
  scan_cfg_.max_speed_xy = cfg_.scan_speed;   // carrot() paces the scan line at scan_speed
}

// ---------------------------------------------------------------------------------------------------------------
// per-tick plumbing

Outputs Navigator::step(const Inputs& in) {
  const NavState before = state_;
  ingest(in);
  Outputs out;
  out.setpoint = hold_;
  guard_ran_ = false;
  const bool abortable = airborne_ && state_ != NavState::kAbort && state_ != NavState::kReturn &&
                         state_ != NavState::kLand && state_ != NavState::kDone;
  if (in.abort_requested && abortable) {
    startAbort("abort requested", in.t);
  } else {
    updateHold(in);
  }
  switch (state_) {
    case NavState::kIdle: stepIdle(in, out); break;
    case NavState::kTakeoff: stepTakeoff(in, out); break;
    case NavState::kTransit: stepTransit(in, out); break;
    case NavState::kScanOut: stepScanOut(in, out); break;
    case NavState::kScanSettle: stepScanSettle(in, out); break;
    case NavState::kLock: stepLock(in, out); break;
    case NavState::kGlide: stepGlide(in, out); break;
    case NavState::kDose: stepDose(in, out); break;
    case NavState::kReturn: stepReturn(in, out); break;
    case NavState::kLand: stepLand(in, out); break;
    case NavState::kHold: stepHold(in, out); break;
    case NavState::kAbort: stepAbort(in, out); break;
    case NavState::kDone: break;
  }
  return finish(before, in.t, std::move(out));
}

void Navigator::ingest(const Inputs& in) {
  // fresh = |now - stamp| <= pose_max_age_s and finite (as the guard): a NaN or far-future stamp is rejected and
  // never replaces the stored pose; pose_ok_ judges the pose actually stored
  const auto fresh = [&](double stamp) {
    const double age = in.t - stamp;
    return std::isfinite(age) && std::fabs(age) <= cfg_.pose_max_age_s;
  };
  if (in.pose.valid && fresh(in.pose.t) && (!have_pose_ || in.pose.t > pose_t_)) {
    const Point3 p{in.pose.x, in.pose.y, in.pose.z};
    if (have_pose_) speed_ = dist3(p, pos_) / (in.pose.t - pose_t_);
    pos_ = p;
    pose_t_ = in.pose.t;
    have_pose_ = true;
  }
  pose_ok_ = in.pose.valid && have_pose_ && fresh(pose_t_);
  clearance_m_ = in.clearance.valid ? in.clearance.metres : NAN;
  if (!in.map) return;
  map_t_ = in.map->stamp;   // the latest map received decides freshness, whatever its stamp
  map_.clear();
  map_.reserve(in.map->trees.size());
  for (const TreeEntry& t : in.map->trees) {
    if (usable(t)) {
      map_.push_back(t);
    } else if (dropped_invalid_.insert(t.id).second) {
      count(counters_, "trees_dropped_invalid");
    }
  }
  // late trees in locked corridors -> cleanup; trees outside the field are noted (LOCK makes its own call)
  if (!corridors_.empty()) {
    updateAssignment(map_, corridors_, field_.polygon, locked_upto_, false, &assignment_, &counters_);
  }
}

Outputs Navigator::finish(NavState before, double t, Outputs out) {
  const bool hold = state_ == NavState::kHold;
  if (hold) out.setpoint = hold_pos_;   // also when a handler entered HOLD on this tick
  // The guard decided on a GLIDE / DOSE tick applies to the final setpoint, whatever path the handler returned by
  // (a dropped target, a finished or failed attempt, a re-plan); HOLD keeps the position where it began.
  if (guard_ran_ && !hold) guardApply(guard_action_, out);
  // The nozzle opens only in a DOSE that neither starts nor ends on this tick, and never while waiting for the mark.
  out.nozzle = out.nozzle && before == NavState::kDose && state_ == NavState::kDose && !mark_.active;
  // The dose tracker integrates the nozzle actually published, after every gate (guard, HOLD, transitions).
  if (dose_running_) dose_.update(t, out.nozzle);
  const bool on_target = state_ == NavState::kGlide || state_ == NavState::kDose;
  out.target = hold ? last_target_ : on_target ? target_ : kNoTree;   // HOLD: unchanged
  const bool corridor_work = state_ == NavState::kTransit || state_ == NavState::kScanOut ||
                             state_ == NavState::kScanSettle || state_ == NavState::kLock || on_target;
  if (corridor_work && corridor_ >= 0) yaw_ = corridors_[static_cast<std::size_t>(corridor_)].heading_rad;
  out.yaw = yaw_;
  out.phase = phase();
  last_phase_ = out.phase;
  last_target_ = out.target;
  return out;
}

void Navigator::enter(NavState s, double t) {
  if (log_ && s != state_) log_(std::string("state ") + toString(state_) + " -> " + toString(s));
  state_ = s;
  state_t_ = t;
}

std::string Navigator::phase() const {
  switch (state_) {
    case NavState::kIdle: return "IDLE";
    case NavState::kTakeoff:
    case NavState::kTransit: return "SCAN_APPROACH";
    case NavState::kScanOut:
    case NavState::kScanSettle: return "SCAN";
    case NavState::kLock: return "PLANNING";
    case NavState::kGlide: return "HOPPING";
    case NavState::kDose: return mark_.active ? "HOPPING" : "SPRAYING";
    case NavState::kReturn:
    case NavState::kLand: return "RETURN";
    case NavState::kHold: return last_phase_;   // unchanged
    case NavState::kAbort: return "ERROR";
    case NavState::kDone: return "IDLE";
  }
  return "ERROR";
}

bool Navigator::onTarget() const {
  const NavState s = state_ == NavState::kHold ? held_state_ : state_;
  return s == NavState::kGlide || s == NavState::kDose;
}

// ---------------------------------------------------------------------------------------------------------------
// state handlers

void Navigator::stepIdle(const Inputs& in, Outputs& out) {
  hold_ = pos_;
  out.setpoint = pos_;
  if (in.pose.valid) yaw_ = yawOf(in.pose);
  if (!in.start_requested || !pose_ok_) return;

  home_ = {field_.home.x, field_.home.y, pos_.z};
  z_scan_abs_ = pos_.z + cfg_.z_scan;
  const CorridorPlan plan = planCorridors(field_, fov_, cfg_, &counters_);
  if (!plan.ok || plan.corridors.empty()) {
    const std::string why = plan.ok ? "no corridor long enough" : plan.error;
    startAbort("corridor planning failed: " + why, in.t);
    return;
  }
  corridors_ = plan.corridors;
  scan_.clear();
  for (const Corridor& c : corridors_) {
    scan_.push_back({c.index, cfg_.strategy == Strategy::kFieldFirst && c.index % 2 == 1});
  }
  scan_idx_ = 0;
  if (log_) {
    log_("planned " + std::to_string(corridors_.size()) + " corridors, width " + std::to_string(plan.width) + " m");
  }
  setPath(makePath({pos_, {pos_.x, pos_.y, z_scan_abs_}}));
  airborne_ = true;
  enter(NavState::kTakeoff, in.t);
}

void Navigator::stepTakeoff(const Inputs& in, Outputs& out) {
  out.setpoint = follow(cfg_);
  if (atPathEnd(false)) startTransit(in.t);
}

void Navigator::stepTransit(const Inputs& in, Outputs& out) {
  out.setpoint = follow(cfg_);
  if (atPathEnd(false)) startScan(in.t);
}

void Navigator::stepScanOut(const Inputs& in, Outputs& out) {
  out.setpoint = follow(scan_cfg_);
  if (atPathEnd(false)) startSettle(in.t);
}

void Navigator::stepScanSettle(const Inputs& in, Outputs& out) {
  out.setpoint = hold_;
  for (TreeId id : treesInCorridor(corridor_)) {
    if (settle_ids_.insert(id).second) settle_gain_t_ = in.t;   // a new tree in the corridor restarts the timer
  }
  const bool settled = in.t - settle_gain_t_ >= cfg_.settle_s;
  const bool timed_out = in.t - state_t_ >= cfg_.settle_timeout_s;
  if (!settled && !timed_out) return;
  if (!settled) {
    count(counters_, "settle_timeouts");
    if (log_) log_("corridor " + std::to_string(corridor_) + ": settle timed out");
  }
  if (cfg_.strategy == Strategy::kFieldFirst && scan_idx_ + 1 < scan_.size()) {
    ++scan_idx_;
    startTransit(in.t);
  } else {
    startLock(in.t);
  }
}

void Navigator::stepLock(const Inputs& in, Outputs& out) {
  out.setpoint = hold_;
  if (in.t - state_t_ >= cfg_.lock_hold_s) advance(in.t);
}

void Navigator::stepGlide(const Inputs& in, Outputs& out) {
  const GuardAction g = guardUpdate(in);
  if (!keepTarget(in.t)) {   // dropped: advance() set the next path
    out.setpoint = follow(cfg_);
    return;
  }
  if (guard_released_) {
    planToTarget(in.t, "clearance guard released");
  } else if (needsReplan(planned_heights_, map_, cfg_)) {
    planToTarget(in.t, "published height changed");
  }
  out.setpoint = follow(cfg_);
  if (guard_failed_) {
    failAttempt(in.t, "clearance guard");
  } else if (g != GuardAction::kClimb && atPathEnd(true)) {
    startDose(in.t);
  }
}

void Navigator::stepDose(const Inputs& in, Outputs& out) {
  out.setpoint = spray_pt_;
  const GuardAction g = guardUpdate(in);
  if (mark_.active) {   // dosed: waiting for set_tree_sprayed, nozzle off, phase HOPPING with the dosed id
    pollMark(in, out);
    if (state_ == NavState::kDose && mark_.done) finishTarget(in.t);
    return;
  }
  if (guard_failed_) {
    failAttempt(in.t, "clearance guard");
    return;
  }
  if (dose_.done(in.t)) {
    dose_.update(in.t, false);   // credits the last interval before validity is read
    dose_running_ = false;
    if (dose_.valid()) {
      mark_ = Mark{};
      mark_.active = true;
      mark_.first_id = target_;
      mark_.centre = target_centre_;
      sendMark(target_, in.t, out);
    } else {
      failAttempt(in.t, "dose invalid");
    }
    return;
  }
  out.nozzle = g != GuardAction::kClimb && distXY(pos_, spray_pt_) <= cfg_.xy_tol;
}

void Navigator::stepReturn(const Inputs& in, Outputs& out) {
  out.setpoint = follow(cfg_);
  if (atPathEnd(false)) startLand(in.t);
}

void Navigator::stepLand(const Inputs& in, Outputs& out) {
  out.setpoint = follow(cfg_);
  if (atPathEnd(true)) {
    hold_ = home_;
    enter(NavState::kDone, in.t);
  }
}

// ---------------------------------------------------------------------------------------------------------------
// transitions

void Navigator::startTransit(double t) {
  const ScanLeg& leg = scan_[scan_idx_];
  corridor_ = leg.corridor;
  from_id_ = kNoTree;
  climbing_ = blind_ = false;
  setPath(makePath(transitPoints(pos_, scanStart(leg))));
  enter(NavState::kTransit, t);
}

void Navigator::startScan(double t) {
  const ScanLeg& leg = scan_[scan_idx_];
  setPath(makePath({scanStart(leg), scanEnd(leg)}));
  enter(NavState::kScanOut, t);
}

void Navigator::startSettle(double t) {
  hold_ = scanEnd(scan_[scan_idx_]);
  settle_ids_ = treesInCorridor(corridor_);
  settle_gain_t_ = t;
  enter(NavState::kScanSettle, t);
}

void Navigator::startLock(double t) {
  enter(NavState::kLock, t);
  from_id_ = kNoTree;
  targets_.clear();
  next_target_ = 0;
  if (cfg_.strategy == Strategy::kCorridor) {
    locked_upto_ = corridor_;
    const std::vector<TreeId> ids =
        updateAssignment(map_, corridors_, field_.polygon, corridor_, true, &assignment_, &counters_);
    targets_ = orderStops(ids, hold_, entryAt(corridor_), corridor_);
  } else {
    // field_first: one lock over every corridor, then spray them back in reverse scan order, each from the end
    // nearer to where the previous one finished
    std::map<int, std::vector<TreeId>> ids;
    for (const Corridor& c : corridors_) {
      locked_upto_ = c.index;
      ids[c.index] = updateAssignment(map_, corridors_, field_.polygon, c.index, true, &assignment_, &counters_);
    }
    Point3 p = hold_;
    for (auto it = scan_.rbegin(); it != scan_.rend(); ++it) {
      const Point3 a = entryAt(it->corridor), b = exitAt(it->corridor);
      const Point3 end = distXY(p, a) >= distXY(p, b) ? a : b;
      for (const Target& tg : orderStops(ids[it->corridor], p, end, it->corridor)) targets_.push_back(tg);
      p = end;
    }
  }
  if (log_) log_("LOCK: " + std::to_string(targets_.size()) + " targets");
}

void Navigator::startGlide(double t) {
  attempt_climbs_ = 0;
  guard_ok_t_ = t;
  planToTarget(t, nullptr);
  enter(NavState::kGlide, t);
}

void Navigator::startDose(double t) {
  dose_.start(t, doseTime(target_area_, cfg_));
  dose_.update(t, false);
  dose_running_ = true;
  mark_ = Mark{};
  hold_ = spray_pt_;
  enter(NavState::kDose, t);
}

void Navigator::startReturn(double t) {
  abandonMark();
  dose_running_ = false;
  climbing_ = blind_ = false;
  corridor_ = -1;
  cleanup_mode_ = false;
  target_ = kNoTree;
  setPath(makePath(transitPoints(pos_, home_)));
  enter(NavState::kReturn, t);
}

void Navigator::startLand(double t) {
  setPath(makePath({pos_, home_}));
  enter(NavState::kLand, t);
}

void Navigator::finishTarget(double t) {
  attempted_.insert(target_);
  attempted_.insert(mark_.first_id);   // the id it was dosed under, if the mark re-resolved it
  attempted_.erase(kNoTree);
  mark_ = Mark{};
  from_id_ = target_;   // the drone is at its spray point
  advance(t);
}

void Navigator::failAttempt(double t, const std::string& why) {
  dose_running_ = false;
  if (!resolveTarget()) {   // the tree left the map during the attempt (e.g. merged mid-dose): nothing to retry
    from_id_ = target_;
    advance(t);
    return;
  }
  if (!retried_) {
    retried_ = true;
    count(counters_, "dose_retries");
    if (log_) log_("tree " + std::to_string(target_) + ": " + why + ", retrying");
    startGlide(t);
    return;
  }
  count(counters_, "trees_skipped");
  if (++skips_[target_] >= 2) {
    attempted_.insert(target_);   // skipped twice: stays unsprayed, reported
    ++skipped_;
    warnings_.push_back("tree " + std::to_string(target_) + " skipped twice, left unsprayed");
  } else if (std::find(assignment_.cleanup.begin(), assignment_.cleanup.end(), target_) ==
             assignment_.cleanup.end()) {
    assignment_.cleanup.push_back(target_);   // one more try from the cleanup list
  }
  if (log_) log_("tree " + std::to_string(target_) + ": " + why + ", skipped");
  from_id_ = target_;
  advance(t);
}

void Navigator::advance(double t) {
  for (int pass = 0; pass < 2; ++pass) {
    while (next_target_ < targets_.size()) {
      const Target tg = targets_[next_target_++];
      TreeId id = tg.id;
      if (!tree(id)) {   // merged or split away since it was planned
        id = reresolve(tg.centre);
        count(counters_, id == kNoTree ? "ids_dropped_vanished" : "ids_reresolved");
        if (log_) log_("tree " + std::to_string(tg.id) + " vanished -> " + std::to_string(id));
        if (id == kNoTree) continue;
      }
      const TreeEntry* tr = tree(id);
      if (!tr || tr->sprayed || attempted_.count(id)) continue;   // never spray twice
      target_ = id;
      corridor_ = cleanup_mode_ ? -1 : tg.corridor;
      retried_ = false;
      startGlide(t);
      return;
    }
    if (!cleanup_mode_ && cfg_.strategy == Strategy::kCorridor && scan_idx_ + 1 < scan_.size()) {
      ++scan_idx_;
      startTransit(t);
      return;
    }
    // corridors exhausted: the cleanup list (late trees; may hold ids sprayed since), from here towards home
    std::vector<TreeId> ids;
    for (TreeId id : assignment_.cleanup) {
      const TreeEntry* tr = tree(id);
      if (tr && !tr->sprayed && !attempted_.count(id) && std::find(ids.begin(), ids.end(), id) == ids.end()) {
        ids.push_back(id);
      }
    }
    if (ids.empty()) break;
    cleanup_mode_ = true;
    targets_ = orderStops(ids, pos_, {home_.x, home_.y, z_scan_abs_}, -1);
    next_target_ = 0;
  }
  startReturn(t);
}

// ---------------------------------------------------------------------------------------------------------------
// helpers

Point3 Navigator::entryAt(int corridor) const {
  const Corridor& c = corridors_[static_cast<std::size_t>(corridor)];
  return {c.entry.x, c.entry.y, z_scan_abs_};
}

Point3 Navigator::exitAt(int corridor) const {
  const Corridor& c = corridors_[static_cast<std::size_t>(corridor)];
  return {c.exit.x, c.exit.y, z_scan_abs_};
}

Point3 Navigator::scanStart(const ScanLeg& leg) const {
  return leg.reversed ? exitAt(leg.corridor) : entryAt(leg.corridor);
}

Point3 Navigator::scanEnd(const ScanLeg& leg) const {
  return leg.reversed ? entryAt(leg.corridor) : exitAt(leg.corridor);
}

std::vector<Point3> Navigator::transitPoints(const Point3& from, const Point3& to) const {
  return support::transitPoints(from, to, z_scan_abs_);
}

void Navigator::setPath(Path3 path) {
  path_ = std::move(path);
  leg_ = 0;
}

bool Navigator::reached(const Point3& p) const {
  return distXY(pos_, p) <= cfg_.xy_tol && std::fabs(pos_.z - p.z) <= cfg_.z_tol;
}

Point3 Navigator::follow(const Config& cfg) {
  // carrot() has no progress memory and would cut corners: stream it one leg at a time, next leg once this one's
  // end is reached, so climbs stay vertical and overlapping climb / descent legs cannot trap the projection
  const std::vector<Point3>& pts = path_.pts;
  while (leg_ + 2 < pts.size() && reached(pts[leg_ + 1])) ++leg_;
  Path3 leg;
  leg.pts = {pts[leg_], pts[leg_ + 1]};
  return carrot(leg, pos_, cfg);
}

bool Navigator::atPathEnd(bool need_slow) const {
  return leg_ + 2 >= path_.pts.size() && reached(path_.pts.back()) && (!need_slow || speed_ < cfg_.arrive_speed);
}

const TreeEntry* Navigator::tree(TreeId id) const { return support::findTree(map_, id); }

std::set<TreeId> Navigator::treesInCorridor(int k) const {
  std::set<TreeId> ids;
  if (k < 0) return ids;
  const treemapper::Polygon2& strip = corridors_[static_cast<std::size_t>(k)].strip;
  for (const TreeEntry& t : map_) {
    if (treemapper::pointInPolygon(strip, t.centre)) ids.insert(t.id);
  }
  return ids;
}

std::vector<Navigator::Target> Navigator::orderStops(const std::vector<TreeId>& ids, const Point3& start,
                                                     const Point3& end, int corridor) const {
  std::vector<Target> out;
  for (const Stop& s : support::orderStops(ids, map_, start, end, z_scan_abs_, cfg_, field_.polygon)) {
    if (const TreeEntry* tr = tree(s.id)) out.push_back({s.id, corridor, tr->centre});
  }
  return out;
}

// ---------------------------------------------------------------------------------------------------------------
// queries

Status Navigator::status() const {
  Status s;
  s.state = state_;
  const bool on_target = onTarget();
  s.cleanup = cleanup_mode_ && on_target;
  s.corridor = corridor_;
  s.corridors_total = static_cast<int>(corridors_.size());
  s.target = state_ == NavState::kHold ? last_target_ : on_target ? target_ : kNoTree;
  s.sprayed = sprayed_;
  s.skipped = skipped_;
  std::set<TreeId> pending;
  for (const auto& kv : assignment_.corridor_of) pending.insert(kv.first);
  pending.insert(assignment_.cleanup.begin(), assignment_.cleanup.end());
  for (TreeId id : pending) {
    const TreeEntry* t = tree(id);
    if (t && !t->sprayed && !attempted_.count(id)) ++s.remaining;
  }
  s.clearance_m = clearance_m_;
  s.warnings = warnings_;
  return s;
}

std::vector<Path3> Navigator::plannedPaths() const {
  std::vector<Path3> out;
  const bool flying = state_ != NavState::kIdle && state_ != NavState::kAbort && state_ != NavState::kDone;
  if (flying && path_.pts.size() >= 2) out.push_back(path_);
  const bool on_target = onTarget();
  Point3 from = on_target ? spray_pt_ : pos_;
  TreeId from_id = on_target ? target_ : from_id_;
  for (std::size_t i = next_target_; i < targets_.size(); ++i) {
    const TreeEntry* t = tree(targets_[i].id);
    if (!t || t->sprayed || attempted_.count(t->id)) continue;
    Path3 p = glideTo(from, from_id, *t, nullptr);
    from = p.pts.back();
    from_id = t->id;
    out.push_back(std::move(p));
  }
  return out;
}

}  // namespace corridor_nav
