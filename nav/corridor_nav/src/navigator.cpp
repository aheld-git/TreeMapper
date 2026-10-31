#include "corridor_nav/navigator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "corridor_nav/corridors.hpp"
#include "corridor_nav/ordering.hpp"
#include "treemapper/geometry.hpp"

namespace corridor_nav {

namespace {
using treemapper::kNoTree;
using treemapper::Point2;
using treemapper::TreeEntry;
using treemapper::TreeId;

constexpr double kZeroLength = 1e-9;        ///< m, consecutive path points closer than this are merged
constexpr double kHeightSampleStep = 0.25;  ///< m, xy sampling of a path when collecting the trees near it
constexpr double kPairCostNs = 5.0;         ///< ns charged per stop pair and 2-opt pass (deterministic order budget)

void count(Counters& counters, const char* name) { ++counters[name]; }

/// A tree the planners can use: finite centre, height and outline, at least a triangle.
bool usable(const TreeEntry& t) {
  if (!std::isfinite(t.height) || !std::isfinite(t.centre.x) || !std::isfinite(t.centre.y)) return false;
  if (t.outline.size() < 3) return false;
  return std::all_of(t.outline.begin(), t.outline.end(),
                     [](const Point2& p) { return std::isfinite(p.x) && std::isfinite(p.y); });
}

double yawOf(const treemapper::DronePose& p) {
  return std::atan2(2.0 * (p.qw * p.qz + p.qx * p.qy), 1.0 - 2.0 * (p.qy * p.qy + p.qz * p.qz));
}

double distXY(const Point3& a, const Point3& b) { return std::hypot(a.x - b.x, a.y - b.y); }

double dist3(const Point3& a, const Point3& b) { return std::hypot(distXY(a, b), a.z - b.z); }

/// Polyline without zero-length legs; always at least two points (a lone point is repeated).
Path3 makePath(const std::vector<Point3>& pts) {
  Path3 path;
  for (const Point3& p : pts) {
    if (path.pts.empty() || dist3(path.pts.back(), p) >= kZeroLength) path.pts.push_back(p);
  }
  if (path.pts.size() == 1) path.pts.push_back(path.pts.front());
  return path;
}

constexpr double kInf = std::numeric_limits<double>::infinity();

/// Axis-aligned xy bounding box, empty until the first add().
struct Box {
  double x0{kInf}, y0{kInf}, x1{-kInf}, y1{-kInf};
  void add(double x, double y) {
    x0 = std::min(x0, x);
    y0 = std::min(y0, y);
    x1 = std::max(x1, x);
    y1 = std::max(y1, y);
  }
  bool overlaps(const Box& o, double grow) const {
    return x0 - grow <= o.x1 && o.x0 <= x1 + grow && y0 - grow <= o.y1 && o.y0 <= y1 + grow;
  }
};

Box outlineBox(const TreeEntry& t) {
  Box b;
  for (const Point2& p : t.outline) b.add(p.x, p.y);
  return b;
}

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
    case NavState::kHold:    // entered by the hold logic (map stale, no pose, service); hover at hold_
    case NavState::kAbort:   // planning failed on the ground: stay where we are
    case NavState::kDone: break;
  }
  return finish(before, std::move(out));
}

void Navigator::ingest(const Inputs& in) {
  if (in.pose.valid) {
    const Point3 p{in.pose.x, in.pose.y, in.pose.z};
    if (!have_pose_ || in.pose.t > pose_t_) {
      if (have_pose_) speed_ = dist3(p, pos_) / (in.pose.t - pose_t_);
      pos_ = p;
      pose_t_ = in.pose.t;
      have_pose_ = true;
    }
  }
  clearance_m_ = in.clearance.valid ? in.clearance.metres : NAN;
  if (!in.map) return;
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

Outputs Navigator::finish(NavState before, Outputs out) {
  // The nozzle opens only in a DOSE that neither starts nor ends on this tick, and never while waiting for the mark.
  out.nozzle = out.nozzle && before == NavState::kDose && state_ == NavState::kDose && !awaiting_mark_;
  const bool on_target = state_ == NavState::kGlide || state_ == NavState::kDose;
  out.target = on_target ? target_ : kNoTree;
  const bool corridor_work = state_ == NavState::kTransit || state_ == NavState::kScanOut ||
                             state_ == NavState::kScanSettle || state_ == NavState::kLock || on_target;
  if (corridor_work && corridor_ >= 0) yaw_ = corridors_[static_cast<std::size_t>(corridor_)].heading_rad;
  out.yaw = yaw_;
  out.phase = phase();
  last_phase_ = out.phase;
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
    case NavState::kDose: return awaiting_mark_ ? "HOPPING" : "SPRAYING";
    case NavState::kReturn:
    case NavState::kLand: return "RETURN";
    case NavState::kHold: return last_phase_;   // unchanged
    case NavState::kAbort: return "ERROR";
    case NavState::kDone: return "IDLE";
  }
  return "ERROR";
}

// ---------------------------------------------------------------------------------------------------------------
// state handlers

void Navigator::stepIdle(const Inputs& in, Outputs& out) {
  hold_ = pos_;
  out.setpoint = pos_;
  if (in.pose.valid) yaw_ = yawOf(in.pose);
  if (!in.start_requested || !in.pose.valid) return;

  home_ = {field_.home.x, field_.home.y, pos_.z};
  z_scan_abs_ = pos_.z + cfg_.z_scan;
  const CorridorPlan plan = planCorridors(field_, fov_, cfg_, &counters_);
  if (!plan.ok || plan.corridors.empty()) {
    const std::string why = plan.ok ? "no corridor long enough" : plan.error;
    warnings_.push_back("planning failed: " + why);
    if (log_) log_("corridor planning failed: " + why);
    enter(NavState::kAbort, in.t);
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
  out.setpoint = follow(cfg_);
  if (atPathEnd(true)) startDose(in.t);
}

void Navigator::stepDose(const Inputs& in, Outputs& out) {
  out.setpoint = spray_pt_;
  if (awaiting_mark_) {
    if (!in.spray_result || in.spray_result->id != target_) return;
    if (in.spray_result->ok) {
      ++sprayed_;
    } else {
      count(counters_, "spray_mark_failed");
    }
    finishTarget(in.t);
    return;
  }
  const bool gate = distXY(pos_, spray_pt_) <= cfg_.xy_tol;
  if (dose_.done(in.t)) {
    dose_.update(in.t, false);   // credits the last interval before validity is read
    if (dose_.valid()) {
      out.request_sprayed = target_;
      awaiting_mark_ = true;
      return;
    }
    ++skipped_;
    count(counters_, "trees_skipped");
    if (log_) log_("tree " + std::to_string(target_) + ": dose invalid, skipped");
    finishTarget(in.t);
    return;
  }
  dose_.update(in.t, gate);
  out.nozzle = gate;
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
  const TreeEntry& to = *tree(target_);
  setPath(planGlide(pos_, from_id_, to, map_, cfg_, &counters_));
  spray_pt_ = path_.pts.back();
  target_area_ = std::fabs(treemapper::polygonArea(to.outline));
  planned_heights_ = heightsNear(path_, target_);
  enter(NavState::kGlide, t);
}

void Navigator::startDose(double t) {
  dose_.start(t, doseTime(target_area_, cfg_));
  dose_.update(t, false);
  awaiting_mark_ = false;
  hold_ = spray_pt_;
  enter(NavState::kDose, t);
}

void Navigator::startReturn(double t) {
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
  from_id_ = target_;   // the drone is at its spray point
  awaiting_mark_ = false;
  advance(t);
}

void Navigator::advance(double t) {
  for (int pass = 0; pass < 2; ++pass) {
    while (next_target_ < targets_.size()) {
      const Target tg = targets_[next_target_++];
      const TreeEntry* tr = tree(tg.id);
      if (!tr || tr->sprayed || attempted_.count(tg.id)) continue;   // never spray twice
      target_ = tg.id;
      corridor_ = cleanup_mode_ ? -1 : tg.corridor;
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
  return {from, {from.x, from.y, std::max(from.z, z_scan_abs_)}, {to.x, to.y, z_scan_abs_}};
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

const TreeEntry* Navigator::tree(TreeId id) const {
  const auto it = std::find_if(map_.begin(), map_.end(), [id](const TreeEntry& t) { return t.id == id; });
  return it == map_.end() ? nullptr : &*it;
}

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
  std::vector<Stop> stops;
  Box box;
  box.add(start.x, start.y);
  box.add(end.x, end.y);
  for (TreeId id : ids) {
    const TreeEntry* tr = tree(id);
    if (!tr || tr->sprayed) continue;
    stops.push_back({id, sprayPoint(*tr, map_, cfg_, nullptr)});
    box.add(stops.back().p.x, stops.back().p.y);
  }
  if (stops.empty()) return {};

  // Only trees within reach of the stops' bounding box can constrain a glide between them.
  const double reach = cfg_.drone_radius + cfg_.lateral_margin;
  std::vector<TreeEntry> local;
  for (const TreeEntry& t : map_) {
    if (outlineBox(t).overlaps(box, reach)) local.push_back(t);
  }
  const auto stopTree = [&](const Point3& p) -> const TreeEntry* {
    for (const Stop& s : stops) {
      if (s.p.x == p.x && s.p.y == p.y && s.p.z == p.z) return tree(s.id);
    }
    return nullptr;
  };
  // cost = glide time; counters are not touched (only the glide actually flown is counted)
  const CostFn cost = [&](const Point3& a, const Point3& b) {
    const TreeEntry* to = stopTree(b);
    if (!to) return pathTime(makePath(transitPoints(a, b)), cfg_);   // b = the end: climb out, then level
    const TreeEntry* from = stopTree(a);
    return pathTime(planGlide(a, from ? from->id : kNoTree, *to, local, cfg_, nullptr), cfg_);
  };
  // Deterministic budget: each 2-opt pass is charged n^2 pair evaluations, not wall time, so the same inputs always
  // give the same order.
  const double n = static_cast<double>(stops.size());
  const double pass_ms = n * n * kPairCostNs * 1e-6;
  double clock_ms = -pass_ms;
  const auto now_ms = [&]() { return clock_ms += pass_ms; };

  // orderTargets sorts by descending axial coordinate along the heading: point it from `end` towards `start`
  const double heading = std::atan2(start.y - end.y, start.x - end.x);
  std::vector<Target> out;
  for (const Stop& s : orderTargets(stops, start, end, heading, cost, cfg_.order_budget_ms, now_ms)) {
    out.push_back({s.id, corridor});
  }
  return out;
}

std::map<TreeId, double> Navigator::heightsNear(const Path3& path, TreeId target) const {
  std::map<TreeId, double> out;
  if (const TreeEntry* t = tree(target)) out[target] = t->height;
  std::vector<Point2> samples;
  Box box;
  for (std::size_t i = 0; i + 1 < path.pts.size(); ++i) {
    const Point3 &a = path.pts[i], &b = path.pts[i + 1];
    const int n = std::max(1, static_cast<int>(std::ceil(distXY(a, b) / kHeightSampleStep)));
    for (int j = 0; j <= n; ++j) {
      const double f = static_cast<double>(j) / n;
      samples.push_back({a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)});
      box.add(samples.back().x, samples.back().y);
    }
  }
  // half a sample step of slack so a tree between two samples is not missed
  const double reach = cfg_.drone_radius + cfg_.lateral_margin + kHeightSampleStep / 2;
  for (const TreeEntry& k : map_) {
    if (!outlineBox(k).overlaps(box, reach)) continue;
    for (const Point2& s : samples) {
      if (distanceToPolygon(k.outline, s) <= reach) {
        out[k.id] = k.height;
        break;
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------------------------------------------
// queries

Status Navigator::status() const {
  Status s;
  s.state = state_;
  const bool on_target = state_ == NavState::kGlide || state_ == NavState::kDose;
  s.cleanup = cleanup_mode_ && on_target;
  s.corridor = corridor_;
  s.corridors_total = static_cast<int>(corridors_.size());
  s.target = on_target ? target_ : kNoTree;
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
  const bool on_target = state_ == NavState::kGlide || state_ == NavState::kDose;
  Point3 from = on_target ? spray_pt_ : pos_;
  TreeId from_id = on_target ? target_ : from_id_;
  for (std::size_t i = next_target_; i < targets_.size(); ++i) {
    const TreeEntry* t = tree(targets_[i].id);
    if (!t || t->sprayed || attempted_.count(t->id)) continue;
    Path3 p = planGlide(from, from_id, *t, map_, cfg_, nullptr);
    from = p.pts.back();
    from_id = t->id;
    out.push_back(std::move(p));
  }
  return out;
}

}  // namespace corridor_nav
