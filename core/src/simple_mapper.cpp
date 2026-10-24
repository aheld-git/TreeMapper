#include "treemapper/simple_mapper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace treemapper {

namespace {
double percentile(std::vector<double> v, double p) {
  if (v.empty()) return std::nan("");
  const double k = std::clamp(p, 0.0, 100.0) / 100.0 * static_cast<double>(v.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(k));
  std::nth_element(v.begin(), v.begin() + static_cast<long>(lo), v.end());
  const double a = v[lo];
  if (lo + 1 >= v.size()) return a;
  const double b = *std::min_element(v.begin() + static_cast<long>(lo) + 1, v.end());
  return a + (k - static_cast<double>(lo)) * (b - a);
}
double dist(const Point2& a, const Point2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
}  // namespace

PhasePolicy phasePolicy(MissionPhase phase, TreeId hop_target) {
  PhasePolicy p;
  switch (phase) {
    case MissionPhase::kScan:
    case MissionPhase::kScanApproach:
    case MissionPhase::kError:  // a fault is when you most want to see; stay permissive
      break;
    case MissionPhase::kUnknown:
      p.warn_unknown = true;  // fail open: behave as scanning (brief section 6)
      break;
    case MissionPhase::kPlanning:
      p.may_create = false;   // tree list is fixed (brief section 6)
      p.may_merge = false;    // a merge would retire an ID navigation has planned against
      break;
    case MissionPhase::kHopping:
      p.may_create = false;
      p.may_merge = false;
      // Only the target may change. hop_target < 0 -> nothing may change (fail closed).
      p.freeze_non_target = true;
      (void)hop_target;
      break;
    case MissionPhase::kSpraying:
    case MissionPhase::kIdle:
    case MissionPhase::kReturn:
      p.awake = false;
      p.may_create = false;
      p.may_merge = false;
      p.freeze_non_target = true;
      break;
  }
  return p;
}

SimpleMapper::SimpleMapper(const SimpleMapperConfig& cfg, const StereoCalibration& cal)
    : cfg_(cfg), cal_(cal), log_([](const std::string& m) { std::cerr << "[treemapper] " << m << "\n"; }) {}

void SimpleMapper::warnOnce(const std::string& key, const std::string& msg) {
  if (warned_[key]) return;
  warned_[key] = true;
  if (log_) log_(msg);
}

void SimpleMapper::process(const FrameInput& f) {
  const PhasePolicy pol = phasePolicy(f.phase, f.hop_target);
  count("frames_total");
  if (pol.warn_unknown) warnOnce("phase", "mission phase unknown/missing: behaving as SCAN (fail open)");
  if (!pol.awake) {
    state_ = MapperState::kSleeping;
    count("frames_skipped_sleeping");
    return;
  }
  if (!f.pose.valid) {
    state_ = MapperState::kHoldingNoPose;
    count("frames_held_no_pose");
    warnOnce("pose", "drone pose not valid: holding all processing (fail closed)");
    return;
  }
  state_ = MapperState::kAwake;
  if (f.disparity.empty()) {
    count(f.boxes.empty() ? "frames_heartbeat_only" : "frames_dropped_no_disparity");
    return;
  }
  const auto t0 = Clock::now();
  const auto obs = extractObservations(f);
  stage_ms_["extract"] += msSince(t0);
  const auto t1 = Clock::now();
  integrate(f.t, f.phase, f.hop_target, obs);
  stage_ms_["integrate"] += msSince(t1);
  ++stage_frames_;
}

std::vector<Observation> SimpleMapper::extractObservations(const FrameInput& f) {
  std::vector<Observation> out;
  const int gw = f.grid_w, gh = f.grid_h;
  if (gw <= 0 || gh <= 0 || static_cast<int>(f.disparity.size()) != gw * gh) {
    count("frames_dropped_bad_disparity_size");
    return out;
  }
  const Transform world_body{f.pose.x, f.pose.y, f.pose.z, f.pose.qx, f.pose.qy, f.pose.qz, f.pose.qw};
  const Transform world_cam = world_body.compose(cal_.body_cam);
  const int s = std::max(1, cal_.stride), o = s / 2;
  // back-project the whole grid once (the ground estimate needs every point)
  std::vector<Vec3> P(static_cast<std::size_t>(gw * gh));
  std::vector<char> ok(P.size(), 0);
  std::vector<double> zs;
  zs.reserve(P.size());
  for (int r = 0; r < gh; ++r) {
    for (int c = 0; c < gw; ++c) {
      const std::size_t k = static_cast<std::size_t>(r * gw + c);
      Vec3 po;
      if (!backprojectOptical(cal_, c * s + o, r * s + o, f.disparity[k], &po)) continue;
      if (po.z < cfg_.min_range || po.z > cfg_.max_range) continue;
      P[k] = world_cam.apply(po);
      ok[k] = 1;
      zs.push_back(P[k].z);
    }
  }
  const double ground = percentile(zs, cfg_.ground_percentile);
  for (const auto& b : f.boxes) {
    const double cx = b.x + 0.5 * b.width, cy = b.y + 0.5 * b.height;
    const double hw = 0.5 * b.width * cfg_.box_shrink, hh = 0.5 * b.height * cfg_.box_shrink;
    const int c0 = std::max(0, static_cast<int>(std::ceil((cx - hw - o) / s)));
    const int c1 = std::min(gw - 1, static_cast<int>(std::floor((cx + hw - o) / s)));
    const int r0 = std::max(0, static_cast<int>(std::ceil((cy - hh - o) / s)));
    const int r1 = std::min(gh - 1, static_cast<int>(std::floor((cy + hh - o) / s)));
    std::vector<Point2> xy;
    std::vector<double> z;
    for (int r = r0; r <= r1; ++r) {
      for (int c = c0; c <= c1; ++c) {
        const std::size_t k = static_cast<std::size_t>(r * gw + c);
        if (!ok[k]) { count("points_dropped_invalid_or_range"); continue; }
        const Vec3& p = P[k];
        if (p.z > f.pose.z - cfg_.above_drone_margin) { count("points_dropped_above_drone"); continue; }
        if (!(p.z >= ground + cfg_.min_above_ground)) { count("points_dropped_ground"); continue; }
        xy.push_back({p.x, p.y});
        z.push_back(p.z);
      }
    }
    count("points_kept", xy.size());
    if (static_cast<int>(xy.size()) < cfg_.min_obs_points) { count("obs_dropped_too_few_points"); continue; }
    Observation ob;
    ob.n_points = static_cast<int>(xy.size());
    ob.height = percentile(z, cfg_.height_percentile);
    ob.hull = convexHull(std::move(xy));
    if (ob.hull.size() < 3 || !std::isfinite(ob.height)) { count("obs_dropped_degenerate"); continue; }
    if (std::fabs(polygonArea(ob.hull)) > cfg_.max_obs_area) { count("obs_dropped_too_large"); continue; }
    ob.centroid = polygonCentroid(ob.hull);
    out.push_back(std::move(ob));
  }
  return out;
}

void SimpleMapper::integrate(double t, MissionPhase phase, TreeId target, const std::vector<Observation>& obs) {
  const PhasePolicy pol = phasePolicy(phase, target);
  if (!pol.awake) { count("obs_dropped_sleeping", obs.size()); return; }
  if (pol.freeze_non_target && target < 0) {
    warnOnce("notarget", "HOPPING without a valid hop target: freezing every tree (fail closed)");
  }
  for (const auto& ob : obs) {
    if (!std::isfinite(ob.height) || ob.hull.size() < 3) { count("obs_dropped_degenerate"); continue; }
    // nearest entry whose outline contains the centroid, or within assoc_max_dist of its centre
    Entry* best = nullptr;
    double best_d = 1e18;
    for (auto& kv : entries_) {
      Entry& e = kv.second;
      const Point2 c = polygonCentroid(e.outline);
      const double d = dist(c, ob.centroid);
      if ((d < cfg_.assoc_max_dist || pointInPolygon(e.outline, ob.centroid)) && d < best_d) {
        best = &e;
        best_d = d;
      }
    }
    if (best == nullptr) {
      bool in_sprayed = false;
      for (const auto& kv : entries_) in_sprayed |= kv.second.sprayed && pointInPolygon(kv.second.outline, ob.centroid);
      if (in_sprayed) { count("obs_dropped_inside_sprayed"); continue; }
      if (!pol.may_create) { count("obs_dropped_new_tree_locked_phase"); continue; }
      Entry e;
      e.id = next_id_++;
      e.outline = ob.hull;
      e.height_sum = ob.height;
      e.n_obs = 1;
      e.first_seen = e.last_seen = t;
      e.published = cfg_.publish_min_obs <= 1;
      entries_[e.id] = e;
      count("entries_created");
      continue;
    }
    if (best->sprayed) { count("obs_dropped_sprayed_tree_frozen"); continue; }
    // An unconfirmed entry must not become a published tree once the list is locked: publishing it
    // after scanning would be a new tree navigation never planned against (brief section 6).
    if (!pol.may_create && !best->published) { count("obs_dropped_unconfirmed_locked_phase"); continue; }
    if (pol.freeze_non_target && best->id != target) { count("obs_dropped_non_target_frozen"); continue; }
    std::vector<Point2> pts = best->outline;
    pts.insert(pts.end(), ob.hull.begin(), ob.hull.end());
    best->outline = convexHull(std::move(pts));  // grow-only (rule 2)
    best->height_sum += ob.height;
    best->n_obs += 1;
    best->last_seen = t;
    if (!best->published && best->n_obs >= cfg_.publish_min_obs) {
      best->published = true;
      count("entries_published");
    }
  }
  // unpublished entries that stopped being seen are start-up artefacts / false positives
  for (auto it = entries_.begin(); it != entries_.end();) {
    if (!it->second.published && t - it->second.last_seen > cfg_.unconfirmed_timeout) {
      it = entries_.erase(it);
      count("entries_expired_unconfirmed");
    } else {
      ++it;
    }
  }
  if (!pol.may_merge) return;
  bool merged = true;
  while (merged) {
    merged = false;
    for (auto a = entries_.begin(); a != entries_.end() && !merged; ++a) {
      for (auto b = std::next(a); b != entries_.end() && !merged; ++b) {
        Entry& ea = a->second;
        Entry& eb = b->second;
        if (ea.sprayed || eb.sprayed) continue;
        const double aa = std::fabs(polygonArea(ea.outline)), ab = std::fabs(polygonArea(eb.outline));
        const Entry& small = aa <= ab ? ea : eb;
        const Entry& large = aa <= ab ? eb : ea;
        if (!pointInPolygon(large.outline, polygonCentroid(small.outline))) continue;
        const double inter = convexIntersectionArea(ea.outline, eb.outline);
        if (inter < cfg_.merge_min_overlap * std::min(aa, ab)) continue;
        // the better-established entry keeps its identity and height (rule 4)
        const bool a_keeps = ea.published != eb.published ? ea.published
                             : ea.n_obs != eb.n_obs     ? ea.n_obs > eb.n_obs
                                                        : ea.id < eb.id;
        Entry& keep = a_keeps ? ea : eb;
        Entry& lose = a_keeps ? eb : ea;
        std::vector<Point2> pts = keep.outline;
        pts.insert(pts.end(), lose.outline.begin(), lose.outline.end());
        keep.outline = convexHull(std::move(pts));
        keep.last_seen = std::max(keep.last_seen, lose.last_seen);
        const TreeId gone = lose.id;  // retired forever: next_id_ never goes back (rule 1)
        entries_.erase(gone);
        count("entries_merged");
        merged = true;
      }
    }
  }
}

std::vector<TreeEntry> SimpleMapper::publishedTrees() const {
  std::vector<TreeEntry> out;
  for (const auto& kv : entries_) {
    const Entry& e = kv.second;
    if (!e.published) continue;
    TreeEntry te;
    te.id = e.id;
    te.outline = e.outline;
    te.centre = polygonCentroid(e.outline);
    te.height = e.height();
    te.sprayed = e.sprayed;
    if (te.outline.size() < 3 || !std::isfinite(te.height) || !std::isfinite(te.centre.x) ||
        !std::isfinite(te.centre.y)) {
      continue;  // never publish an undefined height or a degenerate outline (rule 3)
    }
    out.push_back(std::move(te));
  }
  return out;
}

bool SimpleMapper::markSprayed(TreeId id) {
  auto it = entries_.find(id);
  if (it == entries_.end() || !it->second.published) {
    count("mark_sprayed_unknown_id");
    return false;
  }
  it->second.sprayed = true;
  return true;
}

std::map<std::string, double> SimpleMapper::takeStageTiming() {
  std::map<std::string, double> out;
  if (stage_frames_ > 0) {
    for (const auto& kv : stage_ms_) out[kv.first] = kv.second / stage_frames_;
  }
  stage_ms_.clear();
  stage_frames_ = 0;
  return out;
}

SimpleMapperConfig simpleMapperConfigFrom(const MapperConfig& config) {
  SimpleMapperConfig c;
  ConfigReader r(config);
  c.box_shrink = r.getDouble("box_shrink", c.box_shrink);
  c.min_range = r.getDouble("min_range", c.min_range);
  c.max_range = r.getDouble("max_range", c.max_range);
  c.above_drone_margin = r.getDouble("above_drone_margin", c.above_drone_margin);
  c.ground_percentile = r.getDouble("ground_percentile", c.ground_percentile);
  c.min_above_ground = r.getDouble("min_above_ground", c.min_above_ground);
  c.min_obs_points = r.getInt("min_obs_points", c.min_obs_points);
  c.height_percentile = r.getDouble("height_percentile", c.height_percentile);
  c.max_obs_area = r.getDouble("max_obs_area", c.max_obs_area);
  c.assoc_max_dist = r.getDouble("assoc_max_dist", c.assoc_max_dist);
  c.publish_min_obs = r.getInt("publish_min_obs", c.publish_min_obs);
  c.unconfirmed_timeout = r.getDouble("unconfirmed_timeout", c.unconfirmed_timeout);
  c.merge_min_overlap = r.getDouble("merge_min_overlap", c.merge_min_overlap);
  r.finish("simple");
  return c;
}

// Registered by name in mapper_registry.cpp (TM_REGISTER_BUILTIN(simple)).
std::unique_ptr<Mapper> tm_create_simple(const MapperConfig& config, const StereoCalibration& cal) {
  return std::make_unique<SimpleMapper>(simpleMapperConfigFrom(config), cal);
}

}  // namespace treemapper
