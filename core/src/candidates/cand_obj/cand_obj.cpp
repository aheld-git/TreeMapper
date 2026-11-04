#include "treemapper/candidates/cand_obj/cand_obj.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

#include "treemapper/simple_mapper.hpp"  // phasePolicy(): the shared pure phase-rule function

namespace treemapper {
namespace cand_obj {

namespace {
using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
double dist(const Point2& a, const Point2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
int floorDiv(double v, double cell) { return static_cast<int>(std::floor(v / cell)); }
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kMaxStepDt = 0.5;  ///< s, the rate limiter never takes one step longer than this

template <typename T>
T clampLog(const char* name, T v, T lo, T hi, const std::function<void(const std::string&)>& log) {
  // NaN compares false with everything, so it would pass the range test below and reach casts and indices
  // (height_quantile = nan -> an out-of-range nth_element index): reject it, and +-inf, outright
  if (!std::isfinite(static_cast<double>(v))) {
    throw std::invalid_argument("cand_obj config: " + std::string(name) + " = " + std::to_string(v) + " is not finite");
  }
  if (v < lo || v > hi) {
    const T c = std::clamp(v, lo, hi);
    if (log) log("cand_obj config: " + std::string(name) + " = " + std::to_string(v) + " clamped to " + std::to_string(c));
    return c;
  }
  return v;
}

/// Union-find over basin ids.
struct DSU {
  std::vector<int> p;
  explicit DSU(int n) : p(n) { std::iota(p.begin(), p.end(), 0); }
  int find(int x) {
    while (p[x] != x) { p[x] = p[p[x]]; x = p[x]; }
    return x;
  }
  void unite(int a, int b) { p[find(a)] = find(b); }  // a joins b
};
}  // namespace

// ------------------------------------------------------------------------------------------ config

Config Config::sanitized(const std::function<void(const std::string&)>& log) const {
  Config c = *this;
  c.min_range = clampLog("min_range", c.min_range, 0.0, 100.0, log);
  c.max_range = clampLog("max_range", c.max_range, c.min_range, 200.0, log);
  c.above_drone_margin = clampLog("above_drone_margin", c.above_drone_margin, 0.0, 10.0, log);
  c.max_frame_noise = clampLog("max_frame_noise", c.max_frame_noise, 0.0, 100.0, log);
  c.height_sample_clearance = clampLog("height_sample_clearance", c.height_sample_clearance, 0.0, 50.0, log);
  c.cell = clampLog("cell", c.cell, 0.1, 2.0, log);
  c.min_cell_points = clampLog("min_cell_points", c.min_cell_points, 1, 1000, log);
  c.max_grid = clampLog("max_grid", c.max_grid, 16, 1024, log);
  c.z_ref = clampLog("z_ref", c.z_ref, 0.1, 1000.0, log);
  c.lateral_k = clampLog("lateral_k", c.lateral_k, 0.0, 1000.0, log);
  c.rot_rate_ref = clampLog("rot_rate_ref", c.rot_rate_ref, 1e-3, 100.0, log);
  c.quality_recover_s = clampLog("quality_recover_s", c.quality_recover_s, 0.0, 60.0, log);
  c.ground_low_fraction = clampLog("ground_low_fraction", c.ground_low_fraction, 0.05, 0.9, log);
  c.ground_inlier_band = clampLog("ground_inlier_band", c.ground_inlier_band, 0.05, 5.0, log);
  c.ground_max_slope = clampLog("ground_max_slope", c.ground_max_slope, 0.0, 2.0, log);
  c.min_above_ground = clampLog("min_above_ground", c.min_above_ground, 0.0, 20.0, log);
  c.min_prominence = clampLog("min_prominence", c.min_prominence, 0.0, 20.0, log);
  c.min_basin_cells = clampLog("min_basin_cells", c.min_basin_cells, 1, 100000, log);
  c.box_dilate = clampLog("box_dilate", c.box_dilate, 0.0, 2.0, log);
  c.peak_border_px = clampLog("peak_border_px", c.peak_border_px, 0.0, 1000.0, log);
  c.window = clampLog("window", c.window, 4, 256, log);
  c.match_radius = clampLog("match_radius", c.match_radius, 0.0, 50.0, log);
  c.match_overlap = clampLog("match_overlap", c.match_overlap, 0.0, 1.0, log);
  c.seed_overlap = clampLog("seed_overlap", c.seed_overlap, 0.0, 1.0, log);
  c.match_reach = clampLog("match_reach", c.match_reach, 0.0, 50.0, log);
  c.max_cell_radius = clampLog("max_cell_radius", c.max_cell_radius, 0.0, 50.0, log);
  c.latch_rel_h = clampLog("latch_rel_h", c.latch_rel_h, 0.0, 1.0, log);
  c.latch_frames = clampLog("latch_frames", c.latch_frames, 1, 10000, log);
  c.latch_ratio = clampLog("latch_ratio", c.latch_ratio, 0.0, 1.0, log);
  c.confirm_frames = clampLog("confirm_frames", c.confirm_frames, 1, 10000, log);
  c.min_height_cells = clampLog("min_height_cells", c.min_height_cells, 1, 10000, log);
  c.min_publish_area = clampLog("min_publish_area", c.min_publish_area, 0.0, 1000.0, log);
  c.min_tree_height = clampLog("min_tree_height", c.min_tree_height, 0.0, 50.0, log);
  c.tentative_timeout = clampLog("tentative_timeout", c.tentative_timeout, 0.0, 1000.0, log);
  c.cell_weight_cap = clampLog("cell_weight_cap", c.cell_weight_cap, 1.0, 1e6, log);
  c.outlier_gate_m = clampLog("outlier_gate_m", c.outlier_gate_m, 0.0, 100.0, log);
  c.gate_min_weight = clampLog("gate_min_weight", c.gate_min_weight, 0.0, 1e6, log);
  c.height_quantile = clampLog("height_quantile", c.height_quantile, 0.0, 1.0, log);
  c.max_height_rate = clampLog("max_height_rate", c.max_height_rate, 0.0, 1000.0, log);
  c.max_centre_rate = clampLog("max_centre_rate", c.max_centre_rate, 0.0, 1000.0, log);
  c.centre_support_ratio = clampLog("centre_support_ratio", c.centre_support_ratio, 0.0, 1.0, log);
  c.merge_overlap = clampLog("merge_overlap", c.merge_overlap, 0.0, 1.0, log);
  c.merge_centre_dist = clampLog("merge_centre_dist", c.merge_centre_dist, 0.0, 50.0, log);
  c.post_spray_hold_radius = clampLog("post_spray_hold_radius", c.post_spray_hold_radius, 0.0, 100.0, log);
  return c;
}

Config configFrom(const MapperConfig& config) {
  Config c;
  ConfigReader r(config);
  c.min_range = r.getDouble("min_range", c.min_range);
  c.max_range = r.getDouble("max_range", c.max_range);
  c.above_drone_margin = r.getDouble("above_drone_margin", c.above_drone_margin);
  c.max_frame_noise = r.getDouble("max_frame_noise", c.max_frame_noise);
  c.height_sample_clearance = r.getDouble("height_sample_clearance", c.height_sample_clearance);
  c.cell = r.getDouble("cell", c.cell);
  c.min_cell_points = r.getInt("min_cell_points", c.min_cell_points);
  c.max_grid = r.getInt("max_grid", c.max_grid);
  c.z_ref = r.getDouble("z_ref", c.z_ref);
  c.lateral_k = r.getDouble("lateral_k", c.lateral_k);
  c.rot_rate_ref = r.getDouble("rot_rate_ref", c.rot_rate_ref);
  c.quality_recover_s = r.getDouble("quality_recover_s", c.quality_recover_s);
  c.ground_low_fraction = r.getDouble("ground_low_fraction", c.ground_low_fraction);
  c.ground_inlier_band = r.getDouble("ground_inlier_band", c.ground_inlier_band);
  c.ground_max_slope = r.getDouble("ground_max_slope", c.ground_max_slope);
  c.min_above_ground = r.getDouble("min_above_ground", c.min_above_ground);
  c.min_prominence = r.getDouble("min_prominence", c.min_prominence);
  c.min_basin_cells = r.getInt("min_basin_cells", c.min_basin_cells);
  c.box_dilate = r.getDouble("box_dilate", c.box_dilate);
  c.peak_border_px = r.getDouble("peak_border_px", c.peak_border_px);
  c.hull_cell_corners = r.getBool("hull_cell_corners", c.hull_cell_corners);
  c.seed_truncated = r.getBool("seed_truncated", c.seed_truncated);
  c.window = r.getInt("window", c.window);
  c.match_radius = r.getDouble("match_radius", c.match_radius);
  c.match_overlap = r.getDouble("match_overlap", c.match_overlap);
  c.seed_overlap = r.getDouble("seed_overlap", c.seed_overlap);
  c.match_reach = r.getDouble("match_reach", c.match_reach);
  c.max_cell_radius = r.getDouble("max_cell_radius", c.max_cell_radius);
  c.latch_rel_h = r.getDouble("latch_rel_h", c.latch_rel_h);
  c.latch_frames = r.getInt("latch_frames", c.latch_frames);
  c.latch_ratio = r.getDouble("latch_ratio", c.latch_ratio);
  c.confirm_frames = r.getInt("confirm_frames", c.confirm_frames);
  c.min_height_cells = r.getInt("min_height_cells", c.min_height_cells);
  c.min_publish_area = r.getDouble("min_publish_area", c.min_publish_area);
  c.min_tree_height = r.getDouble("min_tree_height", c.min_tree_height);
  c.tentative_timeout = r.getDouble("tentative_timeout", c.tentative_timeout);
  c.cell_weight_cap = r.getDouble("cell_weight_cap", c.cell_weight_cap);
  c.outlier_gate_m = r.getDouble("outlier_gate_m", c.outlier_gate_m);
  c.gate_min_weight = r.getDouble("gate_min_weight", c.gate_min_weight);
  c.height_quantile = r.getDouble("height_quantile", c.height_quantile);
  c.max_height_rate = r.getDouble("max_height_rate", c.max_height_rate);
  c.max_centre_rate = r.getDouble("max_centre_rate", c.max_centre_rate);
  c.centre_from_support = r.getBool("centre_from_support", c.centre_from_support);
  c.centre_support_ratio = r.getDouble("centre_support_ratio", c.centre_support_ratio);
  c.merge_overlap = r.getDouble("merge_overlap", c.merge_overlap);
  c.merge_centre_dist = r.getDouble("merge_centre_dist", c.merge_centre_dist);
  c.post_spray_hold_radius = r.getDouble("post_spray_hold_radius", c.post_spray_hold_radius);
  r.finish("cand_obj");
  return c;
}

double frameQuality(double rot_rate, double since_fast_s, const Config& c) {
  if (!(rot_rate >= 0.0)) return 1.0;  // unknown rate: fail open (normal weight)
  const double r = rot_rate / c.rot_rate_ref;
  double q = 1.0 / (1.0 + r * r);
  if (c.quality_recover_s > 0.0 && since_fast_s >= 0.0 && since_fast_s < c.quality_recover_s) {
    q = std::min(q, 0.1 + 0.9 * since_fast_s / c.quality_recover_s);
  }
  return q;
}

// ------------------------------------------------------------------------------------------ lifecycle

CandObj::CandObj(const Config& cfg, const StereoCalibration& cal)
    : cal_(cal), log_([](const std::string& m) { std::cerr << "[cand_obj] " << m << "\n"; }) {
  cfg_ = cfg.sanitized(log_);
}

void CandObj::warnOnce(const std::string& key, const std::string& msg) {
  if (warned_[key]) return;
  warned_[key] = true;
  if (log_) log_(msg);
}

PhasePolicy CandObj::policyFor(MissionPhase phase, TreeId target) {
  // Sticky lock (final review S4): once the tree list has been locked (PLANNING / HOPPING / SPRAYING), only an
  // explicit SCAN unlocks it again. ERROR, UNKNOWN and SCAN_APPROACH stay awake (phasePolicy) but neither create
  // nor merge nor fuse heights, like PLANNING: a fault or a lost /mission_state after the list was planned
  // against must not invent trees or retire IDs navigation holds.
  if (phase == MissionPhase::kPlanning || phase == MissionPhase::kHopping || phase == MissionPhase::kSpraying) {
    list_locked_ = true;
  } else if (phase == MissionPhase::kScan) {
    list_locked_ = false;
  }
  PhasePolicy p = phasePolicy(phase, target);
  if (list_locked_ && p.awake && (phase == MissionPhase::kError || phase == MissionPhase::kUnknown ||
                                  phase == MissionPhase::kScanApproach)) {
    p.may_create = false;
    p.may_merge = false;
    sticky_frame_ = true;
  }
  return p;
}

void CandObj::process(const FrameInput& f) {
  count("frames_total");
  const PhasePolicy pol = policyFor(f.phase, f.hop_target);
  if (pol.warn_unknown) warnOnce("phase", "mission phase unknown/missing: behaving as SCAN (fail open)");
  // Post-spray climb: when SPRAYING ends the drone is still low over the sprayed tree. Keep every
  // non-target frozen until it has moved post_spray_hold_radius away (fail closed; brief section 6).
  if (f.phase == MissionPhase::kSpraying) {
    if (f.pose.valid) { hover_x_ = f.pose.x; hover_y_ = f.pose.y; hover_known_ = true; }
  } else if (prev_phase_ == MissionPhase::kSpraying) {
    hold_after_spray_ = true;
    count("post_spray_holds_started");
  }
  prev_phase_ = f.phase;
  if (!pol.awake) {
    state_ = State::kSleeping;
    count("frames_skipped_sleeping");
    have_prev_ = false;
    return;
  }
  if (!f.pose.valid) {
    state_ = State::kHoldingNoPose;
    count("frames_held_no_pose");
    warnOnce("pose", "drone pose not valid: holding all processing (fail closed)");
    have_prev_ = false;
    return;
  }
  state_ = State::kAwake;
  if (f.disparity.empty()) {
    count(f.boxes.empty() ? "frames_heartbeat_only" : "frames_dropped_no_disparity");
    return;
  }
  // frame quality from the body rotation rate between consecutive frames (turn blur / noise bursts)
  double rate = -1.0;
  if (have_prev_) {
    const double dt = f.stamp - prev_stamp_;
    if (dt > 1e-4 && dt <= 0.5) {
      const double dot = std::fabs(pqx_ * f.pose.qx + pqy_ * f.pose.qy + pqz_ * f.pose.qz + pqw_ * f.pose.qw);
      rate = 2.0 * std::acos(std::min(1.0, dot)) / dt;
    }
  }
  if (rate < 0.0) count("frames_quality_unknown_rate");
  have_prev_ = true;
  prev_stamp_ = f.stamp;
  pqx_ = f.pose.qx; pqy_ = f.pose.qy; pqz_ = f.pose.qz; pqw_ = f.pose.qw;
  if (rate > cfg_.rot_rate_ref) last_fast_t_ = f.stamp;
  const double q = frameQuality(rate, f.stamp - last_fast_t_, cfg_);
  if (q < 0.5) count("frames_downweighted_rotation");

  const auto t0 = Clock::now();
  FrameView view;
  std::vector<Crown> crowns;
  extractCrowns(f, &view, &crowns, q);
  stage_ms_["chm"] += msSince(t0);
  const auto t1 = Clock::now();
  integrate(f.t, f.phase, f.hop_target, f.pose, view, crowns, q);
  stage_ms_["instances"] += msSince(t1);
  ++stage_frames_;
}

// ------------------------------------------------------------------------------------------ CHM + crowns

void CandObj::extractCrowns(const FrameInput& f, FrameView* view, std::vector<Crown>* crowns, double quality) {
  crowns->clear();
  *view = FrameView{};
  const int gw = f.grid_w, gh = f.grid_h;
  if (gw <= 0 || gh <= 0 || static_cast<int>(f.disparity.size()) != gw * gh) {
    count("frames_dropped_bad_disparity_size");
    return;
  }
  const Transform world_body{f.pose.x, f.pose.y, f.pose.z, f.pose.qx, f.pose.qy, f.pose.qz, f.pose.qw};
  const Transform world_cam = world_body.compose(cal_.body_cam);
  const int s = std::max(1, cal_.stride), o = s / 2;
  const std::size_t n = static_cast<std::size_t>(gw * gh);
  px_.assign(n, 0); py_.assign(n, 0); pz_.assign(n, 0); pu_.assign(n, 0); pv_.assign(n, 0); pw_.assign(n, 0); pok_.assign(n, 0);
  const double z_cap = f.pose.z - cfg_.above_drone_margin;
  double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
  std::size_t n_ok = 0;
  for (int r = 0; r < gh; ++r) {
    for (int c = 0; c < gw; ++c) {
      const std::size_t k = static_cast<std::size_t>(r * gw + c);
      Vec3 po;
      const double u = c * s + o, v = r * s + o;
      if (!backprojectOptical(cal_, u, v, f.disparity[k], &po)) { count("points_dropped_invalid"); continue; }
      if (po.z < cfg_.min_range || po.z > cfg_.max_range) { count("points_dropped_range"); continue; }
      const Vec3 w = world_cam.apply(po);
      if (w.z > z_cap) { count("points_dropped_above_drone"); continue; }
      // point weight: stereo depth error ~ Z^2, its lateral component ~ rho * sigma_Z (rho = off-axis tangent)
      const double rho2 = (po.x * po.x + po.y * po.y) / (po.z * po.z);
      const double zr = cfg_.z_ref / po.z;
      px_[k] = w.x; py_[k] = w.y; pz_[k] = w.z; pu_[k] = u; pv_[k] = v; pok_[k] = 1;
      pw_[k] = zr * zr * zr * zr / (1.0 + cfg_.lateral_k * rho2);
      xmin = std::min(xmin, w.x); xmax = std::max(xmax, w.x);
      ymin = std::min(ymin, w.y); ymax = std::max(ymax, w.y);
      ++n_ok;
    }
  }
  if (n_ok < 16) { count("frames_dropped_too_few_points"); return; }
  const double cell = cfg_.cell;
  const int i0 = floorDiv(xmin, cell), j0 = floorDiv(ymin, cell);
  const int W = std::min(cfg_.max_grid, floorDiv(xmax, cell) - i0 + 1);
  const int H = std::min(cfg_.max_grid, floorDiv(ymax, cell) - j0 + 1);
  const int NC = W * H;
  // --- bin points into cells (counting sort), per cell: count, mean pixel, mean weight, median z
  std::vector<int> cnt(NC, 0), cell_of(n, -1);
  for (std::size_t k = 0; k < n; ++k) {
    if (!pok_[k]) continue;
    const int i = floorDiv(px_[k], cell) - i0, j = floorDiv(py_[k], cell) - j0;
    if (i < 0 || j < 0 || i >= W || j >= H) { count("points_dropped_outside_grid_cap"); continue; }
    cell_of[k] = j * W + i;
    ++cnt[cell_of[k]];
  }
  std::vector<int> off(NC + 1, 0);
  for (int c = 0; c < NC; ++c) off[c + 1] = off[c] + cnt[c];
  std::vector<double> zs(static_cast<std::size_t>(off[NC]));
  std::vector<int> fill(off.begin(), off.end() - 1);
  std::vector<double> su(NC, 0.0), sv(NC, 0.0), sw(NC, 0.0);
  for (std::size_t k = 0; k < n; ++k) {
    const int c = cell_of[k];
    if (c < 0) continue;
    zs[static_cast<std::size_t>(fill[c]++)] = pz_[k];
    su[c] += pu_[k]; sv[c] += pv_[k]; sw[c] += pw_[k];
  }
  std::vector<double> z(NC, 0.0);
  std::vector<std::uint8_t> valid(NC, 0);
  std::vector<double> mads;
  std::vector<double> dev;
  int n_valid = 0;
  for (int c = 0; c < NC; ++c) {
    if (cnt[c] < cfg_.min_cell_points) { if (cnt[c] > 0) count("cells_dropped_too_few_points"); continue; }
    auto b = zs.begin() + off[c], e = zs.begin() + off[c + 1];
    auto m = b + (e - b) / 2;
    std::nth_element(b, m, e);
    z[c] = *m;
    valid[c] = 1;
    ++n_valid;
    if (cnt[c] >= 5) {  // per-cell spread of the points: a scene-independent noise measure
      dev.clear();
      for (auto it = b; it != e; ++it) dev.push_back(std::fabs(*it - z[c]));
      auto md = dev.begin() + static_cast<long>(dev.size() / 2);
      std::nth_element(dev.begin(), md, dev.end());
      mads.push_back(*md);
    }
  }
  if (n_valid < 8) { count("frames_dropped_too_few_cells"); return; }
  if (mads.size() >= 8) {
    auto md = mads.begin() + static_cast<long>(mads.size() / 2);
    std::nth_element(mads.begin(), md, mads.end());
    const double sigma = 1.4826 * *md;
    count("frame_noise_bucket_" + std::to_string(std::min(20, static_cast<int>(sigma / 0.5))));
    if (sigma > cfg_.max_frame_noise) {  // noise burst (blur / fast turn): nothing from this frame is trusted
      count("frames_dropped_noise_burst");
      warnOnce("burst", "frame noise above max_frame_noise: dropping frames while the burst lasts (fail closed)");
      return;
    }
  }
  // --- ground plane: least squares over the lowest cells, refit on the inliers
  const double cxw = (i0 + 0.5 * W) * cell, cyw = (j0 + 0.5 * H) * cell;
  std::vector<int> vc;
  vc.reserve(n_valid);
  for (int c = 0; c < NC; ++c) if (valid[c]) vc.push_back(c);
  std::vector<int> low = vc;
  const std::size_t nlow = std::max<std::size_t>(3, static_cast<std::size_t>(cfg_.ground_low_fraction * low.size()));
  std::nth_element(low.begin(), low.begin() + static_cast<long>(std::min(nlow, low.size()) - 1), low.end(),
                   [&](int a, int b) { return z[a] < z[b]; });
  low.resize(std::min(nlow, low.size()));
  double pa = 0, pb = 0, pc0 = 0;
  bool plane_ok = false;
  auto fit = [&](const std::vector<int>& cells) {
    // z = a*x' + b*y' + c with x', y' relative to the window centre
    double sx = 0, sy = 0, sz = 0, sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0;
    const double m = static_cast<double>(cells.size());
    for (int c : cells) {
      const double x = (i0 + c % W + 0.5) * cell - cxw, y = (j0 + c / W + 0.5) * cell - cyw;
      sx += x; sy += y; sz += z[c]; sxx += x * x; syy += y * y; sxy += x * y; sxz += x * z[c]; syz += y * z[c];
    }
    const double mx = sx / m, my = sy / m, mz = sz / m;
    const double cxx = sxx - m * mx * mx, cyy = syy - m * my * my, cxy = sxy - m * mx * my;
    const double cxz = sxz - m * mx * mz, cyz = syz - m * my * mz;
    const double det = cxx * cyy - cxy * cxy;
    if (!(std::fabs(det) > 1e-9)) return false;
    pa = (cxz * cyy - cyz * cxy) / det;
    pb = (cyz * cxx - cxz * cxy) / det;
    pc0 = mz - pa * mx - pb * my;
    return std::fabs(pa) <= cfg_.ground_max_slope && std::fabs(pb) <= cfg_.ground_max_slope;
  };
  if (low.size() >= 6 && fit(low)) {
    std::vector<int> inl;
    for (int c : vc) {
      const double x = (i0 + c % W + 0.5) * cell - cxw, y = (j0 + c / W + 0.5) * cell - cyw;
      if (std::fabs(z[c] - (pa * x + pb * y + pc0)) <= cfg_.ground_inlier_band) inl.push_back(c);
    }
    plane_ok = inl.size() >= 6 && fit(inl);
    if (!plane_ok && low.size() >= 6) plane_ok = fit(low);  // keep the first fit if the refit degenerated
  }
  if (!plane_ok) {
    // flat fallback at the lowest decile of the cells (fail open: a worse ground model, never no ground)
    count("frames_ground_plane_fallback");
    warnOnce("ground", "ground plane fit rejected: using a flat ground estimate for this frame");
    std::vector<int> tmp = vc;
    const std::size_t q = tmp.size() / 10;
    std::nth_element(tmp.begin(), tmp.begin() + static_cast<long>(q), tmp.end(), [&](int a, int b) { return z[a] < z[b]; });
    pa = pb = 0.0;
    pc0 = z[tmp[q]];
  }
  // --- height above ground, 3x3 smoothing, classification
  std::vector<double> hag(NC, 0.0), hs(NC, 0.0);
  for (int c : vc) {
    const double x = (i0 + c % W + 0.5) * cell - cxw, y = (j0 + c / W + 0.5) * cell - cyw;
    hag[c] = z[c] - (pa * x + pb * y + pc0);
  }
  view->i0 = i0; view->j0 = j0; view->w = W; view->h = H;
  view->state.assign(NC, 0);
  view->crown.assign(NC, -1);
  view->z.assign(NC, std::numeric_limits<float>::quiet_NaN());
  view->hwt.assign(NC, 0.0f);
  view->hag.assign(NC, 0.0f);
  for (int c : vc) {
    const int i = c % W, j = c / W;
    double sum = 0; int m = 0;
    for (int dj = -1; dj <= 1; ++dj) {
      for (int di = -1; di <= 1; ++di) {
        const int ii = i + di, jj = j + dj;
        if (ii < 0 || jj < 0 || ii >= W || jj >= H) continue;
        const int cc = jj * W + ii;
        if (!valid[cc]) continue;
        sum += hag[cc]; ++m;
      }
    }
    hs[c] = sum / m;
    view->state[c] = hs[c] >= cfg_.min_above_ground ? 2 : 1;
    view->z[c] = static_cast<float>(z[c]);
    view->hwt[c] = static_cast<float>(quality * sw[c] / cnt[c]);
    view->hag[c] = static_cast<float>(hs[c]);
  }
  // --- watershed: steepest ascent among canopy cells
  std::vector<int> up(NC, -1);
  std::vector<int> canopy;
  for (int c : vc) {
    if (view->state[c] != 2) continue;
    canopy.push_back(c);
    const int i = c % W, j = c / W;
    double best = hs[c]; int bc = -1;
    for (int dj = -1; dj <= 1; ++dj) {
      for (int di = -1; di <= 1; ++di) {
        if (!di && !dj) continue;
        const int ii = i + di, jj = j + dj;
        if (ii < 0 || jj < 0 || ii >= W || jj >= H) continue;
        const int cc = jj * W + ii;
        if (view->state[cc] != 2) continue;
        if (hs[cc] > best || (hs[cc] == best && cc < c && bc < 0)) { best = hs[cc]; bc = cc; }
      }
    }
    up[c] = bc;
  }
  if (canopy.empty()) { count("frames_without_canopy"); return; }
  // roots -> basin ids
  std::vector<int> root(NC, -1), basin_of_root;
  std::unordered_map<int, int> root_id;
  for (int c : canopy) {
    int r = c;
    int guard = 0;
    while (up[r] >= 0 && guard++ < NC) r = up[r];
    root[c] = r;
    if (!root_id.count(r)) { root_id[r] = static_cast<int>(basin_of_root.size()); basin_of_root.push_back(r); }
  }
  const int NB = static_cast<int>(basin_of_root.size());
  std::vector<int> label(NC, -1);
  for (int c : canopy) label[c] = root_id[root[c]];
  std::vector<double> peak_h(NB);
  for (int b = 0; b < NB; ++b) peak_h[b] = hs[basin_of_root[b]];
  // saddles between adjacent basins
  std::map<std::pair<int, int>, double> saddle;
  for (int c : canopy) {
    const int i = c % W, j = c / W;
    for (int dj = -1; dj <= 1; ++dj) {
      for (int di = -1; di <= 1; ++di) {
        if (!di && !dj) continue;
        const int ii = i + di, jj = j + dj;
        if (ii < 0 || jj < 0 || ii >= W || jj >= H) continue;
        const int cc = jj * W + ii;
        if (view->state[cc] != 2 || label[cc] == label[c]) continue;
        const auto key = std::minmax(label[c], label[cc]);
        const double sv2 = std::min(hs[c], hs[cc]);
        auto it = saddle.find(key);
        if (it == saddle.end()) saddle[key] = sv2; else it->second = std::max(it->second, sv2);
      }
    }
  }
  // prominence merging: the lower peak joins the neighbour when its rise above the saddle is small
  DSU dsu(NB);
  bool merged = true;
  while (merged && !saddle.empty()) {
    merged = false;
    double best_prom = std::numeric_limits<double>::infinity();
    std::pair<int, int> best_key{-1, -1};
    for (const auto& kv : saddle) {
      const int a = dsu.find(kv.first.first), b = dsu.find(kv.first.second);
      if (a == b) continue;
      const double prom = std::min(peak_h[a], peak_h[b]) - kv.second;
      if (prom < best_prom) { best_prom = prom; best_key = {a, b}; }
    }
    if (best_key.first >= 0 && best_prom < cfg_.min_prominence) {
      const int a = best_key.first, b = best_key.second;
      const int lo = peak_h[a] <= peak_h[b] ? a : b, hi = lo == a ? b : a;
      dsu.unite(lo, hi);
      peak_h[hi] = std::max(peak_h[hi], peak_h[lo]);
      // re-key saddles onto the surviving basin
      std::map<std::pair<int, int>, double> next;
      for (const auto& kv : saddle) {
        const int x = dsu.find(kv.first.first), y = dsu.find(kv.first.second);
        if (x == y) continue;
        const auto key = std::minmax(x, y);
        auto it = next.find(key);
        if (it == next.end()) next[key] = kv.second; else it->second = std::max(it->second, kv.second);
      }
      saddle.swap(next);
      count("basins_merged_low_prominence");
      merged = true;
    }
  }
  // --- crowns
  std::vector<int> crown_index(NB, -1);
  const double dil = cfg_.box_dilate;
  for (int c : canopy) {
    const int b = dsu.find(label[c]);
    if (crown_index[b] < 0) {
      crown_index[b] = static_cast<int>(crowns->size());
      crowns->push_back(Crown{});
    }
    Crown& cr = (*crowns)[crown_index[b]];
    cr.cells.emplace_back(i0 + c % W, j0 + c / W);
    view->crown[c] = crown_index[b];
  }
  for (int b = 0; b < NB; ++b) {
    if (dsu.find(b) != b || crown_index[b] < 0) continue;
    Crown& cr = (*crowns)[crown_index[b]];
    // the crown top: highest smoothed cell of the (merged) basin
    int pk = -1;
    for (const auto& ij : cr.cells) {
      const int c = (ij.second - j0) * W + (ij.first - i0);
      if (pk < 0 || hs[c] > hs[pk]) pk = c;
    }
    cr.peak = {(i0 + pk % W + 0.5) * cell, (j0 + pk / W + 0.5) * cell};
    cr.peak_z = z[pk];
    cr.peak_hag = hs[pk];
    cr.boxed = false;
    const double um = su[pk] / cnt[pk], vm = sv[pk] / cnt[pk];
    const double bpx = cfg_.peak_border_px;
    cr.truncated = um < bpx || vm < bpx || um > cal_.width - bpx || vm > cal_.height - bpx;
    for (const auto& bx : f.boxes) {
      const double dx = dil * bx.width, dy = dil * bx.height;
      if (um >= bx.x - dx && um <= bx.x + bx.width + dx && vm >= bx.y - dy && vm <= bx.y + bx.height + dy) {
        cr.boxed = true;
        break;
      }
    }
    if (hs[pk] < cfg_.min_tree_height) {
      cr.boxed = false;  // too low to be a tree crown: never seeds, never credits (counted below)
      count("crowns_dropped_too_low");
    }
  }
  // drop tiny crowns (relabel view cells to -1)
  std::vector<Crown> kept;
  std::vector<int> remap(crowns->size(), -1);
  for (std::size_t k = 0; k < crowns->size(); ++k) {
    if (static_cast<int>((*crowns)[k].cells.size()) < cfg_.min_basin_cells) { count("crowns_dropped_too_small"); continue; }
    remap[k] = static_cast<int>(kept.size());
    kept.push_back(std::move((*crowns)[k]));
  }
  for (int c : canopy) view->crown[c] = view->crown[c] >= 0 ? remap[view->crown[c]] : -1;
  crowns->swap(kept);
  count("crowns_total", crowns->size());
}

// ------------------------------------------------------------------------------------------ instances

int CandObj::findInstanceForCrown(const Crown& c) const {
  // 1. the crown covers an instance's latched cells (or is covered by them): it is that instance, even when
  //    its top lies far from the instance's current (partial) centroid. This is what stops a second
  //    instance being born on a published tree whose first view was a slice at the swath edge.
  int best = -1, guard = -1;
  if (cfg_.match_overlap > 0.0 && !c.cells.empty()) {
    // among the covered instances the better-established one wins (confirmed > frames seen > lower ID):
    // a tentative fragment can never take a crown away from a published tree
    auto better = [](const Instance& a, const Instance& b) {
      return a.confirmed != b.confirmed ? a.confirmed : a.frames_seen != b.frames_seen ? a.frames_seen > b.frames_seen : a.id < b.id;
    };
    for (std::size_t k = 0; k < instances_.size(); ++k) {
      const Instance& in = instances_[k];
      if (in.n_latched == 0) continue;
      // the crown must contain the instance's known cells (fraction of the LATCHED set) and its top must
      // be within the instance's own reach: a neighbour's crown touching the tree's edge cells, or a
      // merged basin whose top is on the neighbour, is not credited to this tree. (Reach first: it is O(1),
      // so the cell overlap is only counted for the few instances near the crown, S5.)
      if (dist(in.centre, c.peak) > cfg_.match_reach) continue;
      int ov = 0;
      for (const auto& ij : c.cells) {
        const int li = ij.first - in.ai, lj = ij.second - in.aj;
        if (li < 0 || lj < 0 || li >= cfg_.window || lj >= cfg_.window) continue;
        if (in.latched[static_cast<std::size_t>(lj * cfg_.window + li)]) ++ov;
      }
      if (ov == 0) continue;
      const double frac = static_cast<double>(ov) / static_cast<double>(in.n_latched);
      const double frac_crown = static_cast<double>(ov) / static_cast<double>(c.cells.size());
      if (frac >= cfg_.match_overlap) {
        if (best < 0 || better(in, instances_[static_cast<std::size_t>(best)])) best = static_cast<int>(k);
      } else if (frac_crown >= cfg_.seed_overlap && in.confirmed) {
        // seed guard: a crown partly on a published tree's cells must not be born as a second instance
        // next to it (it would stop growing once the tree takes its crowns and stay a permanent fragment)
        if (guard < 0 || better(in, instances_[static_cast<std::size_t>(guard)])) guard = static_cast<int>(k);
      }
    }
    if (best >= 0) return best;
  }
  // 2. the crown top inside the instance's outline, or near its centre
  double bd = std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < instances_.size(); ++k) {
    const Instance& in = instances_[k];
    const double d = dist(in.centre, c.peak);
    const bool inside = in.hull.size() >= 3 && pointInPolygon(in.hull, c.peak);
    if ((d < cfg_.match_radius || inside) && d < bd) { bd = d; best = static_cast<int>(k); }
  }
  return best >= 0 ? best : guard;
}

bool CandObj::insideSprayed(const Point2& p) const {
  for (const auto& in : instances_) {
    if (!in.sprayed) continue;
    if (in.hull.size() >= 3 && pointInPolygon(in.hull, p)) return true;
    if (dist(in.centre, p) < cfg_.match_radius) return true;
  }
  return false;
}

void CandObj::creditCrown(Instance& in, const Crown& c, std::vector<int>& cell_owner, std::vector<std::uint8_t>& cell_trunc,
                          const FrameView& view) {
  const int self = static_cast<int>(&in - instances_.data());
  const bool trim = cfg_.latch_rel_h > 0.0 && c.peak_hag > 0.0 && view.hag.size() == view.state.size();
  const float min_hag = static_cast<float>(cfg_.latch_rel_h * c.peak_hag);
  for (const auto& ij : c.cells) {
    const int li = ij.first - in.ai, lj = ij.second - in.aj;
    if (li < 0 || lj < 0 || li >= cfg_.window || lj >= cfg_.window) { count("cells_dropped_outside_window"); continue; }
    const Point2 cc{(ij.first + 0.5) * cfg_.cell, (ij.second + 0.5) * cfg_.cell};
    if (dist(cc, in.centre) > cfg_.max_cell_radius) { count("cells_dropped_too_far_from_centre"); continue; }
    const int vi = ij.first - view.i0, vj = ij.second - view.j0;
    if (vi < 0 || vj < 0 || vi >= view.w || vj >= view.h) continue;
    const int vk = vj * view.w + vi;
    // the low skirt (stereo smear of the crown edge into occluded ground) is never claimed: it stays
    // unknown for this instance, so it can neither latch nor count against the tree
    if (trim && view.hag[static_cast<std::size_t>(vk)] < min_hag) { count("cells_skipped_low_relative_height"); continue; }
    cell_owner[static_cast<std::size_t>(vk)] = self;
    cell_trunc[static_cast<std::size_t>(vk)] = c.truncated ? 1 : 0;
  }
}

void CandObj::updateEvidence(Instance& in, const FrameView& view, const std::vector<int>& cell_owner,
                             const std::vector<std::uint8_t>& cell_trunc, int self, double quality, double z_cap) {
  const int li0 = std::max(in.ai, view.i0), lj0 = std::max(in.aj, view.j0);
  const int li1 = std::min(in.ai + cfg_.window, view.i0 + view.w), lj1 = std::min(in.aj + cfg_.window, view.j0 + view.h);
  const float q = static_cast<float>(std::clamp(quality, 0.0, 1.0));
  const bool have_z = view.z.size() == view.state.size();
  const bool have_w = view.hwt.size() == view.state.size();
  const float cap = static_cast<float>(cfg_.cell_weight_cap);
  for (int j = lj0; j < lj1; ++j) {
    for (int i = li0; i < li1; ++i) {
      const int vk = (j - view.j0) * view.w + (i - view.i0);
      const int lk = (j - in.aj) * cfg_.window + (i - in.ai);
      const std::uint8_t st = view.state[vk];
      if (st == 0) continue;  // unknown: no evidence either way
      const int owner = cell_owner[vk];
      // D66: the same judgement weighted by how well this frame saw the cell: its points' mean weight (range and
      // off-axis), capped at one frame, times the frame quality. A far, oblique view counts ~0.15 of a near one.
      // No per-cell weights (tests, empty hwt): the frame quality, like npos / nneg.
      const float wo = have_w ? std::min(view.hwt[vk], q) : q;
      if (owner == self) {
        in.wpos[lk] += wo;
        in.npos[lk] = std::min(65535.0f, in.npos[lk] + q);
        // height fusion of this instance's canopy cell (weighted running mean, saturating, gated)
        if (have_z) {
          const float zc = view.z[vk];
          const float wf = have_w ? view.hwt[vk] : q;
          if (!std::isfinite(zc) || zc >= z_cap) {
            count(std::isinf(z_cap) ? "height_cells_skipped_locked_phase" : "height_cells_dropped_near_drone_or_nan");
          } else if (!(wf > 0.0f)) {
            count("height_cells_dropped_zero_weight");
          } else if (in.hw[lk] >= cfg_.gate_min_weight && std::fabs(zc - in.hz[lk]) > cfg_.outlier_gate_m) {
            count("height_cells_rejected_outlier");
            if (++in.hrej[lk] >= 3) {  // the established value was the outlier: start over
              in.hz[lk] = zc; in.hw[lk] = wf; in.hrej[lk] = 0;
              in.hgood[lk] = cell_trunc[vk] ? 0 : 1;
              count("height_cells_reset");
            }
          } else {
            const float wn = in.hw[lk] + wf;
            in.hz[lk] = (in.hz[lk] * in.hw[lk] + zc * wf) / wn;
            in.hw[lk] = std::min(wn, cap);
            in.hrej[lk] = 0;
            if (!cell_trunc[vk]) in.hgood[lk] = 1;
          }
        }
      } else if (st == 1 || owner >= 0) {
        in.nneg[lk] = std::min(65535.0f, in.nneg[lk] + q);  // ground, or another tree's canopy
        in.wneg[lk] += wo;
      } else {
        continue;  // canopy nobody claimed (unboxed / tiny crown / skirt): unknown
      }
      if (!in.latched[lk] && in.npos[lk] >= static_cast<float>(cfg_.latch_frames) &&
          in.npos[lk] >= cfg_.latch_ratio * (in.npos[lk] + in.nneg[lk])) {
        in.latched[lk] = 1;
        ++in.n_latched;
        in.lat_sum_x += (i + 0.5) * cfg_.cell;
        in.lat_sum_y += (j + 0.5) * cfg_.cell;
        in.hull_dirty = true;
        count("cells_latched");
      }
    }
  }
}

void CandObj::refreshHull(Instance& in) {
  if (!in.hull_dirty) return;
  in.hull_dirty = false;
  if (in.n_latched > 0) {
    in.centre = {in.lat_sum_x / in.n_latched, in.lat_sum_y / in.n_latched};
  } else if (in.n_peaks > 0) {
    in.centre = {in.peak_sum_x / in.n_peaks, in.peak_sum_y / in.n_peaks};
  }
  std::vector<Point2> pts;
  pts.reserve(static_cast<std::size_t>(in.n_latched) * 4);
  const double h = cfg_.cell;
  for (int lj = 0; lj < cfg_.window; ++lj) {
    for (int li = 0; li < cfg_.window; ++li) {
      if (!in.latched[lj * cfg_.window + li]) continue;
      const double x = (in.ai + li) * h, y = (in.aj + lj) * h;
      if (cfg_.hull_cell_corners) {
        pts.push_back({x, y}); pts.push_back({x + h, y}); pts.push_back({x + h, y + h}); pts.push_back({x, y + h});
      } else {
        pts.push_back({x + 0.5 * h, y + 0.5 * h});  // the true edge lies somewhere inside the edge cell
      }
    }
  }
  in.hull = pts.size() >= 3 ? convexHull(std::move(pts)) : Polygon2{};
}

void CandObj::refreshCentreSupport(Instance& in) {
  // D66: the published centre's target. Latching is permanent, so the latched centroid keeps every cell a far,
  // oblique first view latched (displaced toward that track) and misses the far half of the crown that this view
  // judged ground. The centroid of the cells whose observation-weighted evidence currently says "this tree's
  // canopy" follows the better (nearer, more nadir) views instead. Fails open to the latched centroid (counted)
  // while fewer than min_height_cells cells qualify.
  in.has_centre_support = false;
  if (!cfg_.centre_from_support) return;
  const float wmin = static_cast<float>(cfg_.latch_frames);
  const float ratio = static_cast<float>(cfg_.centre_support_ratio);
  double sx = 0.0, sy = 0.0;
  int n = 0;
  for (int lj = 0; lj < cfg_.window; ++lj) {
    for (int li = 0; li < cfg_.window; ++li) {
      const int lk = lj * cfg_.window + li;
      const float wp = in.wpos[lk];
      if (wp < wmin || wp < ratio * (wp + in.wneg[lk])) continue;
      sx += (in.ai + li + 0.5) * cfg_.cell;
      sy += (in.aj + lj + 0.5) * cfg_.cell;
      ++n;
    }
  }
  if (n < cfg_.min_height_cells) {
    if (in.confirmed) count("centre_support_fallback_latched");
    return;
  }
  in.centre_support = {sx / n, sy / n};
  in.has_centre_support = true;
}

void CandObj::refreshHeight(Instance& in) {
  // the tree height is a high quantile of the latched cells' fused surface: a partial first view cannot
  // pin it to the top of the visible slice, and no single cell (or frame) defines it
  hbuf_.clear();
  const std::size_t n = static_cast<std::size_t>(cfg_.window) * cfg_.window;
  int good = 0;
  for (std::size_t k = 0; k < n; ++k) {
    if (in.latched[k] && in.hw[k] > 0.0f && std::isfinite(in.hz[k])) {
      hbuf_.push_back(in.hz[k]);
      if (in.hgood[k]) ++good;
    }
  }
  in.n_height_cells = static_cast<int>(hbuf_.size());
  in.n_good_height_cells = good;
  if (in.n_height_cells < cfg_.min_height_cells) { in.height_target = kNaN; return; }
  const std::size_t i = static_cast<std::size_t>(std::floor(cfg_.height_quantile * static_cast<double>(hbuf_.size() - 1) + 0.5));
  std::nth_element(hbuf_.begin(), hbuf_.begin() + static_cast<long>(i), hbuf_.end());
  in.height_target = hbuf_[i];
}

void CandObj::stepPublished(Instance& in, double t) {
  // published centre and height move toward their targets at a bounded rate (rule 2: no jumps); the
  // outline itself is never delayed. Called only for instances the phase policy allows to change.
  const double dt = std::clamp(t - in.last_step_t, 0.0, kMaxStepDt);
  in.last_step_t = t;
  if (!in.confirmed) return;
  if (std::isfinite(in.height_target)) {
    const double d = in.height_target - in.height_pub;
    const double lim = cfg_.max_height_rate * dt;
    if (std::fabs(d) > lim) count("height_steps_clamped");
    in.height_pub += std::clamp(d, -lim, lim);
  }
  const Point2& target = centreTarget(in);
  const double dx = target.x - in.centre_pub.x, dy = target.y - in.centre_pub.y;
  const double dd = std::hypot(dx, dy), lim = cfg_.max_centre_rate * dt;
  if (dd > lim) {
    count("centre_steps_clamped");
    if (dd > 0.0) { in.centre_pub.x += dx * lim / dd; in.centre_pub.y += dy * lim / dd; }
  } else {
    in.centre_pub = target;
  }
}

bool CandObj::publishable(const Instance& in) const {
  return in.confirmed && in.hull.size() >= 3 && in.n_height_cells >= cfg_.min_height_cells && std::isfinite(in.height_pub) &&
         std::isfinite(in.centre_pub.x) && std::isfinite(in.centre_pub.y);
}

void CandObj::absorb(Instance& keep, const Instance& lose) {
  const float cap = static_cast<float>(cfg_.cell_weight_cap);
  for (int lj = 0; lj < cfg_.window; ++lj) {
    for (int li = 0; li < cfg_.window; ++li) {
      const int lk = lj * cfg_.window + li;
      if (lose.npos[lk] <= 0.0f && lose.nneg[lk] <= 0.0f && !lose.latched[lk]) continue;
      const int gi = lose.ai + li, gj = lose.aj + lj;
      const int ki = gi - keep.ai, kj = gj - keep.aj;
      if (ki < 0 || kj < 0 || ki >= cfg_.window || kj >= cfg_.window) { count("merge_cells_dropped_outside_window"); continue; }
      const int kk = kj * cfg_.window + ki;
      keep.npos[kk] = std::min(65535.0f, keep.npos[kk] + lose.npos[lk]);
      keep.nneg[kk] = std::min(65535.0f, keep.nneg[kk] + lose.nneg[lk]);
      keep.wpos[kk] += lose.wpos[lk];
      keep.wneg[kk] += lose.wneg[lk];
      if (lose.hw[lk] > 0.0f) {  // the loser's surface measurements of cells the winner had not measured
        const float wn = keep.hw[kk] + lose.hw[lk];
        keep.hz[kk] = (keep.hz[kk] * keep.hw[kk] + lose.hz[lk] * lose.hw[lk]) / wn;
        keep.hw[kk] = std::min(wn, cap);
        if (lose.hgood[lk]) keep.hgood[kk] = 1;
      }
      if (lose.latched[lk] && !keep.latched[kk]) {
        keep.latched[kk] = 1;
        ++keep.n_latched;
        keep.lat_sum_x += (gi + 0.5) * cfg_.cell;
        keep.lat_sum_y += (gj + 0.5) * cfg_.cell;
      }
    }
  }
  keep.frames_seen = std::max(keep.frames_seen, lose.frames_seen);
  keep.first_seen = std::min(keep.first_seen, lose.first_seen);
  keep.last_seen = std::max(keep.last_seen, lose.last_seen);
  keep.hull_dirty = true;  // the winner keeps its ID, its published height and centre (rule 4); both move
                           // toward the union's values only at the bounded rate
}

void CandObj::mergeInstances(double t) {
  (void)t;
  // Candidate pairs from a uniform grid on the instance centres (cell = the reach 2 x max_cell_radius), so a sweep
  // costs O(N) instead of O(N^2) pair tests (S5: 17.9 ms/frame at 1600 trees before). The pairs are visited in
  // exactly the old order (a ascending, then b ascending) and the first one that merges wins, so the result is
  // identical to the full double loop: pairs farther apart than the reach were skipped there too. refreshHull()
  // is a pure function of the latched cells, so refreshing every candidate up front changes nothing either.
  const double reach = 2.0 * cfg_.max_cell_radius;
  const double gcell = std::max(reach * 1.0001, 1e-3);  // a hair over the reach: no pair in reach is missed by rounding
  std::unordered_map<std::int64_t, std::vector<std::size_t>> grid;
  auto key = [](std::int64_t gi, std::int64_t gj) { return (gi << 32) ^ (gj & 0xffffffffLL); };
  std::vector<std::size_t> cand;
  bool merged = true;
  while (merged) {
    merged = false;
    grid.clear();
    for (std::size_t k = 0; k < instances_.size(); ++k) {
      Instance& in = instances_[k];
      if (in.sprayed || in.n_latched == 0) continue;
      refreshHull(in);
      grid[key(static_cast<std::int64_t>(std::floor(in.centre.x / gcell)), static_cast<std::int64_t>(std::floor(in.centre.y / gcell)))]
          .push_back(k);
    }
    for (std::size_t a = 0; a < instances_.size() && !merged; ++a) {
      Instance& ea = instances_[a];
      if (ea.sprayed || ea.n_latched == 0) continue;
      const std::int64_t gi = static_cast<std::int64_t>(std::floor(ea.centre.x / gcell));
      const std::int64_t gj = static_cast<std::int64_t>(std::floor(ea.centre.y / gcell));
      cand.clear();
      for (std::int64_t dj = -1; dj <= 1; ++dj) {
        for (std::int64_t di = -1; di <= 1; ++di) {
          auto it = grid.find(key(gi + di, gj + dj));
          if (it == grid.end()) continue;
          for (std::size_t b : it->second) if (b > a) cand.push_back(b);
        }
      }
      std::sort(cand.begin(), cand.end());
      for (std::size_t ci = 0; ci < cand.size() && !merged; ++ci) {
        const std::size_t b = cand[ci];
        Instance& eb = instances_[b];
        if (ea.sprayed || eb.sprayed) continue;
        if (ea.n_latched == 0 || eb.n_latched == 0) continue;
        if (dist(ea.centre, eb.centre) > reach) continue;  // cannot overlap
        bool same = dist(ea.centre, eb.centre) < cfg_.merge_centre_dist;
        if (!same) {
          // latched overlap on the shared lattice
          int overlap = 0;
          const int i0 = std::max(ea.ai, eb.ai), j0 = std::max(ea.aj, eb.aj);
          const int i1 = std::min(ea.ai, eb.ai) + cfg_.window, j1 = std::min(ea.aj, eb.aj) + cfg_.window;
          for (int j = j0; j < j1; ++j) {
            for (int i = i0; i < i1; ++i) {
              if (ea.latched[(j - ea.aj) * cfg_.window + (i - ea.ai)] && eb.latched[(j - eb.aj) * cfg_.window + (i - eb.ai)]) ++overlap;
            }
          }
          same = overlap >= cfg_.merge_overlap * std::min(ea.n_latched, eb.n_latched);
        }
        if (!same) continue;
        // the better-established instance keeps identity and height (rule 4)
        const bool a_keeps = ea.confirmed != eb.confirmed ? ea.confirmed
                             : ea.frames_seen != eb.frames_seen ? ea.frames_seen > eb.frames_seen
                                                                : ea.id < eb.id;
        Instance& keep = a_keeps ? ea : eb;
        const std::size_t lose_idx = a_keeps ? b : a;
        absorb(keep, instances_[lose_idx]);
        refreshHull(keep);
        refreshCentreSupport(keep);
        refreshHeight(keep);
        instances_.erase(instances_.begin() + static_cast<long>(lose_idx));  // its ID is retired forever (rule 1)
        count("instances_merged");
        merged = true;
      }
    }
  }
}

void CandObj::integrate(double t, MissionPhase phase, TreeId target, const DronePose& pose, const FrameView& view,
                        std::vector<Crown>& crowns, double quality) {
  sticky_frame_ = false;
  PhasePolicy pol = policyFor(phase, target);
  if (sticky_frame_) {
    count("frames_locked_after_planning");
    warnOnce("sticky", std::string("phase ") + toString(phase) +
                           " after the tree list was locked: no new trees, no merges until SCAN (sticky lock)");
  }
  if (!pol.awake) { count("crowns_dropped_sleeping", crowns.size()); return; }
  // post-spray climb hold (see process()): freeze non-targets until the drone has left the hover point
  if (hold_after_spray_) {
    const bool scanning = phase == MissionPhase::kScan || phase == MissionPhase::kScanApproach;
    if (!hover_known_) { if (pose.valid) { hover_x_ = pose.x; hover_y_ = pose.y; hover_known_ = true; } }
    const bool away = hover_known_ && pose.valid && std::hypot(pose.x - hover_x_, pose.y - hover_y_) > cfg_.post_spray_hold_radius;
    if (away || scanning) {
      hold_after_spray_ = false;
      hover_known_ = false;
    } else {
      pol.freeze_non_target = true;
      pol.may_merge = false;
      pol.may_create = false;
      count("frames_post_spray_hold");
    }
  }
  if (pol.freeze_non_target && target < 0) {
    warnOnce("notarget", "descent/hold without a valid hop target: freezing every tree (fail closed)");
  }
  const double z_cap = pose.z - std::max(cfg_.above_drone_margin, cfg_.height_sample_clearance);
  std::vector<int> cell_owner(view.state.size(), -1);
  std::vector<std::uint8_t> cell_trunc(view.state.size(), 0);  ///< the cell's crown top was truncated (S8)
  std::vector<char> seen(instances_.size(), 0);
  std::vector<char> updatable(instances_.size(), 0);
  for (std::size_t k = 0; k < instances_.size(); ++k) {
    const Instance& in = instances_[k];
    if (in.sprayed) continue;
    if (pol.freeze_non_target && in.id != target) continue;
    if (!pol.may_create && !in.confirmed) continue;  // cannot be confirmed in a locked phase: do not grow it
    updatable[k] = 1;
  }
  // 1. match crowns to instances; seed new instances from unmatched, boxed crowns
  for (std::size_t ci = 0; ci < crowns.size(); ++ci) {
    Crown& c = crowns[ci];
    if (!c.boxed) { count("crowns_dropped_unboxed"); continue; }
    int k = findInstanceForCrown(c);
    if (k < 0) {
      if (insideSprayed(c.peak)) { count("crowns_dropped_inside_sprayed"); continue; }
      if (!pol.may_create) { count("crowns_dropped_new_tree_locked_phase"); continue; }
      if (c.truncated && !cfg_.seed_truncated) { count("crowns_dropped_truncated_seed"); continue; }
      if (c.truncated) count("instances_seeded_from_truncated_crown");
      Instance in;
      in.id = next_id_++;
      in.ai = floorDiv(c.peak.x, cfg_.cell) - cfg_.window / 2;
      in.aj = floorDiv(c.peak.y, cfg_.cell) - cfg_.window / 2;
      const std::size_t n = static_cast<std::size_t>(cfg_.window) * cfg_.window;
      in.npos.assign(n, 0.0f); in.nneg.assign(n, 0.0f); in.latched.assign(n, 0);
      in.hz.assign(n, 0.0f); in.hw.assign(n, 0.0f); in.hrej.assign(n, 0); in.hgood.assign(n, 0);
      in.wpos.assign(n, 0.0f); in.wneg.assign(n, 0.0f);
      in.centre = in.centre_pub = c.peak;
      in.height_target = in.height_pub = kNaN;
      in.first_seen = in.last_seen = in.last_step_t = t;
      instances_.push_back(std::move(in));
      seen.push_back(0);
      updatable.push_back(1);
      k = static_cast<int>(instances_.size()) - 1;
      count("instances_created");
    }
    Instance& in = instances_[static_cast<std::size_t>(k)];
    if (in.sprayed) { count("crowns_dropped_sprayed_tree"); continue; }
    if (!updatable[static_cast<std::size_t>(k)]) {
      count(pol.freeze_non_target && in.id != target ? "crowns_dropped_non_target_frozen" : "crowns_dropped_unconfirmed_locked_phase");
      continue;
    }
    seen[static_cast<std::size_t>(k)] = 1;
    creditCrown(in, c, cell_owner, cell_trunc, view);
    in.peak_sum_x += c.peak.x; in.peak_sum_y += c.peak.y; ++in.n_peaks;
  }
  // 2. evidence + height update for every updatable instance in view; rate-limited publication step
  for (std::size_t k = 0; k < instances_.size(); ++k) {
    if (!updatable[k]) continue;
    Instance& in = instances_[k];
    if (seen[k]) { ++in.frames_seen; in.last_seen = t; }
    const bool in_view = !(in.ai + cfg_.window <= view.i0 || in.aj + cfg_.window <= view.j0 ||
                           in.ai >= view.i0 + view.w || in.aj >= view.j0 + view.h);
    if (in_view) {
      // the latched surface's heights are fused in creation phases only: once the list is locked the
      // descent reference must not drift (a close, oblique look at a shoulder during the approach would
      // re-measure cells the scan saw from above); outlines keep growing under the phase policy
      updateEvidence(in, view, cell_owner, cell_trunc, static_cast<int>(k), quality, pol.may_create ? z_cap : -std::numeric_limits<double>::infinity());
      refreshHull(in);
      refreshCentreSupport(in);
      if (pol.may_create) refreshHeight(in);
    }
    if (!in.confirmed && pol.may_create && in.frames_seen >= cfg_.confirm_frames && std::isfinite(in.height_target) &&
        latchedArea(in) >= cfg_.min_publish_area && in.hull.size() >= 3) {
      if (insideSprayed(in.centre)) { count("instances_refused_confirm_inside_sprayed"); continue; }
      // S8: the height must rest on crowns whose top was inside the image; a tree seen only truncated (top
      // outside the image) would publish an under-estimated height, which is the unsafe direction for a descent
      if (in.n_good_height_cells < cfg_.min_height_cells) { count("instances_waiting_untruncated_height"); stepPublished(in, t); continue; }
      in.confirmed = true;
      in.height_pub = in.height_target;  // first publication: no previous value to rate-limit from
      in.centre_pub = centreTarget(in);
      in.last_step_t = t;
      count("instances_confirmed");
      continue;
    }
    stepPublished(in, t);
  }
  // 3. tentative instances that stopped being seen are noise / false positives
  for (auto it = instances_.begin(); it != instances_.end();) {
    if (!it->confirmed && t - it->last_seen > cfg_.tentative_timeout) {
      it = instances_.erase(it);
      count("instances_expired_unconfirmed");
    } else {
      ++it;
    }
  }
  // 4. merges (creation phases only: a merge retires an ID navigation may hold)
  if (pol.may_merge) mergeInstances(t);
  for (auto& in : instances_) refreshHull(in);
}

// ------------------------------------------------------------------------------------------ navigation API

std::vector<TreeEntry> CandObj::publishedTrees() const {
  std::vector<TreeEntry> out;
  for (const auto& in : instances_) {
    if (!publishable(in)) continue;
    TreeEntry te;
    te.id = in.id;
    te.outline = in.hull;
    te.centre = in.centre_pub;
    te.height = in.height();
    te.sprayed = in.sprayed;
    te.observations = in.frames_seen;
    out.push_back(std::move(te));
  }
  std::sort(out.begin(), out.end(), [](const TreeEntry& a, const TreeEntry& b) { return a.id < b.id; });
  return out;
}

bool CandObj::markSprayed(TreeId id) {
  for (auto& in : instances_) {
    if (in.id != id) continue;
    if (!publishable(in)) break;
    in.sprayed = true;
    count("trees_marked_sprayed");
    return true;
  }
  count("mark_sprayed_unknown_id");
  return false;
}

std::map<std::string, double> CandObj::takeStageTiming() {
  std::map<std::string, double> out;
  if (stage_frames_ > 0) {
    for (const auto& kv : stage_ms_) out[kv.first] = kv.second / stage_frames_;
  }
  stage_ms_.clear();
  stage_frames_ = 0;
  return out;
}

}  // namespace cand_obj

// Registered in mapper_registry.cpp as TM_REGISTER_CANDIDATE(cand_obj).
std::unique_ptr<Mapper> tm_create_cand_obj(const MapperConfig& config, const StereoCalibration& cal) {
  return std::make_unique<cand_obj::CandObj>(cand_obj::configFrom(config), cal);
}

}  // namespace treemapper
