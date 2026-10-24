#include "treemapper/candidates/cand_obj/cand_obj.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
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

template <typename T>
T clampLog(const char* name, T v, T lo, T hi, const std::function<void(const std::string&)>& log) {
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
  c.cell = clampLog("cell", c.cell, 0.1, 2.0, log);
  c.min_cell_points = clampLog("min_cell_points", c.min_cell_points, 1, 1000, log);
  c.max_grid = clampLog("max_grid", c.max_grid, 16, 1024, log);
  c.ground_low_fraction = clampLog("ground_low_fraction", c.ground_low_fraction, 0.05, 0.9, log);
  c.ground_inlier_band = clampLog("ground_inlier_band", c.ground_inlier_band, 0.05, 5.0, log);
  c.ground_max_slope = clampLog("ground_max_slope", c.ground_max_slope, 0.0, 2.0, log);
  c.min_above_ground = clampLog("min_above_ground", c.min_above_ground, 0.0, 20.0, log);
  c.min_prominence = clampLog("min_prominence", c.min_prominence, 0.0, 20.0, log);
  c.min_basin_cells = clampLog("min_basin_cells", c.min_basin_cells, 1, 100000, log);
  c.box_dilate = clampLog("box_dilate", c.box_dilate, 0.0, 2.0, log);
  c.border_px = clampLog("border_px", c.border_px, 0.0, 1000.0, log);
  c.window = clampLog("window", c.window, 4, 256, log);
  c.match_radius = clampLog("match_radius", c.match_radius, 0.0, 50.0, log);
  c.max_cell_radius = clampLog("max_cell_radius", c.max_cell_radius, 0.0, 50.0, log);
  c.latch_frames = clampLog("latch_frames", c.latch_frames, 1, 10000, log);
  c.latch_ratio = clampLog("latch_ratio", c.latch_ratio, 0.0, 1.0, log);
  c.confirm_frames = clampLog("confirm_frames", c.confirm_frames, 1, 10000, log);
  c.min_height_samples = clampLog("min_height_samples", c.min_height_samples, 1, 10000, log);
  c.min_publish_area = clampLog("min_publish_area", c.min_publish_area, 0.0, 1000.0, log);
  c.min_tree_height = clampLog("min_tree_height", c.min_tree_height, 0.0, 50.0, log);
  c.tentative_timeout = clampLog("tentative_timeout", c.tentative_timeout, 0.0, 1000.0, log);
  c.height_buffer = clampLog("height_buffer", c.height_buffer, 1, 100000, log);
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
  c.cell = r.getDouble("cell", c.cell);
  c.min_cell_points = r.getInt("min_cell_points", c.min_cell_points);
  c.max_grid = r.getInt("max_grid", c.max_grid);
  c.ground_low_fraction = r.getDouble("ground_low_fraction", c.ground_low_fraction);
  c.ground_inlier_band = r.getDouble("ground_inlier_band", c.ground_inlier_band);
  c.ground_max_slope = r.getDouble("ground_max_slope", c.ground_max_slope);
  c.min_above_ground = r.getDouble("min_above_ground", c.min_above_ground);
  c.min_prominence = r.getDouble("min_prominence", c.min_prominence);
  c.min_basin_cells = r.getInt("min_basin_cells", c.min_basin_cells);
  c.box_dilate = r.getDouble("box_dilate", c.box_dilate);
  c.border_px = r.getDouble("border_px", c.border_px);
  c.window = r.getInt("window", c.window);
  c.match_radius = r.getDouble("match_radius", c.match_radius);
  c.max_cell_radius = r.getDouble("max_cell_radius", c.max_cell_radius);
  c.latch_frames = r.getInt("latch_frames", c.latch_frames);
  c.latch_ratio = r.getDouble("latch_ratio", c.latch_ratio);
  c.confirm_frames = r.getInt("confirm_frames", c.confirm_frames);
  c.min_height_samples = r.getInt("min_height_samples", c.min_height_samples);
  c.min_publish_area = r.getDouble("min_publish_area", c.min_publish_area);
  c.min_tree_height = r.getDouble("min_tree_height", c.min_tree_height);
  c.tentative_timeout = r.getDouble("tentative_timeout", c.tentative_timeout);
  c.height_buffer = r.getInt("height_buffer", c.height_buffer);
  c.merge_overlap = r.getDouble("merge_overlap", c.merge_overlap);
  c.merge_centre_dist = r.getDouble("merge_centre_dist", c.merge_centre_dist);
  c.post_spray_hold_radius = r.getDouble("post_spray_hold_radius", c.post_spray_hold_radius);
  r.finish("cand_obj");
  return c;
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

void CandObj::process(const FrameInput& f) {
  count("frames_total");
  const PhasePolicy pol = phasePolicy(f.phase, f.hop_target);
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
    return;
  }
  if (!f.pose.valid) {
    state_ = State::kHoldingNoPose;
    count("frames_held_no_pose");
    warnOnce("pose", "drone pose not valid: holding all processing (fail closed)");
    return;
  }
  state_ = State::kAwake;
  if (f.disparity.empty()) {
    count(f.boxes.empty() ? "frames_heartbeat_only" : "frames_dropped_no_disparity");
    return;
  }
  const auto t0 = Clock::now();
  FrameView view;
  std::vector<Crown> crowns;
  extractCrowns(f, &view, &crowns);
  stage_ms_["chm"] += msSince(t0);
  const auto t1 = Clock::now();
  integrate(f.t, f.phase, f.hop_target, f.pose, view, crowns);
  stage_ms_["instances"] += msSince(t1);
  ++stage_frames_;
}

// ------------------------------------------------------------------------------------------ CHM + crowns

void CandObj::extractCrowns(const FrameInput& f, FrameView* view, std::vector<Crown>* crowns) {
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
  px_.assign(n, 0); py_.assign(n, 0); pz_.assign(n, 0); pu_.assign(n, 0); pv_.assign(n, 0); pok_.assign(n, 0);
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
      px_[k] = w.x; py_[k] = w.y; pz_[k] = w.z; pu_[k] = u; pv_[k] = v; pok_[k] = 1;
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
  // --- bin points into cells (counting sort), per cell: count, mean pixel, median z
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
  std::vector<double> su(NC, 0.0), sv(NC, 0.0);
  for (std::size_t k = 0; k < n; ++k) {
    const int c = cell_of[k];
    if (c < 0) continue;
    zs[static_cast<std::size_t>(fill[c]++)] = pz_[k];
    su[c] += pu_[k]; sv[c] += pv_[k];
  }
  std::vector<double> z(NC, 0.0);
  std::vector<std::uint8_t> valid(NC, 0);
  int n_valid = 0;
  for (int c = 0; c < NC; ++c) {
    if (cnt[c] < cfg_.min_cell_points) { if (cnt[c] > 0) count("cells_dropped_too_few_points"); continue; }
    auto b = zs.begin() + off[c], e = zs.begin() + off[c + 1];
    auto m = b + (e - b) / 2;
    std::nth_element(b, m, e);
    z[c] = *m;
    valid[c] = 1;
    ++n_valid;
  }
  if (n_valid < 8) { count("frames_dropped_too_few_cells"); return; }
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
    const double um = su[c] / cnt[c], vm = sv[c] / cnt[c];
    if (um < cfg_.border_px || vm < cfg_.border_px || um > cal_.width - cfg_.border_px || vm > cal_.height - cfg_.border_px) {
      cr.truncated = true;
    }
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
    cr.boxed = false;
    const double um = su[pk] / cnt[pk], vm = sv[pk] / cnt[pk];
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

int CandObj::findInstanceForPeak(const Point2& peak) const {
  int best = -1;
  double bd = std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < instances_.size(); ++k) {
    const Instance& in = instances_[k];
    const double d = dist(in.centre, peak);
    const bool inside = in.hull.size() >= 3 && pointInPolygon(in.hull, peak);
    if ((d < cfg_.match_radius || inside) && d < bd) { bd = d; best = static_cast<int>(k); }
  }
  return best;
}

bool CandObj::insideSprayed(const Point2& p) const {
  for (const auto& in : instances_) {
    if (!in.sprayed) continue;
    if (in.hull.size() >= 3 && pointInPolygon(in.hull, p)) return true;
    if (dist(in.centre, p) < cfg_.match_radius) return true;
  }
  return false;
}

void CandObj::creditCrown(Instance& in, const Crown& c, std::vector<int>& cell_owner, const FrameView& view) {
  const int self = static_cast<int>(&in - instances_.data());
  for (const auto& ij : c.cells) {
    const int li = ij.first - in.ai, lj = ij.second - in.aj;
    if (li < 0 || lj < 0 || li >= cfg_.window || lj >= cfg_.window) { count("cells_dropped_outside_window"); continue; }
    const Point2 cc{(ij.first + 0.5) * cfg_.cell, (ij.second + 0.5) * cfg_.cell};
    if (dist(cc, in.centre) > cfg_.max_cell_radius) { count("cells_dropped_too_far_from_centre"); continue; }
    const int vi = ij.first - view.i0, vj = ij.second - view.j0;
    if (vi < 0 || vj < 0 || vi >= view.w || vj >= view.h) continue;
    cell_owner[vj * view.w + vi] = self;
  }
}

void CandObj::updateEvidence(Instance& in, const FrameView& view, const std::vector<int>& cell_owner, int self) {
  const int li0 = std::max(in.ai, view.i0), lj0 = std::max(in.aj, view.j0);
  const int li1 = std::min(in.ai + cfg_.window, view.i0 + view.w), lj1 = std::min(in.aj + cfg_.window, view.j0 + view.h);
  for (int j = lj0; j < lj1; ++j) {
    for (int i = li0; i < li1; ++i) {
      const int vk = (j - view.j0) * view.w + (i - view.i0);
      const int lk = (j - in.aj) * cfg_.window + (i - in.ai);
      const std::uint8_t st = view.state[vk];
      if (st == 0) continue;  // unknown: no evidence either way
      const int owner = cell_owner[vk];
      if (owner == self) {
        if (in.npos[lk] < 65535) ++in.npos[lk];
      } else if (st == 1 || owner >= 0) {
        if (in.nneg[lk] < 65535) ++in.nneg[lk];  // ground, or another tree's canopy
      } else {
        continue;  // canopy nobody claimed (unboxed / tiny crown): unknown
      }
      if (!in.latched[lk] && in.npos[lk] >= cfg_.latch_frames &&
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
      pts.push_back({x, y}); pts.push_back({x + h, y}); pts.push_back({x + h, y + h}); pts.push_back({x, y + h});
    }
  }
  in.hull = pts.size() >= 3 ? convexHull(std::move(pts)) : Polygon2{};
}

bool CandObj::publishable(const Instance& in) const {
  return in.confirmed && in.hull.size() >= 3 && in.hasHeight(cfg_.min_height_samples) && std::isfinite(in.height()) &&
         std::isfinite(in.centre.x) && std::isfinite(in.centre.y);
}

void CandObj::absorb(Instance& keep, const Instance& lose) {
  for (int lj = 0; lj < cfg_.window; ++lj) {
    for (int li = 0; li < cfg_.window; ++li) {
      const int lk = lj * cfg_.window + li;
      if (!lose.npos[lk] && !lose.nneg[lk] && !lose.latched[lk]) continue;
      const int gi = lose.ai + li, gj = lose.aj + lj;
      const int ki = gi - keep.ai, kj = gj - keep.aj;
      if (ki < 0 || kj < 0 || ki >= cfg_.window || kj >= cfg_.window) { count("merge_cells_dropped_outside_window"); continue; }
      const int kk = kj * cfg_.window + ki;
      keep.npos[kk] = static_cast<std::uint16_t>(std::min(65535, keep.npos[kk] + lose.npos[lk]));
      keep.nneg[kk] = static_cast<std::uint16_t>(std::min(65535, keep.nneg[kk] + lose.nneg[lk]));
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
  keep.hull_dirty = true;  // the loser's height samples are dropped: the winner keeps its height (rule 4)
}

void CandObj::mergeInstances(double t) {
  (void)t;
  bool merged = true;
  while (merged) {
    merged = false;
    for (std::size_t a = 0; a < instances_.size() && !merged; ++a) {
      for (std::size_t b = a + 1; b < instances_.size() && !merged; ++b) {
        Instance& ea = instances_[a];
        Instance& eb = instances_[b];
        if (ea.sprayed || eb.sprayed) continue;
        if (ea.n_latched == 0 || eb.n_latched == 0) continue;
        refreshHull(ea);
        refreshHull(eb);
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
        instances_.erase(instances_.begin() + static_cast<long>(lose_idx));  // its ID is retired forever (rule 1)
        count("instances_merged");
        merged = true;
      }
    }
  }
}

void CandObj::integrate(double t, MissionPhase phase, TreeId target, const DronePose& pose, const FrameView& view,
                        std::vector<Crown>& crowns) {
  PhasePolicy pol = phasePolicy(phase, target);
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
  const double z_cap = pose.z - cfg_.above_drone_margin;
  std::vector<int> cell_owner(view.state.size(), -1);
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
    int k = findInstanceForPeak(c.peak);
    if (k < 0) {
      if (insideSprayed(c.peak)) { count("crowns_dropped_inside_sprayed"); continue; }
      if (!pol.may_create) { count("crowns_dropped_new_tree_locked_phase"); continue; }
      if (c.truncated) { count("crowns_dropped_truncated_seed"); continue; }
      Instance in;
      in.id = next_id_++;
      in.ai = floorDiv(c.peak.x, cfg_.cell) - cfg_.window / 2;
      in.aj = floorDiv(c.peak.y, cfg_.cell) - cfg_.window / 2;
      const std::size_t n = static_cast<std::size_t>(cfg_.window) * cfg_.window;
      in.npos.assign(n, 0); in.nneg.assign(n, 0); in.latched.assign(n, 0);
      in.centre = c.peak;
      in.first_seen = in.last_seen = t;
      instances_.push_back(std::move(in));
      seen.push_back(0);
      updatable.push_back(1);
      cell_owner.resize(view.state.size(), -1);
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
    creditCrown(in, c, cell_owner, view);
    in.peak_sum_x += c.peak.x; in.peak_sum_y += c.peak.y; ++in.n_peaks;
    if (!c.truncated) {
      if (std::isfinite(c.peak_z) && c.peak_z < z_cap) {
        auto pos = std::upper_bound(in.heights.begin(), in.heights.end(), c.peak_z);
        in.heights.insert(pos, c.peak_z);
        if (static_cast<int>(in.heights.size()) > cfg_.height_buffer) {
          // keep the buffer centred on the median: drop the extreme farther from it
          const double med = in.heights[in.heights.size() / 2];
          if (med - in.heights.front() > in.heights.back() - med) in.heights.erase(in.heights.begin());
          else in.heights.pop_back();
        }
      } else {
        count("height_samples_dropped_above_drone_or_nan");
      }
    } else {
      count("height_samples_skipped_truncated_crown");
    }
  }
  // 2. evidence update for every updatable instance in view
  for (std::size_t k = 0; k < instances_.size(); ++k) {
    if (!updatable[k]) continue;
    Instance& in = instances_[k];
    if (seen[k]) { ++in.frames_seen; in.last_seen = t; }
    if (in.ai + cfg_.window <= view.i0 || in.aj + cfg_.window <= view.j0 || in.ai >= view.i0 + view.w || in.aj >= view.j0 + view.h) continue;
    updateEvidence(in, view, cell_owner, static_cast<int>(k));
    refreshHull(in);
    if (!in.confirmed && pol.may_create && in.frames_seen >= cfg_.confirm_frames && in.hasHeight(cfg_.min_height_samples) &&
        latchedArea(in) >= cfg_.min_publish_area && in.hull.size() >= 3) {
      if (insideSprayed(in.centre)) { count("instances_refused_confirm_inside_sprayed"); continue; }
      in.confirmed = true;
      count("instances_confirmed");
    }
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
    te.centre = in.centre;
    te.height = in.height();
    te.sprayed = in.sprayed;
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
