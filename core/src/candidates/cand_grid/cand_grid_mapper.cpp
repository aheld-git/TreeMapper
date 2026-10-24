// cand_grid: the mapper (evidence integration, extraction, persistent instances). See cand_grid.hpp.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>

#include "treemapper/candidates/cand_grid/cand_grid.hpp"

namespace treemapper {
namespace cand_grid {

namespace {
using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
const char* verdictName(PublishVerdict v) {
  switch (v) {
    case PublishVerdict::kPublish: return "publish";
    case PublishVerdict::kLockedPhase: return "locked_phase";
    case PublishVerdict::kTooSmall: return "too_small";
    case PublishVerdict::kTooLow: return "too_low";
    case PublishVerdict::kNoBoxSupport: return "no_box_support";
    case PublishVerdict::kNotConfirmed: return "not_confirmed";
    case PublishVerdict::kNoHeight: return "no_height";
    case PublishVerdict::kOpen: return "open";
  }
  return "?";
}
constexpr int kMaxWindowCells = 480;  // per side (120 m at 0.25 m): bounds one extraction's cost
}  // namespace

CandGridMapper::CandGridMapper(const Config& cfg, const StereoCalibration& cal)
    : cfg_(cfg), cal_(cal), log_([](const std::string& m) { std::cerr << "[cand_grid] " << m << "\n"; }) {
  for (const auto& m : validate(&cfg_)) log_(m);
}

void CandGridMapper::warnOnce(const std::string& key, const std::string& msg) {
  if (warned_[key]) return;
  warned_[key] = true;
  if (log_) log_(msg);
}

CandGridMapper::Inst* CandGridMapper::byId(TreeId id) {
  if (id < 0) return nullptr;
  for (auto& kv : inst_) {
    if (kv.second.published && kv.second.id == id) return &kv.second;
  }
  return nullptr;
}

void CandGridMapper::process(const FrameInput& f) {
  count("frames_total");
  // descent climb-out tracking: SPRAYING -> anything keeps the descent freeze until the drone has
  // moved climb_release_m away from the hover point (fail closed while the pose is invalid)
  if (f.phase == MissionPhase::kSpraying) {
    if (f.pose.valid) { spray_x_ = f.pose.x; spray_y_ = f.pose.y; }
    if (f.hop_target >= 0) last_target_ = f.hop_target;
  } else if (prev_phase_ == MissionPhase::kSpraying) {
    climbing_ = true;
    climb_x_ = spray_x_;
    climb_y_ = spray_y_;
    climb_target_ = last_target_;
    count("climb_out_started");
  }
  if (f.phase != MissionPhase::kSpraying && f.hop_target >= 0) last_target_ = f.hop_target;
  prev_phase_ = f.phase;
  if (climbing_ && f.pose.valid && std::hypot(f.pose.x - climb_x_, f.pose.y - climb_y_) > cfg_.climb_release_m) {
    climbing_ = false;
    count("climb_out_released");
  }
  FrameInput fe;
  const FrameInput* fp = &f;
  if (climbing_ && f.hop_target != climb_target_ && f.phase != MissionPhase::kHopping) {
    fe = f;
    fe.hop_target = climb_target_;
    fp = &fe;
  }
  const PhasePolicy pol = climbOutPolicy(phasePolicy(f.phase, fp->hop_target), climbing_);
  if (climbing_) count("frames_climb_out_frozen");
  process2(*fp, pol);
}

void CandGridMapper::process2(const FrameInput& f, const PhasePolicy& pol) {
  if (pol.warn_unknown) warnOnce("phase", "mission phase unknown/missing: behaving as SCAN (fail open)");
  if (!pol.awake) {
    count("frames_skipped_sleeping");  // heartbeat: sleeping, not dead
    have_prev_ = false;
    return;
  }
  if (!f.pose.valid) {
    count("frames_held_no_pose");
    warnOnce("pose", "drone pose not valid: holding all processing (fail closed)");
    have_prev_ = false;
    return;
  }
  if (f.disparity.empty()) {
    count(f.boxes.empty() ? "frames_heartbeat_only" : "frames_dropped_no_disparity");
    return;
  }
  if (pol.freeze_non_target && f.hop_target < 0) {
    warnOnce("notarget", "HOPPING without a valid hop target: freezing every tree (fail closed)");
  }
  const auto t0 = Clock::now();
  integrateFrame(f, pol);
  stage_ms_["integrate"] += msSince(t0);
  if (f.t - last_extract_t_ >= cfg_.extract_period && have_win_) {
    const auto t1 = Clock::now();
    extract(f.t, pol, f.hop_target);
    stage_ms_["extract"] += msSince(t1);
  }
  ++stage_frames_;
}

// ------------------------------------------------------------------------------------------------
// Evidence integration: every valid disparity point -> world -> per-frame per-cell median -> DSM.

void CandGridMapper::integrateFrame(const FrameInput& f, const PhasePolicy& pol) {
  const int gw = f.grid_w, gh = f.grid_h;
  if (gw <= 0 || gh <= 0 || static_cast<int>(f.disparity.size()) != gw * gh) {
    count("frames_dropped_bad_disparity_size");
    return;
  }
  // frame quality from the body rotation rate (turn blur / noise bursts)
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

  // descent freeze: only cells near the target may receive evidence
  bool region = false;
  double rx0 = 0, ry0 = 0, rx1 = 0, ry1 = 0;
  if (pol.freeze_non_target) {
    const Inst* tg = byId(f.hop_target);
    if (!tg || tg->outline.size() < 3) {
      count("frames_dropped_frozen_no_target");
      return;
    }
    region = true;
    rx0 = ry0 = 1e18;
    rx1 = ry1 = -1e18;
    for (const auto& p : tg->outline) {
      rx0 = std::min(rx0, p.x); ry0 = std::min(ry0, p.y);
      rx1 = std::max(rx1, p.x); ry1 = std::max(ry1, p.y);
    }
    rx0 -= cfg_.target_margin; ry0 -= cfg_.target_margin;
    rx1 += cfg_.target_margin; ry1 += cfg_.target_margin;
  }

  const Transform world_body{f.pose.x, f.pose.y, f.pose.z, f.pose.qx, f.pose.qy, f.pose.qz, f.pose.qw};
  const Transform world_cam = world_body.compose(cal_.body_cam);
  const int s = std::max(1, cal_.stride), o = s / 2;
  // detector support mask on the grid (full boxes: support only, never geometry)
  std::vector<char> inbox(static_cast<std::size_t>(gw * gh), 0);
  for (const auto& b : f.boxes) {
    const int c0 = std::max(0, static_cast<int>(std::ceil((b.x - o) / s)));
    const int c1 = std::min(gw - 1, static_cast<int>(std::floor((b.x + b.width - o) / s)));
    const int r0 = std::max(0, static_cast<int>(std::ceil((b.y - o) / s)));
    const int r1 = std::min(gh - 1, static_cast<int>(std::floor((b.y + b.height - o) / s)));
    for (int r = r0; r <= r1; ++r) {
      for (int c = c0; c <= c1; ++c) inbox[static_cast<std::size_t>(r * gw + c)] = 1;
    }
  }
  struct Pt { int ix, iy; float z, w; bool box; };
  std::vector<Pt> pts;
  pts.reserve(f.disparity.size());
  const double inv_cell = 1.0 / cfg_.cell;
  std::uint64_t n_invalid = 0, n_range = 0, n_above = 0, n_region = 0;
  for (int r = 0; r < gh; ++r) {
    for (int c = 0; c < gw; ++c) {
      const std::size_t k = static_cast<std::size_t>(r * gw + c);
      Vec3 po;
      const double u = c * s + o, v = r * s + o;
      if (!backprojectOptical(cal_, u, v, f.disparity[k], &po)) { ++n_invalid; continue; }
      if (po.z < cfg_.min_range || po.z > cfg_.max_range) { ++n_range; continue; }
      const Vec3 p = world_cam.apply(po);
      if (p.z > f.pose.z - cfg_.above_drone_margin) { ++n_above; continue; }
      if (region && (p.x < rx0 || p.x > rx1 || p.y < ry0 || p.y > ry1)) { ++n_region; continue; }
      const double rho2 = (po.x * po.x + po.y * po.y) / (po.z * po.z);
      const double zr = cfg_.z_ref / po.z;
      const double w = zr * zr * zr * zr / (1.0 + cfg_.lateral_k * rho2);
      pts.push_back({static_cast<int>(std::floor(p.x * inv_cell)), static_cast<int>(std::floor(p.y * inv_cell)),
                     static_cast<float>(p.z), static_cast<float>(w), inbox[k] != 0});
    }
  }
  count("points_dropped_invalid_disparity", n_invalid);
  count("points_dropped_range", n_range);
  count("points_dropped_above_drone", n_above);
  if (n_region) count("points_dropped_descent_freeze", n_region);
  if (pts.empty()) return;
  int bx0 = pts[0].ix, bx1 = bx0, by0 = pts[0].iy, by1 = by0;
  for (const auto& p : pts) {
    bx0 = std::min(bx0, p.ix); bx1 = std::max(bx1, p.ix);
    by0 = std::min(by0, p.iy); by1 = std::max(by1, p.iy);
  }
  const long bw = bx1 - bx0 + 1, bh = by1 - by0 + 1;
  if (bw * bh > 4000000L) {
    count("frames_dropped_footprint_too_large");
    count("points_dropped_footprint_too_large", pts.size());
    return;
  }
  // counting sort by cell
  std::vector<int> start(static_cast<std::size_t>(bw * bh + 1), 0);
  for (const auto& p : pts) ++start[static_cast<std::size_t>((p.iy - by0) * bw + (p.ix - bx0) + 1)];
  for (std::size_t i = 1; i < start.size(); ++i) start[i] += start[i - 1];
  std::vector<int> fill(start.begin(), start.end() - 1);
  std::vector<int> idx(pts.size());
  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    const auto& p = pts[static_cast<std::size_t>(i)];
    idx[static_cast<std::size_t>(fill[static_cast<std::size_t>((p.iy - by0) * bw + (p.ix - bx0))]++)] = i;
  }
  std::vector<float> zs;
  std::uint64_t n_capped = 0, n_rej = 0, n_reset = 0, n_cells = 0;
  for (long cidx = 0; cidx < bw * bh; ++cidx) {
    const int a = start[static_cast<std::size_t>(cidx)], b = start[static_cast<std::size_t>(cidx + 1)];
    if (a == b) continue;
    zs.clear();
    double wsum = 0.0;
    bool box = false;
    for (int j = a; j < b; ++j) {
      const Pt& p = pts[static_cast<std::size_t>(idx[static_cast<std::size_t>(j)])];
      zs.push_back(p.z);
      wsum += p.w;
      box |= p.box;
    }
    const int n = b - a;
    const int ne = std::min(n, cfg_.frame_cell_cap);
    n_capped += static_cast<std::uint64_t>(n - ne);
    std::nth_element(zs.begin(), zs.begin() + n / 2, zs.end());
    const float med = zs[static_cast<std::size_t>(n / 2)];
    const float wf = static_cast<float>(q * (wsum / n) * ne);
    const int ix = bx0 + static_cast<int>(cidx % bw), iy = by0 + static_cast<int>(cidx / bw);
    Cell* cell = grid_.get(ix, iy, true);
    const FuseResult fr = fuseCell(cell, med, wf, cfg_);
    if (fr == FuseResult::kRejected) n_rej += static_cast<std::uint64_t>(n);
    if (fr == FuseResult::kReset) ++n_reset;
    if (box && fr != FuseResult::kRejected && cell->box < 65535) ++cell->box;
    ++n_cells;
  }
  count("points_integrated", pts.size());
  count("points_weight_capped_per_cell", n_capped);  // used for the median, not for extra weight
  if (n_rej) count("points_rejected_cell_outlier", n_rej);
  if (n_reset) count("cells_reset_after_disagreement", n_reset);
  count("cell_updates", n_cells);
  counters_["gauge_grid_tiles"] = grid_.tiles();  // gauges (not monotonic): memory of the sparse grid
  counters_["gauge_grid_bytes"] = grid_.bytes();
  if (!have_win_) {
    wx0_ = bx0; wx1_ = bx1; wy0_ = by0; wy1_ = by1;
    have_win_ = true;
  } else {
    wx0_ = std::min(wx0_, bx0); wx1_ = std::max(wx1_, bx1);
    wy0_ = std::min(wy0_, by0); wy1_ = std::max(wy1_, by1);
  }
}

// ------------------------------------------------------------------------------------------------
// Extraction: segment the recent window and update the persistent instances.

double CandGridMapper::cellsHeight(const std::vector<std::pair<int, int>>& cells) const {
  std::vector<float> h;
  h.reserve(cells.size());
  for (const auto& c : cells) {
    const Cell* k = grid_.find(c.first, c.second);
    if (k && k->w >= cfg_.latch_weight && std::isfinite(k->h)) h.push_back(k->h);
  }
  if (h.empty()) return std::numeric_limits<double>::quiet_NaN();
  const std::size_t i = static_cast<std::size_t>(std::floor(cfg_.height_quantile * static_cast<double>(h.size() - 1) + 0.5));
  std::nth_element(h.begin(), h.begin() + static_cast<long>(i), h.end());
  return h[i];
}

void CandGridMapper::rebuildOutline(Inst* in) {
  std::vector<Point2> pts;
  pts.reserve(in->cells.size() * 4);
  for (const auto& c : in->cells) {
    const double x = c.first * cfg_.cell, y = c.second * cfg_.cell;
    pts.push_back({x, y});
    pts.push_back({x + cfg_.cell, y});
    pts.push_back({x, y + cfg_.cell});
    pts.push_back({x + cfg_.cell, y + cfg_.cell});
  }
  for (const auto& p : in->outline) pts.push_back(p);  // grow-only: the old outline stays inside (rule 2)
  in->outline = convexHull(std::move(pts));
}

void CandGridMapper::extract(double t, const PhasePolicy& pol, TreeId target) {
  // height rate limit: the step allowed in this extraction (first extraction / long gaps: one period)
  const double since = t - last_extract_t_;
  const double max_step = cfg_.max_height_rate * std::min(std::max(since, 0.0), std::max(cfg_.extract_period, 1e-3) * 2.0);
  last_extract_t_ = t;
  const int m = static_cast<int>(std::ceil(cfg_.window_margin / cfg_.cell));
  Window win;
  win.x0 = wx0_ - m;
  win.y0 = wy0_ - m;
  win.w = wx1_ - wx0_ + 1 + 2 * m;
  win.h = wy1_ - wy0_ + 1 + 2 * m;
  have_win_ = false;
  if (win.w > kMaxWindowCells) { win.x0 += (win.w - kMaxWindowCells) / 2; win.w = kMaxWindowCells; count("extract_window_clipped"); }
  if (win.h > kMaxWindowCells) { win.y0 += (win.h - kMaxWindowCells) / 2; win.h = kMaxWindowCells; count("extract_window_clipped"); }
  win.cells.resize(static_cast<std::size_t>(win.w * win.h));
  for (int y = 0; y < win.h; ++y) {
    for (int x = 0; x < win.w; ++x) {
      const Cell* k = grid_.find(win.x0 + x, win.y0 + y);
      if (k) win.cells[static_cast<std::size_t>(y * win.w + x)] = *k;
    }
  }
  const Segmentation seg = segmentCanopy(win, cfg_);
  count("extractions");

  auto gx = [&](int i) { return win.x0 + i % win.w; };
  auto gy = [&](int i) { return win.y0 + i / win.w; };
  auto centre = [&](int i) { return Point2{(gx(i) + 0.5) * cfg_.cell, (gy(i) + 0.5) * cfg_.cell}; };
  auto latchable = [&](int i, const Segment& sg) {
    const Cell& k = win.at(i);
    const float chm = seg.chm[static_cast<std::size_t>(i)];
    return k.w >= cfg_.latch_weight && k.nf >= cfg_.latch_min_frames && k.spread() <= cfg_.latch_max_spread &&
           chm >= std::max(cfg_.canopy_min_h, cfg_.latch_rel_h * sg.peak_chm);
  };
  std::set<std::pair<int, int>> voted;  // (survivor-candidate key, other key) seen in one basin this time

  for (const Segment& sg : seg.segments) {
    if (!sg.tree) { count("segments_rejected_too_low"); continue; }
    if (sg.touches_border) { count("segments_skipped_window_border"); continue; }
    const bool closed = sg.peak_observed && sg.open_fraction <= cfg_.max_open_fraction;
    const Point2 pk = centre(sg.peak);
    // owners among published trees
    std::map<int, int> owners;
    for (int i : sg.cells) {
      const int o = win.at(i).owner;
      if (o >= 0) ++owners[o];
    }
    Inst* T = nullptr;
    int best_n = 0;
    for (const auto& kv : owners) {
      auto it = inst_.find(kv.first);
      if (it == inst_.end()) continue;
      if (kv.second > best_n) { best_n = kv.second; T = &it->second; }
    }
    if (T) {
      const bool owns_peak = win.at(sg.peak).owner == T->key;
      const bool near_peak = std::hypot(pk.x - T->peak_x, pk.y - T->peak_y) <= cfg_.peak_match_dist;
      const bool majority = 2 * best_n >= static_cast<int>(sg.cells.size());
      if (!(owns_peak || near_peak || majority)) T = nullptr;  // a different dome next to T's spill cells
    }
    // rule 5: a basin on a sprayed tree never becomes a new tree
    bool in_sprayed = false;
    for (const auto& kv : inst_) {
      if (kv.second.sprayed && pointInPolygon(kv.second.outline, pk)) in_sprayed = true;
    }
    if (!T && in_sprayed) { count("segments_rejected_inside_sprayed"); continue; }
    if (!T) {
      // unpublished instance by peak proximity, else a new one
      double bd = cfg_.peak_match_dist;
      for (auto& kv : inst_) {
        Inst& in = kv.second;
        if (in.published) continue;
        const double d = std::hypot(pk.x - in.peak_x, pk.y - in.peak_y);
        if (d <= bd) { bd = d; T = &in; }
      }
      if (!T) {
        if (!pol.may_create) { count("segments_rejected_new_tree_locked_phase"); continue; }
        Inst in;
        in.key = next_key_++;
        in.peak_x = pk.x;
        in.peak_y = pk.y;
        T = &inst_.emplace(in.key, in).first->second;
        count("instances_created");
      }
    }
    if (!instanceMayChange(T->published, T->sprayed, T->id, pol, target)) {
      count(T->sprayed ? "segments_ignored_sprayed" : "segments_ignored_frozen");
      continue;
    }
    T->last_seen = t;
    // other published trees whose peak lies in this basin: fragments of T (rule 4)
    for (const auto& kv : owners) {
      if (kv.first == T->key) continue;
      auto it = inst_.find(kv.first);
      if (it == inst_.end() || !it->second.published) continue;
      const Inst& O = it->second;
      const int ox = static_cast<int>(std::floor(O.peak_x / cfg_.cell)) - win.x0;
      const int oy = static_cast<int>(std::floor(O.peak_y / cfg_.cell)) - win.y0;
      if (ox < 0 || oy < 0 || ox >= win.w || oy >= win.h) continue;
      const int sidx = seg.label[static_cast<std::size_t>(oy * win.w + ox)];
      if (sidx < 0 || &seg.segments[static_cast<std::size_t>(sidx)] != &sg) continue;
      voted.insert({std::min(T->key, O.key), std::max(T->key, O.key)});
    }

    if (!T->published) {
      T->cells.clear();
      for (int i : sg.cells) {
        const Cell& k = win.at(i);
        if (k.owner < 0 && latchable(i, sg)) T->cells.push_back({gx(i), gy(i)});
      }
      T->peak_x = pk.x;
      T->peak_y = pk.y;
      T->peak_chm = sg.peak_chm;
      T->confirm = closed ? T->confirm + 1 : 0;
      InstanceEvidence ev;
      ev.area = static_cast<double>(T->cells.size()) * cfg_.cell * cfg_.cell;
      ev.peak_chm = sg.peak_chm;
      ev.box_fraction = sg.box_fraction;
      ev.confirm = T->confirm;
      ev.height = cellsHeight(T->cells);
      ev.closed = closed;
      const PublishVerdict v = publishDecision(ev, pol, cfg_);
      if (v != PublishVerdict::kPublish) {
        count(std::string("publish_blocked_") + verdictName(v));
        continue;
      }
      T->published = true;
      T->id = next_id_++;
      for (const auto& c : T->cells) {
        Cell* k = grid_.get(c.first, c.second, true);
        k->owner = T->key;
      }
      T->height = ev.height;
      T->outline.clear();
      rebuildOutline(T);
      count("instances_published");
      count("cells_latched", T->cells.size());
      continue;
    }
    // published: grow by persistent, confident, closed evidence only (rule 2)
    if (!closed) { count("growth_skipped_open_segment"); continue; }
    if (owns_peak_or_near(T, pk)) {
      T->peak_x = pk.x;
      T->peak_y = pk.y;
      T->peak_chm = std::max(T->peak_chm, static_cast<double>(sg.peak_chm));
    }
    std::size_t latched = 0;
    for (int i : sg.cells) {
      const Cell& kw = win.at(i);
      if (kw.owner >= 0 || !latchable(i, sg)) continue;
      Cell* k = grid_.get(gx(i), gy(i), true);
      if (k->cand == T->key) {
        if (k->cand_n < 255) ++k->cand_n;
      } else {
        k->cand = T->key;
        k->cand_n = 1;
      }
      if (k->cand_n >= cfg_.latch_confirm) {
        k->owner = T->key;
        T->cells.push_back({gx(i), gy(i)});
        ++latched;
      }
    }
    if (latched) {
      rebuildOutline(T);
      count("cells_latched", latched);
    }
    const double h = cellsHeight(T->cells);
    if (std::isfinite(h)) {
      const double d = h - T->height;
      if (std::fabs(d) > max_step) count("height_step_clamped");
      T->height += std::max(-max_step, std::min(max_step, d));
    } else {
      count("height_update_undefined_kept_last");
    }
  }

  // merges of persistent instances that share one basin for merge_confirm extractions (rule 4)
  for (auto& kv : inst_) {
    for (auto it = kv.second.merge_votes.begin(); it != kv.second.merge_votes.end();) {
      if (!voted.count({kv.first, it->first})) it = kv.second.merge_votes.erase(it);
      else ++it;
    }
  }
  for (const auto& pr : voted) {
    auto ia = inst_.find(pr.first), ib = inst_.find(pr.second);
    if (ia == inst_.end() || ib == inst_.end()) continue;
    int& votes = ia->second.merge_votes[pr.second];
    ++votes;
    if (votes < cfg_.merge_confirm) continue;
    Inst& A = ia->second;
    Inst& B = ib->second;
    if (!pol.may_merge) { count("merge_blocked_phase"); continue; }
    if (A.sprayed || B.sprayed) { count("merge_blocked_sprayed"); continue; }
    const double aa = static_cast<double>(A.cells.size()), ab = static_cast<double>(B.cells.size());
    if (!mergeAllowedBySize(std::min(aa, ab), std::max(aa, ab), cfg_)) { count("merge_blocked_size"); continue; }
    const bool a_keeps = mergeSurvivor({A.published, aa, A.key}, {B.published, ab, B.key});
    Inst& keep = a_keeps ? A : B;
    Inst& lose = a_keeps ? B : A;
    for (const auto& c : lose.cells) {
      Cell* k = grid_.get(c.first, c.second, true);
      k->owner = keep.key;
      keep.cells.push_back(c);
    }
    for (const auto& p : lose.outline) keep.outline.push_back(p);
    keep.outline = convexHull(std::move(keep.outline));  // survivor keeps identity + height (rule 4)
    keep.merge_votes.clear();
    count("instances_merged");
    inst_.erase(lose.key);  // its ID is retired forever: next_id_ never goes back (rule 1)
  }
  // unpublished instances that stopped being seen are noise / false positives
  for (auto it = inst_.begin(); it != inst_.end();) {
    if (!it->second.published && t - it->second.last_seen > cfg_.unconfirmed_timeout) {
      it = inst_.erase(it);
      count("instances_expired_unconfirmed");
    } else {
      ++it;
    }
  }
}

bool CandGridMapper::owns_peak_or_near(const Inst* T, const Point2& pk) const {
  const Cell* k = grid_.find(static_cast<int>(std::floor(pk.x / cfg_.cell)), static_cast<int>(std::floor(pk.y / cfg_.cell)));
  return (k && k->owner == T->key) || std::hypot(pk.x - T->peak_x, pk.y - T->peak_y) <= cfg_.peak_match_dist;
}

std::vector<TreeEntry> CandGridMapper::publishedTrees() const {
  std::vector<TreeEntry> out;
  for (const auto& kv : inst_) {
    const Inst& in = kv.second;
    if (!in.published) continue;
    TreeEntry e;
    e.id = in.id;
    e.outline = in.outline;
    e.centre = polygonCentroid(in.outline);
    e.height = in.height;
    e.sprayed = in.sprayed;
    if (e.outline.size() < 3 || !std::isfinite(e.height) || !std::isfinite(e.centre.x) || !std::isfinite(e.centre.y)) {
      continue;  // rule 3: never an undefined height or a degenerate outline
    }
    out.push_back(std::move(e));
  }
  std::sort(out.begin(), out.end(), [](const TreeEntry& a, const TreeEntry& b) { return a.id < b.id; });
  return out;
}

bool CandGridMapper::markSprayed(TreeId id) {
  Inst* in = byId(id);
  if (!in) {
    count("mark_sprayed_unknown_id");
    return false;
  }
  in->sprayed = true;
  return true;
}

std::map<std::string, double> CandGridMapper::takeStageTiming() {
  std::map<std::string, double> out;
  if (stage_frames_ > 0) {
    for (const auto& kv : stage_ms_) out[kv.first] = kv.second / stage_frames_;
  }
  stage_ms_.clear();
  stage_frames_ = 0;
  return out;
}

}  // namespace cand_grid

// Registered in mapper_registry.cpp (TM_REGISTER_CANDIDATE(cand_grid)).
std::unique_ptr<Mapper> tm_create_cand_grid(const MapperConfig& config, const StereoCalibration& cal) {
  auto log = [](const std::string& m) { std::cerr << "[cand_grid] " << m << "\n"; };
  return std::make_unique<cand_grid::CandGridMapper>(cand_grid::configFrom(config, log), cal);
}

}  // namespace treemapper
