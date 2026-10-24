// cand_grid: configuration, pure decision functions, tiled grid and the canopy segmentation.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>

#include "treemapper/candidates/cand_grid/cand_grid.hpp"

namespace treemapper {
namespace cand_grid {

// ------------------------------------------------------------------------------------------------
// Configuration

#define CAND_GRID_FIELDS(X)                                                                         \
  X(cell) X(min_range) X(max_range) X(above_drone_margin) X(z_ref) X(lateral_k) X(frame_cell_cap)    \
  X(rot_rate_ref) X(quality_recover_s) X(cell_weight_cap) X(outlier_gate_m) X(gate_min_weight)       \
  X(coarse) X(ground_quantile) X(ground_radius) X(ground_band) X(ground_smooth) X(ground_min_frames) X(ground_min_weight) X(extract_period) X(window_margin)                   \
  X(min_cell_weight) X(canopy_min_h) X(tree_min_h) X(merge_saddle) X(max_tree_radius) X(peak_radius) \
  X(max_open_fraction) X(latch_weight) X(latch_rel_h) X(latch_max_spread) X(latch_min_frames) X(latch_confirm) X(box_min_frames)        \
  X(box_min_fraction) X(confirm_extractions) X(min_area) X(height_quantile) X(max_height_rate)       \
  X(peak_match_dist) X(merge_confirm) X(frag_max_area_ratio) X(unconfirmed_timeout) X(target_margin) X(climb_release_m)

namespace {
void readField(ConfigReader& r, const char* k, double* v) { *v = r.getDouble(k, *v); }
void readField(ConfigReader& r, const char* k, int* v) { *v = r.getInt(k, *v); }
std::string fmt(double v) {
  char b[64];
  std::snprintf(b, sizeof b, "%.10g", v);
  return b;
}
std::string fmt(int v) { return std::to_string(v); }

template <typename T>
void clampField(const char* name, T* v, T lo, T hi, std::vector<std::string>* msgs) {
  const T old = *v;
  if (!(*v >= lo)) *v = lo;  // also catches NaN
  if (*v > hi) *v = hi;
  if (!(old == *v)) {
    msgs->push_back(std::string("cand_grid config: ") + name + " = " + fmt(old) + " clamped to " + fmt(*v));
  }
}
}  // namespace

std::map<std::string, std::string> configKeyValues(const Config& c) {
  std::map<std::string, std::string> m;
#define X(f) m[#f] = fmt(c.f);
  CAND_GRID_FIELDS(X)
#undef X
  return m;
}

std::vector<std::string> validate(Config* c) {
  std::vector<std::string> m;
  const double inf = std::numeric_limits<double>::infinity();
  clampField("cell", &c->cell, 0.05, 2.0, &m);
  clampField("min_range", &c->min_range, 0.0, 100.0, &m);
  clampField("max_range", &c->max_range, c->min_range + 0.1, 200.0, &m);
  clampField("above_drone_margin", &c->above_drone_margin, 0.0, 5.0, &m);
  clampField("z_ref", &c->z_ref, 0.5, 100.0, &m);
  clampField("lateral_k", &c->lateral_k, 0.0, 100.0, &m);
  clampField("frame_cell_cap", &c->frame_cell_cap, 1, 1000, &m);
  clampField("rot_rate_ref", &c->rot_rate_ref, 1e-3, 100.0, &m);
  clampField("quality_recover_s", &c->quality_recover_s, 0.0, 30.0, &m);
  clampField("latch_weight", &c->latch_weight, 0.0, 1e6, &m);
  clampField("cell_weight_cap", &c->cell_weight_cap, std::max(c->latch_weight, c->min_cell_weight), 1e6, &m);
  clampField("outlier_gate_m", &c->outlier_gate_m, 0.05, inf, &m);
  clampField("gate_min_weight", &c->gate_min_weight, 0.0, 1e6, &m);
  clampField("coarse", &c->coarse, 1, 32, &m);
  clampField("ground_quantile", &c->ground_quantile, 0.0, 1.0, &m);
  clampField("ground_radius", &c->ground_radius, 0.0, 50.0, &m);
  clampField("ground_band", &c->ground_band, 0.0, 10.0, &m);
  clampField("ground_smooth", &c->ground_smooth, 0.0, 20.0, &m);
  clampField("ground_min_frames", &c->ground_min_frames, 1, 255, &m);
  clampField("ground_min_weight", &c->ground_min_weight, 0.0, 1e6, &m);
  clampField("extract_period", &c->extract_period, 0.0, 10.0, &m);
  clampField("window_margin", &c->window_margin, 0.0, 50.0, &m);
  clampField("min_cell_weight", &c->min_cell_weight, 1e-6, 1e6, &m);
  clampField("canopy_min_h", &c->canopy_min_h, 0.0, 20.0, &m);
  clampField("tree_min_h", &c->tree_min_h, c->canopy_min_h, 50.0, &m);
  clampField("merge_saddle", &c->merge_saddle, 0.0, 50.0, &m);
  clampField("max_tree_radius", &c->max_tree_radius, c->cell, 50.0, &m);
  clampField("peak_radius", &c->peak_radius, 0.0, 10.0, &m);
  clampField("max_open_fraction", &c->max_open_fraction, 0.0, 1.0, &m);
  clampField("latch_rel_h", &c->latch_rel_h, 0.0, 1.0, &m);
  clampField("latch_max_spread", &c->latch_max_spread, 0.0, 1e6, &m);
  clampField("latch_min_frames", &c->latch_min_frames, 1, 255, &m);
  clampField("latch_confirm", &c->latch_confirm, 1, 255, &m);
  clampField("box_min_frames", &c->box_min_frames, 0, 65535, &m);
  clampField("box_min_fraction", &c->box_min_fraction, 0.0, 1.0, &m);
  clampField("confirm_extractions", &c->confirm_extractions, 1, 1000, &m);
  clampField("min_area", &c->min_area, 0.0, 1e4, &m);
  clampField("height_quantile", &c->height_quantile, 0.0, 1.0, &m);
  clampField("max_height_rate", &c->max_height_rate, 0.0, 100.0, &m);
  clampField("peak_match_dist", &c->peak_match_dist, 0.0, 50.0, &m);
  clampField("merge_confirm", &c->merge_confirm, 1, 1000, &m);
  clampField("frag_max_area_ratio", &c->frag_max_area_ratio, 0.0, 1.0, &m);
  clampField("unconfirmed_timeout", &c->unconfirmed_timeout, 0.0, 1e4, &m);
  clampField("target_margin", &c->target_margin, 0.0, 50.0, &m);
  clampField("climb_release_m", &c->climb_release_m, 0.0, 100.0, &m);
  return m;
}

Config configFrom(const MapperConfig& config, const std::function<void(const std::string&)>& log) {
  Config c;
  ConfigReader r(config);
#define X(f) readField(r, #f, &c.f);
  CAND_GRID_FIELDS(X)
#undef X
  r.finish("cand_grid");
  for (const auto& msg : validate(&c)) {
    if (log) log(msg);
  }
  return c;
}

// ------------------------------------------------------------------------------------------------
// Pure decisions

double frameQuality(double rot_rate, double since_fast_s, const Config& c) {
  if (!(rot_rate >= 0.0)) return 1.0;  // unknown rate: fail open (normal weight)
  const double r = rot_rate / c.rot_rate_ref;
  double q = 1.0 / (1.0 + r * r);
  if (c.quality_recover_s > 0.0 && since_fast_s >= 0.0 && since_fast_s < c.quality_recover_s) {
    q = std::min(q, 0.1 + 0.9 * since_fast_s / c.quality_recover_s);
  }
  return q;
}

FuseResult fuseCell(Cell* cell, float med, float wf, const Config& c) {
  if (!(wf > 0.0f) || !std::isfinite(med)) return FuseResult::kRejected;
  if (cell->w >= c.gate_min_weight && std::fabs(med - cell->h) > c.outlier_gate_m) {
    if (++cell->rej < 3) return FuseResult::kRejected;
    // three consecutive frames disagree with the established value: the established value was wrong
    cell->h = med;
    cell->w = wf;
    cell->m2 = 0.0f;
    cell->nf = 1;
    cell->rej = 0;
    return FuseResult::kReset;
  }
  cell->rej = 0;
  const float wn = cell->w + wf;
  const float delta = med - cell->h;
  cell->h += (wf / wn) * delta;
  cell->m2 += wf * delta * (med - cell->h);  // West's weighted running variance
  const float cap = static_cast<float>(c.cell_weight_cap);
  if (wn > cap) {  // saturating confidence: rescale the spread accumulator with the weight
    cell->m2 *= cap / wn;
    cell->w = cap;
  } else {
    cell->w = wn;
  }
  if (cell->nf < 255) ++cell->nf;
  return FuseResult::kAccepted;
}

PublishVerdict publishDecision(const InstanceEvidence& e, const PhasePolicy& pol, const Config& c) {
  if (!pol.may_create) return PublishVerdict::kLockedPhase;
  if (!e.closed) return PublishVerdict::kOpen;
  if (!std::isfinite(e.height)) return PublishVerdict::kNoHeight;
  if (e.peak_chm < c.tree_min_h) return PublishVerdict::kTooLow;
  if (e.area < c.min_area) return PublishVerdict::kTooSmall;
  if (e.box_fraction < c.box_min_fraction) return PublishVerdict::kNoBoxSupport;
  if (e.confirm < c.confirm_extractions) return PublishVerdict::kNotConfirmed;
  return PublishVerdict::kPublish;
}

bool mergeSurvivor(const Establishment& a, const Establishment& b) {
  if (a.published != b.published) return a.published;
  if (a.area != b.area) return a.area > b.area;
  return a.key < b.key;
}

bool mergeAllowedBySize(double area_small, double area_large, const Config& c) {
  if (area_small > area_large) std::swap(area_small, area_large);
  return area_small <= c.frag_max_area_ratio * area_large;
}

bool instanceMayChange(bool published, bool sprayed, TreeId id, const PhasePolicy& pol, TreeId target) {
  if (sprayed) return false;                                  // rule 5
  if (!pol.awake) return false;
  if (!pol.may_create && !published) return false;           // no new trees after scanning
  if (pol.freeze_non_target && (target < 0 || id != target)) return false;  // descent: target only
  return true;
}

PhasePolicy climbOutPolicy(PhasePolicy p, bool climbing) {
  if (!climbing || !p.awake) return p;
  p.may_create = false;
  p.may_merge = false;
  p.freeze_non_target = true;
  return p;
}

// ------------------------------------------------------------------------------------------------
// Tiled grid

Cell* TiledGrid::get(int ix, int iy, bool create) {
  const int tx = ix >> kTileBits, ty = iy >> kTileBits;
  const std::int64_t k = key(tx, ty);
  Tile* t = nullptr;
  if (last_ && last_key_ == k) {
    t = last_;
  } else {
    auto it = tiles_.find(k);
    if (it == tiles_.end()) {
      if (!create) return nullptr;
      it = tiles_.emplace(k, std::make_unique<Tile>()).first;
    }
    t = it->second.get();
    last_ = t;
    last_key_ = k;
  }
  return &t->c[((iy & (kTile - 1)) << kTileBits) | (ix & (kTile - 1))];
}

const Cell* TiledGrid::find(int ix, int iy) const {
  auto it = tiles_.find(key(ix >> kTileBits, iy >> kTileBits));
  if (it == tiles_.end()) return nullptr;
  return &it->second->c[((iy & (kTile - 1)) << kTileBits) | (ix & (kTile - 1))];
}

// ------------------------------------------------------------------------------------------------
// Segmentation

namespace {
const float kNaN = std::numeric_limits<float>::quiet_NaN();

float quantileInPlace(std::vector<float>* v, double q) {
  if (v->empty()) return kNaN;
  const std::size_t k = static_cast<std::size_t>(std::floor(q * static_cast<double>(v->size() - 1) + 0.5));
  std::nth_element(v->begin(), v->begin() + static_cast<long>(k), v->end());
  return (*v)[k];
}

/// 1D running min over finite values, window [i-R, i+R] (O(n R), R is small).
void minFilter1D(const float* in, float* out, int n, int stride, int R) {
  for (int i = 0; i < n; ++i) {
    float m = std::numeric_limits<float>::infinity();
    for (int j = std::max(0, i - R); j <= std::min(n - 1, i + R); ++j) {
      const float v = in[j * stride];
      if (std::isfinite(v) && v < m) m = v;
    }
    out[i * stride] = std::isfinite(m) ? m : kNaN;
  }
}

int findRoot(std::vector<int>& p, int x) {
  while (p[static_cast<std::size_t>(x)] != x) {
    p[static_cast<std::size_t>(x)] = p[static_cast<std::size_t>(p[static_cast<std::size_t>(x)])];
    x = p[static_cast<std::size_t>(x)];
  }
  return x;
}
}  // namespace

Segmentation segmentCanopy(const Window& win, const Config& c) {
  const int W = win.w, H = win.h, N = W * H;
  Segmentation out;
  out.label.assign(static_cast<std::size_t>(N), -1);
  out.ground.assign(static_cast<std::size_t>(N), kNaN);
  out.chm.assign(static_cast<std::size_t>(N), kNaN);
  if (W <= 0 || H <= 0 || static_cast<int>(win.cells.size()) != N) return out;
  auto known = [&](int i) {
    const Cell& k = win.at(i);
    return k.nf > 0 && k.w >= c.min_cell_weight;
  };
  // the ground is defined only by well-observed cells: a cell seen once or twice at the swath edge is
  // too noisy, and a min filter over noise drags the ground down and turns noise into "canopy"
  auto groundCell = [&](int i) {
    const Cell& k = win.at(i);
    return k.nf >= c.ground_min_frames && k.w >= c.ground_min_weight;
  };

  // 1. ground: coarse low quantile of the surface, then a min filter of radius ground_radius
  const int K = std::max(1, c.coarse);
  const int CW = (W + K - 1) / K, CH = (H + K - 1) / K;
  std::vector<float> cv(static_cast<std::size_t>(CW * CH), kNaN), buf;
  const std::size_t min_known = static_cast<std::size_t>(std::max(3, K * K / 4));
  for (int cy = 0; cy < CH; ++cy) {
    for (int cx = 0; cx < CW; ++cx) {
      buf.clear();
      for (int y = cy * K; y < std::min(H, cy * K + K); ++y) {
        for (int x = cx * K; x < std::min(W, cx * K + K); ++x) {
          const int i = y * W + x;
          if (groundCell(i)) buf.push_back(win.at(i).h);
        }
      }
      if (buf.size() >= min_known) cv[static_cast<std::size_t>(cy * CW + cx)] = quantileInPlace(&buf, c.ground_quantile);
    }
  }
  const int R = static_cast<int>(std::ceil(c.ground_radius / (c.cell * K)));
  std::vector<float> tmp(cv.size()), gc(cv.size());
  for (int cy = 0; cy < CH; ++cy) minFilter1D(&cv[static_cast<std::size_t>(cy * CW)], &tmp[static_cast<std::size_t>(cy * CW)], CW, 1, R);
  for (int cx = 0; cx < CW; ++cx) minFilter1D(&tmp[static_cast<std::size_t>(cx)], &gc[static_cast<std::size_t>(cx)], CH, CW, R);
  // 1b. refinement: the min filter is biased low (cell noise, slope, undulation). Re-estimate the ground
  //     as the mean of the "ground-band" cells (within ground_band of the min-filter ground) in a small
  //     neighbourhood; keep the min-filter value where no band cell exists (fail safe: lower ground ->
  //     taller CHM -> a tree is never lost, only its edge may grow).
  if (c.ground_band > 0.0) {
    std::vector<float> sum(cv.size(), 0.0f), cnt(cv.size(), 0.0f);
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const int i = y * W + x;
        const std::size_t ci = static_cast<std::size_t>((y / K) * CW + x / K);
        const float g = gc[ci];
        if (!groundCell(i) || !std::isfinite(g)) continue;
        const float hh = win.at(i).h;
        if (hh < g + c.ground_band) { sum[ci] += hh; cnt[ci] += 1.0f; }
      }
    }
    const int R2 = std::max(1, static_cast<int>(std::ceil(c.ground_smooth / (c.cell * K))));
    std::vector<float> g2(gc);
    for (int cy = 0; cy < CH; ++cy) {
      for (int cx = 0; cx < CW; ++cx) {
        float s = 0.0f, n = 0.0f;
        for (int yy = std::max(0, cy - R2); yy <= std::min(CH - 1, cy + R2); ++yy) {
          for (int xx = std::max(0, cx - R2); xx <= std::min(CW - 1, cx + R2); ++xx) {
            s += sum[static_cast<std::size_t>(yy * CW + xx)];
            n += cnt[static_cast<std::size_t>(yy * CW + xx)];
          }
        }
        if (n >= 4.0f) g2[static_cast<std::size_t>(cy * CW + cx)] = s / n;
      }
    }
    gc.swap(g2);
  }

  // 2. CHM and canopy mask
  std::vector<char> canopy(static_cast<std::size_t>(N), 0);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const int i = y * W + x;
      const float g = gc[static_cast<std::size_t>((y / K) * CW + x / K)];
      out.ground[static_cast<std::size_t>(i)] = g;
      if (!known(i) || !std::isfinite(g)) continue;
      const float chm = win.at(i).h - g;
      if (chm >= c.canopy_min_h) {
        canopy[static_cast<std::size_t>(i)] = 1;
        out.chm[static_cast<std::size_t>(i)] = chm;
      }
    }
  }
  // 3. smoothed CHM (3x3 mean over canopy cells)
  std::vector<float> S(static_cast<std::size_t>(N), kNaN);
  std::vector<int> order;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const int i = y * W + x;
      if (!canopy[static_cast<std::size_t>(i)]) continue;
      float s = 0.0f;
      int n = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int xx = x + dx, yy = y + dy;
          if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
          const int j = yy * W + xx;
          if (canopy[static_cast<std::size_t>(j)]) { s += out.chm[static_cast<std::size_t>(j)]; ++n; }
        }
      }
      S[static_cast<std::size_t>(i)] = s / static_cast<float>(n);
      order.push_back(i);
    }
  }
  // 4. persistence watershed: flood from the highest cell down; where two basins meet, the lower one
  //    survives as a separate tree only if its peak rises >= merge_saddle above the saddle.
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    const float sa = S[static_cast<std::size_t>(a)], sb = S[static_cast<std::size_t>(b)];
    return sa != sb ? sa > sb : a < b;
  });
  std::vector<int> lab(static_cast<std::size_t>(N), -1), parent, peak_idx;
  std::vector<float> peak_s;
  int nb[8];
  for (int i : order) {
    const int x = i % W, y = i / W;
    int nn = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (!dx && !dy) continue;
        const int xx = x + dx, yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
        const int l = lab[static_cast<std::size_t>(yy * W + xx)];
        if (l < 0) continue;
        const int r = findRoot(parent, l);
        bool dup = false;
        for (int k = 0; k < nn; ++k) dup |= nb[k] == r;
        if (!dup) nb[nn++] = r;
      }
    }
    const float si = S[static_cast<std::size_t>(i)];
    if (nn == 0) {
      lab[static_cast<std::size_t>(i)] = static_cast<int>(parent.size());
      parent.push_back(static_cast<int>(parent.size()));
      peak_s.push_back(si);
      peak_idx.push_back(i);
      continue;
    }
    int best = nb[0];
    for (int k = 1; k < nn; ++k) {
      if (peak_s[static_cast<std::size_t>(nb[k])] > peak_s[static_cast<std::size_t>(best)]) best = nb[k];
    }
    lab[static_cast<std::size_t>(i)] = best;
    for (int k = 0; k < nn; ++k) {
      const int r = nb[k];
      if (r == best) continue;
      if (peak_s[static_cast<std::size_t>(r)] - si < c.merge_saddle) parent[static_cast<std::size_t>(r)] = best;
    }
  }
  // 5. segments (radius-limited around the peak) and their statistics
  std::vector<int> seg_of_root(parent.size(), -1);
  const double r2max = std::pow(c.max_tree_radius / c.cell, 2);
  for (int i : order) {
    const int r = findRoot(parent, lab[static_cast<std::size_t>(i)]);
    const int p = peak_idx[static_cast<std::size_t>(r)];
    const double dx = i % W - p % W, dy = i / W - p / W;
    if (dx * dx + dy * dy > r2max) continue;
    int& s = seg_of_root[static_cast<std::size_t>(r)];
    if (s < 0) {
      s = static_cast<int>(out.segments.size());
      Segment sg;
      sg.peak = p;
      sg.peak_chm = peak_s[static_cast<std::size_t>(r)];
      sg.tree = sg.peak_chm >= c.tree_min_h;
      out.segments.push_back(sg);
    }
    out.segments[static_cast<std::size_t>(s)].cells.push_back(i);
    out.label[static_cast<std::size_t>(i)] = s;
  }
  const int pr = static_cast<int>(std::ceil(c.peak_radius / c.cell));
  for (std::size_t s = 0; s < out.segments.size(); ++s) {
    Segment& sg = out.segments[s];
    int boundary = 0, open = 0, boxed = 0;
    for (int i : sg.cells) {
      const int x = i % W, y = i / W;
      if (x == 0 || y == 0 || x == W - 1 || y == H - 1) sg.touches_border = true;
      if (win.at(i).box >= c.box_min_frames) ++boxed;
      bool is_b = false, is_open = false;
      const int dxs[4] = {1, -1, 0, 0}, dys[4] = {0, 0, 1, -1};
      for (int k = 0; k < 4; ++k) {
        const int xx = x + dxs[k], yy = y + dys[k];
        if (xx < 0 || yy < 0 || xx >= W || yy >= H) { is_b = is_open = true; continue; }
        const int j = yy * W + xx;
        if (out.label[static_cast<std::size_t>(j)] != static_cast<int>(s)) {
          is_b = true;
          if (!known(j)) is_open = true;
        }
      }
      boundary += is_b;
      open += is_open;
    }
    sg.open_fraction = boundary > 0 ? static_cast<double>(open) / boundary : 0.0;
    sg.box_fraction = sg.cells.empty() ? 0.0 : static_cast<double>(boxed) / static_cast<double>(sg.cells.size());
    const int px = sg.peak % W, py = sg.peak / W;
    for (int dy = -pr; dy <= pr && sg.peak_observed; ++dy) {
      for (int dx = -pr; dx <= pr; ++dx) {
        if (dx * dx + dy * dy > pr * pr) continue;
        const int xx = px + dx, yy = py + dy;
        if (xx < 0 || yy < 0 || xx >= W || yy >= H || !known(yy * W + xx)) { sg.peak_observed = false; break; }
      }
    }
  }
  return out;
}

}  // namespace cand_grid
}  // namespace treemapper
