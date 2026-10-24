// cand_grid: map-centric candidate (M3b). Design: docs/candidates/cand_grid.md.
//
// Evidence is fused into a world-fixed, sparse, tiled 2.5D surface grid (a DSM: per cell a weighted
// canopy-surface height, a capped confidence weight, a frame count and detector support). Trees are
// extracted from that grid, not from individual detections:
//   ground (low quantile of the DSM, min-filtered) -> canopy height model (CHM = DSM - ground)
//   -> persistence watershed on the smoothed CHM (one basin per dome; touching trees separate at their
//      valley even when the gap is below the per-point depth noise, because the valley is a property of
//      the fused surface, not of single points)
//   -> persistent instances. Published instances own "latched" cells; their outline is the convex hull
//      of the latched cells and only ever grows (rule 2). Unpublished instances are re-derived every
//      extraction, so pre-publication segmentation mistakes cost nothing.
// The detector boxes only provide semantic support ("this surface is a tree"), never geometry: a box
// corner on the ground or on a neighbour cannot put points into an outline.
//
// All decision logic sits in small pure functions below (frameQuality, fuseCell, segmentCanopy,
// publishDecision, mergeSurvivor, instanceMayChange) so each rule is testable on its own.
#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "treemapper/frame.hpp"
#include "treemapper/geometry.hpp"
#include "treemapper/mapper.hpp"
#include "treemapper/simple_mapper.hpp"  // phasePolicy() (shared pure phase rules)
#include "treemapper/types.hpp"

namespace treemapper {
namespace cand_grid {

/// Every tunable of the candidate. Defaults == config/cand_grid.cfg (enforced by a test).
struct Config {
  // --- input gates
  double cell{0.25};               ///< m, grid cell side
  double min_range{0.5};           ///< m, optical depth gate
  double max_range{25.0};          ///< m, beyond this stereo depth is unreliable
  double above_drone_margin{0.3};  ///< points above drone z - margin are impossible (dropped)
  // --- evidence weighting
  double z_ref{15.0};              ///< m, range of unit point weight (weight ~ (z_ref/Z)^4: sigma_Z ~ Z^2)
  double lateral_k{10.0};          ///< weight /= 1 + k*rho^2 (rho = off-axis tangent; lateral error ~ rho*sigma_Z)
  int frame_cell_cap{6};           ///< points of one cell in one frame that count (hover/density invariance)
  double rot_rate_ref{0.35};       ///< rad/s; frame weight *= 1/(1+(rate/ref)^2) (turn blur / noise bursts)
  double quality_recover_s{1.5};   ///< s, frame weight recovers linearly to 1 after a fast rotation
  double cell_weight_cap{30.0};    ///< DSM confidence saturates here (hovering cannot inflate it)
  double outlier_gate_m{2.0};      ///< frame median this far from an established cell height is rejected
  double gate_min_weight{6.0};     ///< ... once the cell has this much weight
  // --- ground
  int coarse{4};                   ///< fine cells per coarse ground cell side
  double ground_quantile{0.2};     ///< coarse ground value = this quantile of its known fine cells
  double ground_radius{4.5};       ///< m, first ground guess = min of coarse values within this radius
  double ground_band{0.6};         ///< m, refined ground = mean of cells within this band above the guess
  double ground_smooth{1.5};       ///< m, neighbourhood radius of that mean (0 band = no refinement)
  int ground_min_frames{3};        ///< only cells seen in this many frames define the ground ...
  double ground_min_weight{3.0};   ///< ... with at least this weight (noise at the swath edge drags it down)
  // --- extraction / segmentation
  double extract_period{0.2};      ///< s between extractions
  double window_margin{6.0};       ///< m around the recent footprint
  double min_cell_weight{1.0};     ///< cell counts as observed ("known")
  double canopy_min_h{0.5};        ///< m above ground: canopy cell
  double tree_min_h{1.8};          ///< m above ground: a basin peak lower than this is not a tree
  double merge_saddle{0.8};        ///< m, basins whose lower peak rises less than this above the saddle merge
  double max_tree_radius{3.6};     ///< m, segment cells farther from the peak are cut
  double peak_radius{1.0};         ///< m, the peak neighbourhood must be fully observed
  double max_open_fraction{0.6};   ///< segment boundary cells touching unobserved cells, at most
  // --- confirmation / publication (confidence model)
  double latch_weight{3.0};        ///< cell weight to latch
  double latch_rel_h{0.6};         ///< a cell latches only if its CHM >= this fraction of its tree's peak CHM
  double latch_max_spread{1.5};    ///< m, cells whose frame medians spread more than this never latch
  int latch_min_frames{3};         ///< distinct frames a cell must have been seen in to latch
  int latch_confirm{2};            ///< consecutive extractions a cell must be assigned to the same tree
  int box_min_frames{2};           ///< frames with detector support for a cell to count as box-supported
  double box_min_fraction{0.3};    ///< fraction of a segment's cells that must be box-supported
  int confirm_extractions{2};      ///< consecutive extractions before an instance is published
  double min_area{5.0};            ///< m^2, smaller instances are not published
  double height_quantile{0.93};    ///< published height = this quantile of the latched cells' surface z
  double max_height_step{0.25};    ///< m per extraction a published height may move (no jumps)
  double peak_match_dist{1.5};     ///< m, unpublished instance <-> segment peak association
  // --- persistent-instance merge (rule 4)
  int merge_confirm{3};            ///< consecutive extractions two instances share one basin
  double frag_max_area_ratio{0.6}; ///< the absorbed one must be at most this fraction of the survivor
  double unconfirmed_timeout{3.0}; ///< s, unpublished instances unseen this long are dropped
  double target_margin{1.5};       ///< m, during a descent only cells near the target are updated
  double climb_release_m{1.5};     ///< m, after SPRAYING the descent freeze holds until the drone moved this far
};

/// Parses the key=value configuration (keys = field names). Unknown keys throw; values are validated
/// and clamped in one place (validate()), every clamp is reported through `log`.
Config configFrom(const MapperConfig& c, const std::function<void(const std::string&)>& log = {});
/// Clamps out-of-range values into their valid range; returns one message per clamp.
std::vector<std::string> validate(Config* c);
/// Every key with its value as text (the shipped config/cand_grid.cfg must equal configKeyValues(Config{})).
std::map<std::string, std::string> configKeyValues(const Config& c);

// ------------------------------------------------------------------------------------------------
// Pure decision functions

/// Frame weight from the body rotation rate (rad/s) and the time since the last fast rotation.
/// Unknown rate (< 0) -> 1 (fail open: one frame without a predecessor carries normal weight).
double frameQuality(double rot_rate, double since_fast_s, const Config& c);

/// One grid cell of the fused surface.
struct Cell {
  float h{0.0f};       ///< fused surface height (world z)
  float w{0.0f};       ///< confidence weight, saturates at cell_weight_cap
  float m2{0.0f};      ///< weighted sum of squared deviations of the frame medians (spread = sqrt(m2 / w))
  float spread() const { return w > 0.0f ? std::sqrt(m2 / w) : 0.0f; }
  std::int32_t owner{-1};   ///< instance key of a published tree that latched this cell
  std::int32_t cand{-1};    ///< instance this cell was last assigned to (latch persistence)
  std::uint16_t box{0};     ///< frames in which a detection box covered this cell (capped)
  std::uint8_t nf{0};       ///< frames that updated this cell (capped)
  std::uint8_t cand_n{0};   ///< consecutive extractions assigned to `cand`
  std::uint8_t rej{0};      ///< consecutive rejected frame medians
};

enum class FuseResult { kAccepted, kRejected, kReset };
/// Fuses one frame's robust per-cell measurement (median z of the cell's points, weight wf).
/// Established cells reject medians beyond the outlier gate; 3 consecutive rejections re-initialise
/// the cell (the established value was the outlier).
FuseResult fuseCell(Cell* cell, float med, float wf, const Config& c);

/// Dense copy of a window of the grid, input of the segmentation.
struct Window {
  int x0{0}, y0{0}, w{0}, h{0};  ///< cell index of the lower-left corner, size in cells
  std::vector<Cell> cells;       ///< row-major (iy - y0) * w + (ix - x0)
  const Cell& at(int i) const { return cells[static_cast<std::size_t>(i)]; }
};

struct Segment {
  std::vector<int> cells;     ///< window indices
  int peak{-1};               ///< window index of the highest smoothed-CHM cell
  float peak_chm{0.0f};       ///< smoothed CHM at the peak, m above ground
  double open_fraction{0.0};  ///< boundary cells adjacent to unobserved / out-of-window cells
  bool peak_observed{true};   ///< every cell within peak_radius of the peak observed and inside the window
  bool touches_border{false}; ///< a cell on the window border (the tree may continue outside)
  double box_fraction{0.0};   ///< fraction of cells with box support
  bool tree{false};           ///< peak_chm >= tree_min_h
};

struct Segmentation {
  std::vector<int> label;     ///< per window cell: segment index or -1
  std::vector<float> ground;  ///< per window cell (NaN = unknown)
  std::vector<float> chm;     ///< per window cell (NaN = not canopy)
  std::vector<Segment> segments;
};
/// Ground -> CHM -> persistence watershed. Pure function of the window.
Segmentation segmentCanopy(const Window& win, const Config& c);

/// Why an unpublished instance is (not) published yet.
enum class PublishVerdict { kPublish, kLockedPhase, kTooSmall, kTooLow, kNoBoxSupport, kNotConfirmed,
                            kNoHeight, kOpen };
struct InstanceEvidence {
  double area{0.0};          ///< m^2 of latchable cells
  double peak_chm{0.0};
  double box_fraction{0.0};
  int confirm{0};            ///< consecutive extractions seen as a closed tree segment
  double height{0.0};        ///< candidate published height (NaN = undefined)
  bool closed{false};        ///< segment closed and peak observed
};
PublishVerdict publishDecision(const InstanceEvidence& e, const PhasePolicy& pol, const Config& c);

/// Rank of establishment for merges (rule 4): published > more latched area > lower key.
struct Establishment {
  bool published{false};
  double area{0.0};
  int key{0};
};
/// True if `a` survives a merge with `b`.
bool mergeSurvivor(const Establishment& a, const Establishment& b);
/// Size-aware merge gate: a fragment (smaller <= ratio * larger) merges; two large trees never do.
bool mergeAllowedBySize(double area_small, double area_large, const Config& c);

/// May this instance change now? Sprayed trees never; frozen non-targets never; in a locked phase
/// unpublished instances never (they could become a new tree navigation did not plan against).
bool instanceMayChange(bool published, bool sprayed, TreeId id, const PhasePolicy& pol, TreeId target);

/// The climb out of a spray hover is still part of the descent (the drone is low over a tree whatever
/// navigation's phase says): while `climbing`, nothing may be created or merged and only the last target
/// may change. Pure: base policy in, effective policy out.
PhasePolicy climbOutPolicy(PhasePolicy base, bool climbing);

// ------------------------------------------------------------------------------------------------
// Sparse tiled grid (bounded memory: only observed 8 m x 8 m tiles exist).

class TiledGrid {
 public:
  static constexpr int kTileBits = 5;  ///< 32 x 32 cells per tile
  static constexpr int kTile = 1 << kTileBits;
  Cell* get(int ix, int iy, bool create);
  const Cell* find(int ix, int iy) const;
  std::size_t tiles() const { return tiles_.size(); }
  std::size_t bytes() const { return tiles_.size() * sizeof(Tile); }

 private:
  struct Tile { Cell c[kTile * kTile]; };
  static std::int64_t key(int tx, int ty) {
    return (static_cast<std::int64_t>(tx) << 32) ^ static_cast<std::uint32_t>(ty);
  }
  std::unordered_map<std::int64_t, std::unique_ptr<Tile>> tiles_;
  std::int64_t last_key_{0};
  Tile* last_{nullptr};
};

// ------------------------------------------------------------------------------------------------

class CandGridMapper : public Mapper {
 public:
  explicit CandGridMapper(const Config& cfg = {}, const StereoCalibration& cal = {});

  void process(const FrameInput& frame) override;
  std::vector<TreeEntry> publishedTrees() const override;
  bool markSprayed(TreeId id) override;
  std::map<std::string, double> takeStageTiming() override;
  const std::map<std::string, std::uint64_t>& counters() const override { return counters_; }
  void setLogger(std::function<void(const std::string&)> log) override { log_ = std::move(log); }

  // --- exposed for tests
  const TiledGrid& grid() const { return grid_; }
  std::size_t instanceCount() const { return inst_.size(); }

 private:
  struct Inst {
    int key{0};
    TreeId id{kNoTree};
    bool published{false}, sprayed{false};
    std::vector<std::pair<int, int>> cells;  ///< latched (published) or tentative (unpublished) cells
    Polygon2 outline;                         ///< published: hull of latched cells (grow-only)
    double height{0.0};
    double peak_x{0.0}, peak_y{0.0}, peak_chm{0.0};
    int confirm{0};
    double last_seen{0.0};
    std::map<int, int> merge_votes;  ///< other key -> consecutive extractions in one basin
  };

  void process2(const FrameInput& f, const PhasePolicy& pol);
  void integrateFrame(const FrameInput& f, const PhasePolicy& pol);
  void extract(double t, const PhasePolicy& pol, TreeId target);
  void rebuildOutline(Inst* in);
  double cellsHeight(const std::vector<std::pair<int, int>>& cells) const;
  bool owns_peak_or_near(const Inst* T, const Point2& pk) const;
  void count(const std::string& k, std::uint64_t n = 1) { counters_[k] += n; }
  void warnOnce(const std::string& key, const std::string& msg);
  Inst* byId(TreeId id);

  Config cfg_;
  StereoCalibration cal_;
  TiledGrid grid_;
  std::map<int, Inst> inst_;
  int next_key_{0};
  TreeId next_id_{0};  ///< monotonic, only published trees get one (rule 1)
  // pose history for the rotation-rate frame quality
  bool have_prev_{false};
  double prev_stamp_{0.0}, last_fast_t_{-1e9};
  double pqx_{0}, pqy_{0}, pqz_{0}, pqw_{1};
  // extraction window = union of footprints since the last extraction
  bool have_win_{false};
  int wx0_{0}, wy0_{0}, wx1_{0}, wy1_{0};
  double last_extract_t_{-1e18};
  // descent climb-out (after SPRAYING until the drone left the hover point)
  MissionPhase prev_phase_{MissionPhase::kUnknown};
  bool climbing_{false};
  double climb_x_{0.0}, climb_y_{0.0}, spray_x_{0.0}, spray_y_{0.0};
  TreeId climb_target_{kNoTree}, last_target_{kNoTree};
  std::map<std::string, std::uint64_t> counters_;
  std::map<std::string, double> stage_ms_;
  int stage_frames_{0};
  std::map<std::string, bool> warned_;
  std::function<void(const std::string&)> log_;
};

}  // namespace cand_grid
}  // namespace treemapper
