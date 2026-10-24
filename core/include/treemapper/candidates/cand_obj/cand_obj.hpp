// cand_obj: object-centric tree instance mapper (milestone M3 candidate).
//
// Per frame, the stereo grid is turned into a canopy height model (CHM) on a fixed world lattice: per
// cell a robust (median) height, a per-frame ground plane, 3x3 smoothing. Canopy cells are split into
// crowns by a watershed on the smoothed CHM (steepest ascent to a local maximum, then low-prominence
// basins are merged into their neighbours). A crown whose peak lies inside a detection box is an
// observation of one tree. Observations are matched to tree INSTANCES by 3D position (peak inside the
// instance's outline or near its centre): there are no 2D tracks to lose or to jump.
//
// Each instance owns a local evidence grid on the same lattice: per cell the number of frames the cell
// was its canopy (positive) and the number of frames the cell was seen as ground or as another
// instance's canopy (negative). A cell is LATCHED, permanently, when it has been positive in at least
// `latch_frames` frames and in at least `latch_ratio` of the frames it was judged. The published outline
// is the convex hull of the latched cells: it can only grow (rule 2), and no single frame can add a cell
// to it. Height = median of per-frame crown-top samples (rule 3: published only once there are enough
// finite samples). Identity: IDs are monotonic; a merge keeps the better-established instance's ID and
// height (rule 4); sprayed instances never change (rule 5). Every drop is counted by reason (rule 7).
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "treemapper/frame.hpp"
#include "treemapper/geometry.hpp"
#include "treemapper/mapper.hpp"
#include "treemapper/types.hpp"

namespace treemapper {
namespace cand_obj {

/// All tunables. In-struct defaults == config/cand_obj.cfg (enforced by a test). Clamping lives in
/// sanitized() only, and every clamp is logged.
struct Config {
  // --- points -> cells
  double min_range{0.5};            ///< m, optical depth gate
  double max_range{30.0};           ///< m
  double above_drone_margin{0.5};   ///< m, drop points higher than drone z - margin
  double cell{0.4};                 ///< m, world lattice cell (shared by the frame CHM and every instance)
  int min_cell_points{2};           ///< fewer valid points -> cell unknown
  int max_grid{160};                ///< cap of the per-frame CHM window, cells per side
  // --- ground plane (per frame, from the lowest cells)
  double ground_low_fraction{0.4};  ///< fit the plane to this lowest fraction of the cells
  double ground_inlier_band{0.6};   ///< m, refit with cells within +- band of the first plane
  double ground_max_slope{0.3};     ///< above this |slope| the fit is rejected (flat fallback)
  double min_above_ground{1.0};     ///< m, a smoothed cell this far above the plane is canopy
  // --- crowns (watershed basins)
  double min_prominence{0.8};       ///< m, a peak this little above its saddle is merged into the neighbour
  int min_basin_cells{4};           ///< smaller crowns are ignored
  double box_dilate{0.15};          ///< boxes grown by this fraction of their size before the peak test
  double border_px{8.0};            ///< px, a crown with cells this close to the image edge is truncated
  // --- instances
  int window{28};                   ///< cells per side of an instance's evidence window (anchored at birth)
  double match_radius{2.0};         ///< m, a peak this close to an instance centre is that instance
  double max_cell_radius{4.0};      ///< m, cells farther from the instance centre are not credited (spill guard)
  int latch_frames{3};              ///< positive frames before a cell latches
  double latch_ratio{0.5};          ///< ... and positive in at least this fraction of the judged frames
  int confirm_frames{4};            ///< frames with a matched crown before an instance may publish
  int min_height_samples{3};        ///< finite crown-top samples before an instance may publish
  double min_publish_area{3.0};     ///< m^2 of latched cells before an instance may publish
  double min_tree_height{1.5};      ///< m above ground, smaller crowns never seed an instance
  double tentative_timeout{1.5};    ///< s, unconfirmed instances unseen this long are deleted
  int height_buffer{64};            ///< crown-top samples kept per instance (median of them)
  // --- merges (rule 4)
  double merge_overlap{0.5};        ///< latched overlap / smaller latched area
  double merge_centre_dist{1.5};    ///< m, or centres closer than this
  // --- phases
  double post_spray_hold_radius{2.0};  ///< m, after SPRAYING everything but the target stays frozen until
                                       ///< the drone has moved this far from the hover point (the climb)

  /// Range-checked copy; each clamp is reported through `log` (may be empty).
  Config sanitized(const std::function<void(const std::string&)>& log) const;
};

/// Key/value -> Config (keys = field names); unknown keys throw std::invalid_argument.
Config configFrom(const MapperConfig& config);
/// Number of tunables, for the config-file test.
constexpr int kConfigKeyCount = 28;

/// One crown observation handed to the instance layer (built by the CHM stage, or by tests).
struct Crown {
  std::vector<std::pair<int, int>> cells;  ///< lattice cells (i, j) of the crown
  Point2 peak;                             ///< world xy of the crown top
  double peak_z{0.0};                      ///< world z of the crown top (height sample)
  bool boxed{true};                        ///< peak lies inside a detection box
  bool truncated{false};                   ///< touches the image border (no height sample)
};

/// The per-frame view the instance layer needs besides the crowns: which lattice cells were judged.
struct FrameView {
  int i0{0}, j0{0}, w{0}, h{0};      ///< lattice window of the CHM
  std::vector<std::uint8_t> state;  ///< per window cell: 0 unknown, 1 ground, 2 canopy
  std::vector<int> crown;           ///< per window cell: crown index or -1
  bool valid() const { return w > 0 && h > 0 && state.size() == static_cast<std::size_t>(w * h); }
};

enum class State { kAwake, kSleeping, kHoldingNoPose };

class CandObj : public Mapper {
 public:
  explicit CandObj(const Config& cfg = {}, const StereoCalibration& cal = {});

  void process(const FrameInput& frame) override;
  std::vector<TreeEntry> publishedTrees() const override;
  bool markSprayed(TreeId id) override;
  std::map<std::string, double> takeStageTiming() override;
  const std::map<std::string, std::uint64_t>& counters() const override { return counters_; }
  void setLogger(std::function<void(const std::string&)> log) override { log_ = std::move(log); }

  /// Stage 1+2: points -> CHM -> crowns. Exposed for tests.
  void extractCrowns(const FrameInput& frame, FrameView* view, std::vector<Crown>* crowns);
  /// Stage 3: crowns -> instances under the phase policy. Exposed for rule tests (a test builds its own
  /// crowns and view). `drone_z` gates height samples; `drone_xy` drives the post-spray hold.
  void integrate(double t, MissionPhase phase, TreeId hop_target, const DronePose& pose, const FrameView& view,
                 std::vector<Crown>& crowns);

  State state() const { return state_; }
  const Config& config() const { return cfg_; }
  double cellSize() const { return cfg_.cell; }

 private:
  struct Instance {
    TreeId id{kNoTree};
    int ai{0}, aj{0};                    ///< lattice origin of the evidence window
    std::vector<std::uint16_t> npos, nneg;
    std::vector<std::uint8_t> latched;
    int n_latched{0};
    double lat_sum_x{0.0}, lat_sum_y{0.0};  ///< sum of latched cell centres (world)
    double peak_sum_x{0.0}, peak_sum_y{0.0};
    int n_peaks{0};
    int frames_seen{0};
    double first_seen{0.0}, last_seen{0.0};
    std::vector<double> heights;  ///< sorted
    bool confirmed{false};
    bool sprayed{false};
    bool hull_dirty{true};
    Polygon2 hull;
    Point2 centre;                ///< latched centroid, else running peak mean
    double height() const { return heights.empty() ? 0.0 : heights[heights.size() / 2]; }
    bool hasHeight(int min_samples) const { return static_cast<int>(heights.size()) >= min_samples; }
  };

  void count(const std::string& k, std::uint64_t n = 1) { counters_[k] += n; }
  void warnOnce(const std::string& key, const std::string& msg);
  int findInstanceForPeak(const Point2& peak) const;
  bool insideSprayed(const Point2& p) const;
  void creditCrown(Instance& in, const Crown& c, std::vector<int>& cell_owner, const FrameView& view);
  void updateEvidence(Instance& in, const FrameView& view, const std::vector<int>& cell_owner, int self_index);
  void refreshHull(Instance& in);
  void mergeInstances(double t);
  void absorb(Instance& keep, const Instance& lose);
  double latchedArea(const Instance& in) const { return in.n_latched * cfg_.cell * cfg_.cell; }
  bool publishable(const Instance& in) const;

  Config cfg_;
  StereoCalibration cal_;
  std::vector<Instance> instances_;  ///< live instances (tentative + confirmed), in creation order
  TreeId next_id_{0};
  State state_{State::kAwake};
  bool hold_after_spray_{false};
  bool hover_known_{false};
  double hover_x_{0.0}, hover_y_{0.0};
  MissionPhase prev_phase_{MissionPhase::kUnknown};
  std::map<std::string, std::uint64_t> counters_;
  std::map<std::string, double> stage_ms_;
  int stage_frames_{0};
  std::map<std::string, bool> warned_;
  std::function<void(const std::string&)> log_;
  // scratch buffers (per frame)
  std::vector<double> px_, py_, pz_, pu_, pv_;
  std::vector<char> pok_;
};

}  // namespace cand_obj
}  // namespace treemapper
