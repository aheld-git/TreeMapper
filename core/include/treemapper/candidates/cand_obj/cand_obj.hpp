// cand_obj: object-centric tree instance mapper (milestone M3 candidate, hardened in M4).
//
// Per frame, the stereo grid is turned into a canopy height model (CHM) on a fixed world lattice: per
// cell a robust (median) height, a per-frame ground plane, 3x3 smoothing. Canopy cells are split into
// crowns by a watershed on the smoothed CHM (steepest ascent to a local maximum, then low-prominence
// basins are merged into their neighbours). A crown whose peak lies inside a detection box is an
// observation of one tree. Observations are matched to tree INSTANCES by 3D position (the crown covers
// the instance's latched cells, or its peak lies inside the instance's outline or near its centre):
// there are no 2D tracks to lose or to jump.
//
// Each instance owns a local evidence grid on the same lattice: per cell the (frame-quality weighted)
// number of frames the cell was its canopy (positive) and the number of frames the cell was seen as
// ground or as another instance's canopy (negative). A cell is LATCHED, permanently, when it has been
// positive in at least `latch_frames` frames and in at least `latch_ratio` of the frames it was judged;
// cells below `latch_rel_h` of the crown's peak height (the low skirt) are never claimed. The published
// outline is the convex hull of the latched cells: it can only grow (rule 2), and no single frame can add
// a cell to it.
//
// Height (M4): every latched cell carries a fused surface height (range / off-axis / rotation-rate
// weighted running mean with a saturating weight and an outlier gate); the tree height is a high
// quantile (`height_quantile`) of the latched cells' surface, so a partial first view (crown entering at
// the swath edge) cannot pin the height to the top of the visible slice. Published height and centre are
// rate-limited (`max_height_rate`, `max_centre_rate`), so they never jump; sprayed and frozen instances
// are never stepped.
//
// Centre (D66): latching is permanent, and a crown first seen from a far track at the image edge latches cells
// displaced toward that track (stereo range error along the oblique ray). The published centre is therefore not
// the latched centroid but the centroid of the cells the instance's OBSERVATION-WEIGHTED evidence currently calls
// its canopy (each frame counts min(point weight, 1) x frame quality, so a far, oblique view counts ~0.15 of a
// near-nadir one): once near views exist the far view's ghost band drops out and the unlatched far half of the
// crown comes in. The outline (latched hull) and everything that matches or merges instances are unchanged. Rule 3: published only once enough latched cells carry a finite height. Identity:
// IDs are monotonic; a merge keeps the better-established instance's ID (rule 4); sprayed instances never
// change (rule 5). Every drop is counted by reason (rule 7).
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "treemapper/frame.hpp"
#include "treemapper/geometry.hpp"
#include "treemapper/mapper.hpp"
#include "treemapper/simple_mapper.hpp"  // PhasePolicy
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
  double max_frame_noise{2.0};      ///< m, frame dropped when 1.4826 * median per-cell MAD of z exceeds this (noise burst)
  double height_sample_clearance{2.0};  ///< m, a cell closer than this to the drone altitude gives no height
                                        ///< sample (the above-drone cap may have cut the real top: a shoulder would be sampled)
  double cell{0.4};                 ///< m, world lattice cell (shared by the frame CHM and every instance)
  int min_cell_points{2};           ///< fewer valid points -> cell unknown
  int max_grid{160};                ///< cap of the per-frame CHM window, cells per side
  // --- point and frame weighting (M4, from cand_grid): enters the height fusion and the evidence
  double z_ref{15.0};               ///< m, range of unit point weight (weight ~ (z_ref/Z)^4: sigma_Z ~ Z^2)
  double lateral_k{10.0};           ///< point weight /= 1 + k*rho^2 (rho = off-axis tangent; lateral error ~ rho*sigma_Z)
  double rot_rate_ref{1.0};         ///< rad/s; frame quality = 1/(1+(rate/ref)^2) (turn blur / noise bursts)
  double quality_recover_s{1.5};    ///< s, frame quality recovers linearly to 1 after a fast rotation
  // --- ground plane (per frame, from the lowest cells)
  double ground_low_fraction{0.4};  ///< fit the plane to this lowest fraction of the cells
  double ground_inlier_band{0.6};   ///< m, refit with cells within +- band of the first plane
  double ground_max_slope{0.3};     ///< above this |slope| the fit is rejected (flat fallback)
  double min_above_ground{1.0};     ///< m, a smoothed cell this far above the plane is canopy
  // --- crowns (watershed basins)
  double min_prominence{0.6};       ///< m, a peak this little above its saddle is merged into the neighbour
  int min_basin_cells{4};           ///< smaller crowns are ignored
  double box_dilate{0.15};          ///< boxes grown by this fraction of their size before the peak test
  double peak_border_px{24.0};      ///< px, a crown whose top is this close to the image edge is truncated
                                    ///< (its top may be outside the image): see seed_truncated
  bool hull_cell_corners{false};    ///< outline from latched cell corners (true) or cell centres (false)
  // --- instances
  bool seed_truncated{true};        ///< a crown whose top is still outside the image may seed an instance (its
                                    ///< cells start latching early). Whatever this says, an instance is confirmed
                                    ///< only once min_height_cells latched cells carry a height sample from a
                                    ///< NON-truncated crown: a tree seen only with its top outside the image never
                                    ///< publishes (its height would be under-estimated: the unsafe direction)
  int window{28};                   ///< cells per side of an instance's evidence window (anchored at birth)
  double match_radius{2.0};         ///< m, a peak this close to an instance centre is that instance
  double match_overlap{0.5};        ///< a crown covering this fraction of an instance's latched cells is that instance,
                                    ///< wherever its top is ...
  double match_reach{3.0};          ///< ... provided its top is within this distance of the instance centre
  double seed_overlap{0.25};        ///< a crown with this fraction of its cells on a published tree's latched cells is
                                    ///< never born as a new instance: it is credited to that tree (seed guard)
  double max_cell_radius{4.0};      ///< m, cells farther from the instance centre are not credited (spill guard)
  double latch_rel_h{0.0};          ///< a cell is claimed only if its height above ground >= this x the crown's peak (skirt trim)
  int latch_frames{5};              ///< positive (quality-weighted) frames before a cell latches
  double latch_ratio{0.75};         ///< ... and positive in at least this fraction of the judged frames
  int confirm_frames{3};            ///< frames with a matched crown before an instance may publish
  int min_height_cells{3};          ///< latched cells with a fused height (and as many sampled from non-truncated
                                    ///< crowns) before an instance may publish
  double min_publish_area{3.0};     ///< m^2 of latched cells before an instance may publish
  double min_tree_height{1.5};      ///< m above ground, smaller crowns never seed an instance
  double tentative_timeout{1.5};    ///< s, unconfirmed instances unseen this long are deleted
  // --- height model (M4)
  double cell_weight_cap{30.0};     ///< a cell's fused-height weight saturates here (hovering cannot inflate it)
  double outlier_gate_m{2.0};       ///< m, a frame height this far from an established cell's value is rejected
  double gate_min_weight{3.0};      ///< ... once the cell has this much weight (3 rejections in a row re-initialise it)
  double height_quantile{0.99};     ///< published height = this quantile of the latched cells' fused surface
  double max_height_rate{0.5};      ///< m/s a published height may move (no jumps)
  double max_centre_rate{0.8};      ///< m/s a published centre may move (no jumps; the outline is not delayed)
  // --- centre (D66)
  bool centre_from_support{true};   ///< published centre = centroid of the cells with observation-weighted canopy
                                    ///< support (true) or of the latched cells (false: the M4 centre)
  double centre_support_ratio{0.5};  ///< a cell supports the centre while its observation-weighted canopy evidence is
                                     ///< >= this share of all its weighted evidence and >= latch_frames weighted frames;
                                     ///< fewer than min_height_cells such cells: the latched centroid (counted)
  double centre_max_outside{1.0};   ///< m, the supported centroid is held within this distance of the latched outline
                                    ///< (counted): the navigator accepts a clearance-guard release over the target only
                                    ///< within drone_radius + lateral_margin (1.3 m) of its outline
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
constexpr int kConfigKeyCount = 48;

/// Frame quality from the body rotation rate (rad/s) and the time since the last fast rotation.
/// Unknown rate (< 0) -> 1 (fail open). Pure, tested.
double frameQuality(double rot_rate, double since_fast_s, const Config& c);

/// One crown observation handed to the instance layer (built by the CHM stage, or by tests).
struct Crown {
  std::vector<std::pair<int, int>> cells;  ///< lattice cells (i, j) of the crown
  Point2 peak;                             ///< world xy of the crown top
  double peak_z{0.0};                      ///< world z of the crown top (tests: the height of every crown cell)
  double peak_hag{0.0};                    ///< smoothed height above ground at the top (0 = unknown: no skirt trim)
  bool boxed{true};                        ///< peak lies inside a detection box
  bool truncated{false};                   ///< top too close to the image border (no seed)
};

/// The per-frame view the instance layer needs besides the crowns: which lattice cells were judged.
struct FrameView {
  int i0{0}, j0{0}, w{0}, h{0};      ///< lattice window of the CHM
  std::vector<std::uint8_t> state;  ///< per window cell: 0 unknown, 1 ground, 2 canopy
  std::vector<int> crown;           ///< per window cell: crown index or -1
  std::vector<float> z;             ///< per window cell: median world z (empty = none: no height fusion)
  std::vector<float> hwt;           ///< per window cell: height weight (empty = frame quality everywhere)
  std::vector<float> hag;           ///< per window cell: smoothed height above ground (empty = no skirt trim)
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

  /// Stage 1+2: points -> CHM -> crowns. Exposed for tests. `quality` scales the cells' height weights.
  void extractCrowns(const FrameInput& frame, FrameView* view, std::vector<Crown>* crowns, double quality = 1.0);
  /// Stage 3: crowns -> instances under the phase policy. Exposed for rule tests (a test builds its own
  /// crowns and view). `pose.z` gates height fusion; `pose.xy` drives the post-spray hold; `quality`
  /// (0..1, from the rotation rate) weights this frame's evidence.
  void integrate(double t, MissionPhase phase, TreeId hop_target, const DronePose& pose, const FrameView& view,
                 std::vector<Crown>& crowns, double quality = 1.0);

  State state() const { return state_; }
  const Config& config() const { return cfg_; }
  double cellSize() const { return cfg_.cell; }

 private:
  struct Instance {
    TreeId id{kNoTree};
    int ai{0}, aj{0};                    ///< lattice origin of the evidence window
    std::vector<float> npos, nneg;       ///< quality-weighted positive / negative frames per cell
    std::vector<std::uint8_t> latched;
    std::vector<float> hz, hw;           ///< per cell: fused surface height, its saturating weight
    std::vector<std::uint8_t> hrej;      ///< per cell: consecutive rejected frame heights
    std::vector<std::uint8_t> hgood;     ///< per cell: has an accepted height sample from a non-truncated crown
    std::vector<float> wpos, wneg;       ///< per cell: observation-weighted positive / negative evidence (D66)
    int n_latched{0};
    double lat_sum_x{0.0}, lat_sum_y{0.0};  ///< sum of latched cell centres (world)
    double peak_sum_x{0.0}, peak_sum_y{0.0};
    int n_peaks{0};
    int frames_seen{0};
    double first_seen{0.0}, last_seen{0.0};
    bool confirmed{false};
    bool sprayed{false};
    bool hull_dirty{true};
    Polygon2 hull;
    Point2 centre;                ///< latched centroid, else running peak mean (internal, instantaneous)
    Point2 centre_support;        ///< centroid of the cells with weighted canopy support (D66)
    bool has_centre_support{false};  ///< ... valid (enough supported cells); else the published centre follows `centre`
    Point2 centre_pub;            ///< published centre: rate-limited toward `centre`
    double height_target{0.0};    ///< quantile of the latched cells' fused heights (NaN until enough cells)
    double height_pub{0.0};       ///< published height: rate-limited toward `height_target`
    int n_height_cells{0};        ///< latched cells with a fused height
    int n_good_height_cells{0};   ///< ... of which sampled at least once from a non-truncated crown (S8 guard)
    double last_step_t{0.0};      ///< time of the last rate-limit step
    double height() const { return height_pub; }
  };

  void count(const std::string& k, std::uint64_t n = 1) { counters_[k] += n; }
  void warnOnce(const std::string& key, const std::string& msg);
  int findInstanceForCrown(const Crown& c) const;
  bool insideSprayed(const Point2& p) const;
  void creditCrown(Instance& in, const Crown& c, std::vector<int>& cell_owner, std::vector<std::uint8_t>& cell_trunc,
                   const FrameView& view);
  void updateEvidence(Instance& in, const FrameView& view, const std::vector<int>& cell_owner,
                      const std::vector<std::uint8_t>& cell_trunc, int self_index, double quality, double z_cap);
  void refreshHull(Instance& in);
  void refreshCentreSupport(Instance& in);
  /// The published centre's target: the supported centroid when there is one, else the latched centroid.
  const Point2& centreTarget(const Instance& in) const { return in.has_centre_support ? in.centre_support : in.centre; }
  void refreshHeight(Instance& in);
  void stepPublished(Instance& in, double t);
  void mergeInstances(double t);
  void absorb(Instance& keep, const Instance& lose);
  double latchedArea(const Instance& in) const { return in.n_latched * cfg_.cell * cfg_.cell; }
  bool publishable(const Instance& in) const;
  /// phasePolicy() plus the sticky lock (see cand_obj.cpp): updates list_locked_.
  PhasePolicy policyFor(MissionPhase phase, TreeId target);

  Config cfg_;
  StereoCalibration cal_;
  std::vector<Instance> instances_;  ///< live instances (tentative + confirmed), in creation order
  TreeId next_id_{0};
  State state_{State::kAwake};
  bool hold_after_spray_{false};
  bool list_locked_{false};   ///< the list was locked (PLANNING/HOPPING/SPRAYING) and no SCAN followed yet
  bool sticky_frame_{false};  ///< the current integrate() runs under the sticky lock
  bool hover_known_{false};
  double hover_x_{0.0}, hover_y_{0.0};
  MissionPhase prev_phase_{MissionPhase::kUnknown};
  // pose history for the rotation-rate frame quality
  bool have_prev_{false};
  double prev_stamp_{0.0}, last_fast_t_{-1e9};
  double pqx_{0}, pqy_{0}, pqz_{0}, pqw_{1};
  std::map<std::string, std::uint64_t> counters_;
  std::map<std::string, double> stage_ms_;
  int stage_frames_{0};
  std::map<std::string, bool> warned_;
  std::function<void(const std::string&)> log_;
  // scratch buffers (per frame)
  std::vector<double> px_, py_, pz_, pu_, pv_, pw_;
  std::vector<char> pok_;
  std::vector<float> hbuf_;
};

}  // namespace cand_obj
}  // namespace treemapper
