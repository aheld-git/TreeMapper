// SimpleMapper: the deliberately simple baseline (milestone M2). Honest, not tuned.
//
// Per frame: points from the central part of each detection box (disparity -> world) -> gates
// (range, above drone, ground) -> one observation per box (convex hull, centroid, canopy-top height)
// -> nearest-entry association -> grow-only convex-hull fusion -> optional containment merge.
// Mission-phase rules (brief sections 6/7) live in the pure function phasePolicy() so they can be
// tested on their own. Every dropped point / observation / frame is counted by reason (rule 7).
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

struct SimpleMapperConfig {
  double box_shrink{0.7};            ///< keep the central fraction of each box side (corners hit ground/neighbours)
  double min_range{0.5};             ///< m, optical depth gate
  double max_range{30.0};            ///< m
  double above_drone_margin{0.3};    ///< drop points higher than drone z - margin
  double ground_percentile{5.0};     ///< per-frame ground estimate = this percentile of all point heights
  double min_above_ground{1.0};      ///< canopy points must be this far above the ground estimate
  int min_obs_points{25};            ///< fewer surviving points -> observation dropped
  double height_percentile{95.0};    ///< observation height = this percentile of point z (canopy top, world z)
  double max_obs_area{60.0};         ///< m^2, larger observation hulls are dropped (gross outliers)
  double assoc_max_dist{2.0};        ///< m, centroid distance for association (or centroid inside outline)
  int publish_min_obs{3};            ///< observations before an entry is published
  double unconfirmed_timeout{2.0};   ///< s, unpublished entries not seen for this long are deleted
  double merge_min_overlap{0.3};     ///< merge if smaller entry's centroid is inside the larger one and
                                     ///< intersection / smaller area >= this
};

/// What a phase allows (brief sections 6/7). Pure function: the single place phase rules are decided.
struct PhasePolicy {
  bool awake{true};              ///< false: heavy processing sleeps, heartbeat only
  bool may_create{true};         ///< new entries allowed
  bool may_merge{true};          ///< entries may merge (merges retire IDs navigation may hold)
  bool freeze_non_target{false}; ///< only the hop target may change
  bool warn_unknown{false};      ///< phase information missing/unknown -> scanning behaviour + warning
};
/// Unknown -> scanning (fail open, warn). PLANNING: no creation/merge. HOPPING: additionally only the
/// target may change; without a valid target everything is frozen (fail closed). SPRAYING/IDLE/RETURN:
/// asleep.
PhasePolicy phasePolicy(MissionPhase phase, TreeId hop_target);

/// One fused-ready observation of one detection box.
struct Observation {
  Polygon2 hull;       ///< convex hull of the canopy points (world xy)
  Point2 centroid;
  double height{0.0};  ///< canopy top, world z
  int n_points{0};
};

/// Builds a SimpleMapperConfig from string key/values (keys = field names); unknown keys throw.
SimpleMapperConfig simpleMapperConfigFrom(const MapperConfig& config);

enum class MapperState { kAwake, kSleeping, kHoldingNoPose };

class SimpleMapper : public Mapper {
 public:
  explicit SimpleMapper(const SimpleMapperConfig& cfg = {}, const StereoCalibration& cal = {});

  /// Full per-frame step: phase policy, pose hold, observation extraction, integration.
  void process(const FrameInput& frame) override;
  /// Points -> observations (one per box). Exposed for tests and for other candidates.
  std::vector<Observation> extractObservations(const FrameInput& frame);
  /// Association + fusion + merge under the phase policy. Exposed for rule tests.
  void integrate(double t, MissionPhase phase, TreeId hop_target, const std::vector<Observation>& obs);

  /// Navigation API: published trees only (valid outline + finite height, rule 3).
  std::vector<TreeEntry> publishedTrees() const override;
  bool markSprayed(TreeId id) override;

  MapperState state() const { return state_; }
  const std::map<std::string, std::uint64_t>& counters() const override { return counters_; }
  /// Per-stage processing time (ms) accumulated since the last call; returns (stage -> mean ms/frame).
  std::map<std::string, double> takeStageTiming() override;
  void setLogger(std::function<void(const std::string&)> log) override { log_ = std::move(log); }

 private:
  struct Entry {
    TreeId id{kNoTree};
    Polygon2 outline;
    double height_sum{0.0};
    int n_obs{0};
    bool published{false};
    bool sprayed{false};
    double first_seen{0.0}, last_seen{0.0};
    double height() const { return n_obs > 0 ? height_sum / n_obs : 0.0; }
  };
  void count(const std::string& k, std::uint64_t n = 1) { counters_[k] += n; }
  void warnOnce(const std::string& key, const std::string& msg);

  SimpleMapperConfig cfg_;
  StereoCalibration cal_;
  std::map<TreeId, Entry> entries_;
  TreeId next_id_{0};  ///< monotonic: an ID is never handed out twice (rule 1)
  MapperState state_{MapperState::kAwake};
  std::map<std::string, std::uint64_t> counters_;
  std::map<std::string, double> stage_ms_;
  int stage_frames_{0};
  std::map<std::string, bool> warned_;
  std::function<void(const std::string&)> log_;
};

}  // namespace treemapper
