// corridor_nav: the navigator state machine (spec 4.4). One `step` per control tick turns the latest inputs into a
// setpoint, the nozzle flag, the mission phase for TreeMapper and service requests. Deterministic given its inputs:
// no clock, no ROS. The building blocks (corridors, assignment, ordering, glide, spray, guard) are pure functions.
#pragma once

#include <cstddef>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "corridor_nav/assignment.hpp"
#include "corridor_nav/config.hpp"
#include "corridor_nav/glide.hpp"
#include "corridor_nav/guard.hpp"
#include "corridor_nav/spray.hpp"
#include "corridor_nav/types.hpp"
#include "treemapper/types.hpp"

namespace corridor_nav {

enum class NavState { kIdle, kTakeoff, kTransit, kScanOut, kScanSettle, kLock, kGlide, kDose, kReturn, kLand, kHold,
                      kAbort, kDone };
const char* toString(NavState s);

struct MapSnapshot {
  double stamp{0.0};
  std::vector<treemapper::TreeEntry> trees;
};

/// Answer of /treemapper/set_tree_sprayed for an earlier `request_sprayed`.
struct SprayResult {
  treemapper::TreeId id{treemapper::kNoTree};
  bool ok{false};
};

struct Inputs {
  double t{0.0};                       ///< s, now
  treemapper::DronePose pose;          ///< pose.t = its stamp, pose.valid
  std::optional<MapSnapshot> map;      ///< set on ticks where a new map arrived
  Clearance clearance;                 ///< latest stereo nadir clearance
  bool start_requested{false};
  bool abort_requested{false};
  std::optional<SprayResult> spray_result;
};

struct Outputs {
  Point3 setpoint;                     ///< carrot point, world frame
  double yaw{0.0};                     ///< rad
  bool nozzle{false};
  std::string phase;                   ///< /mission_state (spec 4.4)
  treemapper::TreeId target{treemapper::kNoTree};   ///< /spray_target, kNoTree = -1
  std::optional<treemapper::TreeId> request_sprayed; ///< call set_tree_sprayed(id) once
};

struct Status {
  NavState state{NavState::kIdle};
  bool cleanup{false};                 ///< gliding to / dosing a tree of the cleanup list
  int corridor{-1};                    ///< -1 = none
  int corridors_total{0};
  treemapper::TreeId target{treemapper::kNoTree};
  unsigned sprayed{0}, skipped{0}, remaining{0};   ///< remaining: in locked corridors + cleanup list
  double clearance_m{NAN};             ///< NaN when invalid
  std::vector<std::string> warnings;
};

/// The status state string: "CLEANUP" while gliding to / dosing a cleanup tree, else toString(state).
const char* statusName(const Status& s);

class Navigator {
 public:
  /// `config` is sanitized (clamps reported through `log`); `log` may be empty.
  Navigator(Config config, Field field, CameraFov fov, std::function<void(const std::string&)> log = {});

  Outputs step(const Inputs& in);

  NavState state() const { return state_; }
  Status status() const;
  const Counters& counters() const { return counters_; }
  const std::vector<Corridor>& corridors() const { return corridors_; }
  /// The path being flown (if any) followed by the planned glides between the remaining targets, for RViz.
  std::vector<Path3> plannedPaths() const;

 private:
  struct ScanLeg {
    int corridor{0};
    bool reversed{false};   ///< scanned exit -> entry (field_first's odd corridors)
  };
  struct Target {
    treemapper::TreeId id{treemapper::kNoTree};
    int corridor{-1};
  };

  // --- per-tick plumbing
  void ingest(const Inputs& in);
  Outputs finish(NavState before, Outputs out);   ///< the single exit of step(): nozzle gate, phase, target, yaw
  void enter(NavState s, double t);
  std::string phase() const;

  // --- one handler per state; each fills the setpoint and may change state
  void stepIdle(const Inputs& in, Outputs& out);
  void stepTakeoff(const Inputs& in, Outputs& out);
  void stepTransit(const Inputs& in, Outputs& out);
  void stepScanOut(const Inputs& in, Outputs& out);
  void stepScanSettle(const Inputs& in, Outputs& out);
  void stepLock(const Inputs& in, Outputs& out);
  void stepGlide(const Inputs& in, Outputs& out);
  void stepDose(const Inputs& in, Outputs& out);
  void stepReturn(const Inputs& in, Outputs& out);
  void stepLand(const Inputs& in, Outputs& out);

  // --- transitions with their entry actions
  void startTransit(double t);
  void startScan(double t);
  void startSettle(double t);
  void startLock(double t);
  void startGlide(double t);
  void startDose(double t);
  void startReturn(double t);
  void startLand(double t);
  void finishTarget(double t);   ///< the current target is done (sprayed or given up): advance
  void advance(double t);        ///< next target, else next corridor, else cleanup, else RETURN

  // --- helpers
  Point3 scanStart(const ScanLeg& leg) const;
  Point3 scanEnd(const ScanLeg& leg) const;
  Point3 entryAt(int corridor) const;
  Point3 exitAt(int corridor) const;
  void setPath(Path3 path);
  Point3 follow(const Config& cfg);           ///< carrot on the current leg; drops legs whose end is reached
  bool atPathEnd(bool need_slow) const;       ///< on the last leg and at its end (and slow, if asked)
  bool reached(const Point3& p) const;
  const treemapper::TreeEntry* tree(treemapper::TreeId id) const;
  std::set<treemapper::TreeId> treesInCorridor(int k) const;
  std::vector<Target> orderStops(const std::vector<treemapper::TreeId>& ids, const Point3& start, const Point3& end,
                                 int corridor) const;
  /// Climb vertically to scan altitude (if below it), then fly level to above `to`.
  std::vector<Point3> transitPoints(const Point3& from, const Point3& to) const;
  std::map<treemapper::TreeId, double> heightsNear(const Path3& path, treemapper::TreeId target) const;

  Config cfg_, scan_cfg_;
  Field field_;
  CameraFov fov_;
  std::function<void(const std::string&)> log_;

  NavState state_{NavState::kIdle};
  double state_t_{0.0};
  Counters counters_;
  std::vector<std::string> warnings_;
  std::string last_phase_{"IDLE"};

  // vehicle
  Point3 pos_;
  bool have_pose_{false};
  double pose_t_{0.0};
  double speed_{0.0};
  double yaw_{0.0};
  double clearance_m_{NAN};
  Point3 home_;            ///< field home xy at the take-off z
  double z_scan_abs_{0.0}; ///< home z + z_scan
  Point3 hold_;            ///< hover point of the hovering states

  // map
  std::vector<treemapper::TreeEntry> map_;
  std::set<treemapper::TreeId> dropped_invalid_;

  // mission
  std::vector<Corridor> corridors_;
  std::vector<ScanLeg> scan_;
  std::size_t scan_idx_{0};
  int corridor_{-1};
  int locked_upto_{-1};
  Assignment assignment_;
  std::set<treemapper::TreeId> settle_ids_;
  double settle_gain_t_{0.0};

  std::vector<Target> targets_;
  std::size_t next_target_{0};   ///< index into targets_ of the next target to pick
  bool cleanup_mode_{false};
  treemapper::TreeId target_{treemapper::kNoTree};
  treemapper::TreeId from_id_{treemapper::kNoTree};
  double target_area_{0.0};
  Point3 spray_pt_;
  std::map<treemapper::TreeId, double> planned_heights_;   ///< for the re-plan trigger (needsReplan)
  std::set<treemapper::TreeId> attempted_;                 ///< sprayed or skipped by this mission
  unsigned sprayed_{0}, skipped_{0};

  DoseTracker dose_;
  bool awaiting_mark_{false};

  // path being flown
  Path3 path_;
  std::size_t leg_{0};
};

}  // namespace corridor_nav
