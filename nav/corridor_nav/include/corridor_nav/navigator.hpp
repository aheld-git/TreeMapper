// corridor_nav: the navigator state machine (spec 4.4). One `step` per control tick turns the latest inputs into a
// setpoint, the nozzle flag, the mission phase for TreeMapper and service requests. Deterministic given its inputs:
// no clock, no ROS. The building blocks (corridors, assignment, ordering, glide, spray, guard) are pure functions.
#pragma once

#include <cmath>
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

/// m above (published height + spray_clearance): the highest spray floor a guard release may set. A safety cap on what a
/// guard release can drag the dose up to, not a tuning knob, so it is not a Config key.
constexpr double kSprayFloorCapM = 2.0;

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
    treemapper::Point2 centre;   ///< where the tree was when it was planned, for re-resolving a vanished id
  };
  /// set_tree_sprayed of the tree just dosed: one re-resolve on ok = false, `service_retries` retries on silence.
  struct Mark {
    bool active{false}, done{false}, reresolved{false};
    treemapper::TreeId id{treemapper::kNoTree};         ///< the id asked for now
    treemapper::TreeId first_id{treemapper::kNoTree};   ///< the id the tree was dosed under
    treemapper::Point2 centre;
    double sent_t{0.0};
    int tries{0};
  };

  // --- per-tick plumbing
  void ingest(const Inputs& in);
  /// The single exit of step(): nozzle gate, dose tracking of the published nozzle, phase, target, yaw.
  Outputs finish(NavState before, double t, Outputs out);
  void enter(NavState s, double t);
  std::string phase() const;
  bool onTarget() const;   ///< GLIDE / DOSE, or a HOLD entered from one of them

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
  void stepHold(const Inputs& in, Outputs& out);
  void stepAbort(const Inputs& in, Outputs& out);

  // --- transitions with their entry actions
  void startTransit(double t);
  void startScan(double t);
  void startSettle(double t);
  void startLock(double t);
  void startGlide(double t);
  void startDose(double t);
  void startReturn(double t);
  void startLand(double t);
  void startHold(const std::string& reason, double t);
  void startAbort(const std::string& why, double t);
  void finishTarget(double t);   ///< the current target is done (sprayed or its mark given up): advance
  void failAttempt(double t, const std::string& why);   ///< invalid dose / guard: one retry, then skip
  void advance(double t);        ///< next target, else next corridor, else cleanup, else RETURN

  // --- holds, guard, mark, re-planning
  void updateHold(const Inputs& in);   ///< enters HOLD on a missing input; leaves it (or times out)
  /// Decision + episode bookkeeping, once per GLIDE / DOSE tick; finish() applies it to the final setpoint.
  GuardAction guardUpdate(const Inputs& in);
  void guardApply(GuardAction a, Outputs& out) const;   ///< climb / no-descent on the setpoint
  /// On a guard release in GLIDE / DOSE: spray_floor_ = max(spray_floor_, spray_pt_.z, pos_.z) and spray_pt_ raised
  /// to it (counted spray_point_guard_raised). The floor holds for the target until it is finished, skipped or dropped.
  /// A release only counts while the drone's xy is within the target's outline expanded by drone_radius +
  /// lateral_margin (elsewhere it says nothing about this canopy: spray_floor_release_ignored, floor unchanged), and
  /// the floor is capped at published height + spray_clearance + kSprayFloorCapM. Returns false (counted
  /// spray_floor_cap_exceeded, nothing raised) when the release altitude is above the cap: the caller fails the attempt.
  /// The 2 m cap bounds the guard's contribution only: a spray point the planner already raised over an overlapping
  /// neighbour (spray_point_raised) may sit above it, and is kept.
  bool raiseSprayFloor();
  void pollMark(const Inputs& in, Outputs& out);
  void sendMark(treemapper::TreeId id, double t, Outputs& out);
  void abandonMark();
  bool resolveTarget();        ///< re-resolves a vanished target_; false (counted dropped) if nothing stands for it
  bool keepTarget(double t);   ///< resolveTarget(), and advance() if it was dropped
  treemapper::TreeId reresolve(const treemapper::Point2& centre) const;   ///< kNoTree if none or already done
  void planToTarget(double t, const char* replan_why);   ///< glide from here; counts a re-plan if `replan_why`
  Path3 glideTo(const Point3& from, treemapper::TreeId from_id, const treemapper::TreeEntry& to,
                Counters* counters) const;

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
  /// support::heightPlausible against home; an implausible tree is counted target_height_implausible once per id.
  bool plausible(const treemapper::TreeEntry& t);
  std::set<treemapper::TreeId> treesInCorridor(int k) const;
  std::vector<Target> orderStops(const std::vector<treemapper::TreeId>& ids, const Point3& start, const Point3& end,
                                 int corridor) const;
  /// Climb vertically to scan altitude (if below it), then fly level to above `to`.
  std::vector<Point3> transitPoints(const Point3& from, const Point3& to) const;

  Config cfg_, scan_cfg_;
  Field field_;
  CameraFov fov_;
  std::function<void(const std::string&)> log_;

  NavState state_{NavState::kIdle};
  double state_t_{0.0};
  Counters counters_;
  std::vector<std::string> warnings_;
  std::string last_phase_{"IDLE"};
  treemapper::TreeId last_target_{treemapper::kNoTree};

  // vehicle
  Point3 pos_;
  bool have_pose_{false};
  bool pose_ok_{false};    ///< pose valid this tick and the stored pose's stamp within +-pose_max_age_s of now
  double pose_t_{0.0};
  double speed_{0.0};
  double yaw_{0.0};
  double clearance_m_{NAN};
  bool airborne_{false};   ///< took off: an abort returns home
  Point3 home_;            ///< field home xy at the take-off z
  double z_scan_abs_{0.0}; ///< home z + z_scan
  Point3 hold_;            ///< hover point of the hovering states

  // map
  std::vector<treemapper::TreeEntry> map_;
  double map_t_{-INFINITY};   ///< stamp of the latest map received
  std::set<treemapper::TreeId> dropped_invalid_;

  // HOLD
  NavState held_state_{NavState::kIdle};
  double held_state_t_{0.0}, hold_t_{0.0};
  Point3 hold_pos_;        ///< where the hold began
  bool service_hold_{false};

  // clearance guard
  bool climbing_{false}, blind_{false}, guard_released_{false}, guard_failed_{false};
  bool guard_ran_{false};                       ///< guardUpdate ran on this tick
  GuardAction guard_action_{GuardAction::kNone};   ///< its decision
  double guard_ok_t_{0.0};   ///< last tick the guard was clear
  int attempt_climbs_{0};    ///< climb episodes in this glide + dose attempt

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
  treemapper::Point2 target_centre_;
  double target_area_{0.0};
  Point3 spray_pt_;
  double spray_floor_{-INFINITY};   ///< spray altitude floor of the current target (guard releases); -inf = none
  std::map<treemapper::TreeId, double> planned_heights_;   ///< for the re-plan trigger (needsReplan)
  std::set<treemapper::TreeId> attempted_;                 ///< sprayed, mark given up, or skipped twice
  std::set<treemapper::TreeId> implausible_;               ///< passed over for a published height above the limit
  std::map<treemapper::TreeId, int> skips_;                ///< times each tree was skipped
  bool retried_{false};                                    ///< the current target had its one retry
  unsigned sprayed_{0}, skipped_{0};                       ///< skipped_: trees skipped twice (left unsprayed)

  DoseTracker dose_;
  bool dose_running_{false};   ///< the tracker integrates the published nozzle
  Mark mark_;

  // path being flown
  Path3 path_;
  std::size_t leg_{0};
};

}  // namespace corridor_nav
