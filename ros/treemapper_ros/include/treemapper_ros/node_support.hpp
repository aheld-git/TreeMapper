// ROS-free helpers of treemapper_node: the mapper config file, phase classification for health, and the
// per-tree diagnostics the Mapper interface does not carry (first published, last changed, locked).
#pragma once

#include <cstdint>
#include <exception>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "treemapper/mapper.hpp"
#include "treemapper/types.hpp"

namespace treemapper_ros {

/// Reads a core config file (config/<mapper>.cfg): `key = value` per line, '#' starts a comment (also
/// after a value), blank lines ignored. Throws std::runtime_error if the file cannot be opened and
/// std::invalid_argument on a line that is neither blank, a comment nor key = value, or a duplicate key.
treemapper::MapperConfig readCfgFile(const std::string& path);

/// Applies `overrides` on top of `base` (the ROS-side `mapper_overrides` dict). An override of a key the
/// file does not have is still passed on: the mapper's ConfigReader rejects unknown keys loudly.
treemapper::MapperConfig applyOverrides(treemapper::MapperConfig base,
                                        const std::map<std::string, std::string>& overrides);

/// Phases in which the mapper sleeps (brief section 6): exactly the phases phasePolicy() marks not awake
/// (IDLE, SPRAYING, RETURN), so health never says SLEEPING while the mapper is mapping. SCAN_APPROACH is
/// awake (the mapper maps it), so a stale input there is reported, not hidden.
bool isSleepPhase(treemapper::MissionPhase p);
/// Phases in which the tree list is fixed (no new standalone trees): PLANNING, HOPPING, SPRAYING.
bool isLockedPhase(treemapper::MissionPhase p);

/// Thrown for configuration errors that must end the node (bad parameters at startup, a mapper that cannot
/// be built). CallbackGuard lets it through; every other exception is contained.
struct FatalConfigError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/// Keeps the node alive when a callback throws (B1): run() executes a callback body, and an exception
/// (other than FatalConfigError) is logged, counted per callback and remembered, so health can report
/// FAULT for `fault_hold_s` after the latest one. Thread-safe (the pose callback runs on its own spinner).
/// Respawning instead would restart the IDs at 0 (rule 1), see README "Crash containment".
class CallbackGuard {
 public:
  explicit CallbackGuard(double fault_hold_s = 10.0) : hold_s_(fault_hold_s) {}
  /// Returns true when `body` completed. `now` = wall seconds (for the FAULT window); `log` gets one line.
  template <typename F, typename L>
  bool run(const char* where, double now, F&& body, L&& log) {
    try {
      body();
      return true;
    } catch (const FatalConfigError&) {
      throw;
    } catch (const std::exception& e) {
      log(record(where, e.what(), now));
    } catch (...) {
      log(record(where, "unknown exception", now));
    }
    return false;
  }
  /// True while the latest exception is less than fault_hold_s old.
  bool faultActive(double now) const;
  /// "callback exception in <where>: <what> (<n> total)", empty if none ever.
  std::string lastError() const;
  std::uint64_t total() const;
  /// callback_exceptions (total) and callback_exceptions_<where> per callback.
  std::map<std::string, std::uint64_t> counters() const;

 private:
  std::string record(const char* where, const std::string& what, double now);
  double hold_s_;
  mutable std::mutex m_;
  std::map<std::string, std::uint64_t> counts_;
  std::uint64_t total_{0};
  double last_t_{-1e300};
  std::string last_;
};

/// Diagnostics per published tree, kept by the node.
struct TreeMeta {
  double first_published{0.0};
  double last_updated{0.0};
  bool locked{false};
};

class TreeMetaTracker {
 public:
  /// Call with every published map (time `t`, phase in force). Returns the IDs whose outline, centre,
  /// height or sprayed flag changed since the previous call (including new trees).
  std::vector<treemapper::TreeId> update(double t, treemapper::MissionPhase phase,
                                         const std::vector<treemapper::TreeEntry>& trees);
  const TreeMeta* find(treemapper::TreeId id) const;
  /// IDs that were published once and are no longer in the map (merged away). Never reused (rule 1);
  /// kept so a disappearance is visible in diagnostics.
  const std::set<treemapper::TreeId>& vanished() const { return vanished_; }

 private:
  std::map<treemapper::TreeId, TreeMeta> meta_;
  std::map<treemapper::TreeId, treemapper::TreeEntry> last_;
  std::set<treemapper::TreeId> vanished_;
};

}  // namespace treemapper_ros
