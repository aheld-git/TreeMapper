// ROS-free helpers of treemapper_node: the mapper config file, phase classification for health, and the
// per-tree diagnostics the Mapper interface does not carry (first published, last changed, locked).
#pragma once

#include <map>
#include <set>
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

/// Phases in which heavy processing may sleep (brief section 6): the detector is dark, so the node
/// reports SLEEPING (alive, deliberately idle) instead of looking dead.
bool isSleepPhase(treemapper::MissionPhase p);
/// Phases in which the tree list is fixed (no new standalone trees): PLANNING, HOPPING, SPRAYING.
bool isLockedPhase(treemapper::MissionPhase p);

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
