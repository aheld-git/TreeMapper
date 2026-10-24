// The published tree map: query and mark-as-sprayed (PROJECT_BRIEF section 4).
// M1 skeleton: storage and the navigation-facing API only. Fusion, gating and
// publication policy are added by later milestones behind this interface.
#pragma once

#include <map>
#include <optional>
#include <vector>

#include "treemapper/types.hpp"

namespace treemapper {

class TreeMap {
 public:
  /// All published trees, ordered by ID.
  std::vector<TreeEntry> allTrees() const;
  /// One tree by ID, or nullopt if unknown.
  std::optional<TreeEntry> findTree(TreeId id) const;
  /// Navigation marks a tree as sprayed. Returns false if the ID is unknown.
  bool markSprayed(TreeId id, bool sprayed = true);
  std::size_t size() const { return trees_.size(); }

  /// Insert or replace an entry (used by the fusion stage; not part of the navigation API).
  /// Rejects entries that would violate the contract: no ID, < 3 vertices or a non-finite height.
  bool upsert(const TreeEntry& entry);

 private:
  std::map<TreeId, TreeEntry> trees_;
};

/// Contract check shared by every producer: valid ID, >= 3 finite vertices, finite centre and height.
bool isPublishable(const TreeEntry& e);

}  // namespace treemapper
