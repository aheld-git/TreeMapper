#include "treemapper/tree_map.hpp"

#include <cmath>

namespace treemapper {

bool isPublishable(const TreeEntry& e) {
  if (e.id < 0 || e.outline.size() < 3) return false;
  if (!std::isfinite(e.height) || !std::isfinite(e.centre.x) || !std::isfinite(e.centre.y)) return false;
  for (const auto& p : e.outline) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
  }
  return true;
}

std::vector<TreeEntry> TreeMap::allTrees() const {
  std::vector<TreeEntry> out;
  out.reserve(trees_.size());
  for (const auto& kv : trees_) out.push_back(kv.second);
  return out;
}

std::optional<TreeEntry> TreeMap::findTree(TreeId id) const {
  auto it = trees_.find(id);
  if (it == trees_.end()) return std::nullopt;
  return it->second;
}

bool TreeMap::markSprayed(TreeId id, bool sprayed) {
  auto it = trees_.find(id);
  if (it == trees_.end()) return false;
  it->second.sprayed = sprayed;
  return true;
}

bool TreeMap::upsert(const TreeEntry& entry) {
  if (!isPublishable(entry)) return false;
  trees_[entry.id] = entry;
  return true;
}

}  // namespace treemapper
