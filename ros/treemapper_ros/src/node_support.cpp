#include "treemapper_ros/node_support.hpp"

#include <fstream>
#include <stdexcept>

namespace treemapper_ros {

using treemapper::MapperConfig;
using treemapper::MissionPhase;
using treemapper::TreeEntry;
using treemapper::TreeId;

namespace {
std::string trim(const std::string& s) {
  const auto a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
  return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

bool sameEntry(const TreeEntry& a, const TreeEntry& b) {
  if (a.outline.size() != b.outline.size() || a.sprayed != b.sprayed || a.height != b.height ||
      a.centre.x != b.centre.x || a.centre.y != b.centre.y) {
    return false;
  }
  for (std::size_t k = 0; k < a.outline.size(); ++k) {
    if (a.outline[k].x != b.outline[k].x || a.outline[k].y != b.outline[k].y) return false;
  }
  return true;
}
}  // namespace

MapperConfig readCfgFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open mapper config file '" + path + "'");
  MapperConfig c;
  std::string line;
  int n = 0;
  while (std::getline(in, line)) {
    ++n;
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    line = trim(line);
    if (line.empty()) continue;
    const auto eq = line.find('=');
    const std::string key = eq == std::string::npos ? std::string() : trim(line.substr(0, eq));
    if (key.empty()) {
      throw std::invalid_argument(path + ":" + std::to_string(n) + ": expected 'key = value', got '" + line + "'");
    }
    if (c.count(key)) throw std::invalid_argument(path + ":" + std::to_string(n) + ": duplicate key '" + key + "'");
    c[key] = trim(line.substr(eq + 1));
  }
  return c;
}

MapperConfig applyOverrides(MapperConfig base, const std::map<std::string, std::string>& overrides) {
  for (const auto& kv : overrides) base[kv.first] = kv.second;
  return base;
}

bool isSleepPhase(MissionPhase p) {
  return p == MissionPhase::kIdle || p == MissionPhase::kScanApproach || p == MissionPhase::kSpraying ||
         p == MissionPhase::kReturn;
}

bool isLockedPhase(MissionPhase p) {
  return p == MissionPhase::kPlanning || p == MissionPhase::kHopping || p == MissionPhase::kSpraying;
}

std::vector<TreeId> TreeMetaTracker::update(double t, MissionPhase phase, const std::vector<TreeEntry>& trees) {
  std::vector<TreeId> changed;
  std::set<TreeId> now;
  for (const auto& e : trees) {
    now.insert(e.id);
    auto it = last_.find(e.id);
    TreeMeta& m = meta_[e.id];
    if (it == last_.end()) {
      m.first_published = t;
      m.last_updated = t;
      changed.push_back(e.id);
      vanished_.erase(e.id);
    } else if (!sameEntry(it->second, e)) {
      m.last_updated = t;
      changed.push_back(e.id);
    }
    if (isLockedPhase(phase)) m.locked = true;
    last_[e.id] = e;
  }
  for (auto it = last_.begin(); it != last_.end();) {
    if (!now.count(it->first)) {
      vanished_.insert(it->first);
      it = last_.erase(it);
    } else {
      ++it;
    }
  }
  return changed;
}

const TreeMeta* TreeMetaTracker::find(TreeId id) const {
  auto it = meta_.find(id);
  return it == meta_.end() ? nullptr : &it->second;
}

}  // namespace treemapper_ros
