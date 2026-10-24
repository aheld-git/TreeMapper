// Mapper: the interface every mapping system implements (SimpleMapper, and the M3 candidates).
//
// This is exactly what a runner needs: feed frames, read the published map, report sprayed trees, read
// timing and drop counters. Runners (treemapper_replay today, the ROS node later) only see this header
// and mapper_registry.hpp, so a new system never changes a runner.
//
// Contract every implementation must keep (PROJECT_BRIEF sections 4-7, rules enforced by the scorer):
//  * publishedTrees() returns only valid entries: outline >= 3 vertices, finite height (rule 3).
//  * IDs are never reused for a different tree (rule 1).
//  * process() applies the mission-phase rules (phasePolicy() in simple_mapper.hpp is a reusable pure
//    function for them).
//  * Every dropped point / observation / frame is counted by reason in counters() (rule 7).
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "treemapper/frame.hpp"
#include "treemapper/types.hpp"

namespace treemapper {

/// Free-form configuration of one system: key -> value, both strings. A system parses what it knows
/// (see ConfigReader) and rejects unknown keys so a typo can never silently run the defaults.
using MapperConfig = std::map<std::string, std::string>;

class Mapper {
 public:
  virtual ~Mapper() = default;

  /// Full per-frame step. Heartbeat frames (no boxes / no disparity) and frames without a valid pose
  /// must be handled; they never throw.
  virtual void process(const FrameInput& frame) = 0;

  /// Navigation API: the published map (valid outline + finite height only).
  virtual std::vector<TreeEntry> publishedTrees() const = 0;

  /// Navigation reports a tree sprayed. False if the ID is unknown. Sprayed trees never change again.
  virtual bool markSprayed(TreeId id) = 0;

  /// Per-stage processing time since the last call, as stage -> mean ms/frame; resets the accumulators.
  /// Written into the map stream as `stage_ms` (the scorer's cost section).
  virtual std::map<std::string, double> takeStageTiming() = 0;

  /// Monotonic drop / event counters by reason (rule 7), written to counters.json by the runner.
  virtual const std::map<std::string, std::uint64_t>& counters() const = 0;

  /// Optional diagnostics sink; the default ignores it.
  virtual void setLogger(std::function<void(const std::string&)> /*log*/) {}
};

/// Reads typed values out of a MapperConfig and remembers which keys were used, so finish() can reject
/// the leftovers. Throws std::invalid_argument with the offending key on a malformed value.
///
///   ConfigReader r(cfg);
///   p.alpha = r.getDouble("alpha", p.alpha);
///   r.finish("cand_a");
class ConfigReader {
 public:
  explicit ConfigReader(const MapperConfig& c) : c_(c) {}

  double getDouble(const std::string& key, double def) {
    const std::string* s = find(key);
    if (!s) return def;
    std::size_t used = 0;
    double v = 0.0;
    try { v = std::stod(*s, &used); } catch (const std::exception&) { used = 0; }
    if (used == 0 || used != s->size()) bad(key, *s, "a number");
    return v;
  }
  int getInt(const std::string& key, int def) {
    const std::string* s = find(key);
    if (!s) return def;
    std::size_t used = 0;
    int v = 0;
    try { v = std::stoi(*s, &used); } catch (const std::exception&) { used = 0; }
    if (used == 0 || used != s->size()) bad(key, *s, "an integer");
    return v;
  }
  bool getBool(const std::string& key, bool def) {
    const std::string* s = find(key);
    if (!s) return def;
    if (*s == "1" || *s == "true") return true;
    if (*s == "0" || *s == "false") return false;
    bad(key, *s, "true/false/1/0");
    return def;
  }
  std::string getString(const std::string& key, const std::string& def) {
    const std::string* s = find(key);
    return s ? *s : def;
  }
  /// Throws std::invalid_argument naming every key that no get*() asked for.
  void finish(const std::string& system) const {
    std::string unknown;
    for (const auto& kv : c_) {
      if (!used_.count(kv.first)) unknown += (unknown.empty() ? "" : ", ") + kv.first;
    }
    if (!unknown.empty()) throw std::invalid_argument("system '" + system + "': unknown config key(s): " + unknown);
  }

 private:
  const std::string* find(const std::string& key) {
    used_[key] = true;
    auto it = c_.find(key);
    return it == c_.end() ? nullptr : &it->second;
  }
  [[noreturn]] static void bad(const std::string& key, const std::string& v, const char* want) {
    throw std::invalid_argument("config key '" + key + "': '" + v + "' is not " + want);
  }
  const MapperConfig& c_;
  std::map<std::string, bool> used_;
};

}  // namespace treemapper
