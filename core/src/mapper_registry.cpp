#include "treemapper/mapper_registry.hpp"

#include <algorithm>
#include <map>

namespace treemapper {

// ---------------------------------------------------------------------------------------------------
// REGISTRATION. One line per system, in registry() below. A registered name "x" expects the system to
// define, in namespace treemapper, a function
//     std::unique_ptr<Mapper> tm_create_x(const MapperConfig&, const StereoCalibration&);
//
//  * TM_REGISTER_CANDIDATE(x): for a candidate under core/src/candidates/x/. The declaration is weak:
//    if the candidate is switched off with -DTM_CANDIDATES=... (see CMakeLists.txt) its line registers
//    nothing, so one broken candidate never blocks the build of another. (Candidate objects are linked
//    in directly, not through the static library, which is what makes the weak reference resolve.)
//  * TM_REGISTER_BUILTIN(x): for systems inside treemapper_core itself (only "simple"): a normal
//    reference, which also makes the linker pull the system out of the static library.
// ---------------------------------------------------------------------------------------------------
#define TM_REGISTER_BUILTIN(x)                                                                  \
  std::unique_ptr<Mapper> tm_create_##x(const MapperConfig&, const StereoCalibration&);         \
  r[#x] = &tm_create_##x;
#define TM_REGISTER_CANDIDATE(x)                                                                \
  std::unique_ptr<Mapper> tm_create_##x(const MapperConfig&, const StereoCalibration&)          \
      __attribute__((weak));                                                                    \
  if (&tm_create_##x != nullptr) r[#x] = &tm_create_##x;

// static, not in an anonymous namespace: the macros declare tm_create_x in the enclosing namespace,
// which must be treemapper itself.
static const std::map<std::string, MapperFactory>& registry() {
  static const std::map<std::string, MapperFactory> table = [] {
    std::map<std::string, MapperFactory> r;
    // --- registered systems (add yours below, keep alphabetical) ---------------------------------
    TM_REGISTER_BUILTIN(simple)
    TM_REGISTER_CANDIDATE(cand_grid)
    TM_REGISTER_CANDIDATE(cand_obj)
    // ---------------------------------------------------------------------------------------------
    return r;
  }();
  return table;
}

std::vector<std::string> registeredMappers() {
  std::vector<std::string> names;
  for (const auto& kv : registry()) names.push_back(kv.first);
  return names;  // std::map iterates sorted
}

std::unique_ptr<Mapper> createMapper(const std::string& name, const MapperConfig& config,
                                     const StereoCalibration& calibration) {
  auto it = registry().find(name);
  if (it == registry().end()) {
    std::string known;
    for (const auto& n : registeredMappers()) known += (known.empty() ? "" : ", ") + n;
    throw std::invalid_argument("unknown system '" + name + "' (registered: " + known + ")");
  }
  return it->second(config, calibration);
}

MapperConfig parseMapperConfig(const std::string& text) {
  MapperConfig out;
  std::size_t pos = 0;
  while (pos < text.size()) {
    std::size_t end = text.find(';', pos);
    if (end == std::string::npos) end = text.size();
    const std::string pair = text.substr(pos, end - pos);
    pos = end + 1;
    if (pair.empty()) continue;
    const std::size_t eq = pair.find('=');
    if (eq == std::string::npos || eq == 0) {
      throw std::invalid_argument("bad config entry '" + pair + "' (expected key=value)");
    }
    out[pair.substr(0, eq)] = pair.substr(eq + 1);
  }
  return out;
}

}  // namespace treemapper
