#include "treemapper/types.hpp"

namespace treemapper {

MissionPhase parseMissionPhase(const std::string& s) {
  if (s == "IDLE") return MissionPhase::kIdle;
  if (s == "SCAN_APPROACH") return MissionPhase::kScanApproach;
  if (s == "SCAN") return MissionPhase::kScan;
  if (s == "PLANNING") return MissionPhase::kPlanning;
  if (s == "HOPPING") return MissionPhase::kHopping;
  if (s == "SPRAYING") return MissionPhase::kSpraying;
  if (s == "RETURN") return MissionPhase::kReturn;
  if (s == "ERROR") return MissionPhase::kError;
  return MissionPhase::kUnknown;
}

const char* toString(MissionPhase p) {
  switch (p) {
    case MissionPhase::kIdle: return "IDLE";
    case MissionPhase::kScanApproach: return "SCAN_APPROACH";
    case MissionPhase::kScan: return "SCAN";
    case MissionPhase::kPlanning: return "PLANNING";
    case MissionPhase::kHopping: return "HOPPING";
    case MissionPhase::kSpraying: return "SPRAYING";
    case MissionPhase::kReturn: return "RETURN";
    case MissionPhase::kError: return "ERROR";
    case MissionPhase::kUnknown: break;
  }
  return "UNKNOWN";
}

}  // namespace treemapper
