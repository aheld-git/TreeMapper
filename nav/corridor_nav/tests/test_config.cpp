#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

#include "corridor_nav/config.hpp"

using namespace corridor_nav;
using treemapper::MapperConfig;

namespace {

std::string repoRoot() {
  std::string p = __FILE__;  // <root>/nav/corridor_nav/tests/test_config.cpp
  for (int k = 0; k < 4; ++k) p = p.substr(0, p.find_last_of('/'));
  return p;
}

MapperConfig readCfgFile(const std::string& path) {
  std::ifstream in(path);
  MapperConfig c;
  std::string line;
  while (std::getline(in, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    const auto eq = line.find('=');
    if (eq == std::string::npos) continue;
    auto trim = [](std::string s) {
      const auto a = s.find_first_not_of(" \t\r"), b = s.find_last_not_of(" \t\r");
      return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
    };
    c[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
  }
  return c;
}

}  // namespace

TEST(NavConfig, ShippedFileEqualsBuiltInDefaults) {
  const std::string path = repoRoot() + "/config/corridor_nav.cfg";
  const MapperConfig file = readCfgFile(path);
  ASSERT_FALSE(file.empty()) << "cannot read " << path;
  int n_fields = 2;  // strategy and dose_mode are enums, checked below
#define COUNT_FIELD(f) ++n_fields;
  CORRIDOR_NAV_FIELDS(COUNT_FIELD)
#undef COUNT_FIELD
  EXPECT_EQ(n_fields, kConfigKeyCount);
  EXPECT_EQ(static_cast<int>(file.size()), kConfigKeyCount) << "every tunable must be in the config file";
  const Config def;
  const Config from_file = configFrom(file);  // throws on an unknown key
#define CHECK_FIELD(f) EXPECT_DOUBLE_EQ(static_cast<double>(from_file.f), static_cast<double>(def.f)) << #f;
  CORRIDOR_NAV_FIELDS(CHECK_FIELD)
#undef CHECK_FIELD
  EXPECT_EQ(from_file.strategy, Strategy::kCorridor);
  EXPECT_EQ(from_file.dose_mode, DoseMode::kArea);
  EXPECT_EQ(def.strategy, Strategy::kCorridor);
  EXPECT_EQ(def.dose_mode, DoseMode::kArea);

  EXPECT_THROW(configFrom({{"not_a_key", "1"}}), std::invalid_argument);
  EXPECT_THROW(configFrom({{"z_scan", "nan"}}), std::invalid_argument);
  EXPECT_THROW(configFrom({{"strategy", "zigzag"}}), std::invalid_argument);
  EXPECT_THROW(configFrom({{"dose_mode", "lots"}}), std::invalid_argument);
}

TEST(NavConfig, EnumsParseFromStrings) {
  const Config c = configFrom({{"strategy", "field_first"}, {"dose_mode", "fixed"}, {"service_retries", "5"}});
  EXPECT_EQ(c.strategy, Strategy::kFieldFirst);
  EXPECT_EQ(c.dose_mode, DoseMode::kFixed);
  EXPECT_EQ(c.service_retries, 5);
}

TEST(NavConfig, SanitizedClampsAndLogsEveryClamp) {
  Config c;
  c.xy_tol = 0.0;           // below the range (> 0)
  c.dose_min_s = 7.0;       // above dose_max_s (6): the max is raised
  c.spray_clearance = 1.0;  // below min_clearance (1.5): raised
  int logged = 0;
  const Config s = c.sanitized([&](const std::string& m) { ++logged; EXPECT_NE(m.find("clamped"), std::string::npos); });
  EXPECT_EQ(logged, 3);
  EXPECT_GT(s.xy_tol, 0.0);
  EXPECT_GE(s.dose_max_s, 7.0);
  EXPECT_DOUBLE_EQ(s.dose_min_s, 7.0);
  EXPECT_DOUBLE_EQ(s.spray_clearance, 1.5);

  int none = 0;
  Config{}.sanitized([&](const std::string&) { ++none; });
  EXPECT_EQ(none, 0) << "defaults are in range";
  EXPECT_EQ(Config{}.sanitized(nullptr).service_retries, 3);
}

TEST(NavConfig, CorridorWidthIsZeroOrAtLeastTheMinimum) {
  // 0 = automatic; a fixed width below corridor_width_min is raised to it (logged "clamped"); 0 and >= min stay
  std::vector<std::string> logs;
  const auto log = [&](const std::string& m) { logs.push_back(m); };
  Config c;
  c.corridor_width_min = 3.0;
  c.corridor_width = 1.0;
  Config s = c.sanitized(log);
  EXPECT_DOUBLE_EQ(s.corridor_width, 3.0);
  ASSERT_EQ(logs.size(), 1u);
  EXPECT_NE(logs[0].find("corridor_width"), std::string::npos);
  EXPECT_NE(logs[0].find("clamped"), std::string::npos);

  logs.clear();
  c.corridor_width = 0.0;
  EXPECT_DOUBLE_EQ(c.sanitized(log).corridor_width, 0.0);
  c.corridor_width = 3.0;
  EXPECT_DOUBLE_EQ(c.sanitized(log).corridor_width, 3.0);
  c.corridor_width = 7.5;
  EXPECT_DOUBLE_EQ(c.sanitized(log).corridor_width, 7.5);
  EXPECT_TRUE(logs.empty());

  // the minimum it is compared with is the sanitized one
  c.corridor_width_min = 0.0;   // clamped up to the smallest length
  c.corridor_width = 1e-4;
  logs.clear();
  s = c.sanitized(log);
  EXPECT_GE(s.corridor_width, s.corridor_width_min);
  EXPECT_GT(s.corridor_width, 0.0);
}
