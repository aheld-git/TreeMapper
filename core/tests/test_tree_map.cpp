#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "treemapper/tree_map.hpp"

using namespace treemapper;

namespace {
TreeEntry square(TreeId id, double cx, double cy, double h = 4.0) {
  TreeEntry e;
  e.id = id;
  e.outline = {{cx - 1, cy - 1}, {cx + 1, cy - 1}, {cx + 1, cy + 1}, {cx - 1, cy + 1}};
  e.centre = {cx, cy};
  e.height = h;
  return e;
}
}  // namespace

TEST(TreeMap, QueryAllAndById) {
  TreeMap m;
  ASSERT_TRUE(m.upsert(square(3, 0, 0)));
  ASSERT_TRUE(m.upsert(square(1, 6, 0)));
  auto all = m.allTrees();
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[0].id, 1);
  EXPECT_EQ(all[1].id, 3);
  ASSERT_TRUE(m.findTree(3).has_value());
  EXPECT_FALSE(m.findTree(7).has_value());
}

TEST(TreeMap, MarkSprayed) {
  TreeMap m;
  m.upsert(square(1, 0, 0));
  EXPECT_TRUE(m.markSprayed(1));
  EXPECT_TRUE(m.findTree(1)->sprayed);
  EXPECT_FALSE(m.markSprayed(42));
}

TEST(TreeMap, RejectsUndefinedHeightAndDegenerateOutline) {
  TreeMap m;
  EXPECT_FALSE(m.upsert(square(1, 0, 0, std::numeric_limits<double>::quiet_NaN())));
  EXPECT_FALSE(m.upsert(square(1, 0, 0, std::numeric_limits<double>::infinity())));
  TreeEntry e = square(2, 0, 0);
  e.outline.resize(2);
  EXPECT_FALSE(m.upsert(e));
  EXPECT_FALSE(m.upsert(square(-1, 0, 0)));
  EXPECT_EQ(m.size(), 0u);
}

TEST(MissionPhase, ParsesNavigatorStringsAndFailsToUnknown) {
  EXPECT_EQ(parseMissionPhase("SCAN"), MissionPhase::kScan);
  EXPECT_EQ(parseMissionPhase("HOPPING"), MissionPhase::kHopping);
  EXPECT_EQ(parseMissionPhase("SCAN_APPROACH"), MissionPhase::kScanApproach);
  EXPECT_EQ(parseMissionPhase("TAKEOFF"), MissionPhase::kUnknown);
  EXPECT_EQ(parseMissionPhase(""), MissionPhase::kUnknown);
  EXPECT_STREQ(toString(MissionPhase::kSpraying), "SPRAYING");
  EXPECT_STREQ(toString(MissionPhase::kUnknown), "UNKNOWN");
}
