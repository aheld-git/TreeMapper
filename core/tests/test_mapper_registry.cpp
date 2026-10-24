// Interface + registry tests: every registered system is exercised through the Mapper interface only,
// so a new candidate gets the same smoke coverage for free.
#include <gtest/gtest.h>

#include <algorithm>

#include "treemapper/mapper_registry.hpp"
#include "treemapper/simple_mapper.hpp"

using namespace treemapper;

TEST(MapperRegistry, SimpleIsRegisteredAndListSorted) {
  const auto names = registeredMappers();
  EXPECT_NE(std::find(names.begin(), names.end(), "simple"), names.end());
  EXPECT_TRUE(std::is_sorted(names.begin(), names.end()));
}

TEST(MapperRegistry, UnknownNameThrowsAndListsRegisteredNames) {
  try {
    createMapper("no_such_system", {}, StereoCalibration{});
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("no_such_system"), std::string::npos);
    EXPECT_NE(std::string(e.what()).find("simple"), std::string::npos);
  }
}

TEST(MapperRegistry, EverySystemSurvivesHeartbeatsAndEmptyFramesThroughTheInterface) {
  for (const auto& name : registeredMappers()) {
    SCOPED_TRACE(name);
    auto m = createMapper(name, {}, StereoCalibration{});
    ASSERT_TRUE(m);
    FrameInput f;  // no boxes, no disparity, invalid pose, unknown phase: a heartbeat
    for (int i = 0; i < 3; ++i) {
      f.t = f.stamp = i * 0.1;
      m->process(f);
    }
    EXPECT_TRUE(m->publishedTrees().empty());
    EXPECT_FALSE(m->markSprayed(12345));  // unknown ID
    (void)m->takeStageTiming();
    (void)m->counters();
  }
}

TEST(MapperRegistry, EverySystemRejectsAnUnknownConfigKey) {
  for (const auto& name : registeredMappers()) {
    SCOPED_TRACE(name);
    EXPECT_THROW(createMapper(name, {{"definitely_not_a_key", "1"}}, StereoCalibration{}), std::invalid_argument);
  }
}

TEST(MapperConfig, ParsesKeyValueText) {
  const auto c = parseMapperConfig("a=1;b=x y;;c=");
  EXPECT_EQ(c.size(), 3u);
  EXPECT_EQ(c.at("a"), "1");
  EXPECT_EQ(c.at("b"), "x y");
  EXPECT_EQ(c.at("c"), "");
  EXPECT_TRUE(parseMapperConfig("").empty());
  EXPECT_THROW(parseMapperConfig("novalue"), std::invalid_argument);
  EXPECT_THROW(parseMapperConfig("=3"), std::invalid_argument);
}

TEST(MapperConfig, ConfigReaderTypesAndLeftovers) {
  MapperConfig c{{"d", "2.5"}, {"i", "7"}, {"b", "true"}, {"s", "hello"}, {"extra", "1"}};
  ConfigReader r(c);
  EXPECT_DOUBLE_EQ(r.getDouble("d", 0.0), 2.5);
  EXPECT_EQ(r.getInt("i", 0), 7);
  EXPECT_TRUE(r.getBool("b", false));
  EXPECT_EQ(r.getString("s", ""), "hello");
  EXPECT_EQ(r.getInt("missing", 4), 4);
  EXPECT_THROW(r.finish("x"), std::invalid_argument);  // "extra" was never read
  MapperConfig bad{{"d", "2.5kg"}};
  ConfigReader rb(bad);
  EXPECT_THROW(rb.getDouble("d", 0.0), std::invalid_argument);
}

TEST(SimpleMapperConfig, FromStringsOverridesOnlyTheNamedFields) {
  const SimpleMapperConfig def;
  const auto c = simpleMapperConfigFrom({{"min_obs_points", "40"}, {"box_shrink", "0.5"}});
  EXPECT_EQ(c.min_obs_points, 40);
  EXPECT_DOUBLE_EQ(c.box_shrink, 0.5);
  EXPECT_DOUBLE_EQ(c.max_range, def.max_range);
  EXPECT_EQ(c.publish_min_obs, def.publish_min_obs);
  const auto d = simpleMapperConfigFrom({});
  EXPECT_EQ(d.min_obs_points, def.min_obs_points);
  EXPECT_DOUBLE_EQ(d.merge_min_overlap, def.merge_min_overlap);
}
