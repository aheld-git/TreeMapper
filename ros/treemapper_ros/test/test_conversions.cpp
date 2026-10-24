// Message conversion tests: contract entry <-> TreeEntry message, detections, pose, counters, markers, and the
// per-tree diagnostics tracker.
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "treemapper/simple_mapper.hpp"
#include "treemapper_ros/conversions.hpp"

using namespace treemapper_ros;
using treemapper::MissionPhase;
using treemapper::TreeEntry;

namespace {
TreeEntry square(int id, double cx, double cy, double h, bool sprayed = false) {
  TreeEntry e;
  e.id = id;
  e.outline = {{cx - 1, cy - 1}, {cx + 1, cy - 1}, {cx + 1, cy + 1}, {cx - 1, cy + 1}};
  e.centre = {cx, cy};
  e.height = h;
  e.sprayed = sprayed;
  return e;
}
}  // namespace

TEST(Conversions, TreeEntryRoundTrip) {
  TreeEntry e = square(7, 3.5, -2.25, 4.75, true);
  e.observations = 9;
  TreeMeta meta;
  meta.first_published = 12.5;
  meta.last_updated = 20.25;
  meta.locked = true;
  const auto m = toMsg(e, &meta);
  EXPECT_EQ(m.id, 7);
  ASSERT_EQ(m.outline.size(), 4u);
  EXPECT_DOUBLE_EQ(m.outline[1].x, 4.5);
  EXPECT_DOUBLE_EQ(m.outline[1].z, 0.0);
  EXPECT_DOUBLE_EQ(m.centre.y, -2.25);
  EXPECT_DOUBLE_EQ(m.height, 4.75);
  EXPECT_TRUE(m.sprayed);
  EXPECT_EQ(m.observations, 9u);
  EXPECT_FLOAT_EQ(m.confidence, 0.0f);  // reserved
  EXPECT_TRUE(m.locked);
  EXPECT_DOUBLE_EQ(m.first_published.toSec(), 12.5);
  EXPECT_DOUBLE_EQ(m.last_updated.toSec(), 20.25);
  const TreeEntry back = fromMsg(m);
  EXPECT_EQ(back.id, e.id);
  ASSERT_EQ(back.outline.size(), e.outline.size());
  for (std::size_t k = 0; k < e.outline.size(); ++k) {
    EXPECT_DOUBLE_EQ(back.outline[k].x, e.outline[k].x);
    EXPECT_DOUBLE_EQ(back.outline[k].y, e.outline[k].y);
  }
  EXPECT_DOUBLE_EQ(back.height, e.height);
  EXPECT_EQ(back.sprayed, e.sprayed);
  const auto nometa = toMsg(e, nullptr);
  EXPECT_FALSE(nometa.locked);
}

TEST(Conversions, MapKeepsOrderAndHeader) {
  std_msgs::Header h;
  h.stamp = ros::Time(42.0);
  h.frame_id = "world";
  const auto m = toMapMsg(h, {square(1, 0, 0, 4), square(5, 6, 0, 5)}, nullptr);
  EXPECT_EQ(m.header.frame_id, "world");
  EXPECT_DOUBLE_EQ(m.header.stamp.toSec(), 42.0);
  ASSERT_EQ(m.trees.size(), 2u);
  EXPECT_EQ(m.trees[0].id, 1);
  EXPECT_EQ(m.trees[1].id, 5);
}

TEST(Conversions, DetectionsScaled) {
  treemapper_msgs::Detection2DArray a;
  treemapper_msgs::Detection2D d;
  d.x = 10; d.y = 20; d.width = 30; d.height = 40; d.score = 0.8f; d.label = 1;
  a.detections.push_back(d);
  auto b = fromMsg(a, 1.0);
  ASSERT_EQ(b.size(), 1u);
  EXPECT_DOUBLE_EQ(b[0].x, 10); EXPECT_DOUBLE_EQ(b[0].height, 40); EXPECT_EQ(b[0].label, 1);
  EXPECT_NEAR(b[0].score, 0.8, 1e-6);
  b = fromMsg(a, 0.5);  // full-res detector boxes onto a half-res calibration
  EXPECT_DOUBLE_EQ(b[0].x, 5); EXPECT_DOUBLE_EQ(b[0].width, 15);
}

TEST(Conversions, PoseFromTransformAndInvalidOnGarbage) {
  geometry_msgs::Transform t;
  t.translation.x = 1; t.translation.y = 2; t.translation.z = 15;
  t.rotation.w = 1;
  auto p = toDronePose(t, 3.0);
  EXPECT_TRUE(p.valid);
  EXPECT_DOUBLE_EQ(p.z, 15);
  EXPECT_DOUBLE_EQ(p.t, 3.0);
  t.rotation.w = 0;  // zero quaternion
  EXPECT_FALSE(toDronePose(t, 3.0).valid);
  t.rotation.w = 1;
  t.translation.z = std::nan("");
  EXPECT_FALSE(toDronePose(t, 3.0).valid);
}

TEST(Conversions, CountersAndTiming) {
  std_msgs::Header h;
  const auto c = toCountersMsg(h, "input", {{"a", 1}, {"b", 22}});
  EXPECT_EQ(c.stage, "input");
  ASSERT_EQ(c.reasons.size(), 2u);
  EXPECT_EQ(c.reasons[1], "b");
  EXPECT_EQ(c.counts[1], 22u);
  const auto s = toTimingMsg(h, {{"extract", 0.5}});
  ASSERT_EQ(s.stages.size(), 1u);
  EXPECT_DOUBLE_EQ(s.ms[0], 0.5);
}

TEST(Conversions, MarkersStartWithDeleteAllAndColourByState) {
  std_msgs::Header h;
  h.frame_id = "world";
  TreeMetaTracker tr;
  tr.update(1.0, MissionPhase::kPlanning, {square(1, 0, 0, 4), square(2, 6, 0, 5, true), square(3, 12, 0, 4)});
  const auto arr = toMarkers(h, {square(1, 0, 0, 4), square(2, 6, 0, 5, true), square(3, 12, 0, 4)}, &tr, 3);
  ASSERT_EQ(arr.markers.size(), 1u + 3u * 3u);
  EXPECT_EQ(arr.markers[0].action, visualization_msgs::Marker::DELETEALL);
  const auto& outline1 = arr.markers[1];
  EXPECT_EQ(outline1.ns, "outline");
  EXPECT_EQ(outline1.points.size(), 5u);  // closed strip
  EXPECT_DOUBLE_EQ(outline1.points[0].z, 4.0);
  EXPECT_GT(outline1.color.r, 0.9f);       // locked = orange
  EXPECT_GT(arr.markers[4].color.g, 0.7f); // sprayed = green
  EXPECT_GT(arr.markers[7].color.r, 0.9f); // target = red
  EXPECT_LT(arr.markers[7].color.g, 0.2f);
  EXPECT_EQ(arr.markers[3].text.substr(0, 1), "1");
}

TEST(TreeMetaTracker, FirstPublishedLastUpdatedLockedVanished) {
  TreeMetaTracker tr;
  auto ch = tr.update(1.0, MissionPhase::kScan, {square(1, 0, 0, 4)});
  ASSERT_EQ(ch.size(), 1u);
  EXPECT_FALSE(tr.find(1)->locked);
  ch = tr.update(2.0, MissionPhase::kScan, {square(1, 0, 0, 4)});
  EXPECT_TRUE(ch.empty());
  EXPECT_DOUBLE_EQ(tr.find(1)->last_updated, 1.0);
  ch = tr.update(3.0, MissionPhase::kPlanning, {square(1, 0, 0, 4.2), square(2, 6, 0, 4)});
  EXPECT_EQ(ch.size(), 2u);
  EXPECT_DOUBLE_EQ(tr.find(1)->first_published, 1.0);
  EXPECT_DOUBLE_EQ(tr.find(1)->last_updated, 3.0);
  EXPECT_TRUE(tr.find(1)->locked);
  tr.update(4.0, MissionPhase::kScan, {square(2, 6, 0, 4)});
  EXPECT_TRUE(tr.find(2)->locked);  // locked stays locked
  EXPECT_EQ(tr.vanished().count(1), 1u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}

// S13: health's SLEEPING must mean exactly "the mapper is asleep" (phasePolicy), so SCAN_APPROACH, where the
// mapper maps, is awake (and a stale input there is warned about, not hidden).
TEST(NodeSupport, SleepPhasesAreExactlyThePhasesTheMapperSleeps) {
  for (int k = 0; k <= static_cast<int>(MissionPhase::kError); ++k) {
    const auto p = static_cast<MissionPhase>(k);
    EXPECT_EQ(isSleepPhase(p), !treemapper::phasePolicy(p, treemapper::kNoTree).awake) << treemapper::toString(p);
  }
  EXPECT_FALSE(isSleepPhase(MissionPhase::kScanApproach));
  EXPECT_FALSE(isSleepPhase(MissionPhase::kScan));
  EXPECT_TRUE(isSleepPhase(MissionPhase::kSpraying));
  EXPECT_TRUE(isSleepPhase(MissionPhase::kIdle));
  EXPECT_TRUE(isSleepPhase(MissionPhase::kReturn));
}

// B1: a throwing callback body is contained (logged, counted, FAULT for the hold window); a fatal
// configuration error still propagates so startup misconfiguration ends the node.
TEST(NodeSupport, CallbackGuardContainsExceptionsButNotFatalConfigErrors) {
  CallbackGuard g(10.0);
  std::vector<std::string> logs;
  auto log = [&](const std::string& m) { logs.push_back(m); };
  int ran = 0;
  EXPECT_TRUE(g.run("ok", 100.0, [&] { ++ran; }, log));
  EXPECT_EQ(ran, 1);
  EXPECT_FALSE(g.faultActive(100.0));
  EXPECT_TRUE(g.lastError().empty());
  EXPECT_FALSE(g.run("disparity", 100.0, [] { throw std::out_of_range("vector::at"); }, log));
  EXPECT_FALSE(g.run("disparity", 101.0, [] { throw 42; }, log));
  ASSERT_EQ(logs.size(), 2u);
  EXPECT_NE(logs[0].find("disparity"), std::string::npos);
  EXPECT_NE(logs[0].find("vector::at"), std::string::npos);
  EXPECT_EQ(g.total(), 2u);
  const auto c = g.counters();
  EXPECT_EQ(c.at("callback_exceptions"), 2u);
  EXPECT_EQ(c.at("callback_exceptions_disparity"), 2u);
  EXPECT_TRUE(g.faultActive(105.0));
  EXPECT_FALSE(g.faultActive(111.5));  // the FAULT window ends; the warning (lastError) stays
  EXPECT_NE(g.lastError().find("2 total"), std::string::npos);
  EXPECT_THROW(g.run("config", 120.0, [] { throw FatalConfigError("bad ~disparity_offset"); }, log), FatalConfigError);
  EXPECT_EQ(g.total(), 2u);
}
