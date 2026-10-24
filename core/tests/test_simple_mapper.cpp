// Rule tests for the simple baseline (brief section 7) and its geometry helpers.
// Each rule test would fail if the rule were broken.
#include <gtest/gtest.h>

#include <cmath>
#include <set>

#include "treemapper/geometry.hpp"
#include "treemapper/simple_mapper.hpp"

using namespace treemapper;

namespace {

Observation square(double cx, double cy, double half, double h = 4.0) {
  Observation o;
  o.hull = {{cx - half, cy - half}, {cx + half, cy - half}, {cx + half, cy + half}, {cx - half, cy + half}};
  o.centroid = {cx, cy};
  o.height = h;
  o.n_points = 100;
  return o;
}

SimpleMapperConfig cfg() {
  SimpleMapperConfig c;
  c.publish_min_obs = 3;
  return c;
}

// publish a tree at (x, y) during SCAN
void addTree(SimpleMapper& m, double& t, double x, double y, double half = 2.0) {
  for (int i = 0; i < 3; ++i) m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(x, y, half)});
}

const TreeEntry* find(const std::vector<TreeEntry>& v, TreeId id) {
  for (const auto& e : v) if (e.id == id) return &e;
  return nullptr;
}

double area(const TreeEntry& e) { return std::fabs(polygonArea(e.outline)); }

}  // namespace

TEST(Geometry, HullAreaCentroidIntersection) {
  Polygon2 h = convexHull({{0, 0}, {2, 0}, {2, 2}, {0, 2}, {1, 1}, {1, 0}});
  ASSERT_EQ(h.size(), 4u);
  EXPECT_NEAR(polygonArea(h), 4.0, 1e-12);  // CCW positive
  Point2 c = polygonCentroid(h);
  EXPECT_NEAR(c.x, 1.0, 1e-12);
  EXPECT_NEAR(c.y, 1.0, 1e-12);
  Polygon2 g = convexHull({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
  EXPECT_NEAR(convexIntersectionArea(h, g), 1.0, 1e-9);
  EXPECT_TRUE(pointInPolygon(h, {0.5, 0.5}));
  EXPECT_FALSE(pointInPolygon(h, {2.5, 0.5}));
}

TEST(Geometry, NadirCameraBackprojection) {
  // Isaac mount: optical z -> body -z, optical x -> body -y, optical y -> body -x.
  StereoCalibration cal;
  cal.fx = cal.fy = 100.0; cal.cx = 50.0; cal.cy = 40.0; cal.baseline = 0.1; cal.stride = 1;
  cal.body_cam = {0, 0, 0, -0.70710678, 0.70710678, 0, 0};
  Transform world_body{1.0, 2.0, 10.0, 0, 0, 0, 1};
  Vec3 po;
  ASSERT_TRUE(backprojectOptical(cal, 60.0, 40.0, 1.0, &po));  // Z = 10 m, 1 m right of centre
  Vec3 w = world_body.compose(cal.body_cam).apply(po);
  EXPECT_NEAR(w.z, 0.0, 1e-9);
  EXPECT_NEAR(w.x, 1.0, 1e-9);
  EXPECT_NEAR(w.y, 2.0 - 1.0, 1e-9);
  EXPECT_FALSE(backprojectOptical(cal, 0, 0, std::nan(""), &po));
  EXPECT_FALSE(backprojectOptical(cal, 0, 0, 0.0, &po));
}

TEST(SimpleMapper, ExtractObservationFromDisparity) {
  StereoCalibration cal;
  cal.width = 200; cal.height = 200; cal.fx = cal.fy = 200.0; cal.cx = cal.cy = 100.0;
  cal.baseline = 0.1; cal.stride = 2;
  cal.body_cam = {0, 0, 0, -0.70710678, 0.70710678, 0, 0};
  SimpleMapper m(SimpleMapperConfig{}, cal);
  FrameInput f;
  f.t = 1.0; f.pose.valid = true; f.pose.z = 10.0; f.phase = MissionPhase::kScan;
  f.grid_w = f.grid_h = 100;
  f.disparity.assign(100 * 100, 200.0f * 0.1f / 10.0f);  // ground at range 10 -> z = 0
  for (int r = 30; r < 70; ++r)
    for (int c = 30; c < 70; ++c) f.disparity[r * 100 + c] = 200.0f * 0.1f / 6.0f;  // canopy top z = 4
  Detection2D b; b.x = 30; b.y = 30; b.width = 140; b.height = 140; b.score = 0.9;
  f.boxes.push_back(b);
  auto obs = m.extractObservations(f);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].height, 4.0, 1e-3);
  EXPECT_NEAR(obs[0].centroid.x, 0.0, 0.1);
  EXPECT_NEAR(obs[0].centroid.y, 0.0, 0.1);
  EXPECT_GT(m.counters().at("points_dropped_ground"), 0u);  // box corners on the ground were dropped
}

// Rules 1 + 4: IDs never reused (after expiry and merges); the better-established entry keeps its
// identity and height when a fragment merges into it.
TEST(SimpleMapperRules, IdsNeverReusedAndMergeKeepsEstablished) {
  SimpleMapper m(cfg());
  double t = 0;
  m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(50, 50, 1)});  // ID 0: unconfirmed, expires
  t += 5.0;
  for (int i = 0; i < 6; ++i) m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(0, 0, 2.0, 4.0)});
  for (int i = 0; i < 3; ++i) m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(3.0, 0, 0.5, 9.0)});
  auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u);
  EXPECT_EQ(trees[0].id, 1);  // the expired entry's ID 0 was not handed out again
  EXPECT_EQ(trees[1].id, 2);  // the fragment
  EXPECT_EQ(m.counters().at("entries_expired_unconfirmed"), 1u);
  // tree 1 grows over the fragment -> merge; tree 1 (6 obs) keeps identity and height
  m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(0.5, 0, 3.0, 4.0)});
  trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u);
  EXPECT_EQ(trees[0].id, 1);
  EXPECT_NEAR(trees[0].height, 4.0, 1e-9);
  EXPECT_EQ(m.counters().at("entries_merged"), 1u);
  addTree(m, t, 20, 0);
  trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u);
  EXPECT_EQ(trees[1].id, 3);  // neither 0 (expired) nor 2 (merged away) comes back
}

// Rule 3: no undefined height published; nothing published before it is confirmed.
TEST(SimpleMapperRules, NoUndefinedHeightPublished) {
  SimpleMapper m(cfg());
  double t = 0;
  Observation bad = square(0, 0, 2.0, std::nan(""));
  for (int i = 0; i < 5; ++i) m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {bad});
  EXPECT_TRUE(m.publishedTrees().empty());
  m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(0, 0, 2.0)});
  m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(0, 0, 2.0)});
  EXPECT_TRUE(m.publishedTrees().empty());  // 2 < publish_min_obs
  m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(0, 0, 2.0)});
  m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {bad});
  auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u);
  EXPECT_TRUE(std::isfinite(trees[0].height));
  EXPECT_NEAR(trees[0].height, 4.0, 1e-9);
}

// Brief section 6: no new standalone trees after scanning (PLANNING, HOPPING, sleeping phases).
TEST(SimpleMapperRules, NoNewTreesAfterScanning) {
  for (MissionPhase ph : {MissionPhase::kPlanning, MissionPhase::kHopping, MissionPhase::kSpraying,
                          MissionPhase::kIdle, MissionPhase::kReturn}) {
    SimpleMapper m(cfg());
    double t = 0;
    addTree(m, t, 0, 0);
    for (int i = 0; i < 10; ++i) m.integrate(t += 0.1, ph, 0, {square(15, 15, 2.0)});
    EXPECT_EQ(m.publishedTrees().size(), 1u) << toString(ph);
  }
  // an entry created during SCAN but still unconfirmed must not get published after the list locks
  {
    SimpleMapper m(cfg());
    double t = 0;
    m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(0, 0, 2.0)});
    m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(0, 0, 2.0)});
    for (int i = 0; i < 5; ++i) m.integrate(t += 0.1, MissionPhase::kPlanning, kNoTree, {square(0, 0, 2.0)});
    EXPECT_TRUE(m.publishedTrees().empty());
    EXPECT_EQ(m.counters().at("obs_dropped_unconfirmed_locked_phase"), 5u);
  }
  // ...but an unknown phase behaves as scanning (fail open)
  SimpleMapper m(cfg());
  double t = 0;
  addTree(m, t, 0, 0);
  for (int i = 0; i < 3; ++i) m.integrate(t += 0.1, MissionPhase::kUnknown, kNoTree, {square(15, 15, 2.0)});
  EXPECT_EQ(m.publishedTrees().size(), 2u);
}

// Brief section 6 / scorer safety: during a descent only the target may change.
TEST(SimpleMapperRules, OnlyTargetChangesDuringDescent) {
  SimpleMapper m(cfg());
  double t = 0;
  addTree(m, t, 0, 0);
  addTree(m, t, 8, 0);
  const auto before = m.publishedTrees();
  ASSERT_EQ(before.size(), 2u);
  const TreeId target = before[0].id, other = before[1].id;
  // growing observations on both trees while hopping onto `target`
  for (int i = 0; i < 5; ++i) {
    m.integrate(t += 0.1, MissionPhase::kHopping, target, {square(0.5, 0, 2.5), square(8.5, 0, 2.5)});
  }
  auto after = m.publishedTrees();
  EXPECT_GT(area(*find(after, target)), area(*find(before, target)) + 1.0);  // the target grew
  EXPECT_NEAR(area(*find(after, other)), area(*find(before, other)), 1e-9);  // the other did not change
  EXPECT_EQ(find(after, other)->height, find(before, other)->height);
  // no valid target while hopping -> everything frozen (fail closed)
  const auto mid = m.publishedTrees();
  for (int i = 0; i < 5; ++i) m.integrate(t += 0.1, MissionPhase::kHopping, kNoTree, {square(0.7, 0, 3.5)});
  EXPECT_NEAR(area(*find(m.publishedTrees(), target)), area(*find(mid, target)), 1e-9);
}

// Rule 5: sprayed trees stop changing; a detection inside one does not create a new tree.
TEST(SimpleMapperRules, SprayedTreesFrozen) {
  SimpleMapper m(cfg());
  double t = 0;
  addTree(m, t, 0, 0, 3.0);
  const TreeId id = m.publishedTrees()[0].id;
  ASSERT_TRUE(m.markSprayed(id));
  EXPECT_FALSE(m.markSprayed(12345));
  const auto before = m.publishedTrees();
  for (int i = 0; i < 6; ++i) {
    m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(0.5, 0.5, 4.0, 7.0)});  // fuses -> refused
    m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, {square(2.5, 2.5, 0.3, 7.0)});  // inside, not associated
  }
  auto after = m.publishedTrees();
  ASSERT_EQ(after.size(), 1u);
  EXPECT_TRUE(after[0].sprayed);
  EXPECT_NEAR(area(after[0]), area(before[0]), 1e-12);
  EXPECT_EQ(after[0].height, before[0].height);
}

TEST(SimpleMapperRules, PhasePolicyTable) {
  EXPECT_TRUE(phasePolicy(MissionPhase::kScan, kNoTree).may_create);
  EXPECT_TRUE(phasePolicy(MissionPhase::kUnknown, kNoTree).warn_unknown);
  EXPECT_TRUE(phasePolicy(MissionPhase::kUnknown, kNoTree).may_create);
  EXPECT_FALSE(phasePolicy(MissionPhase::kPlanning, kNoTree).may_create);
  EXPECT_FALSE(phasePolicy(MissionPhase::kPlanning, kNoTree).freeze_non_target);
  EXPECT_TRUE(phasePolicy(MissionPhase::kHopping, 3).freeze_non_target);
  EXPECT_FALSE(phasePolicy(MissionPhase::kSpraying, 3).awake);
  EXPECT_FALSE(phasePolicy(MissionPhase::kIdle, kNoTree).awake);
}

TEST(SimpleMapperRules, HoldsWithoutPoseAndSleeps) {
  SimpleMapper m(cfg());
  FrameInput f;
  f.phase = MissionPhase::kScan;
  f.pose.valid = false;
  m.process(f);
  EXPECT_EQ(m.state(), MapperState::kHoldingNoPose);
  EXPECT_EQ(m.counters().at("frames_held_no_pose"), 1u);
  f.phase = MissionPhase::kSpraying;
  m.process(f);
  EXPECT_EQ(m.state(), MapperState::kSleeping);
}
