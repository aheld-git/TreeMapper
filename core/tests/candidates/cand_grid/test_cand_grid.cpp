// cand_grid rule tests (PROJECT_BRIEF sections 6 and 7). Every test would fail if its rule broke.
//
// A tiny ray-cast stereo world drives the full mapper: a nadir camera (the recorded mount rotation),
// flat ground at z = 0, dome-shaped trees, seeded disparity noise. Pure decision functions are tested
// directly as well.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <set>
#include <sstream>
#include <string>

#include "treemapper/candidates/cand_grid/cand_grid.hpp"
#include "treemapper/mapper_registry.hpp"

using namespace treemapper;
using namespace treemapper::cand_grid;

namespace {

struct Dome {
  double x, y, r, top, base;
};

struct World {
  std::vector<Dome> trees;
  double surface(double x, double y) const {
    double h = 0.0;
    for (const auto& t : trees) {
      const double rho = std::hypot(x - t.x, y - t.y) / t.r;
      if (rho < 1.0) h = std::max(h, t.base + (t.top - t.base) * std::sqrt(1.0 - rho * rho));
    }
    return h;
  }
};

StereoCalibration testCal() {
  StereoCalibration c;
  c.width = 640;
  c.height = 400;
  c.fx = c.fy = 454.53;
  c.cx = 320.0;
  c.cy = 200.0;
  c.baseline = 0.16;
  c.stride = 4;
  c.body_cam = Transform{0.0, 0.0, 0.0, -0.70710678, 0.70710678, 0.0, 0.0};  // exact nadir (recorded mount)
  return c;
}

/// Optical point of a world point for an identity-attitude body at (px, py, pz) (nadir mount:
/// optical x = -world y, optical y = -world x, optical z = down).
Vec3 toOptical(double px, double py, double pz, double x, double y, double z) {
  return {-(y - py), -(x - px), pz - z};
}

struct FrameOpts {
  double sigma_px{0.1};
  bool boxes{true};
  double disparity_bias_px{0.0};  ///< added to every disparity (a "bad frame" when large)
  bool pose_valid{true};
  double dropout{0.02};  ///< fraction of pixels without disparity
};

FrameInput makeFrame(const World& w, const StereoCalibration& cal, double t, MissionPhase phase, double px, double py,
                     double pz, std::mt19937* rng, const FrameOpts& o = {}, TreeId target = kNoTree) {
  FrameInput f;
  f.t = f.stamp = t;
  f.phase = phase;
  f.hop_target = target;
  f.pose.t = t;
  f.pose.x = px;
  f.pose.y = py;
  f.pose.z = pz;
  f.pose.valid = o.pose_valid;
  f.grid_w = cal.width / cal.stride;
  f.grid_h = cal.height / cal.stride;
  f.disparity.assign(static_cast<std::size_t>(f.grid_w * f.grid_h), std::nanf(""));
  std::normal_distribution<double> n(0.0, o.sigma_px);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  double top_max = 0.0;
  for (const auto& tr : w.trees) top_max = std::max(top_max, tr.top);
  const Transform wb{px, py, pz, 0, 0, 0, 1};
  const Transform wc = wb.compose(cal.body_cam);
  const double s = cal.stride, off = cal.stride / 2;
  for (int r = 0; r < f.grid_h; ++r) {
    for (int c = 0; c < f.grid_w; ++c) {
      const double u = c * s + off, v = r * s + off;
      const double dx = (u - cal.cx) / cal.fx, dy = (v - cal.cy) / cal.fy;
      if (o.dropout > 0.0 && u01(*rng) < o.dropout) continue;  // stereo dropout (no disparity)
      // identity attitude + nadir mount: optical depth == vertical drop, so march from the highest top
      double lo = std::max(0.02, pz - top_max - 0.05), hi = -1.0;
      for (double d = lo; d < pz + 1.0; d += 0.1) {
        const Vec3 p = wc.apply({dx * d, dy * d, d});
        if (p.z <= w.surface(p.x, p.y)) { hi = d; break; }
        lo = d;
      }
      if (hi <= 0) continue;
      for (int it = 0; it < 6; ++it) {
        const double mid = 0.5 * (lo + hi);
        const Vec3 p = wc.apply({dx * mid, dy * mid, mid});
        (p.z <= w.surface(p.x, p.y) ? hi : lo) = mid;
      }
      const double depth = hi;
      f.disparity[static_cast<std::size_t>(r * f.grid_w + c)] =
          static_cast<float>(cal.fx * cal.baseline / depth + n(*rng) + o.disparity_bias_px);
    }
  }
  if (o.boxes) {
    for (const auto& tr : w.trees) {
      double u0 = 1e9, v0 = 1e9, u1 = -1e9, v1 = -1e9;
      for (int k = 0; k < 4; ++k) {
        const double x = tr.x + (k & 1 ? tr.r : -tr.r), y = tr.y + (k & 2 ? tr.r : -tr.r);
        const Vec3 p = toOptical(px, py, pz, x, y, tr.base);
        const double u = cal.fx * p.x / p.z + cal.cx, v = cal.fy * p.y / p.z + cal.cy;
        u0 = std::min(u0, u); v0 = std::min(v0, v); u1 = std::max(u1, u); v1 = std::max(v1, v);
      }
      u0 = std::max(0.0, u0); v0 = std::max(0.0, v0);
      u1 = std::min<double>(cal.width, u1); v1 = std::min<double>(cal.height, v1);
      if (u1 - u0 < 10 || v1 - v0 < 10) continue;
      Detection2D b;
      b.x = u0; b.y = v0; b.width = u1 - u0; b.height = v1 - v0; b.score = 0.9;
      f.boxes.push_back(b);
    }
  }
  return f;
}

/// A straight scan pass along y at 15 m, 4 m/s, 10 Hz, from y0 to y1 at x.
void scanPass(CandGridMapper* m, const World& w, const StereoCalibration& cal, double* t, double x, double y0,
              double y1, std::mt19937* rng, MissionPhase phase = MissionPhase::kScan, const FrameOpts& o = {}) {
  const double dir = y1 > y0 ? 1.0 : -1.0;
  for (double y = y0; dir * (y1 - y) >= 0; y += dir * 0.4) {
    m->process(makeFrame(w, cal, *t, phase, x, y, 15.0, rng, o));
    *t += 0.1;
  }
}

const TreeEntry* nearest(const std::vector<TreeEntry>& trees, double x, double y) {
  const TreeEntry* best = nullptr;
  double bd = 1e9;
  for (const auto& e : trees) {
    const double d = std::hypot(e.centre.x - x, e.centre.y - y);
    if (d < bd) { bd = d; best = &e; }
  }
  return bd < 2.0 ? best : nullptr;
}

double area(const Polygon2& p) { return std::fabs(polygonArea(p)); }

World twoTrees(double gap) {
  World w;
  const double r = 2.2;
  w.trees.push_back({0.0, 0.0, r, 5.0, 1.0});
  w.trees.push_back({0.0, 2 * r + gap, r, 4.5, 1.0});
  return w;
}

Config testConfig() { return Config{}; }

}  // namespace

// ------------------------------------------------------------------------------------------------
// Configuration

TEST(CandGridConfig, ShippedConfigFileEqualsBuiltInDefaults) {
  std::string path = __FILE__;  // .../core/tests/candidates/cand_grid/test_cand_grid.cpp
  for (int i = 0; i < 5; ++i) path = path.substr(0, path.find_last_of('/'));
  std::ifstream in(path + "/config/cand_grid.cfg");
  ASSERT_TRUE(in.good()) << path + "/config/cand_grid.cfg";
  MapperConfig file;
  std::string line;
  while (std::getline(in, line)) {
    line = line.substr(0, line.find('#'));
    const auto eq = line.find('=');
    if (eq == std::string::npos) continue;
    auto trim = [](std::string s) {
      s.erase(0, s.find_first_not_of(" \t"));
      s.erase(s.find_last_not_of(" \t\r") + 1);
      return s;
    };
    file[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
  }
  const auto defaults = configKeyValues(Config{});
  ASSERT_EQ(file.size(), defaults.size()) << "every key once in the shipped config";
  const auto parsed = configKeyValues(configFrom(file));
  for (const auto& kv : defaults) {
    ASSERT_TRUE(file.count(kv.first)) << "missing key " << kv.first;
    EXPECT_EQ(parsed.at(kv.first), kv.second) << kv.first;
  }
}

TEST(CandGridConfig, UnknownKeyIsRejectedAndClampsAreLogged) {
  EXPECT_THROW(configFrom({{"no_such_key", "1"}}), std::invalid_argument);
  EXPECT_THROW(configFrom({{"cell", "abc"}}), std::invalid_argument);
  std::vector<std::string> logs;
  const Config c = configFrom({{"height_quantile", "1.7"}}, [&](const std::string& m) { logs.push_back(m); });
  EXPECT_DOUBLE_EQ(c.height_quantile, 1.0);
  ASSERT_EQ(logs.size(), 1u);
  EXPECT_NE(logs[0].find("height_quantile"), std::string::npos);
}

TEST(CandGridConfig, RegisteredByName) {
  auto m = createMapper("cand_grid", {}, testCal());
  ASSERT_TRUE(m);
  EXPECT_TRUE(m->publishedTrees().empty());
}

// ------------------------------------------------------------------------------------------------
// Pure functions

TEST(CandGridPure, HoveringSaturatesConfidenceAndDoesNotDistortHeight) {
  Config c;
  Cell once, hover;
  fuseCell(&once, 5.0f, 6.0f, c);
  for (int i = 0; i < 5000; ++i) fuseCell(&hover, 5.0f, 6.0f, c);
  EXPECT_LE(hover.w, c.cell_weight_cap + 1e-6);  // capped: a long hover cannot inflate confidence
  EXPECT_NEAR(hover.h, once.h, 1e-5);
  EXPECT_LE(hover.spread(), 1e-3);
}

TEST(CandGridPure, OutlierFrameRejectedThenPersistentChangeAccepted) {
  Config c;
  Cell k;
  for (int i = 0; i < 10; ++i) fuseCell(&k, 1.0f, 3.0f, c);
  EXPECT_EQ(fuseCell(&k, 9.0f, 3.0f, c), FuseResult::kRejected);  // one bad frame
  EXPECT_NEAR(k.h, 1.0f, 1e-5);
  EXPECT_EQ(fuseCell(&k, 9.0f, 3.0f, c), FuseResult::kRejected);
  EXPECT_EQ(fuseCell(&k, 9.0f, 3.0f, c), FuseResult::kReset);  // three in a row: the old value was wrong
  EXPECT_NEAR(k.h, 9.0f, 1e-5);
}

TEST(CandGridPure, FrameQualityDownweightsFastRotationAndFailsOpenWithoutRate) {
  Config c;
  EXPECT_DOUBLE_EQ(frameQuality(-1.0, 100.0, c), 1.0);
  EXPECT_GT(frameQuality(0.0, 100.0, c), 0.99);
  EXPECT_LT(frameQuality(1.5, 0.0, c), 0.1);
  EXPECT_LT(frameQuality(0.0, 0.2, c), 0.3);  // still recovering right after a turn
}

TEST(CandGridPure, PublishDecisionNeedsEveryKindOfEvidence) {
  Config c;
  PhasePolicy scan = phasePolicy(MissionPhase::kScan, kNoTree);
  InstanceEvidence e;
  e.area = 10.0;
  e.peak_chm = 4.0;
  e.box_fraction = 0.8;
  e.confirm = 5;
  e.height = 5.0;
  e.closed = true;
  EXPECT_EQ(publishDecision(e, scan, c), PublishVerdict::kPublish);
  auto v = e; v.height = std::nan("");
  EXPECT_EQ(publishDecision(v, scan, c), PublishVerdict::kNoHeight);  // rule 3
  v = e; v.box_fraction = 0.0;
  EXPECT_EQ(publishDecision(v, scan, c), PublishVerdict::kNoBoxSupport);
  v = e; v.confirm = 1;
  EXPECT_EQ(publishDecision(v, scan, c), PublishVerdict::kNotConfirmed);
  v = e; v.area = 1.0;
  EXPECT_EQ(publishDecision(v, scan, c), PublishVerdict::kTooSmall);
  v = e; v.peak_chm = 0.9;
  EXPECT_EQ(publishDecision(v, scan, c), PublishVerdict::kTooLow);
  v = e; v.closed = false;
  EXPECT_EQ(publishDecision(v, scan, c), PublishVerdict::kOpen);
  EXPECT_EQ(publishDecision(e, phasePolicy(MissionPhase::kPlanning, kNoTree), c), PublishVerdict::kLockedPhase);
  EXPECT_EQ(publishDecision(e, phasePolicy(MissionPhase::kHopping, 3), c), PublishVerdict::kLockedPhase);
}

TEST(CandGridPure, MergeRulesAreSizeAwareAndKeepTheEstablishedTree) {
  Config c;
  EXPECT_TRUE(mergeAllowedBySize(2.0, 20.0, c));    // small fragment into a large tree
  EXPECT_FALSE(mergeAllowedBySize(18.0, 20.0, c));  // two large trees never merge on overlap alone
  EXPECT_TRUE(mergeSurvivor({true, 5.0, 9}, {false, 50.0, 1}));   // published beats unpublished
  EXPECT_TRUE(mergeSurvivor({true, 20.0, 9}, {true, 5.0, 1}));    // then more area
  EXPECT_TRUE(mergeSurvivor({true, 20.0, 1}, {true, 20.0, 9}));   // then the older one
  EXPECT_FALSE(mergeSurvivor({true, 20.0, 9}, {true, 20.0, 1}));
}

TEST(CandGridPure, PhaseRulesForInstanceChanges) {
  const PhasePolicy scan = phasePolicy(MissionPhase::kScan, kNoTree);
  const PhasePolicy plan = phasePolicy(MissionPhase::kPlanning, kNoTree);
  const PhasePolicy hop = phasePolicy(MissionPhase::kHopping, 4);
  const PhasePolicy hop_none = phasePolicy(MissionPhase::kHopping, kNoTree);
  EXPECT_TRUE(instanceMayChange(false, false, kNoTree, scan, kNoTree));
  EXPECT_FALSE(instanceMayChange(true, true, 1, scan, kNoTree));     // sprayed: never (rule 5)
  EXPECT_FALSE(instanceMayChange(false, false, kNoTree, plan, kNoTree));  // no new tree after scanning
  EXPECT_TRUE(instanceMayChange(true, false, 2, plan, kNoTree));     // existing trees may grow
  EXPECT_TRUE(instanceMayChange(true, false, 4, hop, 4));            // the target may change
  EXPECT_FALSE(instanceMayChange(true, false, 5, hop, 4));           // a neighbour may not
  EXPECT_FALSE(instanceMayChange(true, false, 4, hop_none, kNoTree)); // no target: freeze (fail closed)
  const PhasePolicy climb = climbOutPolicy(plan, true);
  EXPECT_FALSE(climb.may_create);
  EXPECT_FALSE(climb.may_merge);
  EXPECT_TRUE(climb.freeze_non_target);
  EXPECT_FALSE(instanceMayChange(true, false, 2, climb, 4));
  EXPECT_FALSE(climbOutPolicy(plan, false).freeze_non_target);
}

TEST(CandGridPure, SegmentationSeparatesTouchingDomesAndMergesNoisePeaks) {
  // Two domes whose canopies touch (gap 0): the fused surface still has a valley -> two segments.
  Config c;
  World w = twoTrees(0.0);
  Window win;
  win.x0 = -60; win.y0 = -40; win.w = 120; win.h = 120;
  win.cells.resize(static_cast<std::size_t>(win.w * win.h));
  std::mt19937 rng(3);
  std::normal_distribution<double> bump(0.0, 0.1);  // fused-surface noise (well below the valley depth)
  for (int y = 0; y < win.h; ++y) {
    for (int x = 0; x < win.w; ++x) {
      Cell& k = win.cells[static_cast<std::size_t>(y * win.w + x)];
      const double wx = (win.x0 + x + 0.5) * c.cell, wy = (win.y0 + y + 0.5) * c.cell;
      k.h = static_cast<float>(w.surface(wx, wy) + bump(rng));
      k.w = 10.0f;
      k.nf = 10;
      k.box = 10;
    }
  }
  const Segmentation s = segmentCanopy(win, c);
  int trees = 0;
  for (const auto& sg : s.segments) trees += sg.tree;
  EXPECT_EQ(trees, 2);
}

// ------------------------------------------------------------------------------------------------
// End-to-end rules through the mapper

class CandGridScenario : public ::testing::Test {
 protected:
  StereoCalibration cal = testCal();
  std::mt19937 rng{7};
  double t = 0.0;
};

TEST_F(CandGridScenario, Rule4_TouchingTreesBelowDepthNoiseStayTwoEntriesEvenUnderAMergedBox) {
  // gap 0.2 m; per-point depth noise at 10 m range with 0.3 px is ~0.4 m (gap < noise)
  World w = twoTrees(0.2);
  CandGridMapper m(testConfig(), cal);
  FrameOpts o;
  o.sigma_px = 0.3;
  o.boxes = false;
  // one merged box over both trees (the detector's merged-box failure)
  const double t0 = t;
  for (double y = -14; y <= 20; y += 0.4) {
    FrameInput f = makeFrame(w, cal, t, MissionPhase::kScan, 0.0, y, 15.0, &rng, o);
    Detection2D b;
    b.x = 0; b.y = 0; b.width = 640; b.height = 400; b.score = 0.9;  // covers everything in view
    f.boxes.push_back(b);
    m.process(f);
    t += 0.1;
  }
  (void)t0;
  const auto trees = m.publishedTrees();
  for (const auto& e : trees) {
    std::printf("  entry %d centre (%.2f, %.2f) area %.1f h %.2f\n", e.id, e.centre.x, e.centre.y, area(e.outline), e.height);
  }
  ASSERT_EQ(trees.size(), 2u);
  EXPECT_NE(nearest(trees, 0.0, 0.0), nullptr);
  EXPECT_NE(nearest(trees, 0.0, 4.6), nullptr);
  EXPECT_NE(nearest(trees, 0.0, 0.0), nearest(trees, 0.0, 4.6));
  // outlines must not swallow the neighbour: each centre stays inside its own canopy
  for (const auto& e : trees) {
    const double d0 = std::hypot(e.centre.x, e.centre.y), d1 = std::hypot(e.centre.x, e.centre.y - 4.6);
    EXPECT_LT(std::min(d0, d1), 1.0);
  }
}

TEST_F(CandGridScenario, Rule1_IdsAreStableAndNeverReused) {
  World w;
  for (int i = 0; i < 4; ++i) w.trees.push_back({0.0, i * 6.0, 2.2, 5.0, 1.0});
  CandGridMapper m(testConfig(), cal);
  std::map<TreeId, Point2> first_centre;
  TreeId max_seen = -1;
  for (double y = -14; y <= 32; y += 0.4) {
    m.process(makeFrame(w, cal, t, MissionPhase::kScan, 0.0, y, 15.0, &rng));
    t += 0.1;
    for (const auto& e : m.publishedTrees()) {
      if (!first_centre.count(e.id)) {
        EXPECT_GT(e.id, max_seen) << "a new tree must get a fresh, never-used ID";
        max_seen = std::max(max_seen, e.id);
        first_centre[e.id] = e.centre;
      } else {
        // an ID keeps its meaning: its centre stays on the same tree
        EXPECT_LT(std::hypot(e.centre.x - first_centre[e.id].x, e.centre.y - first_centre[e.id].y), 1.5);
      }
    }
  }
  EXPECT_EQ(m.publishedTrees().size(), 4u);
  EXPECT_FALSE(m.markSprayed(999));
}

TEST_F(CandGridScenario, Rule2_BadFramesNeverShrinkJumpOrDeformAPublishedTree) {
  World w = twoTrees(1.0);
  CandGridMapper m(testConfig(), cal);
  scanPass(&m, w, cal, &t, 0.0, -14, 20, &rng);
  const auto before = m.publishedTrees();
  ASSERT_EQ(before.size(), 2u);
  // five garbage frames hovering over the trees: every disparity +2 px (everything ~4 m too high / near),
  // then a burst of heavy noise
  FrameOpts bad;
  bad.disparity_bias_px = 2.0;
  for (int i = 0; i < 3; ++i) { m.process(makeFrame(w, cal, t, MissionPhase::kScan, 0.0, 2.0, 15.0, &rng, bad)); t += 0.1; }
  bad.disparity_bias_px = 0.0;
  bad.sigma_px = 3.0;
  for (int i = 0; i < 2; ++i) { m.process(makeFrame(w, cal, t, MissionPhase::kScan, 0.0, 2.0, 15.0, &rng, bad)); t += 0.1; }
  const auto after = m.publishedTrees();
  ASSERT_EQ(after.size(), 2u);
  for (const auto& b : before) {
    const TreeEntry* a = nullptr;
    for (const auto& e : after) if (e.id == b.id) a = &e;
    ASSERT_NE(a, nullptr) << "published tree vanished";
    EXPECT_GE(area(a->outline) + 1e-9, area(b.outline));                              // no shrink
    EXPECT_NEAR(convexIntersectionArea(a->outline, b.outline), area(b.outline), 1e-6);  // old outline kept inside
    EXPECT_LT(std::hypot(a->centre.x - b.centre.x, a->centre.y - b.centre.y), 0.5);     // no jump
    EXPECT_LT(std::fabs(a->height - b.height), 0.6);                                    // no height jump
    EXPECT_LT(area(a->outline), 1.3 * area(b.outline));                                  // no blow-up
  }
}

TEST_F(CandGridScenario, Rule3_HeightIsAlwaysFiniteAndAccurate) {
  World w = twoTrees(1.0);
  CandGridMapper m(testConfig(), cal);
  for (double y = -14; y <= 20; y += 0.4) {
    m.process(makeFrame(w, cal, t, MissionPhase::kScan, 0.0, y, 15.0, &rng));
    t += 0.1;
    for (const auto& e : m.publishedTrees()) {
      ASSERT_TRUE(std::isfinite(e.height));
      ASSERT_GE(e.outline.size(), 3u);
    }
  }
  const auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u);
  EXPECT_NEAR(nearest(trees, 0, 0)->height, 5.0, 0.4);
  EXPECT_NEAR(nearest(trees, 0, 5.4)->height, 4.5, 0.4);
}

TEST_F(CandGridScenario, Rule5_SprayedTreeNeverChangesAndNoNewTreeInsideIt) {
  World w = twoTrees(1.0);
  CandGridMapper m(testConfig(), cal);
  scanPass(&m, w, cal, &t, 0.0, -14, 20, &rng);
  const auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u);
  const TreeEntry sprayed = *nearest(trees, 0, 0);
  ASSERT_TRUE(m.markSprayed(sprayed.id));
  // the sprayed tree "grows" (a wider, taller dome at the same place) and is scanned again
  World w2 = w;
  w2.trees[0].r = 3.0;
  w2.trees[0].top = 6.0;
  scanPass(&m, w2, cal, &t, 0.0, 20, -14, &rng);
  const auto after = m.publishedTrees();
  ASSERT_EQ(after.size(), 2u) << "no new tree may appear inside a sprayed tree";
  const TreeEntry* s = nullptr;
  for (const auto& e : after) if (e.id == sprayed.id) s = &e;
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(s->sprayed);
  EXPECT_EQ(s->outline.size(), sprayed.outline.size());
  EXPECT_DOUBLE_EQ(area(s->outline), area(sprayed.outline));
  EXPECT_DOUBLE_EQ(s->height, sprayed.height);
}

TEST_F(CandGridScenario, Section6_NoNewTreesAfterScanningButExistingTreesMayGrow) {
  World w;
  w.trees.push_back({0.0, 0.0, 2.2, 5.0, 1.0});
  CandGridMapper m(testConfig(), cal);
  scanPass(&m, w, cal, &t, 0.0, -12, 8, &rng);
  ASSERT_EQ(m.publishedTrees().size(), 1u);
  // PLANNING: fly over a tree never seen before
  World w2 = w;
  w2.trees.push_back({0.0, 20.0, 2.2, 5.0, 1.0});
  scanPass(&m, w2, cal, &t, 0.0, 8, 32, &rng, MissionPhase::kPlanning);
  EXPECT_EQ(m.publishedTrees().size(), 1u);
  EXPECT_GT(m.counters().at("segments_rejected_new_tree_locked_phase"), 0u);
}

TEST_F(CandGridScenario, Section6_DuringADescentOnlyTheTargetChanges) {
  World w = twoTrees(0.6);
  CandGridMapper m(testConfig(), cal);
  // a short pass that only sees part of the second tree -> it would grow if allowed
  scanPass(&m, w, cal, &t, 0.0, -14, 4, &rng);
  const auto before = m.publishedTrees();
  ASSERT_GE(before.size(), 1u);
  const TreeEntry target = *nearest(before, 0, 0);
  // descent onto the target between the trees, low (7 m): low-angle views of the neighbour
  for (double z = 12.0; z >= 7.0; z -= 0.1) {
    m.process(makeFrame(w, cal, t, MissionPhase::kHopping, 0.0, 1.5, z, &rng, {}, target.id));
    t += 0.1;
  }
  const auto after = m.publishedTrees();
  for (const auto& b : before) {
    if (b.id == target.id) continue;
    const TreeEntry* a = nullptr;
    for (const auto& e : after) if (e.id == b.id) a = &e;
    ASSERT_NE(a, nullptr);
    EXPECT_DOUBLE_EQ(area(a->outline), area(b.outline));
    EXPECT_DOUBLE_EQ(a->height, b.height);
  }
  EXPECT_EQ(after.size(), before.size()) << "no new tree during a descent";
  // without a valid target everything is frozen (fail closed)
  CandGridMapper m2(testConfig(), cal);
  scanPass(&m2, w, cal, &t, 0.0, -14, 4, &rng);
  const auto b2 = m2.publishedTrees();
  std::vector<std::string> logs;
  m2.setLogger([&](const std::string& s) { logs.push_back(s); });
  for (int i = 0; i < 20; ++i) { m2.process(makeFrame(w, cal, t, MissionPhase::kHopping, 0.0, 1.5, 8.0, &rng)); t += 0.1; }
  const auto a2 = m2.publishedTrees();
  ASSERT_EQ(a2.size(), b2.size());
  for (std::size_t i = 0; i < a2.size(); ++i) EXPECT_DOUBLE_EQ(area(a2[i].outline), area(b2[i].outline));
  EXPECT_FALSE(logs.empty());
}

TEST_F(CandGridScenario, Section6_ClimbOutAfterSprayingStaysFrozenUntilTheDroneLeaves) {
  World w = twoTrees(0.6);
  CandGridMapper m(testConfig(), cal);
  scanPass(&m, w, cal, &t, 0.0, -14, 4, &rng);
  const auto before = m.publishedTrees();
  ASSERT_GE(before.size(), 1u);
  const TreeEntry target = *nearest(before, 0, 0);
  for (int i = 0; i < 10; ++i) { m.process(makeFrame(w, cal, t, MissionPhase::kSpraying, 0.0, 1.5, 7.0, &rng, {}, target.id)); t += 0.1; }
  m.markSprayed(target.id);
  // navigation already says PLANNING while the drone climbs over the hover point
  for (double z = 7.0; z <= 12.0; z += 0.2) {
    m.process(makeFrame(w, cal, t, MissionPhase::kPlanning, 0.0, 1.5, z, &rng));
    t += 0.1;
  }
  const auto after = m.publishedTrees();
  ASSERT_EQ(after.size(), before.size());
  for (std::size_t i = 0; i < after.size(); ++i) {
    EXPECT_DOUBLE_EQ(area(after[i].outline), area(before[i].outline));
  }
  EXPECT_GT(m.counters().at("frames_climb_out_frozen"), 0u);
}

TEST_F(CandGridScenario, Section6_SleepsWithHeartbeatAndUnknownPhaseBehavesAsScan) {
  World w;
  w.trees.push_back({0.0, 0.0, 2.2, 5.0, 1.0});
  CandGridMapper sleeping(testConfig(), cal);
  scanPass(&sleeping, w, cal, &t, 0.0, -12, 8, &rng, MissionPhase::kSpraying);
  EXPECT_TRUE(sleeping.publishedTrees().empty());
  EXPECT_GT(sleeping.counters().at("frames_skipped_sleeping"), 0u);
  EXPECT_EQ(sleeping.grid().tiles(), 0u) << "no processing while asleep";
  CandGridMapper unknown(testConfig(), cal);
  std::vector<std::string> logs;
  unknown.setLogger([&](const std::string& s) { logs.push_back(s); });
  scanPass(&unknown, w, cal, &t, 0.0, -12, 8, &rng, MissionPhase::kUnknown);
  EXPECT_EQ(unknown.publishedTrees().size(), 1u);
  ASSERT_FALSE(logs.empty());
  EXPECT_NE(logs[0].find("unknown"), std::string::npos);
}

TEST_F(CandGridScenario, Rule6_InvalidPoseHoldsEverythingAndSaysSo) {
  World w;
  w.trees.push_back({0.0, 0.0, 2.2, 5.0, 1.0});
  CandGridMapper m(testConfig(), cal);
  std::vector<std::string> logs;
  m.setLogger([&](const std::string& s) { logs.push_back(s); });
  FrameOpts o;
  o.pose_valid = false;
  scanPass(&m, w, cal, &t, 0.0, -12, 8, &rng, MissionPhase::kScan, o);
  EXPECT_TRUE(m.publishedTrees().empty());
  EXPECT_EQ(m.grid().tiles(), 0u) << "nothing may be frozen into the map before the pose is valid";
  EXPECT_GT(m.counters().at("frames_held_no_pose"), 0u);
  ASSERT_FALSE(logs.empty());
  EXPECT_NE(logs[0].find("pose"), std::string::npos);
}

TEST_F(CandGridScenario, Rule7_DroppedPointsAreCountedByReason) {
  World w;
  w.trees.push_back({0.0, 0.0, 2.2, 5.0, 1.0});
  Config c = testConfig();
  c.max_range = 12.0;  // ground at 15 m range is beyond: dropped by range
  CandGridMapper m(c, cal);
  scanPass(&m, w, cal, &t, 0.0, -4, 4, &rng);
  EXPECT_GT(m.counters().at("points_dropped_range"), 0u);
  EXPECT_GT(m.counters().at("points_dropped_invalid_disparity"), 0u);  // sky / no-return pixels
  // a tree taller than the drone: its top points are physically impossible -> dropped and counted
  World tall;
  tall.trees.push_back({0.1, 0.0, 2.2, 8.0, 1.0});
  Config c2 = testConfig();
  c2.min_range = 0.05;  // so the range gate does not catch these first
  CandGridMapper m2(c2, cal);
  for (int i = 0; i < 5; ++i) { m2.process(makeFrame(tall, cal, t, MissionPhase::kScan, 0.0, 0.0, 8.1, &rng)); t += 0.1; }
  EXPECT_GT(m2.counters().at("points_dropped_above_drone"), 0u);
  // a frame with boxes but without disparity is counted, not silently ignored
  FrameInput f = makeFrame(w, cal, t, MissionPhase::kScan, 0.0, 0.0, 15.0, &rng);
  f.disparity.clear();
  m2.process(f);
  EXPECT_EQ(m2.counters().at("frames_dropped_no_disparity"), 1u);
}

TEST_F(CandGridScenario, Section8_HoveringDoesNotInflateShapeComparedWithOnePass) {
  World w;
  w.trees.push_back({0.0, 0.0, 2.3, 5.0, 1.0});
  CandGridMapper pass(testConfig(), cal), hover(testConfig(), cal);
  double t1 = 0.0, t2 = 0.0;
  std::mt19937 r1(11), r2(11);
  scanPass(&pass, w, cal, &t1, 0.0, -12, 12, &r1);
  scanPass(&hover, w, cal, &t2, 0.0, -12, 0, &r2);
  for (int i = 0; i < 300; ++i) { hover.process(makeFrame(w, cal, t2, MissionPhase::kScan, 0.0, 0.0, 15.0, &r2)); t2 += 0.1; }
  scanPass(&hover, w, cal, &t2, 0.0, 0, 12, &r2);
  const auto a = pass.publishedTrees(), b = hover.publishedTrees();
  ASSERT_EQ(a.size(), 1u);
  ASSERT_EQ(b.size(), 1u);
  EXPECT_LT(area(b[0].outline), 1.25 * area(a[0].outline));
  EXPECT_NEAR(b[0].height, a[0].height, 0.3);
  // true canopy area pi r^2 = 16.6 m^2; the single pass already captures most of it
  EXPECT_GT(area(a[0].outline), 0.5 * M_PI * 2.3 * 2.3);
}
