#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "fake_world.hpp"

using namespace corridor_nav;
using corridor_nav::fake::FakeWorld;
using corridor_nav::fake::Sample;
using corridor_nav::fake::TruthTree;
using treemapper::TreeId;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.05;
constexpr double kMaxSim = 2000.0;

Field field30x24() {
  Field f;
  f.polygon = {{0, 0}, {30, 0}, {30, 24}, {0, 24}};
  f.heading_rad = 0.0;
  f.home = {0.0, -2.0};
  return f;
}

CameraFov fov90() { return {kPi / 2, kPi / 2}; }

/// Rows at the given y, four trees each at x = 6, 12, 18, 24 (6 m spacing), slightly different heights.
std::vector<TruthTree> grid(const std::vector<double>& rows) {
  std::vector<TruthTree> out;
  for (double y : rows) {
    for (double x : {6.0, 12.0, 18.0, 24.0}) {
      TruthTree t;
      t.x = x;
      t.y = y;
      t.r = 1.5;
      t.height = 4.0 + 0.25 * static_cast<double>(out.size() % 3);
      out.push_back(t);
    }
  }
  return out;
}

FakeWorld world(const Config& cfg, std::vector<TruthTree> trees) {
  FakeWorld w;
  w.cfg = cfg;
  w.trees = std::move(trees);
  w.drone = {field30x24().home.x, field30x24().home.y, 0.0};
  return w;
}

template <typename T>
std::vector<T> collapse(const std::vector<T>& v) {
  std::vector<T> out;
  for (const T& x : v) {
    if (out.empty() || !(out.back() == x)) out.push_back(x);
  }
  return out;
}

std::vector<std::string> phases(const FakeWorld& w) {
  std::vector<std::string> out;
  for (const Sample& s : w.samples) out.push_back(s.phase);
  return out;
}

std::uint64_t counter(const Navigator& nav, const std::string& name) {
  const auto it = nav.counters().find(name);
  return it == nav.counters().end() ? 0u : it->second;
}

}  // namespace

TEST(Navigator, FullCorridorMissionSpraysEveryTreeOnce) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  EXPECT_EQ(nav.state(), NavState::kDone);
  EXPECT_EQ(nav.corridors().size(), 3u);
  for (std::size_t i = 0; i < w.trees.size(); ++i) EXPECT_EQ(w.trees[i].spray_calls, 1) << "tree " << i + 1;
  EXPECT_EQ(w.requests.size(), w.trees.size());
  EXPECT_EQ(counter(nav, "trees_skipped"), 0u);
  EXPECT_EQ(nav.status().skipped, 0u);
  EXPECT_EQ(nav.status().sprayed, 12u);
  EXPECT_EQ(nav.status().remaining, 0u);
  EXPECT_GE(w.min_clearance, cfg.min_clearance);
  // landed back home
  EXPECT_NEAR(w.drone.x, 0.0, cfg.xy_tol);
  EXPECT_NEAR(w.drone.y, -2.0, cfg.xy_tol);
  EXPECT_NEAR(w.drone.z, 0.0, cfg.z_tol);
}

TEST(Navigator, PhaseSequencePerCorridor) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));

  std::vector<std::string> expected = {"IDLE"};
  for (int c = 0; c < 3; ++c) {
    for (const char* p : {"SCAN_APPROACH", "SCAN", "PLANNING"}) expected.push_back(p);
    for (int i = 0; i < 4; ++i) {
      expected.push_back("HOPPING");
      expected.push_back("SPRAYING");
    }
    expected.push_back("HOPPING");   // waiting for set_tree_sprayed of the last tree
  }
  expected.push_back("RETURN");
  expected.push_back("IDLE");        // DONE, landed
  EXPECT_EQ(collapse(phases(w)), expected);

  bool planned = false;
  for (const std::string& p : phases(w)) {
    if (p == "PLANNING") planned = true;
    if (p == "SCAN_APPROACH") planned = false;
    EXPECT_FALSE(p == "SCAN" && planned) << "SCAN after PLANNING without SCAN_APPROACH";
  }
  for (const Sample& s : w.samples) {
    if (s.phase != "HOPPING" && s.phase != "SPRAYING") {
      EXPECT_EQ(s.target, treemapper::kNoTree) << s.phase;
    }
  }
}

TEST(Navigator, NozzleOnlyInDoseWithinTolerance) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  int open = 0;
  for (const Sample& s : w.samples) {
    if (!s.nozzle) continue;
    ++open;
    EXPECT_EQ(s.state, NavState::kDose) << "t=" << s.t;
    EXPECT_LE(s.xy_err, cfg.xy_tol) << "t=" << s.t;
  }
  EXPECT_GT(open, 0);
}

TEST(Navigator, SettleHappensInScan) {
  const Config cfg;
  std::vector<TruthTree> trees = grid({6, 12, 18});
  TruthTree late;            // near corridor 0's exit (30, 4); the map confirms it 1 s into the hover
  late.x = 28.0;
  late.y = 3.0;
  late.r = 1.2;
  late.height = 4.2;
  late.settle_only = true;
  late.dwell_needed = 20;
  trees.push_back(late);
  const TreeId late_id = FakeWorld::idOf(trees.size() - 1);
  FakeWorld w = world(cfg, trees);
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));

  const TruthTree& t = w.trees.back();
  ASSERT_TRUE(t.published);
  EXPECT_EQ(t.published_state, NavState::kScanSettle);
  EXPECT_EQ(t.published_phase, "SCAN");
  EXPECT_EQ(t.spray_calls, 1);
  EXPECT_EQ(counter(nav, "late_trees"), 0u);
  int targeted = 0;
  for (const Sample& s : w.samples) {
    if (s.target != late_id) continue;
    ++targeted;
    EXPECT_FALSE(s.cleanup);
    EXPECT_EQ(s.corridor, 0);
  }
  EXPECT_GT(targeted, 0);
}

TEST(Navigator, EmptyCorridorGoesStraightToNext) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 18}));   // corridor 1 (y 8..16) holds no tree
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  for (const TruthTree& t : w.trees) EXPECT_EQ(t.spray_calls, 1);

  std::vector<std::pair<NavState, int>> seq;
  for (const Sample& s : w.samples) seq.emplace_back(s.state, s.corridor);
  seq = collapse(seq);
  const auto it = std::find(seq.begin(), seq.end(), std::make_pair(NavState::kScanSettle, 1));
  ASSERT_NE(it, seq.end());
  ASSERT_LT(std::distance(seq.begin(), it) + 2, static_cast<std::ptrdiff_t>(seq.size()));
  EXPECT_EQ(*(it + 1), std::make_pair(NavState::kLock, 1));
  EXPECT_EQ(*(it + 2), std::make_pair(NavState::kTransit, 2));
  for (const Sample& s : w.samples) EXPECT_FALSE(s.corridor == 1 && s.state == NavState::kGlide);
}

TEST(Navigator, SprayedTreeNeverTargeted) {
  const Config cfg;
  std::vector<TruthTree> trees = grid({6, 12, 18});
  trees[5].sprayed = true;
  const TreeId done_id = FakeWorld::idOf(5);
  FakeWorld w = world(cfg, trees);
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  for (const Sample& s : w.samples) EXPECT_NE(s.target, done_id);
  EXPECT_EQ(w.trees[5].spray_calls, 0);
  for (std::size_t i = 0; i < w.trees.size(); ++i) {
    if (i != 5) {
      EXPECT_EQ(w.trees[i].spray_calls, 1) << "tree " << i + 1;
    }
  }
  EXPECT_EQ(nav.status().sprayed, 11u);
}

TEST(Navigator, FieldFirstScansAllThenSprays) {
  Config cfg;
  cfg.strategy = Strategy::kFieldFirst;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  for (const TruthTree& t : w.trees) EXPECT_EQ(t.spray_calls, 1);
  EXPECT_GE(w.min_clearance, cfg.min_clearance);

  const std::vector<std::string> ph = phases(w);
  const auto first_planning = std::find(ph.begin(), ph.end(), "PLANNING");
  ASSERT_NE(first_planning, ph.end());
  EXPECT_EQ(std::find(first_planning, ph.end(), "SCAN"), ph.end());
  const std::vector<std::string> c = collapse(ph);
  EXPECT_EQ(std::count(c.begin(), c.end(), "PLANNING"), 1);

  // boustrophedon: corridor 1 is scanned exit -> entry (against the heading, x decreasing)
  std::vector<double> xs;
  for (const Sample& s : w.samples) {
    if (s.state == NavState::kScanOut && s.corridor == 1) xs.push_back(s.setpoint.x);
  }
  ASSERT_GE(xs.size(), 2u);
  EXPECT_GT(xs.front(), xs.back() + 20.0);
}
