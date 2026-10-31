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

// ------------------------------------------------------------------------------------------------------------------
// Task 7: holds, clearance guard, dose retries, ID re-resolution, re-plans, cleanup and abort

namespace {

/// Index of the first sample at or after `from` that satisfies `pred`, or samples.size().
template <typename Pred>
std::size_t findSample(const FakeWorld& w, Pred pred, std::size_t from = 0) {
  for (std::size_t i = from; i < w.samples.size(); ++i) {
    if (pred(w.samples[i])) return i;
  }
  return w.samples.size();
}

std::size_t countRequests(const FakeWorld& w, TreeId id) {
  return static_cast<std::size_t>(std::count(w.requests.begin(), w.requests.end(), id));
}

/// Number of ticks on which the navigator entered DOSE on `id` (with the given cleanup flag, if any).
int doseEntries(const FakeWorld& w, TreeId id, int cleanup = -1) {
  int n = 0;
  for (std::size_t i = 1; i < w.samples.size(); ++i) {
    const Sample &a = w.samples[i - 1], &b = w.samples[i];
    if (b.state != NavState::kDose || b.target != id || (a.state == NavState::kDose && a.target == id)) continue;
    if (cleanup < 0 || b.cleanup == (cleanup == 1)) ++n;
  }
  return n;
}

bool samePoint(const Point3& a, const Point3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

/// An L: the 30 x 24 rectangle without its top-right 18 x 8 corner (x > 12, y > 16).
Field fieldL() {
  Field f;
  f.polygon = {{0, 0}, {30, 0}, {30, 16}, {12, 16}, {12, 24}, {0, 24}};
  f.heading_rad = 0.0;
  f.home = {0.0, -2.0};
  return f;
}

TruthTree tree(double x, double y, double r, double height) {
  TruthTree t;
  t.x = x;
  t.y = y;
  t.r = r;
  t.height = height;
  return t;
}

}  // namespace

TEST(Navigator, MapStaleHoldsThenReturns) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  double stop_t = -1.0;
  w.before_step = [&](FakeWorld&, const Navigator& n, Inputs& in) {
    if (stop_t < 0.0 && n.state() == NavState::kGlide && n.status().sprayed >= 1) stop_t = in.t;   // mid-GLIDE
    if (stop_t >= 0.0) in.map.reset();
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  ASSERT_GE(stop_t, 0.0);

  const std::size_t h = findSample(w, [](const Sample& s) { return s.state == NavState::kHold; });
  ASSERT_LT(h, w.samples.size());
  ASSERT_GT(h, 0u);
  // the last map came one tick before stop_t
  EXPECT_LE(w.samples[h].t - stop_t, cfg.map_stale_s + kDt + 1e-9);
  EXPECT_GE(w.samples[h].t - stop_t, cfg.map_stale_s - kDt - 1e-9);
  const Sample& before = w.samples[h - 1];
  const Point3 hold = w.samples[h].pos;
  std::size_t i = h;
  for (; i < w.samples.size() && w.samples[i].state == NavState::kHold; ++i) {
    const Sample& s = w.samples[i];
    EXPECT_FALSE(s.nozzle);
    EXPECT_EQ(s.phase, before.phase);     // unchanged
    EXPECT_EQ(s.target, before.target);   // unchanged
    EXPECT_TRUE(samePoint(s.setpoint, hold)) << "t=" << s.t;
  }
  ASSERT_LT(i, w.samples.size());
  EXPECT_NEAR(w.samples[i].t - w.samples[h].t, cfg.hold_timeout_s, kDt + 1e-9);
  EXPECT_EQ(w.samples[i].state, NavState::kReturn);
  EXPECT_EQ(w.samples[i].phase, "RETURN");
  EXPECT_EQ(counter(nav, "hold_map_stale"), 1u);
  EXPECT_EQ(counter(nav, "hold_timeouts"), 1u);
}

TEST(Navigator, PoseLossHolds) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  double freeze_t = -1.0, invalid_t = -1.0;
  treemapper::DronePose frozen;
  TreeId glide_target = treemapper::kNoTree;
  w.before_step = [&](FakeWorld&, const Navigator& n, Inputs& in) {
    if (freeze_t < 0.0 && n.state() == NavState::kGlide && n.status().sprayed >= 1) {
      freeze_t = in.t;   // the pose stamp stops advancing for 1 s, mid-GLIDE
      frozen = in.pose;
      glide_target = n.status().target;
    }
    if (freeze_t >= 0.0 && in.t < freeze_t + 1.0) in.pose = frozen;
    if (invalid_t < 0.0 && n.state() == NavState::kDose && n.status().sprayed >= 3) invalid_t = in.t;
    if (invalid_t >= 0.0 && in.t < invalid_t + 0.5) in.pose.valid = false;   // no pose in TF for 0.5 s
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  ASSERT_GE(freeze_t, 0.0);
  ASSERT_GE(invalid_t, 0.0);

  const std::size_t h = findSample(w, [](const Sample& s) { return s.state == NavState::kHold; });
  ASSERT_LT(h, w.samples.size());
  EXPECT_LE(w.samples[h].t - freeze_t, cfg.pose_max_age_s + kDt + 1e-9);
  const std::size_t r = findSample(w, [](const Sample& s) { return s.state != NavState::kHold; }, h);
  ASSERT_LT(r, w.samples.size());
  EXPECT_EQ(w.samples[r].state, NavState::kGlide);   // resumed the previous state
  EXPECT_EQ(w.samples[r].target, glide_target);
  for (std::size_t i = h; i < r; ++i) EXPECT_TRUE(samePoint(w.samples[i].setpoint, w.samples[h].setpoint));

  const std::size_t h2 = findSample(w, [&](const Sample& s) { return s.state == NavState::kHold && s.t >= invalid_t; });
  ASSERT_LT(h2, w.samples.size());
  EXPECT_NEAR(w.samples[h2].t, invalid_t, 1e-9);   // valid = false holds at once
  EXPECT_EQ(counter(nav, "hold_no_pose"), 2u);
  EXPECT_EQ(counter(nav, "hold_timeouts"), 0u);
  for (const TruthTree& t : w.trees) EXPECT_TRUE(t.sprayed);
  for (const Sample& s : w.samples) EXPECT_FALSE(s.state == NavState::kHold && s.nozzle);
}

TEST(Navigator, GuardClimbsAndBlocksDescent) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  double climb_t = -1.0, blind_t = -1.0, dose_climb_t = -1.0;
  w.before_step = [&](FakeWorld&, const Navigator& n, Inputs& in) {
    const Status st = n.status();
    // 1) 1 s of 1.0 m clearance in a low GLIDE (after the first tree)
    if (climb_t < 0.0 && n.state() == NavState::kGlide && st.sprayed >= 1) climb_t = in.t;
    if (climb_t >= 0.0 && in.t < climb_t + 1.0) in.clearance.metres = 1.0;
    // 2) 1 s blind at the start of corridor 1's first glide (a long descent from z_scan)
    if (blind_t < 0.0 && n.state() == NavState::kGlide && st.corridor == 1) blind_t = in.t;
    if (blind_t >= 0.0 && in.t < blind_t + 1.0) {
      in.clearance.valid = false;
      in.clearance.metres = NAN;
    }
    // 3) 0.5 s of 1.0 m clearance while spraying in corridor 2
    if (dose_climb_t < 0.0 && n.state() == NavState::kDose && st.corridor == 2 && w.samples.back().nozzle) {
      dose_climb_t = in.t;
    }
    if (dose_climb_t >= 0.0 && in.t < dose_climb_t + 0.5) in.clearance.metres = 1.0;
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  ASSERT_GE(climb_t, 0.0);
  ASSERT_GE(blind_t, 0.0);
  ASSERT_GE(dose_climb_t, 0.0);

  // climb: straight up from the current position at max_speed_z, nozzle closed
  std::size_t i = findSample(w, [&](const Sample& s) { return s.t >= climb_t - 1e-9; });
  ASSERT_LT(i + 20, w.samples.size());
  for (std::size_t k = i; k < i + 20; ++k) {
    const Sample& s = w.samples[k];
    EXPECT_FALSE(s.nozzle);
    EXPECT_DOUBLE_EQ(s.setpoint.x, s.pos.x);
    EXPECT_DOUBLE_EQ(s.setpoint.y, s.pos.y);
    EXPECT_GT(s.setpoint.z, s.pos.z);
    if (k > i) {
      EXPECT_NEAR(s.pos.z - w.samples[k - 1].pos.z, cfg.max_speed_z * kDt, 1e-9);
    }
  }
  // blind: the setpoint never goes below the drone, and the drone does not descend
  i = findSample(w, [&](const Sample& s) { return s.t >= blind_t - 1e-9; });
  ASSERT_LT(i + 20, w.samples.size());
  const double z0 = w.samples[i].pos.z;
  for (std::size_t k = i; k < i + 20; ++k) {
    EXPECT_GE(w.samples[k].setpoint.z, w.samples[k].pos.z) << "t=" << w.samples[k].t;
    EXPECT_GE(w.samples[k].pos.z, z0 - 1e-9);
  }
  EXPECT_LT(w.samples[i + 40].pos.z, z0 - 1.0);   // it descends once the guard sees again
  // climb during a dose closes the nozzle
  i = findSample(w, [&](const Sample& s) { return s.t >= dose_climb_t - 1e-9; });
  for (std::size_t k = i; k < i + 10; ++k) EXPECT_FALSE(w.samples[k].nozzle) << "t=" << w.samples[k].t;

  EXPECT_EQ(counter(nav, "guard_triggers"), 2u);   // one per climb episode
  EXPECT_EQ(counter(nav, "guard_blind"), 1u);
  EXPECT_GE(counter(nav, "glide_replans"), 1u);    // the glide is re-planned when the guard releases
  for (const TruthTree& t : w.trees) EXPECT_TRUE(t.sprayed);
}

TEST(Navigator, DoseRetryThenSkip) {
  const Config cfg;
  std::vector<TruthTree> trees = grid({6, 12, 18});
  trees[1].dose_push = 0.5;   // the drone is held 0.5 m off tree 2 for every dose
  const TreeId bad = FakeWorld::idOf(1);
  FakeWorld w = world(cfg, trees);
  Navigator nav(cfg, field30x24(), fov90());
  long skipped_at_cleanup = -1;
  w.before_step = [&](FakeWorld&, const Navigator& n, Inputs&) {
    const Status st = n.status();
    if (skipped_at_cleanup < 0 && st.cleanup && st.target == bad) {
      skipped_at_cleanup = static_cast<long>(counter(n, "trees_skipped"));
    }
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));

  EXPECT_EQ(skipped_at_cleanup, 1);                // one retry, then skipped, then into cleanup
  EXPECT_EQ(doseEntries(w, bad, 0), 2);           // dose + retry in its corridor
  EXPECT_EQ(doseEntries(w, bad, 1), 2);           // dose + retry in cleanup, once
  EXPECT_EQ(counter(nav, "dose_retries"), 2u);
  EXPECT_EQ(counter(nav, "trees_skipped"), 2u);   // skipped twice: stays unsprayed
  EXPECT_EQ(countRequests(w, bad), 0u);
  EXPECT_FALSE(w.trees[1].sprayed);
  EXPECT_EQ(nav.status().skipped, 1u);
  EXPECT_EQ(nav.status().sprayed, 11u);
  for (std::size_t k = 0; k < w.trees.size(); ++k) {
    if (k != 1) {
      EXPECT_EQ(w.trees[k].spray_calls, 1) << "tree " << k + 1;
    }
  }
  // the nozzle xy gate: closed on every DOSE tick off by more than xy_tol
  int off = 0;
  for (const Sample& s : w.samples) {
    if (s.state != NavState::kDose || !(s.xy_err > cfg.xy_tol)) continue;
    ++off;
    EXPECT_FALSE(s.nozzle) << "t=" << s.t;
  }
  EXPECT_GT(off, 0);
}

TEST(Navigator, VanishedIdIsReresolved) {
  const Config cfg;
  const TreeId old_id = FakeWorld::idOf(4);   // corridor 1, sprayed last there (x = 6)
  {
    FakeWorld w = world(cfg, grid({6, 12, 18}));
    Navigator nav(cfg, field30x24(), fov90());
    bool merged = false;
    w.before_step = [&](FakeWorld& fw, const Navigator& n, Inputs&) {
      if (!merged && n.state() == NavState::kGlide && n.status().corridor == 1) {
        fw.trees[4].published_id = 50;   // the map replaces 5 with 50, same outline
        merged = true;
      }
    };
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    ASSERT_TRUE(merged);
    EXPECT_EQ(counter(nav, "ids_reresolved"), 1u);
    EXPECT_EQ(counter(nav, "ids_dropped_vanished"), 0u);
    EXPECT_EQ(countRequests(w, old_id), 0u);
    EXPECT_EQ(countRequests(w, 50), 1u);
    EXPECT_TRUE(w.trees[4].sprayed);
    EXPECT_LT(findSample(w, [](const Sample& s) { return s.target == 50 && s.state == NavState::kGlide; }),
              w.samples.size());
    for (const TruthTree& t : w.trees) EXPECT_EQ(t.spray_calls, 1);
  }
  {
    FakeWorld w = world(cfg, grid({6, 12, 18}));
    Navigator nav(cfg, field30x24(), fov90());
    std::size_t gone_at = 0;
    w.before_step = [&](FakeWorld& fw, const Navigator& n, Inputs&) {
      if (gone_at == 0 && n.state() == NavState::kGlide && n.status().corridor == 1) {
        fw.trees[4].hidden = true;   // replaced by nothing
        gone_at = fw.samples.size();
      }
    };
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    ASSERT_GT(gone_at, 0u);
    EXPECT_EQ(counter(nav, "ids_dropped_vanished"), 1u);
    EXPECT_EQ(counter(nav, "ids_reresolved"), 0u);
    EXPECT_EQ(findSample(w, [&](const Sample& s) { return s.target == old_id; }, gone_at), w.samples.size());
    for (std::size_t k = 0; k < w.trees.size(); ++k) {
      if (k != 4) {
        EXPECT_EQ(w.trees[k].spray_calls, 1) << "tree " << k + 1;
      }
    }
  }
}

TEST(Navigator, TargetVanishingMidGlideIsReresolvedOrDropped) {
  const Config cfg;
  const TreeId id = FakeWorld::idOf(4);
  for (const bool replaced : {true, false}) {
    FakeWorld w = world(cfg, grid({6, 12, 18}));
    Navigator nav(cfg, field30x24(), fov90());
    std::size_t at = 0;
    w.before_step = [&](FakeWorld& fw, const Navigator& n, Inputs& in) {
      if (at == 0 && n.state() == NavState::kGlide && n.status().target == id) {
        if (replaced) {
          fw.trees[4].published_id = 50;
        } else {
          fw.trees[4].hidden = true;
        }
        in.map = MapSnapshot{in.t, fw.map()};
        at = fw.samples.size();
      }
    };
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    ASSERT_GT(at, 0u);
    EXPECT_EQ(counter(nav, "ids_reresolved"), replaced ? 1u : 0u);
    EXPECT_EQ(counter(nav, "ids_dropped_vanished"), replaced ? 0u : 1u);
    EXPECT_EQ(counter(nav, "glide_replans"), replaced ? 1u : 0u);
    EXPECT_EQ(w.samples[at].target, replaced ? 50 : treemapper::kNoTree) << "replaced " << replaced;
    EXPECT_EQ(w.trees[4].sprayed, replaced);
    for (std::size_t k = 0; k < w.trees.size(); ++k) {
      if (k != 4) {
        EXPECT_EQ(w.trees[k].spray_calls, 1) << "tree " << k + 1;
      }
    }
  }
}

TEST(Navigator, GuardGivesUpAStuckAttempt) {
  // the guard reads 1.0 m whenever tree 2 is the target: the climb is capped at scan altitude, and an attempt with no
  // clear reading for hold_timeout_s fails like an invalid dose (one retry, skip, once more from cleanup)
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  const TreeId bad = FakeWorld::idOf(1);
  Navigator nav(cfg, field30x24(), fov90());
  w.before_step = [&](FakeWorld&, const Navigator& n, Inputs& in) {
    if (n.status().target == bad) in.clearance.metres = 1.0;
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  EXPECT_EQ(counter(nav, "guard_triggers"), 2u);   // one long episode per approach (corridor, cleanup)
  EXPECT_EQ(counter(nav, "dose_retries"), 2u);
  EXPECT_EQ(counter(nav, "trees_skipped"), 2u);
  EXPECT_EQ(countRequests(w, bad), 0u);
  EXPECT_EQ(nav.status().skipped, 1u);
  for (std::size_t k = 0; k < w.trees.size(); ++k) {
    if (k != 1) {
      EXPECT_TRUE(w.trees[k].sprayed) << "tree " << k + 1;
    }
  }
  for (const Sample& s : w.samples) {
    EXPECT_LE(s.setpoint.z, cfg.z_scan + 1e-9) << "t=" << s.t;
    if (s.target == bad) {
      EXPECT_FALSE(s.nozzle);
    }
  }
}

TEST(Navigator, GuardGivesUpAfterThreeClimbs) {
  // short low-clearance readings near tree 2 make the guard climb, release and re-plan, again and again: the third
  // climb episode of one attempt fails it
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  const TreeId bad = FakeWorld::idOf(1);
  Navigator nav(cfg, field30x24(), fov90());
  w.before_step = [&](FakeWorld&, const Navigator& n, Inputs& in) {
    if (n.status().target == bad && std::fmod(in.t, 1.0) < 0.3) in.clearance.metres = 1.0;
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  // 4 attempts (glide + retry, twice), each ended by its third climb or, if it got that far, an invalid dose; an
  // attempt that never reached DOSE can only have been ended by the climb count (clear readings come every second)
  EXPECT_LE(counter(nav, "guard_triggers"), 12u);
  EXPECT_GE(counter(nav, "guard_triggers"), 8u);
  EXPECT_LT(doseEntries(w, bad), 4);
  EXPECT_EQ(counter(nav, "dose_retries"), 2u);
  EXPECT_EQ(counter(nav, "trees_skipped"), 2u);
  EXPECT_GE(counter(nav, "glide_replans"), 4u);     // releases re-plan
  EXPECT_EQ(countRequests(w, bad), 0u);
  for (std::size_t k = 0; k < w.trees.size(); ++k) {
    if (k != 1) {
      EXPECT_TRUE(w.trees[k].sprayed) << "tree " << k + 1;
    }
  }
}

TEST(Navigator, SprayMarkFailureReresolvesThenContinues) {
  const Config cfg;
  const TreeId id = FakeWorld::idOf(1);
  {  // ok = false once (the id was merged away into 50): re-resolve, retry on 50, done
    std::vector<TruthTree> trees = grid({6, 12, 18});
    trees[1].mark_fail = 1;
    trees[1].rename_on_fail = 50;
    FakeWorld w = world(cfg, trees);
    Navigator nav(cfg, field30x24(), fov90());
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    EXPECT_EQ(countRequests(w, id), 1u);
    EXPECT_EQ(countRequests(w, 50), 1u);
    EXPECT_EQ(counter(nav, "ids_reresolved"), 1u);
    EXPECT_EQ(counter(nav, "spray_mark_failed"), 0u);
    EXPECT_TRUE(w.trees[1].sprayed);
    EXPECT_EQ(nav.status().sprayed, 12u);
  }
  {  // the retry on the new id fails too: spray_mark_failed, the mission goes on
    std::vector<TruthTree> trees = grid({6, 12, 18});
    trees[1].mark_fail = 2;
    trees[1].rename_on_fail = 50;
    FakeWorld w = world(cfg, trees);
    Navigator nav(cfg, field30x24(), fov90());
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    EXPECT_EQ(countRequests(w, id), 1u);
    EXPECT_EQ(countRequests(w, 50), 1u);   // never retried later, not even in cleanup
    EXPECT_EQ(counter(nav, "ids_reresolved"), 1u);
    EXPECT_EQ(counter(nav, "spray_mark_failed"), 1u);
    for (std::size_t k = 0; k < w.trees.size(); ++k) {
      if (k != 1) {
        EXPECT_TRUE(w.trees[k].sprayed) << "tree " << k + 1;
      }
    }
    EXPECT_EQ(nav.status().sprayed, 11u);
  }
  {  // no answer: 2 s timeouts, service_retries retries, then HOLD(service) until it answers
    std::vector<TruthTree> trees = grid({6, 12, 18});
    trees[1].mark_silent = 1 + cfg.service_retries + 0;   // the first request and every retry go unanswered
    FakeWorld w = world(cfg, trees);
    Navigator nav(cfg, field30x24(), fov90());
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    EXPECT_EQ(counter(nav, "spray_mark_retries"), static_cast<std::uint64_t>(cfg.service_retries));
    EXPECT_EQ(counter(nav, "hold_service"), 1u);
    EXPECT_EQ(countRequests(w, id), static_cast<std::size_t>(cfg.service_retries) + 2u);   // + one sent in HOLD
    const std::size_t h = findSample(w, [](const Sample& s) { return s.state == NavState::kHold; });
    ASSERT_LT(h, w.samples.size());
    EXPECT_EQ(w.samples[h].phase, "HOPPING");
    EXPECT_EQ(w.samples[h].target, id);
    EXPECT_TRUE(w.trees[1].sprayed);
    EXPECT_EQ(nav.status().sprayed, 12u);
    EXPECT_EQ(counter(nav, "hold_timeouts"), 0u);
  }
}

TEST(Navigator, ReplanOnHeightChange) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  const TreeId id = FakeWorld::idOf(2);
  const double new_height = w.trees[2].height + 0.5;
  Navigator nav(cfg, field30x24(), fov90());
  bool raised = false;
  w.before_step = [&](FakeWorld& fw, const Navigator& n, Inputs& in) {
    if (!raised && n.state() == NavState::kGlide && n.status().target == id) {
      fw.trees[2].height = new_height;
      in.map = MapSnapshot{in.t, fw.map()};
      raised = true;
    }
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  ASSERT_TRUE(raised);
  EXPECT_EQ(counter(nav, "glide_replans"), 1u);
  const std::size_t d = findSample(w, [&](const Sample& s) { return s.state == NavState::kDose && s.target == id; });
  ASSERT_LT(d, w.samples.size());
  EXPECT_NEAR(w.samples[d].setpoint.z, new_height + cfg.spray_clearance, 1e-9);
  EXPECT_NEAR(w.samples[d - 1].setpoint.z, new_height + cfg.spray_clearance, 1e-9);   // the glide's final setpoint
  EXPECT_TRUE(w.trees[2].sprayed);
}

TEST(Navigator, LateTreeSprayedInCleanup) {
  const Config cfg;
  std::vector<TruthTree> trees = grid({6, 12, 18});
  trees.push_back(tree(27.5, 2.0, 1.2, 4.1));
  trees.back().publish_in_corridor = 1;   // corridor 0's tree, published while corridor 1 scans
  const TreeId late = FakeWorld::idOf(trees.size() - 1);
  FakeWorld w = world(cfg, trees);
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  ASSERT_TRUE(w.trees.back().published);
  EXPECT_EQ(counter(nav, "late_trees"), 1u);
  EXPECT_EQ(w.trees.back().spray_calls, 1);
  int dosed = 0;
  for (const Sample& s : w.samples) {
    if (s.target != late) continue;
    EXPECT_TRUE(s.cleanup) << "t=" << s.t;
    if (s.state == NavState::kDose) ++dosed;
  }
  EXPECT_GT(dosed, 0);
  for (const TruthTree& t : w.trees) EXPECT_TRUE(t.sprayed);
}

TEST(Navigator, AbortClosesNozzleAndReturns) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  double abort_t = -1.0;
  w.before_step = [&](FakeWorld& fw, const Navigator&, Inputs& in) {
    if (abort_t < 0.0 && !fw.samples.empty() && fw.samples.back().nozzle) abort_t = in.t;
    in.abort_requested = abort_t >= 0.0 && in.t < abort_t + 1e-9;
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  ASSERT_GE(abort_t, 0.0);
  const std::size_t a = findSample(w, [&](const Sample& s) { return s.t >= abort_t - 1e-9; });
  ASSERT_LT(a + 1, w.samples.size());
  EXPECT_TRUE(w.samples[a - 1].nozzle);
  EXPECT_FALSE(w.samples[a].nozzle);
  EXPECT_EQ(w.samples[a].phase, "ERROR");
  EXPECT_EQ(w.samples[a].target, treemapper::kNoTree);
  EXPECT_EQ(w.samples[a + 1].phase, "RETURN");
  EXPECT_EQ(w.samples[a + 1].state, NavState::kReturn);
  for (std::size_t k = a + 1; k < w.samples.size(); ++k) {
    EXPECT_FALSE(w.samples[k].nozzle);
    EXPECT_TRUE(w.samples[k].phase == "RETURN" || w.samples[k].phase == "IDLE") << w.samples[k].phase;
  }
  EXPECT_EQ(counter(nav, "aborts"), 1u);
  EXPECT_NEAR(w.drone.z, 0.0, cfg.z_tol);   // landed
}

TEST(Navigator, GlideLeavingFieldClimbsOverTheNotch) {
  const Config cfg;
  std::vector<TruthTree> trees;
  for (double x : {6.0, 12.0, 18.0, 24.0}) trees.push_back(tree(x, 6.0, 1.5, 4.0));   // corridor 0
  for (double x : {6.0, 12.0, 18.0}) trees.push_back(tree(x, 12.0, 1.5, 4.2));        // corridor 1
  for (double x : {4.0, 9.0}) trees.push_back(tree(x, 20.0, 1.5, 4.1));               // corridor 2 (x 0..12)
  trees.push_back(tree(26.0, 14.0, 1.2, 4.0));   // corridor 1, seen only while corridor 2 scans
  trees.back().publish_in_corridor = 2;
  const TreeId late = FakeWorld::idOf(trees.size() - 1);
  FakeWorld w = world(cfg, trees);
  const Field field = fieldL();
  Navigator nav(cfg, field, fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  ASSERT_EQ(nav.corridors().size(), 3u);
  for (const TruthTree& t : w.trees) EXPECT_TRUE(t.sprayed);

  // the cleanup glide from corridor 2 to the late tree crosses the notch: up-over at home z + z_scan
  EXPECT_EQ(counter(nav, "glide_outside_field_climbs"), 1u);
  int outside = 0;
  for (const Sample& s : w.samples) {
    if (s.state != NavState::kGlide) continue;
    if (distanceToPolygon(field.polygon, {s.setpoint.x, s.setpoint.y}) <= cfg.xy_tol) continue;
    ++outside;
    EXPECT_EQ(s.target, late);
    EXPECT_NEAR(s.setpoint.z, cfg.z_scan, 1e-9) << "t=" << s.t;
  }
  EXPECT_GT(outside, 0);
}

TEST(Navigator, SettleRestartsOnEachNewTree) {
  // two settle-only trees, confirmed 1.0 s and 2.5 s into the hover: the second after the first settle_s would have
  // expired, so only a restart on the first keeps the corridor open for it
  std::vector<TruthTree> trees = grid({12, 18});
  trees.push_back(tree(28.5, 2.0, 1.0, 4.0));
  trees.back().settle_only = true;
  trees.back().dwell_needed = 20;
  trees.push_back(tree(28.5, 6.0, 1.0, 4.2));
  trees.back().settle_only = true;
  trees.back().dwell_needed = 50;
  for (const double timeout : {8.0, 3.0}) {
    Config cfg;
    cfg.settle_timeout_s = timeout;
    FakeWorld w = world(cfg, trees);
    Navigator nav(cfg, field30x24(), fov90());
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    for (std::size_t k = trees.size() - 2; k < trees.size(); ++k) {
      EXPECT_EQ(w.trees[k].published_state, NavState::kScanSettle);
      const TreeId id = FakeWorld::idOf(k);
      const std::size_t g = findSample(w, [&](const Sample& s) { return s.target == id; });
      ASSERT_LT(g, w.samples.size());
      EXPECT_EQ(w.samples[g].corridor, 0);
      EXPECT_FALSE(w.samples[g].cleanup);
    }
    EXPECT_EQ(counter(nav, "late_trees"), 0u);
    // the second tree came 2.5 s in: it ends the settle at 4.5 s, past a 3 s timeout
    EXPECT_EQ(counter(nav, "settle_timeouts"), timeout < 4.0 ? 1u : 0u) << "timeout " << timeout;
  }
}

TEST(Navigator, InvalidTreesDroppedAtIngest) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  treemapper::TreeEntry nan_height = w.entry(0);
  nan_height.id = 100;
  nan_height.height = NAN;
  treemapper::TreeEntry two_points = w.entry(0);
  two_points.id = 101;
  two_points.outline.resize(2);
  w.extra = {nan_height, two_points};
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  EXPECT_EQ(counter(nav, "trees_dropped_invalid"), 2u);   // once per id, although published on every tick
  for (const Sample& s : w.samples) EXPECT_TRUE(s.target != 100 && s.target != 101);
  for (const TruthTree& t : w.trees) EXPECT_EQ(t.spray_calls, 1);
}

TEST(Navigator, PlanningFailureAborts) {
  const Config cfg;
  FakeWorld w = world(cfg, grid({6}));
  Navigator nav(cfg, field30x24(), CameraFov{0.1, 0.1});   // footprint far too narrow: coverage impossible
  for (int k = 0; k < 40; ++k) w.tick(nav, kDt);
  EXPECT_EQ(nav.state(), NavState::kAbort);
  EXPECT_EQ(w.samples.back().phase, "ERROR");
  EXPECT_EQ(w.samples.back().target, treemapper::kNoTree);
  EXPECT_FALSE(w.samples.back().nozzle);
  EXPECT_NEAR(w.samples.back().setpoint.z, 0.0, 1e-9);   // stays on the ground
  EXPECT_FALSE(nav.status().warnings.empty());
}

// ------------------------------------------------------------------------------------------------------------------
// Task 7 review round 1 (ported from the reviewer's Rev7 probes) and the coverage gaps it listed

TEST(Navigator, TargetVanishingDuringDoseIsResolvedBeforeTheRetry) {
  // the dose on tree 2 is invalid (drone held 0.5 m off) and the tree leaves the map during it: the retry must not
  // glide to a tree that is gone (was a null dereference)
  const Config cfg;
  const TreeId bad = FakeWorld::idOf(1);
  for (const bool merged : {false, true}) {
    std::vector<TruthTree> trees = grid({6, 12, 18});
    trees[1].dose_push = 0.5;
    FakeWorld w = world(cfg, trees);
    Navigator nav(cfg, field30x24(), fov90());
    w.before_step = [&](FakeWorld& fw, const Navigator& n, Inputs& in) {
      if (n.state() != NavState::kDose || n.status().target != bad) return;
      if (merged) {
        fw.trees[1].published_id = 50;
      } else {
        fw.trees[1].hidden = true;
      }
      in.map = MapSnapshot{in.t, fw.map()};
    };
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim)) << "merged " << merged;
    EXPECT_EQ(countRequests(w, bad), 0u);
    EXPECT_EQ(countRequests(w, 50), 0u);
    if (merged) {   // the retry and the cleanup pass go to 50, which stays off-target: skipped twice
      EXPECT_EQ(counter(nav, "ids_reresolved"), 1u);
      EXPECT_EQ(counter(nav, "dose_retries"), 2u);
      EXPECT_EQ(counter(nav, "trees_skipped"), 2u);
    } else {        // gone: dropped, no retry, no skip
      EXPECT_EQ(counter(nav, "ids_dropped_vanished"), 1u);
      EXPECT_EQ(counter(nav, "dose_retries"), 0u);
      EXPECT_EQ(counter(nav, "trees_skipped"), 0u);
    }
    for (std::size_t k = 0; k < w.trees.size(); ++k) {
      if (k != 1) {
        EXPECT_EQ(w.trees[k].spray_calls, 1) << "tree " << k + 1;
      }
    }
  }
}

TEST(Navigator, MarkAnsweredDuringHoldFinishesOnce) {
  const Config cfg;
  {  // the pose is lost right after the first set_tree_sprayed; the answer arrives in HOLD(no_pose)
    FakeWorld w = world(cfg, grid({6, 12, 18}));
    Navigator nav(cfg, field30x24(), fov90());
    double lost_t = -1.0;
    TreeId first = treemapper::kNoTree;
    w.before_step = [&](FakeWorld& fw, const Navigator&, Inputs& in) {
      if (lost_t < 0.0 && fw.requests.size() == 1) {
        lost_t = in.t;
        first = fw.requests[0];
      }
      if (lost_t >= 0.0 && in.t < lost_t + 3.0) in.pose.valid = false;
    };
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    EXPECT_EQ(countRequests(w, first), 1u);   // not sent again on resume
    EXPECT_EQ(counter(nav, "spray_mark_retries"), 0u);
    EXPECT_EQ(counter(nav, "hold_no_pose"), 1u);
    EXPECT_EQ(nav.status().sprayed, 12u);
  }
  {  // HOLD(service) with retries used up; the pose is lost as it starts, the answer arrives in the hold
    std::vector<TruthTree> trees = grid({6, 12, 18});
    trees[1].mark_silent = 1 + cfg.service_retries;
    FakeWorld w = world(cfg, trees);
    Navigator nav(cfg, field30x24(), fov90());
    double lost_t = -1.0;
    w.before_step = [&](FakeWorld& fw, const Navigator& n, Inputs& in) {
      if (lost_t < 0.0 && n.state() == NavState::kHold) lost_t = in.t;
      if (lost_t >= 0.0 && in.t < lost_t + 3.0) in.pose.valid = false;
      (void)fw;
    };
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    ASSERT_GE(lost_t, 0.0);
    EXPECT_EQ(counter(nav, "hold_timeouts"), 0u);
    EXPECT_EQ(counter(nav, "hold_service"), 1u);
    EXPECT_EQ(countRequests(w, FakeWorld::idOf(1)), static_cast<std::size_t>(cfg.service_retries) + 2u);
    EXPECT_EQ(counter(nav, "spray_mark_failed"), 0u);
    EXPECT_EQ(nav.status().sprayed, 12u);
  }
}

TEST(Navigator, MapFreshnessFollowsTheLatestMap) {
  const Config cfg;
  {  // maps keep arriving, but their stamp stops advancing mid-GLIDE: stale after map_stale_s
    FakeWorld w = world(cfg, grid({6, 12, 18}));
    Navigator nav(cfg, field30x24(), fov90());
    double frozen = -1.0;
    w.before_step = [&](FakeWorld&, const Navigator& n, Inputs& in) {
      if (frozen < 0.0 && n.state() == NavState::kGlide && n.status().sprayed >= 1) frozen = in.t;
      if (frozen >= 0.0) in.map->stamp = frozen;
    };
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    const std::size_t h = findSample(w, [](const Sample& s) { return s.state == NavState::kHold; });
    ASSERT_LT(h, w.samples.size());
    EXPECT_GT(w.samples[h].t - frozen, cfg.map_stale_s - 1e-9);
    EXPECT_LE(w.samples[h].t - frozen, cfg.map_stale_s + kDt + 1e-9);
    EXPECT_EQ(counter(nav, "hold_map_stale"), 1u);
    EXPECT_EQ(counter(nav, "hold_timeouts"), 1u);
  }
  {  // one far-future stamp, then no map: that map is not fresh (it used to latch "fresh" for good)
    FakeWorld w = world(cfg, grid({6, 12, 18}));
    Navigator nav(cfg, field30x24(), fov90());
    double bad_t = -1.0;
    w.before_step = [&](FakeWorld&, const Navigator& n, Inputs& in) {
      if (bad_t < 0.0 && n.state() == NavState::kGlide && n.status().sprayed >= 1) {
        bad_t = in.t;
        in.map->stamp = in.t + 1000.0;
        return;
      }
      if (bad_t >= 0.0) in.map.reset();
    };
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    const std::size_t h = findSample(w, [](const Sample& s) { return s.state == NavState::kHold; });
    ASSERT_LT(h, w.samples.size());
    EXPECT_NEAR(w.samples[h].t, bad_t, 1e-9);
    EXPECT_EQ(counter(nav, "hold_map_stale"), 1u);
  }
  {  // one NaN or future stamp among fresh maps: a one-tick hold, then the mission goes on
    FakeWorld w = world(cfg, grid({6, 12, 18}));
    Navigator nav(cfg, field30x24(), fov90());
    int glitches = 0;
    w.before_step = [&](FakeWorld&, const Navigator& n, Inputs& in) {
      if (glitches == 0 && n.state() == NavState::kGlide && n.status().sprayed >= 1) {
        in.map->stamp = NAN;
        ++glitches;
      } else if (glitches == 1 && n.state() == NavState::kGlide && n.status().sprayed >= 5) {
        in.map->stamp = in.t + 1000.0;
        ++glitches;
      }
    };
    ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
    EXPECT_EQ(glitches, 2);
    EXPECT_EQ(counter(nav, "hold_map_stale"), 2u);
    EXPECT_EQ(counter(nav, "hold_timeouts"), 0u);
    for (const TruthTree& t : w.trees) EXPECT_TRUE(t.sprayed);
  }
}

TEST(Navigator, NoMapYetHoldsAtScanOut) {
  // TreeMapper never publishes: take-off and transit need no map, the scan does
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  w.before_step = [](FakeWorld&, const Navigator&, Inputs& in) { in.map.reset(); };
  Navigator nav(cfg, field30x24(), fov90());
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  const std::size_t h = findSample(w, [](const Sample& s) { return s.state == NavState::kHold; });
  ASSERT_LT(h, w.samples.size());
  ASSERT_GT(h, 0u);
  EXPECT_EQ(w.samples[h - 1].state, NavState::kScanOut);   // held on the first tick in SCAN_OUT
  EXPECT_EQ(w.samples[h].phase, "SCAN");                   // unchanged
  for (std::size_t k = 0; k < h; ++k) EXPECT_NE(w.samples[k].state, NavState::kHold);
  EXPECT_EQ(counter(nav, "hold_map_stale"), 1u);
  EXPECT_EQ(counter(nav, "hold_timeouts"), 1u);
  EXPECT_TRUE(w.requests.empty());
  EXPECT_NEAR(w.drone.z, 0.0, cfg.z_tol);   // went home and landed
}

TEST(Navigator, PoseWithBadStampIsRejected) {
  // one far-future and one NaN pose stamp mid-mission: neither replaces the stored pose (a future one used to freeze
  // the position for good), the stored pose is still fresh, so no hold
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  int glitches = 0;
  w.before_step = [&](FakeWorld&, const Navigator& n, Inputs& in) {
    if (glitches == 0 && n.state() == NavState::kGlide && n.status().sprayed >= 1) {
      in.pose.t = in.t + 100.0;
      ++glitches;
    } else if (glitches == 1 && n.state() == NavState::kGlide && n.status().sprayed >= 5) {
      in.pose.t = NAN;
      ++glitches;
    }
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  EXPECT_EQ(glitches, 2);
  EXPECT_EQ(counter(nav, "hold_no_pose"), 0u);
  for (const TruthTree& t : w.trees) EXPECT_EQ(t.spray_calls, 1);
}

TEST(Navigator, GuardAppliesOnTheTickATargetIsDropped) {
  // the guard is climbing on the glide to tree 8 when tree 8 vanishes: the next glide's first setpoint must still
  // obey the guard
  const Config cfg;
  FakeWorld w = world(cfg, grid({6, 12, 18}));
  Navigator nav(cfg, field30x24(), fov90());
  const TreeId victim = FakeWorld::idOf(7);
  int climb_ticks = 0;
  bool hidden = false;
  w.before_step = [&](FakeWorld& fw, const Navigator& n, Inputs& in) {
    if (hidden) return;
    if (n.state() == NavState::kGlide && n.status().target == victim && n.status().sprayed >= 4) {
      in.clearance.metres = 1.0;
      if (++climb_ticks == 8) {
        fw.trees[7].hidden = true;
        in.map = MapSnapshot{in.t, fw.map()};
        hidden = true;
      }
    }
  };
  ASSERT_TRUE(w.run(nav, kDt, kMaxSim));
  ASSERT_TRUE(hidden);
  EXPECT_EQ(counter(nav, "ids_dropped_vanished"), 1u);
  const std::size_t d = findSample(w, [&](const Sample& s) { return s.t > 0.0 && s.target != victim; },
                                   findSample(w, [&](const Sample& s) { return s.target == victim; }));
  ASSERT_LT(d, w.samples.size());
  // still climbing straight up (here already at scan altitude, where the climb is capped): no descent, no xy move
  EXPECT_DOUBLE_EQ(w.samples[d].setpoint.x, w.samples[d].pos.x);
  EXPECT_DOUBLE_EQ(w.samples[d].setpoint.y, w.samples[d].pos.y);
  EXPECT_GE(w.samples[d].setpoint.z, w.samples[d].pos.z);
}
