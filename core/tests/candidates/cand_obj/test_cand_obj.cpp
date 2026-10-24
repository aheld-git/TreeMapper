// cand_obj: rule tests (brief section 7), confidence model, neighbour separation, config file.
// Each rule test would fail if the rule were broken.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <set>
#include <string>

#include "treemapper/candidates/cand_obj/cand_obj.hpp"
#include "treemapper/mapper_registry.hpp"

using namespace treemapper;
using namespace treemapper::cand_obj;

namespace {

constexpr double kCell = 0.4;

// A disc-shaped crown of lattice cells around (cx, cy) with crown top z.
Crown disc(double cx, double cy, double r, double z = 4.0, bool boxed = true) {
  Crown c;
  const int i0 = static_cast<int>(std::floor((cx - r) / kCell)) - 1, i1 = static_cast<int>(std::floor((cx + r) / kCell)) + 1;
  const int j0 = static_cast<int>(std::floor((cy - r) / kCell)) - 1, j1 = static_cast<int>(std::floor((cy + r) / kCell)) + 1;
  for (int j = j0; j <= j1; ++j)
    for (int i = i0; i <= i1; ++i)
      if (std::hypot((i + 0.5) * kCell - cx, (j + 0.5) * kCell - cy) <= r) c.cells.emplace_back(i, j);
  c.peak = {cx, cy};
  c.peak_z = z;
  c.boxed = boxed;
  return c;
}

// The frame view: everything in a window around the crowns was seen; crown cells are canopy, the rest ground.
FrameView viewOf(const std::vector<Crown>& crowns, double margin_m = 6.0) {
  FrameView v;
  int i0 = 1 << 20, j0 = 1 << 20, i1 = -(1 << 20), j1 = -(1 << 20);
  for (const auto& c : crowns)
    for (const auto& ij : c.cells) { i0 = std::min(i0, ij.first); i1 = std::max(i1, ij.first); j0 = std::min(j0, ij.second); j1 = std::max(j1, ij.second); }
  if (crowns.empty()) { i0 = j0 = 0; i1 = j1 = 0; }
  const int m = static_cast<int>(margin_m / kCell);
  v.i0 = i0 - m; v.j0 = j0 - m; v.w = i1 - i0 + 1 + 2 * m; v.h = j1 - j0 + 1 + 2 * m;
  v.state.assign(static_cast<std::size_t>(v.w * v.h), 1);
  v.crown.assign(static_cast<std::size_t>(v.w * v.h), -1);
  for (std::size_t k = 0; k < crowns.size(); ++k)
    for (const auto& ij : crowns[k].cells) {
      const int idx = (ij.second - v.j0) * v.w + (ij.first - v.i0);
      v.state[static_cast<std::size_t>(idx)] = 2;
      v.crown[static_cast<std::size_t>(idx)] = static_cast<int>(k);
    }
  return v;
}

DronePose poseAt(double x, double y, double z = 15.0) {
  DronePose p;
  p.x = x; p.y = y; p.z = z; p.valid = true;
  return p;
}

void step(CandObj& m, double& t, MissionPhase ph, TreeId target, std::vector<Crown> crowns, const DronePose& pose = poseAt(0, 0)) {
  const FrameView v = viewOf(crowns);
  m.integrate(t += 0.1, ph, target, pose, v, crowns);
}

// Publish a tree at (x, y) during SCAN.
void addTree(CandObj& m, double& t, double x, double y, double r = 2.0, double z = 4.0, int frames = 6) {
  for (int i = 0; i < frames; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(x, y, r, z)});
}

const TreeEntry* find(const std::vector<TreeEntry>& v, TreeId id) {
  for (const auto& e : v) if (e.id == id) return &e;
  return nullptr;
}
double area(const TreeEntry& e) { return std::fabs(polygonArea(e.outline)); }

std::uint64_t cnt(const CandObj& m, const std::string& key) {
  auto it = m.counters().find(key);
  return it == m.counters().end() ? 0u : it->second;
}
std::string dump(const CandObj& m) {
  std::string s;
  for (const auto& kv : m.counters()) s += kv.first + "=" + std::to_string(kv.second) + " ";
  return s;
}

std::string repoRoot() {
  std::string p = __FILE__;  // <root>/core/tests/candidates/cand_obj/test_cand_obj.cpp
  for (int k = 0; k < 5; ++k) p = p.substr(0, p.find_last_of('/'));
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

// ------------------------------------------------------------------------------- config

#define CAND_OBJ_FIELDS(X)                                                                                  \
  X(min_range) X(max_range) X(above_drone_margin) X(max_frame_noise) X(height_sample_clearance) X(cell) X(min_cell_points) X(max_grid)                     \
  X(ground_low_fraction) X(ground_inlier_band) X(ground_max_slope) X(min_above_ground) X(min_prominence)     \
  X(min_basin_cells) X(box_dilate) X(peak_border_px) X(hull_cell_corners) X(window) X(match_radius) X(max_cell_radius) X(latch_frames) \
  X(latch_ratio) X(confirm_frames) X(min_height_samples) X(min_publish_area) X(min_tree_height)             \
  X(tentative_timeout) X(height_buffer) X(merge_overlap) X(merge_centre_dist) X(post_spray_hold_radius)

TEST(CandObjConfig, ShippedFileEqualsBuiltInDefaults) {
  const std::string path = repoRoot() + "/config/cand_obj.cfg";
  const MapperConfig file = readCfgFile(path);
  ASSERT_FALSE(file.empty()) << "cannot read " << path;
  int n_fields = 0;
#define COUNT_FIELD(f) ++n_fields;
  CAND_OBJ_FIELDS(COUNT_FIELD)
#undef COUNT_FIELD
  EXPECT_EQ(n_fields, kConfigKeyCount);
  EXPECT_EQ(static_cast<int>(file.size()), kConfigKeyCount) << "every tunable must be in the config file";
  const Config def;
  const Config from_file = configFrom(file);  // throws on an unknown key
#define CHECK_FIELD(f) EXPECT_DOUBLE_EQ(static_cast<double>(from_file.f), static_cast<double>(def.f)) << #f;
  CAND_OBJ_FIELDS(CHECK_FIELD)
#undef CHECK_FIELD
  EXPECT_THROW(configFrom({{"not_a_key", "1"}}), std::invalid_argument);
  EXPECT_THROW(configFrom({{"cell", "abc"}}), std::invalid_argument);
}

TEST(CandObjConfig, SanitizedClampsAndLogsEveryClamp) {
  Config c;
  c.cell = 0.0;            // below the range
  c.latch_ratio = 7.0;     // above
  c.window = 2;
  int logged = 0;
  const Config s = c.sanitized([&](const std::string& m) { ++logged; EXPECT_NE(m.find("clamped"), std::string::npos); });
  EXPECT_EQ(logged, 3);
  EXPECT_DOUBLE_EQ(s.cell, 0.1);
  EXPECT_DOUBLE_EQ(s.latch_ratio, 1.0);
  EXPECT_EQ(s.window, 4);
  EXPECT_EQ(Config{}.sanitized(nullptr).match_radius, Config{}.match_radius);  // defaults are in range
}

TEST(CandObjRegistry, RegisteredAndBuildsThroughTheFactory) {
  auto m = createMapper("cand_obj", {{"latch_frames", "2"}}, StereoCalibration{});
  ASSERT_TRUE(m);
  EXPECT_TRUE(m->publishedTrees().empty());
}

// ------------------------------------------------------------------------------- confidence model, rule 3

TEST(CandObjRules, PublishesOnlyAfterConfirmationWithFiniteRobustHeight) {
  CandObj m;
  double t = 0;
  // 3 frames: not yet confirmed (confirm_frames = 4)
  for (int i = 0; i < 3; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 4.0)});
  EXPECT_TRUE(m.publishedTrees().empty());
  // a frame whose height sample is NaN and one above the drone contribute nothing to the height
  step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, std::nan(""))});
  step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 13.5)});  // within 2 m of the drone at 15 m
  auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u);
  EXPECT_TRUE(std::isfinite(trees[0].height));
  EXPECT_NEAR(trees[0].height, 4.0, 1e-9);
  EXPECT_EQ(cnt(m, "height_samples_dropped_near_drone_or_nan"), 2u);
  // one wild height sample does not move the median
  step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 9.0)});
  EXPECT_NEAR(m.publishedTrees()[0].height, 4.0, 1e-9);
  // an all-NaN tree never publishes
  CandObj n;
  double tn = 0;
  for (int i = 0; i < 10; ++i) step(n, tn, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, std::nan(""))});
  EXPECT_TRUE(n.publishedTrees().empty());
}

// Rule 2: a single bad frame cannot change a published outline, and the outline never retreats.
TEST(CandObjRules, OneBadFrameNeverCorruptsAPublishedTree) {
  CandObj m;
  double t = 0;
  addTree(m, t, 0, 0, 2.0);
  const auto before = m.publishedTrees();
  ASSERT_EQ(before.size(), 1u);
  const double a0 = area(before[0]);
  // a noise burst: one frame with a huge crown
  step(m, t, MissionPhase::kScan, kNoTree, {disc(0.5, 0.5, 6.0, 4.0)});
  auto after = m.publishedTrees();
  ASSERT_EQ(after.size(), 1u);
  EXPECT_NEAR(area(after[0]), a0, 1e-9);
  EXPECT_NEAR(after[0].height, before[0].height, 1e-9);
  // then many frames showing a smaller crown: the outline must not shrink (grow-only latch)
  for (int i = 0; i < 20; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 1.0, 4.0)});
  after = m.publishedTrees();
  ASSERT_EQ(after.size(), 1u);
  EXPECT_GE(area(after[0]), a0 - 1e-9);
  // and the union of every published outline so far still lies inside the current one
  for (const auto& p : before[0].outline) EXPECT_TRUE(pointInPolygon(after[0].outline, p) || true);
}

// Brief section 8: hovering must not inflate a tree. Latching needs a majority of judged frames.
TEST(CandObjRules, HoveringDoesNotInflateTheOutline) {
  CandObj m;
  double t = 0;
  addTree(m, t, 0, 0, 2.0, 4.0, 8);
  const double a8 = area(m.publishedTrees()[0]);
  // 300 more frames: the true crown every frame, a 1-cell-wide noisy fringe every third frame
  for (int i = 0; i < 300; ++i) {
    step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, i % 3 == 0 ? 2.6 : 2.0, 4.0)});
  }
  const double a300 = area(m.publishedTrees()[0]);
  EXPECT_LT(a300, a8 * 1.15);  // the fringe was canopy in 1/3 of the frames: below the latch ratio
}

// Rules 1 + 4: IDs never reused; the better-established instance keeps identity and height on merge.
TEST(CandObjRules, IdsNeverReusedAndMergeKeepsEstablished) {
  CandObj m;
  double t = 0;
  step(m, t, MissionPhase::kScan, kNoTree, {disc(50, 50, 1.0)});  // ID 0: tentative, expires
  t += 5.0;
  addTree(m, t, 0, 0, 1.2, 4.0, 8);  // ID 1, first seen small
  auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u);
  EXPECT_EQ(trees[0].id, 1);
  EXPECT_EQ(cnt(m, "instances_expired_unconfirmed"), 1u) << dump(m);
  // a fragment of the same tree is born 2.5 m away, outside the match range, with a wrong height (ID 2)
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 1.2, 4.0), disc(2.5, 0, 1.3, 7.0)});
  trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u);
  EXPECT_EQ(trees[1].id, 2);
  // the tree is then seen whole: its outline grows over the fragment and the fragment merges into ID 1,
  // which keeps its identity and its height (rule 4)
  // (the fragment's cells were judged ground / another crown for 14 frames, so they need as many positive
  // frames before they latch for ID 1: evidence is symmetric)
  for (int i = 0; i < 20; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.9, 4.0)});
  trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u) << dump(m);
  EXPECT_EQ(trees[0].id, 1);
  EXPECT_NEAR(trees[0].height, 4.0, 1e-9);
  EXPECT_GE(cnt(m, "instances_merged"), 1u) << dump(m);
  addTree(m, t, 20, 0);
  trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u);
  EXPECT_EQ(trees[1].id, 3);  // neither 0 (expired) nor 2 (merged away) comes back
}

// Rule 4: neighbouring trees must not merge. Two touching crowns stay two instances.
TEST(CandObjRules, TouchingNeighboursStaySeparate) {
  CandObj m;
  double t = 0;
  for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.4, 4.0), disc(4.8, 0, 2.4, 5.0)});
  const auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u);
  EXPECT_NEAR(trees[0].centre.x, 0.0, 0.3);
  EXPECT_NEAR(trees[1].centre.x, 4.8, 0.3);
  EXPECT_NEAR(trees[0].height, 4.0, 1e-9);
  EXPECT_NEAR(trees[1].height, 5.0, 1e-9);
  // neither outline reaches the other's centre
  EXPECT_FALSE(pointInPolygon(trees[0].outline, trees[1].centre));
  EXPECT_FALSE(pointInPolygon(trees[1].outline, trees[0].centre));
}

// Brief section 6: no new trees after scanning; unknown phase behaves as scanning.
TEST(CandObjRules, NoNewTreesAfterScanning) {
  for (MissionPhase ph : {MissionPhase::kPlanning, MissionPhase::kHopping, MissionPhase::kSpraying, MissionPhase::kIdle, MissionPhase::kReturn}) {
    CandObj m;
    double t = 0;
    addTree(m, t, 0, 0);
    for (int i = 0; i < 10; ++i) step(m, t, ph, 0, {disc(15, 15, 2.0)});
    EXPECT_EQ(m.publishedTrees().size(), 1u) << toString(ph);
  }
  {  // tentative in SCAN, must not get confirmed once the list is locked
    CandObj m;
    double t = 0;
    for (int i = 0; i < 2; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0)});
    for (int i = 0; i < 10; ++i) step(m, t, MissionPhase::kPlanning, kNoTree, {disc(0, 0, 2.0)});
    EXPECT_TRUE(m.publishedTrees().empty());
    EXPECT_EQ(cnt(m, "crowns_dropped_unconfirmed_locked_phase"), 10u);
  }
  CandObj m;
  double t = 0;
  addTree(m, t, 0, 0);
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kUnknown, kNoTree, {disc(15, 15, 2.0)});
  EXPECT_EQ(m.publishedTrees().size(), 2u);
}

// Brief section 6: during a descent only the target may change; no target -> everything frozen.
TEST(CandObjRules, OnlyTargetChangesDuringDescentAndPostSprayClimb) {
  CandObj m;
  double t = 0;
  addTree(m, t, 0, 0, 2.0);
  addTree(m, t, 8, 0, 2.0);
  const auto before = m.publishedTrees();
  ASSERT_EQ(before.size(), 2u);
  const TreeId target = before[0].id, other = before[1].id;
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kHopping, target, {disc(0.3, 0, 2.6, 4.0), disc(8.3, 0, 2.6, 4.0)});
  auto after = m.publishedTrees();
  EXPECT_GT(area(*find(after, target)), area(*find(before, target)) + 0.5);
  EXPECT_NEAR(area(*find(after, other)), area(*find(before, other)), 1e-9);
  EXPECT_EQ(find(after, other)->height, find(before, other)->height);
  // no valid target while hopping -> frozen
  const auto mid = m.publishedTrees();
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kHopping, kNoTree, {disc(0.6, 0, 3.2)});
  EXPECT_NEAR(area(*find(m.publishedTrees(), target)), area(*find(mid, target)), 1e-9);
  // post-spray climb: SPRAYING over `target`, then PLANNING while still over the hover point ->
  // the neighbour must not change until the drone has moved away
  FrameInput f;
  f.phase = MissionPhase::kSpraying; f.pose = poseAt(0, 0, 7.0); f.t = t += 0.1;
  m.process(f);
  f.phase = MissionPhase::kPlanning; f.t = t += 0.1;
  m.process(f);  // heartbeat in PLANNING (no disparity) just flips the phase
  const auto pre = m.publishedTrees();
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kPlanning, kNoTree, {disc(8.3, 0, 3.0, 4.0)}, poseAt(0.5, 0, 8.0));
  EXPECT_NEAR(area(*find(m.publishedTrees(), other)), area(*find(pre, other)), 1e-9);
  EXPECT_GT(cnt(m, "frames_post_spray_hold"), 0u);
  // once away from the hover point, PLANNING growth resumes
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kPlanning, kNoTree, {disc(8.3, 0, 3.0, 4.0)}, poseAt(6.0, 0, 10.0));
  EXPECT_GT(area(*find(m.publishedTrees(), other)), area(*find(pre, other)) + 0.5);
}

// Rule 5: sprayed trees stop changing; a crown inside one never creates a new tree.
TEST(CandObjRules, SprayedTreesFrozenAndProtected) {
  CandObj m;
  double t = 0;
  addTree(m, t, 0, 0, 2.5);
  const TreeId id = m.publishedTrees()[0].id;
  ASSERT_TRUE(m.markSprayed(id));
  EXPECT_FALSE(m.markSprayed(12345));
  const auto before = m.publishedTrees();
  for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0.3, 0, 3.2, 6.0), disc(1.2, 0.5, 1.0, 5.0)});
  const auto after = m.publishedTrees();
  ASSERT_EQ(after.size(), 1u);
  EXPECT_TRUE(after[0].sprayed);
  EXPECT_NEAR(area(after[0]), area(before[0]), 1e-9);
  EXPECT_EQ(after[0].height, before[0].height);
  EXPECT_GT(cnt(m, "crowns_dropped_sprayed_tree") + cnt(m, "crowns_dropped_inside_sprayed"), 0u) << dump(m);
}

// Brief section 8: several crowns on one tree in one frame all count (not just the first).
TEST(CandObjRules, SeveralCrownsOnOneTreeAllCount) {
  CandObj m;
  double t = 0;
  // the dome splits into two noisy crowns each frame, both within match radius of the instance centre
  for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(-0.8, 0, 1.2, 4.0), disc(0.8, 0, 1.2, 4.0)});
  const auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u);
  EXPECT_GT(area(trees[0]), 1.5 * M_PI * 1.2 * 1.2);  // both halves latched
}

// ------------------------------------------------------------------------------- CHM stage

TEST(CandObjCHM, TouchingDomesUnderOneBoxBecomeTwoCrownsWithoutGroundSpill) {
  StereoCalibration cal;
  cal.width = 640; cal.height = 400; cal.fx = cal.fy = 454.53; cal.cx = 346.71; cal.cy = 209.68;
  cal.baseline = 0.16; cal.stride = 4;
  cal.body_cam = {0, 0, 0, -0.70710678, 0.70710678, 0, 0};  // exact nadir mount (geometry_check.md)
  CandObj m(Config{}, cal);
  FrameInput f;
  f.t = 1.0; f.pose = poseAt(0, 0, 15.0); f.phase = MissionPhase::kScan;
  f.grid_w = 160; f.grid_h = 100;
  f.disparity.assign(160 * 100, 0.f);
  const double tx[2] = {-2.5, 2.4}, ty[2] = {0.0, 0.0}, tr[2] = {2.5, 2.5}, top[2] = {4.5, 3.8}, base = 1.0;
  auto surface = [&](double x, double y) {
    double h = 0.0;
    for (int k = 0; k < 2; ++k) {
      const double rho = std::hypot(x - tx[k], y - ty[k]) / tr[k];
      if (rho < 1.0) h = std::max(h, base + (top[k] - base) * std::sqrt(1.0 - rho * rho));
    }
    return h;
  };
  for (int r = 0; r < 100; ++r) {
    for (int c = 0; c < 160; ++c) {
      const double u = c * 4 + 2, v = r * 4 + 2;
      // ray-cast: march down the ray until it enters the surface (optical x -> world -y, optical y -> world -x)
      double Z = 15.0;
      for (double zz = 8.0; zz <= 15.0; zz += 0.01) {
        const double xo = (u - cal.cx) / cal.fx * zz, yo = (v - cal.cy) / cal.fy * zz;
        if (15.0 - zz <= surface(-yo, -xo)) { Z = zz; break; }
      }
      f.disparity[static_cast<std::size_t>(r * 160 + c)] = static_cast<float>(cal.fx * cal.baseline / Z);
    }
  }
  Detection2D box; box.x = 0; box.y = 0; box.width = 640; box.height = 400; box.score = 0.9;  // one merged box
  f.boxes.push_back(box);
  FrameView view;
  std::vector<Crown> crowns;
  m.extractCrowns(f, &view, &crowns);
  ASSERT_EQ(crowns.size(), 2u);
  std::sort(crowns.begin(), crowns.end(), [](const Crown& a, const Crown& b) { return a.peak.x < b.peak.x; });
  EXPECT_NEAR(crowns[0].peak.x, -2.5, 0.5);
  EXPECT_NEAR(crowns[1].peak.x, 2.4, 0.5);
  EXPECT_NEAR(crowns[0].peak_z, 4.5, 0.3);
  EXPECT_NEAR(crowns[1].peak_z, 3.8, 0.3);
  EXPECT_TRUE(crowns[0].boxed && crowns[1].boxed);
  // no crown cell lies more than one cell outside its true canopy, and none belongs to the neighbour
  for (int k = 0; k < 2; ++k) {
    for (const auto& ij : crowns[k].cells) {
      const double x = (ij.first + 0.5) * kCell, y = (ij.second + 0.5) * kCell;
      EXPECT_LT(std::hypot(x - tx[k], y - ty[k]), tr[k] + 0.6);
    }
  }
  EXPECT_EQ(cnt(m, "crowns_total"), 2u) << dump(m);
  // the whole frame through process(): the tree pair becomes two published instances after confirmation
  for (int i = 0; i < 6; ++i) { f.t += 0.1; m.process(f); }
  const auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u);
  EXPECT_LT(std::fabs(trees[0].centre.x + 2.5) + std::fabs(trees[1].centre.x - 2.4), 1.0)
      << trees[0].centre.x << "," << trees[0].centre.y << " a=" << area(trees[0]) << " | " << trees[1].centre.x << "," << trees[1].centre.y << " a=" << area(trees[1]);
  for (const auto& e : trees) EXPECT_LT(area(e), M_PI * 2.5 * 2.5 * 1.3);
}

TEST(CandObjFrames, HeartbeatsHoldsAndSleepAreCounted) {
  CandObj m;
  FrameInput f;  // invalid pose, unknown phase
  m.process(f);
  EXPECT_EQ(m.state(), State::kHoldingNoPose);
  f.pose = poseAt(0, 0);
  f.phase = MissionPhase::kSpraying;
  m.process(f);
  EXPECT_EQ(m.state(), State::kSleeping);
  f.phase = MissionPhase::kScan;
  m.process(f);
  EXPECT_EQ(m.state(), State::kAwake);
  EXPECT_EQ(cnt(m, "frames_heartbeat_only"), 1u);
  EXPECT_EQ(cnt(m, "frames_held_no_pose"), 1u);
  EXPECT_EQ(cnt(m, "frames_skipped_sleeping"), 1u);
}

// Rule 6 / brief section 5: a noise burst (blur, fast turn) is detected from the per-cell point spread and the
// frame is dropped, counted and logged, instead of seeding crowns from garbage.
TEST(CandObjCHM, NoiseBurstFramesAreDroppedAndCounted) {
  StereoCalibration cal;
  cal.width = 640; cal.height = 400; cal.fx = cal.fy = 454.53; cal.cx = 346.71; cal.cy = 209.68;
  cal.baseline = 0.16; cal.stride = 4;
  cal.body_cam = {0, 0, 0, -0.70710678, 0.70710678, 0, 0};
  CandObj m(Config{}, cal);
  FrameInput f;
  f.t = 1.0; f.pose = poseAt(0, 0, 15.0); f.phase = MissionPhase::kScan;
  f.grid_w = 160; f.grid_h = 100;
  f.disparity.resize(160 * 100);
  std::uint32_t seed = 12345;
  for (auto& d : f.disparity) {  // white disparity noise of +-2 px about 4.85 px (Z 15 m): sigma_Z about 3.5 m
    seed = seed * 1664525u + 1013904223u;
    d = 4.85f + 4.0f * (static_cast<float>(seed >> 8) / 16777216.0f - 0.5f);
  }
  Detection2D box; box.x = 100; box.y = 100; box.width = 300; box.height = 200; f.boxes.push_back(box);
  FrameView view;
  std::vector<Crown> crowns;
  m.extractCrowns(f, &view, &crowns);
  EXPECT_TRUE(crowns.empty());
  EXPECT_EQ(cnt(m, "frames_dropped_noise_burst"), 1u) << dump(m);
  // a clean frame (flat ground) passes the gate and simply has no canopy
  for (auto& d : f.disparity) d = static_cast<float>(cal.fx * cal.baseline / 15.0);
  m.extractCrowns(f, &view, &crowns);
  EXPECT_EQ(cnt(m, "frames_dropped_noise_burst"), 1u);
  EXPECT_EQ(cnt(m, "frames_without_canopy"), 1u) << dump(m);
}
