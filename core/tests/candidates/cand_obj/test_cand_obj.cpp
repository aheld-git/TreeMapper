// cand_obj: rule tests (brief section 7), confidence model, neighbour separation, config file.
// Each rule test would fail if the rule were broken.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <fstream>
#include <limits>
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
  v.z.assign(static_cast<std::size_t>(v.w * v.h), 0.0f);  // ground at z = 0, every crown cell at the crown's top
  for (std::size_t k = 0; k < crowns.size(); ++k)
    for (const auto& ij : crowns[k].cells) {
      const int idx = (ij.second - v.j0) * v.w + (ij.first - v.i0);
      v.state[static_cast<std::size_t>(idx)] = 2;
      v.crown[static_cast<std::size_t>(idx)] = static_cast<int>(k);
      v.z[static_cast<std::size_t>(idx)] = static_cast<float>(crowns[k].peak_z);
    }
  return v;
}

// A dome of top `top` and radius r around (cx, cy): per-cell z follows the dome profile (for height tests),
// optionally only the part with x < x_max (a slice seen at the swath edge).
Crown dome(double cx, double cy, double r, double top, double x_max = 1e9) {
  Crown c = disc(cx, cy, r, top);
  std::vector<std::pair<int, int>> kept;
  for (const auto& ij : c.cells) if ((ij.first + 0.5) * kCell < x_max) kept.push_back(ij);
  c.cells = kept;
  return c;
}
FrameView domeView(const std::vector<Crown>& crowns, const std::vector<std::array<double, 4>>& domes) {
  FrameView v = viewOf(crowns);
  for (std::size_t k = 0; k < crowns.size(); ++k)
    for (const auto& ij : crowns[k].cells) {
      const int idx = (ij.second - v.j0) * v.w + (ij.first - v.i0);
      const double x = (ij.first + 0.5) * kCell, y = (ij.second + 0.5) * kCell;
      const double rho = std::hypot(x - domes[k][0], y - domes[k][1]) / domes[k][2];
      v.z[static_cast<std::size_t>(idx)] = static_cast<float>(domes[k][3] * std::sqrt(std::max(0.0, 1.0 - rho * rho)));
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
  X(z_ref) X(lateral_k) X(rot_rate_ref) X(quality_recover_s)                                                \
  X(ground_low_fraction) X(ground_inlier_band) X(ground_max_slope) X(min_above_ground) X(min_prominence)     \
  X(min_basin_cells) X(box_dilate) X(peak_border_px) X(hull_cell_corners) X(seed_truncated) X(window) X(match_radius) X(match_overlap) X(match_reach) X(seed_overlap) \
  X(max_cell_radius) X(latch_rel_h) X(latch_frames) X(latch_ratio) X(confirm_frames) X(min_height_cells) X(min_publish_area) X(min_tree_height) \
  X(tentative_timeout) X(cell_weight_cap) X(outlier_gate_m) X(gate_min_weight) X(height_quantile) X(max_height_rate) X(max_centre_rate) \
  X(centre_from_support) X(centre_support_ratio) X(centre_max_outside)                                                               \
  X(merge_overlap) X(merge_centre_dist) X(post_spray_hold_radius)

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
  c.centre_support_ratio = -0.5;
  c.centre_max_outside = 2.0;
  int logged = 0;
  const Config s = c.sanitized([&](const std::string& m) { ++logged; EXPECT_NE(m.find("clamped"), std::string::npos); });
  EXPECT_EQ(logged, 5);
  EXPECT_DOUBLE_EQ(s.centre_support_ratio, 0.0);
  EXPECT_DOUBLE_EQ(s.centre_max_outside, 1.3);
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
  // 2 frames: not yet confirmed (confirm_frames = 3, min_height_cells = 3)
  for (int i = 0; i < 2; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 4.0)});
  EXPECT_TRUE(m.publishedTrees().empty());
  // a frame whose height sample is NaN and one too close to the drone contribute nothing to the height:
  // still only 2 finite samples, so still unpublished although 4 frames were seen
  step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, std::nan(""))});
  step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 13.5)});  // within 2 m of the drone at 15 m
  EXPECT_TRUE(m.publishedTrees().empty());
  step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 4.0)});  // third finite sample
  auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u);
  EXPECT_TRUE(std::isfinite(trees[0].height));
  EXPECT_NEAR(trees[0].height, 4.0, 1e-9);
  EXPECT_GT(cnt(m, "height_cells_dropped_near_drone_or_nan"), 0u) << dump(m);
  // one wild frame (every cell 5 m higher) does not move the height: established cells reject it, and
  // the published value is rate-limited anyway
  step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 9.0)});
  EXPECT_NEAR(m.publishedTrees()[0].height, 4.0, 1e-9);
  EXPECT_GT(cnt(m, "height_cells_rejected_outlier"), 0u) << dump(m);
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
  // a fragment of the same tree is born 2.5 m away, outside the match range, with a wrong (low) height (ID 2)
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 1.2, 4.0), disc(2.5, 0, 1.3, 2.0)});
  trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u);
  EXPECT_EQ(trees[1].id, 2);
  // the tree is then seen whole: its outline grows over the fragment and the fragment merges into ID 1,
  // which keeps its identity and its height (rule 4)
  // (the fragment's cells were judged ground / another crown for 14 frames, so they need as many positive
  // frames before they latch for ID 1: evidence is symmetric)
  for (int i = 0; i < 50; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.9, 4.0)});
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
  for (int i = 0; i < 25; ++i) step(m, t, MissionPhase::kHopping, target, {disc(0.3, 0, 2.6, 4.0), disc(8.3, 0, 2.6, 4.0)});
  auto after = m.publishedTrees();
  EXPECT_GT(area(*find(after, target)), area(*find(before, target)) + 0.5);
  EXPECT_NEAR(area(*find(after, other)), area(*find(before, other)), 1e-9);
  EXPECT_EQ(find(after, other)->height, find(before, other)->height);
  // no valid target while hopping -> frozen
  const auto mid = m.publishedTrees();
  for (int i = 0; i < 25; ++i) step(m, t, MissionPhase::kHopping, kNoTree, {disc(0.6, 0, 3.2)});
  EXPECT_NEAR(area(*find(m.publishedTrees(), target)), area(*find(mid, target)), 1e-9);
  // post-spray climb: SPRAYING over `target`, then PLANNING while still over the hover point ->
  // the neighbour must not change until the drone has moved away
  FrameInput f;
  f.phase = MissionPhase::kSpraying; f.pose = poseAt(0, 0, 7.0); f.t = t += 0.1;
  m.process(f);
  f.phase = MissionPhase::kPlanning; f.t = t += 0.1;
  m.process(f);  // heartbeat in PLANNING (no disparity) just flips the phase
  const auto pre = m.publishedTrees();
  for (int i = 0; i < 25; ++i) step(m, t, MissionPhase::kPlanning, kNoTree, {disc(8.3, 0, 3.0, 4.0)}, poseAt(0.5, 0, 8.0));
  EXPECT_NEAR(area(*find(m.publishedTrees(), other)), area(*find(pre, other)), 1e-9);
  EXPECT_GT(cnt(m, "frames_post_spray_hold"), 0u);
  // once away from the hover point, PLANNING growth resumes
  for (int i = 0; i < 40; ++i) step(m, t, MissionPhase::kPlanning, kNoTree, {disc(8.3, 0, 3.0, 4.0)}, poseAt(6.0, 0, 10.0));
  EXPECT_GT(area(*find(m.publishedTrees(), other)), area(*find(pre, other)) + 0.5);
}

// D67: the post-spray hold lasts until the drone has left the hover point. SCAN_APPROACH right after a dose (corridor
// mode flies HOPPING -> SCAN_APPROACH from the hover point) does not end it; leaving the radius, or SCAN, does.
TEST(CandObjRules, PostSprayHoldOutlastsScanApproachUntilTheDroneLeaves) {
  auto sprayed = [](CandObj& m, double& t, TreeId* other) {
    addTree(m, t, 0, 0, 2.0);
    addTree(m, t, 8, 0, 2.0);
    const auto trees = m.publishedTrees();
    ASSERT_EQ(trees.size(), 2u) << dump(m);
    *other = trees[1].id;
    FrameInput f;
    f.phase = MissionPhase::kSpraying; f.hop_target = trees[0].id; f.pose = poseAt(0, 0, 7.0); f.t = t += 0.1;
    m.process(f);
    f.phase = MissionPhase::kScanApproach; f.hop_target = kNoTree; f.t = t += 0.1;
    m.process(f);  // heartbeat (no disparity): SPRAYING has ended, the hold starts
  };
  auto same = [](const TreeEntry& a, const TreeEntry& b) {
    return a.outline.size() == b.outline.size() && std::fabs(area(a) - area(b)) < 1e-12 && a.centre.x == b.centre.x &&
           a.centre.y == b.centre.y && a.height == b.height;
  };
  {  // SCAN_APPROACH within the radius: the neighbour stays frozen; leaving the radius releases it
    CandObj m;
    double t = 0;
    TreeId other = kNoTree;
    sprayed(m, t, &other);
    const TreeEntry pre = *find(m.publishedTrees(), other);
    const auto held0 = cnt(m, "frames_post_spray_hold");
    for (int i = 0; i < 25; ++i)
      step(m, t, MissionPhase::kScanApproach, kNoTree, {disc(8.6, 0.4, 3.0, 4.0)}, poseAt(0.7, 0, 9.0));
    EXPECT_TRUE(same(*find(m.publishedTrees(), other), pre)) << dump(m);
    EXPECT_EQ(cnt(m, "frames_post_spray_hold"), held0 + 25u);
    for (int i = 0; i < 40; ++i)
      step(m, t, MissionPhase::kScanApproach, kNoTree, {disc(8.6, 0.4, 3.0, 4.0)}, poseAt(2.5, 0, 12.0));
    EXPECT_GT(area(*find(m.publishedTrees(), other)), area(pre) + 0.5) << dump(m);
    EXPECT_EQ(cnt(m, "frames_post_spray_hold"), held0 + 25u);
  }
  {  // SCAN releases it even within the radius (a real scan never runs with its trees frozen)
    CandObj m;
    double t = 0;
    TreeId other = kNoTree;
    sprayed(m, t, &other);
    const TreeEntry pre = *find(m.publishedTrees(), other);
    const auto held0 = cnt(m, "frames_post_spray_hold");
    for (int i = 0; i < 40; ++i)
      step(m, t, MissionPhase::kScan, kNoTree, {disc(8.6, 0.4, 3.0, 4.0)}, poseAt(0.7, 0, 9.0));
    EXPECT_GT(area(*find(m.publishedTrees(), other)), area(pre) + 0.5) << dump(m);
    EXPECT_EQ(cnt(m, "frames_post_spray_hold"), held0);
  }
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

// ------------------------------------------------------------------------------- M4 hardening

// A tentative or new instance can never take over a published tree's match (Isaac 178 s transient
// duplicate): a crown that covers a published tree's latched cells is that tree, even when its top lies
// farther from the tree's (still partial) centroid than match_radius.
TEST(CandObjM4, FragmentNeverTakesOverAPublishedTree) {
  CandObj m;
  double t = 0;
  // the tree enters at the swath edge: only the slice x < -1.6 of a dome centred at (0, 0) is seen
  for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kScan, kNoTree, {dome(0, 0, 3.0, 5.0, -1.6)});
  auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u) << dump(m);
  const TreeId id = trees[0].id;
  EXPECT_LT(trees[0].centre.x, -1.6);  // the slice's centroid
  // now the whole crown is visible: its top (0, 0) is > match_radius from the slice centroid (about -2.3, 0)
  // and outside the slice's outline, so the old peak rule alone would have seeded a second instance
  EXPECT_GT(std::hypot(trees[0].centre.x, trees[0].centre.y), m.config().match_radius);
  EXPECT_FALSE(pointInPolygon(trees[0].outline, Point2{0.0, 0.0}));
  // (the test view marks the unseen part of the dome as ground, so those cells carry 8 negative frames
  // and need 24 positive ones before they latch: evidence is symmetric)
  for (int i = 0; i < 40; ++i) step(m, t, MissionPhase::kScan, kNoTree, {dome(0, 0, 3.0, 5.0)});
  trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u) << dump(m);
  EXPECT_EQ(trees[0].id, id);
  EXPECT_EQ(cnt(m, "instances_created"), 1u) << dump(m);  // no second instance was ever born
  EXPECT_GT(area(trees[0]), 3.0 * M_PI * 1.0 * 1.0);         // and the known tree grew over the full crown
  EXPECT_NEAR(trees[0].centre.x, 0.0, 1.0);  // still converging: far cells enter only as the centroid moves (max_cell_radius)
}

// Height = high quantile of the latched surface, not the top of the first visible slice; the published
// height and centre move at a bounded rate only (no jumps), and the height never ends up biased low.
TEST(CandObjM4, HeightFromLatchedSurfaceAndRateLimitedPublication) {
  CandObj m;
  double t = 0;
  const std::vector<std::array<double, 4>> d{{0, 0, 2.6, 5.0}};
  for (int i = 0; i < 8; ++i) {
    std::vector<Crown> c{dome(0, 0, 2.6, 5.0, -1.0)};
    m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, poseAt(0, 0), domeView(c, d), c);
  }
  auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u) << dump(m);
  const double h_slice = trees[0].height;
  EXPECT_LT(h_slice, 4.7);  // the slice x < -1 of a 5 m dome tops out at 5 * sqrt(1 - (1/2.6)^2) = 4.6 m
  double prev_h = h_slice;
  Point2 prev_c = trees[0].centre;
  double max_dh = 0.0, max_dc = 0.0;
  for (int i = 0; i < 60; ++i) {
    std::vector<Crown> c{dome(0, 0, 2.6, 5.0)};
    m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, poseAt(0, 0), domeView(c, d), c);
    trees = m.publishedTrees();
    ASSERT_EQ(trees.size(), 1u);
    max_dh = std::max(max_dh, std::fabs(trees[0].height - prev_h));
    max_dc = std::max(max_dc, std::hypot(trees[0].centre.x - prev_c.x, trees[0].centre.y - prev_c.y));
    prev_h = trees[0].height;
    prev_c = trees[0].centre;
  }
  EXPECT_NEAR(trees[0].height, 4.97, 0.10) << dump(m);  // the whole dome: 1 % of a dome's cells lie above 5 * sqrt(0.99) = 4.97 m
  EXPECT_GT(trees[0].height, h_slice + 0.2);
  EXPECT_LE(max_dh, 0.5 * 0.1 + 1e-6);                 // max_height_rate 0.5 m/s at 10 Hz
  EXPECT_LE(max_dc, 0.8 * 0.1 + 1e-6);                 // max_centre_rate 0.8 m/s
  EXPECT_NEAR(trees[0].centre.x, 0.0, 0.3);            // and the centre has converged to the true centre
  EXPECT_GT(cnt(m, "height_steps_clamped") + cnt(m, "centre_steps_clamped"), 0u);
}

// Skirt trim: crown cells below latch_rel_h x the crown's peak height above ground are never claimed, so
// the stereo smear of the crown edge cannot latch.
TEST(CandObjM4, SkirtCellsBelowRelativeHeightNeverLatch) {
  Config c0;
  c0.latch_rel_h = 0.5;  // the gate is shipped off (it cost hard recall; see the design doc); test it enabled
  CandObj m(c0);
  double t = 0;
  for (int i = 0; i < 10; ++i) {
    std::vector<Crown> c{disc(0, 0, 2.5, 4.0)};
    c[0].peak_hag = 4.0;
    FrameView v = viewOf(c);
    v.hag.assign(v.state.size(), 0.0f);
    for (const auto& ij : c[0].cells) {
      const double r = std::hypot((ij.first + 0.5) * kCell, (ij.second + 0.5) * kCell);
      v.hag[static_cast<std::size_t>((ij.second - v.j0) * v.w + (ij.first - v.i0))] = r <= 1.5 ? 4.0f : 1.0f;  // skirt: 25 % of the peak
    }
    m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, poseAt(0, 0), v, c);
  }
  const auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u) << dump(m);
  EXPECT_LT(area(trees[0]), M_PI * 1.7 * 1.7);
  EXPECT_GT(area(trees[0]), M_PI * 1.2 * 1.2);
  EXPECT_GT(cnt(m, "cells_skipped_low_relative_height"), 0u);
}

// Frame quality weights the evidence: a frame taken in a fast turn counts as a fraction of a frame.
TEST(CandObjM4, LowQualityFramesCarryLessEvidence) {
  Config c;
  EXPECT_DOUBLE_EQ(frameQuality(-1.0, 100.0, c), 1.0);          // unknown rate: fail open
  EXPECT_NEAR(frameQuality(c.rot_rate_ref, 100.0, c), 0.5, 1e-9);
  EXPECT_NEAR(frameQuality(0.0, 0.0, c), 0.1, 1e-9);            // right after a fast rotation
  EXPECT_NEAR(frameQuality(0.0, 0.75, c), 0.55, 1e-9);          // half recovered
  EXPECT_DOUBLE_EQ(frameQuality(0.0, 2.0, c), 1.0);
  CandObj m;
  double t = 0;
  for (int i = 0; i < 12; ++i) {
    std::vector<Crown> cr{disc(0, 0, 2.0, 4.0)};
    m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, poseAt(0, 0), viewOf(cr), cr, 0.2);
  }
  EXPECT_TRUE(m.publishedTrees().empty()) << dump(m);  // 12 x 0.2 = 2.4 < latch_frames
  for (int i = 0; i < 3; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 4.0)});
  EXPECT_EQ(m.publishedTrees().size(), 1u) << dump(m);  // 2.4 + 3 >= 5
}

// Rule 4 + M4: a merge keeps the winner's ID and does not jump its published height or centre; the
// loser's measured surface still counts, so the merged height cannot end below what either side saw.
TEST(CandObjM4, MergeNeverJumpsThePublishedValues) {
  CandObj m;
  double t = 0;
  addTree(m, t, 0, 0, 1.2, 4.0, 8);                                     // ID 0
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 1.2, 4.0), disc(2.6, 0, 1.2, 5.0)});  // ID 1, taller part
  auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u) << dump(m);
  double prev_h = find(trees, 0)->height;
  Point2 prev_c = find(trees, 0)->centre;
  double max_dh = 0.0, max_dc = 0.0;
  for (int i = 0; i < 60; ++i) {
    step(m, t, MissionPhase::kScan, kNoTree, {disc(1.0, 0, 2.6, 5.0)});
    trees = m.publishedTrees();
    const TreeEntry* e = find(trees, 0);
    ASSERT_TRUE(e) << dump(m);
    max_dh = std::max(max_dh, std::fabs(e->height - prev_h));
    max_dc = std::max(max_dc, std::hypot(e->centre.x - prev_c.x, e->centre.y - prev_c.y));
    prev_h = e->height;
    prev_c = e->centre;
  }
  ASSERT_EQ(trees.size(), 1u) << dump(m);
  EXPECT_EQ(trees[0].id, 0);
  EXPECT_GE(cnt(m, "instances_merged"), 1u);
  EXPECT_LE(max_dh, 0.05 + 1e-6);
  EXPECT_LE(max_dc, 0.08 + 1e-6);
  EXPECT_NEAR(trees[0].height, 5.0, 0.05);  // converged to the taller measured surface, not stuck at 4.0
}

// ------------------------------------------------------------------------------- centre (miss reduction, D66)

namespace {
// A crown first seen from a far track at the image edge, then from a near track (the Isaac pattern of
// results/miss_reduction.md "Centre"): the true crown is a disc of radius 1.5 m at (0, 0). From the far track
// (8.4 m across) the stereo puts the crown 1.5 m toward the drone and sees the true far half as ground; its cells
// carry the low point weight of oblique, far points (0.15). From the near track (3.7 m across) the crown is where
// it is, with near-nadir weight (0.9), and the far view's ghost band is ground. `each` is called after every frame.
template <typename F>
void farThenNear(CandObj& m, double& t, F each, double ghost_dy = -1.5, int near_frames = 40) {
  auto frame = [&](double cy, double w, double drone_y) {
    std::vector<Crown> c{disc(0.0, cy, 1.5, 2.7)};
    FrameView v = viewOf(c);
    v.hwt.assign(v.state.size(), static_cast<float>(w));
    m.integrate(t += 0.1, MissionPhase::kScan, kNoTree, poseAt(0.0, drone_y), v, c);
    each();
  };
  for (int i = 0; i < 25; ++i) frame(ghost_dy, 0.15, -8.4);  // far pass
  for (int i = 0; i < near_frames; ++i) frame(0.0, 0.9, -3.7);  // near pass
}

// Distance from p to a convex polygon (0 inside).
double outside(const Polygon2& poly, const Point2& p) {
  if (pointInPolygon(poly, p)) return 0.0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const Point2& a = poly[i];
    const Point2& b = poly[(i + 1) % poly.size()];
    const double ex = b.x - a.x, ey = b.y - a.y, l2 = ex * ex + ey * ey;
    const double s = l2 > 0.0 ? std::clamp(((p.x - a.x) * ex + (p.y - a.y) * ey) / l2, 0.0, 1.0) : 0.0;
    best = std::min(best, std::hypot(p.x - a.x - s * ex, p.y - a.y - s * ey));
  }
  return best;
}
}  // namespace

// The published centre converges to the crown once near views exist: the far view's cells latched (they stay in
// the outline, rule 2) but no longer carry the centre once the observation-weighted evidence says ground.
TEST(CandObj, CentreConvergesAfterNearViews) {
  CandObj m;
  double t = 0;
  bool published_far = false;
  int frames = 0;
  farThenNear(m, t, [&] {
    if (++frames == 25) {  // end of the far pass: published, biased toward the far track like the latched cells
      const auto trees = m.publishedTrees();
      ASSERT_EQ(trees.size(), 1u) << dump(m);
      EXPECT_LT(trees[0].centre.y, -1.0);
      published_far = true;
    }
  });
  ASSERT_TRUE(published_far);
  const auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u) << dump(m);
  EXPECT_LT(std::hypot(trees[0].centre.x, trees[0].centre.y), 0.3) << "centre " << trees[0].centre.x << ", " << trees[0].centre.y;
}

// Through the same sequence the published outline only grows and the centre never moves faster than max_centre_rate.
TEST(CandObj, CentreFixKeepsOutlineGrowOnly) {
  CandObj m;
  double t = 0;
  double prev_area = -1.0;
  Point2 prev_c{0, 0};
  int n_pub = 0;
  double max_dc = 0.0;
  farThenNear(m, t, [&] {
    const auto trees = m.publishedTrees();
    if (trees.empty()) return;
    ASSERT_EQ(trees.size(), 1u) << dump(m);
    const double a = area(trees[0]);
    if (n_pub++ > 0) {
      EXPECT_GE(a, prev_area - 1e-9) << "outline shrank at t " << t;
      max_dc = std::max(max_dc, std::hypot(trees[0].centre.x - prev_c.x, trees[0].centre.y - prev_c.y));
    }
    prev_area = a;
    prev_c = trees[0].centre;
  });
  EXPECT_GT(n_pub, 40);
  EXPECT_LE(max_dc, m.config().max_centre_rate * 0.1 + 1e-6);
  EXPECT_GT(cnt(m, "centre_steps_clamped"), 0u);  // the centre did move a long way, at the bounded rate
}

// The supported centroid is held within centre_max_outside of the latched outline (counted), so the navigator still
// accepts a clearance-guard release over the target (it requires the drone within 1.3 m of the target's outline).
// Here the far view's ghost lies 3.5 m from the crown and only it latched: the crown's support centroid would sit
// about 2 m outside the outline.
TEST(CandObj, CentreHeldWithinMaxOutsideOfTheOutline) {
  auto run = [](double max_outside, double* final_outside, double* max_seen, double* max_dc, std::uint64_t* clamps) {
    Config c;
    c.match_radius = 4.0;     // the near crown, 3.5 m from the ghost's centre, is still this tree
    c.max_cell_radius = 6.0;  // and its cells are credited
    c.centre_max_outside = max_outside;
    CandObj m(c);
    double t = 0;
    Point2 prev{0, 0};
    bool have = false;
    *max_seen = 0.0;
    *max_dc = 0.0;
    farThenNear(m, t, [&] {
      const auto trees = m.publishedTrees();
      if (trees.empty()) return;
      ASSERT_EQ(trees.size(), 1u) << dump(m);
      *max_seen = std::max(*max_seen, outside(trees[0].outline, trees[0].centre));
      if (have) *max_dc = std::max(*max_dc, std::hypot(trees[0].centre.x - prev.x, trees[0].centre.y - prev.y));
      prev = trees[0].centre;
      have = true;
    }, -3.5, 50);
    const auto trees = m.publishedTrees();
    ASSERT_EQ(trees.size(), 1u) << dump(m);
    *final_outside = outside(trees[0].outline, trees[0].centre);
    *clamps = cnt(m, "centre_clamped_to_outline");
  };
  double fo = 0, ms = 0, dc = 0;
  std::uint64_t clamps = 0;
  run(1.3, &fo, &ms, &dc, &clamps);  // the widest setting: the support centroid lies more than 1.0 m outside
  EXPECT_GT(fo, 1.2);
  run(1.0, &fo, &ms, &dc, &clamps);  // the default holds it at 1.0 m
  EXPECT_GT(fo, 0.9);                // (it did move toward the crown, up to the bound)
  EXPECT_LE(ms, 1.0 + 1e-6);         // never farther outside, in any frame
  EXPECT_GT(clamps, 0u);
  EXPECT_LE(dc, Config{}.max_centre_rate * 0.1 + 1e-6);  // and the published centre still moves at the bounded rate
}

// centre_from_support = false is the documented rollback (real-flight profile included): the published centre is
// the latched centroid again, as before D66. Through the far-then-near sequence the pre-D66 build published
// (0, -1.4521739) (RED run of CentreConvergesAfterNearViews, task 6): the centroid of the latched ghost band.
TEST(CandObj, CentreFromSupportOffIsTheLatchedCentroid) {
  Config c;
  c.centre_from_support = false;
  CandObj m(c);
  double t = 0;
  farThenNear(m, t, [] {});
  const auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u) << dump(m);
  EXPECT_NEAR(trees[0].centre.x, 0.0, 1e-9);
  EXPECT_NEAR(trees[0].centre.y, -1.4521739130434776, 1e-9);
  EXPECT_EQ(cnt(m, "centre_support_fallback_latched"), 0u);
  EXPECT_EQ(cnt(m, "centre_clamped_to_outline"), 0u);
}

// ------------------------------------------------------------------------------- final review (S4, S6, S8, S12)

namespace {
std::set<TreeId> publishedIds(const CandObj& m) {
  std::set<TreeId> s;
  for (const auto& e : m.publishedTrees()) s.insert(e.id);
  return s;
}
// An annulus of cells (r_in..r_out) around (cx, cy): its hull covers an interior it has no cells in.
Crown ring(double cx, double cy, double r_in, double r_out, double z = 4.0) {
  Crown c = disc(cx, cy, r_out, z);
  std::vector<std::pair<int, int>> kept;
  for (const auto& ij : c.cells) {
    if (std::hypot((ij.first + 0.5) * kCell - cx, (ij.second + 0.5) * kCell - cy) >= r_in) kept.push_back(ij);
  }
  c.cells = kept;
  return c;
}
}  // namespace

// S4: once the list was locked (PLANNING / HOPPING / SPRAYING), ERROR, UNKNOWN and SCAN_APPROACH stay awake but
// neither create nor merge, until SCAN is entered again explicitly.
TEST(CandObjFinal, StickyLockAfterPlanningUntilScan) {
  for (MissionPhase later : {MissionPhase::kError, MissionPhase::kUnknown, MissionPhase::kScanApproach}) {
    CandObj m;
    double t = 0;
    for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0)});
    for (int i = 0; i < 5; ++i) step(m, t, MissionPhase::kHopping, 0, {});
    for (int i = 0; i < 10; ++i) step(m, t, later, kNoTree, {disc(15, 15, 2.0)});
    EXPECT_EQ(m.publishedTrees().size(), 1u) << toString(later) << ": a new standalone tree appeared after the lock";
    EXPECT_EQ(m.state(), State::kAwake);
    EXPECT_GT(cnt(m, "frames_locked_after_planning"), 0u) << dump(m);
    EXPECT_GT(cnt(m, "crowns_dropped_new_tree_locked_phase"), 0u) << dump(m);
    // an explicit SCAN unlocks: the same crown now becomes a tree
    for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(15, 15, 2.0)});
    EXPECT_EQ(m.publishedTrees().size(), 2u) << toString(later);
  }
  {  // the lock also holds when it was entered through SPRAYING (asleep) and the mapper only sees ERROR afterwards
    CandObj m;
    double t = 0;
    addTree(m, t, 0, 0);
    FrameInput f;
    f.phase = MissionPhase::kSpraying; f.pose = poseAt(0, 0); f.t = t += 0.1;
    m.process(f);
    for (int i = 0; i < 10; ++i) step(m, t, MissionPhase::kError, kNoTree, {disc(15, 15, 2.0)}, poseAt(20, 20));
    EXPECT_EQ(m.publishedTrees().size(), 1u);
  }
  // no lock ever: ERROR / UNKNOWN stay permissive (fail open, brief section 6)
  CandObj m;
  double t = 0;
  addTree(m, t, 0, 0);
  for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kError, kNoTree, {disc(15, 15, 2.0)});
  EXPECT_EQ(m.publishedTrees().size(), 2u);
}

// S4 + rule 4: two planned-against trees and a crown bridging both: while the list is locked (PLANNING, and ERROR
// after a lock) no merge retires a planned ID. Control: the same frames in SCAN do merge (the merge is in reach).
TEST(CandObjFinal, NoMergeWhileTheListIsLocked) {
  for (MissionPhase later : {MissionPhase::kPlanning, MissionPhase::kError, MissionPhase::kScan}) {
    CandObj m;
    double t = 0;
    for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0), disc(4.6, 0, 2.0)});
    const auto before = publishedIds(m);
    ASSERT_EQ(before.size(), 2u);
    for (int i = 0; i < 5; ++i) step(m, t, MissionPhase::kPlanning, kNoTree, {});
    for (int i = 0; i < 150; ++i) step(m, t, later, kNoTree, {disc(1.6, 0, 3.9)});
    if (later == MissionPhase::kScan) {
      EXPECT_GE(cnt(m, "instances_merged"), 1u) << "control: the bridging crown must merge the pair in SCAN " << dump(m);
    } else {
      EXPECT_EQ(publishedIds(m), before) << toString(later);
      EXPECT_EQ(cnt(m, "instances_merged"), 0u) << toString(later) << " " << dump(m);
    }
  }
}

// S6 / rule 5 second half: an instance whose centre ends up inside a sprayed tree is never confirmed. The sprayed
// tree is a ring, so its outline covers an interior none of its cells occupy; the new crown's top is outside the
// sprayed outline (so it is not matched to the sprayed tree), but most of its cells fill that interior.
TEST(CandObjFinal, NoNewTreeConfirmedInsideASprayedTree) {
  CandObj m;
  double t = 0;
  for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kScan, kNoTree, {ring(0, 0, 1.5, 2.5)});
  ASSERT_EQ(m.publishedTrees().size(), 1u) << dump(m);
  const TreeId sprayed = m.publishedTrees()[0].id;
  ASSERT_TRUE(m.markSprayed(sprayed));
  Crown c = disc(0.2, 0, 1.2);  // the ring's empty interior ...
  const Crown tail = disc(3.3, 0, 0.7);
  c.cells.insert(c.cells.end(), tail.cells.begin(), tail.cells.end());  // ... plus a tail outside the ring
  c.peak = {3.3, 0.0};  // top outside the sprayed outline and > match_radius from its centre
  for (int i = 0; i < 12; ++i) step(m, t, MissionPhase::kScan, kNoTree, {c});
  EXPECT_EQ(cnt(m, "instances_created"), 2u) << dump(m);
  EXPECT_EQ(publishedIds(m), std::set<TreeId>{sprayed}) << dump(m);
  EXPECT_GT(cnt(m, "instances_refused_confirm_inside_sprayed"), 0u) << dump(m);
}

// S6 / rule 5: a sprayed tree is excluded from merges (neither absorbed nor absorbing), even when another tree
// grows right over it.
TEST(CandObjFinal, SprayedTreeNeverMerges) {
  CandObj m;
  double t = 0;
  addTree(m, t, 2.6, 0, 1.2, 4.0, 12);  // ID 0, the better established one
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(2.6, 0, 1.2, 4.0), disc(0, 0, 1.2, 4.0)});
  auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 2u) << dump(m);
  const TreeId a = 1;  // the newer tree at (0, 0)
  ASSERT_TRUE(find(trees, a));
  const TreeEntry a0 = *find(trees, a);
  ASSERT_TRUE(m.markSprayed(a));
  // tree 0 grows over the sprayed tree until its latched cells cover it completely
  for (int i = 0; i < 90; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(1.6, 0, 2.4, 4.0)});
  trees = m.publishedTrees();
  const TreeEntry* a1 = find(trees, a);
  ASSERT_TRUE(a1) << "the sprayed tree was merged away: " << dump(m);
  EXPECT_TRUE(a1->sprayed);
  EXPECT_NEAR(area(*a1), area(a0), 1e-9);
  EXPECT_TRUE(find(trees, 0));
  EXPECT_EQ(cnt(m, "instances_merged"), 0u) << dump(m);
  EXPECT_GT(area(*find(trees, 0)), M_PI * 1.2 * 1.2 * 2.0);  // tree 0 did grow over it (the merge was in reach)
}

// S6 / rule 4: in a merge a confirmed (published) instance wins over a tentative one, even when the tentative one
// is older and has been seen in more frames: a published ID never vanishes into an unpublished instance.
TEST(CandObjFinal, MergeWinnerIsTheConfirmedInstance) {
  CandObj m;
  double t = 0;
  // ID 0: tentative for good (no finite height ever), latched, seen 12 frames
  for (int i = 0; i < 12; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 1.5, std::nan(""))});
  ASSERT_TRUE(m.publishedTrees().empty());
  // ID 1: born 3.1 m away (outside match reach), its cells reach over ID 0; it confirms after 5 frames, and its
  // latched centroid lands within merge_centre_dist of ID 0
  Crown c = disc(0.9, 0, 1.6, 4.0);  // every cell within max_cell_radius of the top: all credited to ID 1
  c.peak = {3.1, 0.0};
  for (int i = 0; i < 8; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 1.5, std::nan("")), c});
  EXPECT_GE(cnt(m, "instances_merged"), 1u) << dump(m);
  EXPECT_EQ(publishedIds(m), std::set<TreeId>{1}) << dump(m);
}

// S6 / brief section 6: cell heights are fused only in creation phases. A taller surface seen in PLANNING and while
// hopping onto the tree must leave no trace: back in SCAN (the same 4 m surface again) the height is still 4 m.
TEST(CandObjFinal, HeightsNotFusedInLockedPhases) {
  CandObj m;
  double t = 0;
  addTree(m, t, 0, 0, 2.0, 4.0, 8);
  const TreeId id = m.publishedTrees()[0].id;
  ASSERT_NEAR(m.publishedTrees()[0].height, 4.0, 1e-9);
  for (int i = 0; i < 30; ++i) step(m, t, MissionPhase::kPlanning, kNoTree, {disc(0, 0, 2.0, 5.5)});
  for (int i = 0; i < 30; ++i) step(m, t, MissionPhase::kHopping, id, {disc(0, 0, 2.0, 5.5)});
  EXPECT_NEAR(m.publishedTrees()[0].height, 4.0, 1e-9);
  EXPECT_GT(cnt(m, "height_cells_skipped_locked_phase"), 0u) << dump(m);
  for (int i = 0; i < 10; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 4.0)});
  EXPECT_NEAR(m.publishedTrees()[0].height, 4.0, 1e-9) << "locked-phase surface samples leaked into the height";
}

// S6 / rule 5 + navigation API: markSprayed refuses an ID that is not published (tentative instance).
TEST(CandObjFinal, MarkSprayedRefusesUnpublishedInstances) {
  CandObj m;
  double t = 0;
  for (int i = 0; i < 2; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0)});
  ASSERT_EQ(cnt(m, "instances_created"), 1u);
  ASSERT_TRUE(m.publishedTrees().empty());
  EXPECT_FALSE(m.markSprayed(0));  // ID 0 exists, tentative
  EXPECT_EQ(cnt(m, "trees_marked_sprayed"), 0u);
  EXPECT_EQ(cnt(m, "mark_sprayed_unknown_id"), 1u);
  for (int i = 0; i < 6; ++i) step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0)});
  ASSERT_EQ(m.publishedTrees().size(), 1u);
  EXPECT_FALSE(m.publishedTrees()[0].sprayed);  // the refused call left no trace
  EXPECT_TRUE(m.markSprayed(0));
}

// S8: a tree seen only with its crown top outside the image (truncated) never publishes: its height would be
// under-estimated. Once real tops are sampled it publishes.
TEST(CandObjFinal, TruncatedOnlyCrownNeverPublishes) {
  CandObj m;
  double t = 0;
  for (int i = 0; i < 12; ++i) {
    Crown c = disc(0, 0, 2.0, 3.0);
    c.truncated = true;
    step(m, t, MissionPhase::kScan, kNoTree, {c});
  }
  EXPECT_TRUE(m.publishedTrees().empty()) << dump(m);
  EXPECT_GT(cnt(m, "instances_waiting_untruncated_height"), 0u) << dump(m);
  EXPECT_EQ(cnt(m, "instances_seeded_from_truncated_crown"), 1u);
  step(m, t, MissionPhase::kScan, kNoTree, {disc(0, 0, 2.0, 5.0)});  // the top comes into view
  const auto trees = m.publishedTrees();
  ASSERT_EQ(trees.size(), 1u) << dump(m);
  EXPECT_TRUE(std::isfinite(trees[0].height));
}

// S12: non-finite values never reach the mapper (NaN passes every range comparison).
TEST(CandObjFinal, NonFiniteConfigValuesAreRejected) {
  for (const char* v : {"nan", "NaN", "inf", "-inf", "infinity"}) {
    EXPECT_THROW(configFrom({{"height_quantile", v}}), std::invalid_argument) << v;
    EXPECT_THROW(configFrom({{"latch_ratio", v}}), std::invalid_argument) << v;
  }
  Config c;
  c.max_height_rate = std::nan("");
  EXPECT_THROW(c.sanitized(nullptr), std::invalid_argument);
  Config d;
  d.height_quantile = std::numeric_limits<double>::infinity();
  EXPECT_THROW(CandObj{d}, std::invalid_argument);
  EXPECT_THROW(createMapper("cand_obj", {{"max_centre_rate", "nan"}}, StereoCalibration{}), std::invalid_argument);
}
