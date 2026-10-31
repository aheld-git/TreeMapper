#include <gtest/gtest.h>

#include "corridor_nav/assignment.hpp"

using namespace corridor_nav;
using treemapper::kNoTree;
using treemapper::Point2;
using treemapper::Polygon2;
using treemapper::TreeEntry;
using treemapper::TreeId;

namespace {

Polygon2 box(double x0, double y0, double x1, double y1) { return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; }

TreeEntry tree(TreeId id, double x, double y, double r = 0.5, bool sprayed = false) {
  TreeEntry t;
  t.id = id;
  t.centre = {x, y};
  t.outline = box(x - r, y - r, x + r, y + r);
  t.sprayed = sprayed;
  return t;
}

/// Field 20 x 10; corridor 0 = y in [0, 5], corridor 1 = y in [5, 10].
struct Fixture {
  Polygon2 field = box(0, 0, 20, 10);
  std::vector<Corridor> corridors;
  Fixture() {
    Corridor a, b;
    a.index = 0;
    a.strip = box(0, 0, 20, 5);
    b.index = 1;
    b.strip = box(0, 5, 20, 10);
    corridors = {a, b};
  }
};

}  // namespace

TEST(Assignment, AssignsOnLockAndIsPermanent) {
  Fixture f;
  std::vector<TreeEntry> map{tree(1, 3, 2), tree(2, 8, 3), tree(3, 5, 7), tree(4, 12, 8)};
  Assignment a;
  Counters n;
  const auto got0 = updateAssignment(map, f.corridors, f.field, 0, &a, &n);
  EXPECT_EQ(got0, (std::vector<TreeId>{1, 2}));
  EXPECT_EQ(a.corridor_of.at(1), 0);
  EXPECT_EQ(a.corridor_of.at(2), 0);
  EXPECT_EQ(a.corridor_of.count(3), 0u);  // corridor 1 is not locked yet
  EXPECT_TRUE(a.cleanup.empty());

  // tree 1 drifts into corridor 1 (map refinement): its assignment stays
  map[0].centre = {3, 7};
  const auto got1 = updateAssignment(map, f.corridors, f.field, 1, &a, &n);
  EXPECT_EQ(got1, (std::vector<TreeId>{3, 4}));
  EXPECT_EQ(a.corridor_of.at(1), 0);
  EXPECT_TRUE(a.cleanup.empty());
  EXPECT_EQ(n.count("late_trees"), 0u);
  // a repeat call assigns nothing new
  EXPECT_TRUE(updateAssignment(map, f.corridors, f.field, 1, &a, &n).empty());
}

TEST(Assignment, LateTreeGoesToCleanup) {
  Fixture f;
  std::vector<TreeEntry> map{tree(1, 3, 2), tree(2, 5, 7)};
  Assignment a;
  Counters n;
  updateAssignment(map, f.corridors, f.field, 0, &a, &n);
  updateAssignment(map, f.corridors, f.field, 1, &a, &n);
  map.push_back(tree(9, 10, 1));  // corridor 0, published after it locked
  const auto got = updateAssignment(map, f.corridors, f.field, 1, &a, &n);
  EXPECT_TRUE(got.empty());
  EXPECT_EQ(a.cleanup, std::vector<TreeId>{9});
  EXPECT_EQ(a.corridor_of.count(9), 0u);
  EXPECT_EQ(n["late_trees"], 1u);
  // seen again, still once
  updateAssignment(map, f.corridors, f.field, 1, &a, &n);
  updateAssignment(map, f.corridors, f.field, -1, &a, &n);
  EXPECT_EQ(a.cleanup, std::vector<TreeId>{9});
  EXPECT_EQ(n["late_trees"], 1u);
}

TEST(Assignment, JustLockedMinusOneOnlyUpdatesCleanupAndOutside) {
  Fixture f;
  std::vector<TreeEntry> map{tree(1, 3, 2), tree(2, 30, 2)};
  Assignment a;
  Counters n;
  EXPECT_TRUE(updateAssignment(map, f.corridors, f.field, -1, &a, &n).empty());
  EXPECT_TRUE(a.corridor_of.empty());
  EXPECT_TRUE(a.cleanup.empty());
  EXPECT_EQ(a.outside_field, std::set<TreeId>{2});
}

TEST(Assignment, SprayedAndOutsideFieldNeverAssigned) {
  Fixture f;
  std::vector<TreeEntry> map{tree(1, 3, 2, 0.5, /*sprayed=*/true), tree(2, 25, 2), tree(3, 6, 2)};
  Assignment a;
  Counters n;
  const auto got = updateAssignment(map, f.corridors, f.field, 0, &a, &n);
  EXPECT_EQ(got, std::vector<TreeId>{3});
  EXPECT_EQ(a.corridor_of.count(1), 0u);
  EXPECT_EQ(a.corridor_of.count(2), 0u);
  EXPECT_EQ(a.outside_field, std::set<TreeId>{2});
  EXPECT_EQ(n["trees_outside_field"], 1u);
  // outside is counted once, and a sprayed tree is never a late tree either
  updateAssignment(map, f.corridors, f.field, 1, &a, &n);
  EXPECT_EQ(n["trees_outside_field"], 1u);
  EXPECT_TRUE(a.cleanup.empty());
  EXPECT_EQ(n.count("late_trees"), 0u);
}

TEST(Assignment, CentreInNoStripButInsideFieldGoesToNearestStrip) {
  Fixture f;
  f.corridors[0].strip = box(0, 0, 20, 4.9);  // a 0.2 m gap between the strips
  f.corridors[1].strip = box(0, 5.1, 20, 10);
  std::vector<TreeEntry> map{tree(1, 5, 4.95), tree(2, 6, 5.08)};
  Assignment a;
  const auto got = updateAssignment(map, f.corridors, f.field, 0, &a, nullptr);  // null counters are fine
  EXPECT_EQ(got, std::vector<TreeId>{1});
  const auto got1 = updateAssignment(map, f.corridors, f.field, 1, &a, nullptr);
  EXPECT_EQ(got1, std::vector<TreeId>{2});
}

TEST(Assignment, ReresolvePrefersContainingOutlineThenNearest) {
  std::vector<TreeEntry> map{tree(7, 0, 0, 2.0), tree(9, 20, 0, 0.3)};
  EXPECT_EQ(reresolveId({1.5, 0.5}, map, 1.5), 7);   // inside 7's outline
  EXPECT_EQ(reresolveId({21.0, 0.0}, map, 1.5), 9);  // 1.0 m from 9's centre, inside nothing
  EXPECT_EQ(reresolveId({22.0, 0.0}, map, 1.5), kNoTree);  // 2.0 m away
  map[0].sprayed = true;
  EXPECT_EQ(reresolveId({1.5, 0.5}, map, 1.5), kNoTree);  // sprayed tree containing the point is skipped
  EXPECT_EQ(reresolveId({1.5, 0.5}, {}, 1.5), kNoTree);
}

TEST(Assignment, ReresolveContainingOutlineBeatsCloserCentre) {
  // Point inside big tree 1's outline, but nearer to small tree 2's centre.
  std::vector<TreeEntry> map{tree(1, 0, 0, 3.0), tree(2, 3.2, 0, 0.1)};
  EXPECT_EQ(reresolveId({2.9, 0.0}, map, 1.5), 1);
}
