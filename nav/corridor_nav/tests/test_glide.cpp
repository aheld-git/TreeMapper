#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "corridor_nav/glide.hpp"

using namespace corridor_nav;
using treemapper::Point2;
using treemapper::Polygon2;
using treemapper::TreeEntry;
using treemapper::TreeId;

namespace {

constexpr double kPi = 3.14159265358979323846;

Polygon2 circle(double cx, double cy, double r) {
  Polygon2 p;
  for (int i = 0; i < 24; ++i) p.push_back({cx + r * std::cos(2 * kPi * i / 24), cy + r * std::sin(2 * kPi * i / 24)});
  return p;
}

TreeEntry tree(TreeId id, double x, double y, double r, double h) {
  TreeEntry t;
  t.id = id;
  t.outline = circle(x, y, r);
  t.centre = {x, y};
  t.height = h;
  return t;
}

double requirement(const std::vector<TreeEntry>& map, const Config& c, TreeId from_id, TreeId to_id, double x, double y) {
  const double reach = c.drone_radius + c.lateral_margin;
  double req = -1e9;
  for (const TreeEntry& k : map) {
    if (distanceToPolygon(k.outline, {x, y}) > reach) continue;
    if (k.id == to_id || k.id == from_id) req = std::max(req, k.height + (c.spray_clearance - c.descent_slack));
    else req = std::max(req, k.height + c.transit_clearance);
  }
  return req;
}

}  // namespace

TEST(Glide, DistanceToPolygon) {
  Polygon2 sq = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  EXPECT_DOUBLE_EQ(distanceToPolygon(sq, {1, 1}), 0.0);
  EXPECT_DOUBLE_EQ(distanceToPolygon(sq, {5, 1}), 3.0);
  EXPECT_NEAR(distanceToPolygon(sq, {3, 3}), std::sqrt(2.0), 1e-12);
  Polygon2 cw(sq.rbegin(), sq.rend());
  EXPECT_DOUBLE_EQ(distanceToPolygon(cw, {1, 1}), 0.0);
  EXPECT_DOUBLE_EQ(distanceToPolygon(cw, {-1, 1}), 1.0);
}

TEST(Glide, StraightWhenGapIsClear) {
  const Config cfg;
  const std::vector<TreeEntry> map = {tree(1, 0, 0, 1.0, 5.0), tree(2, 5, 0, 1.0, 5.5)};
  const Point3 a = sprayPoint(map[0], map, cfg, nullptr);
  EXPECT_DOUBLE_EQ(a.z, 7.5);
  const Path3 p = planGlide(a, 1, map[1], map, cfg, nullptr);
  ASSERT_EQ(p.pts.size(), 2u);
  EXPECT_FALSE(p.climbed);
  EXPECT_DOUBLE_EQ(p.pts.front().z, 7.5);
  EXPECT_DOUBLE_EQ(p.pts.back().z, 8.0);
  EXPECT_DOUBLE_EQ(p.pts.back().x, 5.0);
}

TEST(Glide, ClimbsOverTallerMiddleTree) {
  const Config cfg;
  const std::vector<TreeEntry> map = {tree(1, 0, 0, 1.0, 4.0), tree(2, 5, 0, 1.0, 7.0), tree(3, 10, 0, 1.0, 4.0)};
  Counters c;
  const Point3 a = sprayPoint(map[0], map, cfg, &c);
  const Path3 p = planGlide(a, 1, map[2], map, cfg, &c);
  EXPECT_TRUE(p.climbed);
  EXPECT_EQ(c["glide_climbs_inserted"], 1u);
  ASSERT_EQ(p.pts.size(), 4u);
  EXPECT_GE(p.pts[1].z, 9.0);
  EXPECT_DOUBLE_EQ(p.pts[1].x, 0.0);
  EXPECT_DOUBLE_EQ(p.pts[2].x, 10.0);
  EXPECT_DOUBLE_EQ(p.pts[2].z, p.pts[1].z);
  EXPECT_DOUBLE_EQ(p.pts[3].z, 6.5);
  // the level leg is the only one that passes C's outline, and it is above 9
  EXPECT_DOUBLE_EQ(p.pts[1].z, 9.0);
}

TEST(Glide, NeverBelowAnyObstacleRequirement) {
  const Config cfg;
  std::mt19937 rng(42);
  auto uni = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
  int climbs = 0;
  for (int iter = 0; iter < 200; ++iter) {
    const int n = 5 + static_cast<int>(rng() % 11);
    std::vector<TreeEntry> map;
    for (int i = 0; i < n; ++i) map.push_back(tree(i + 1, uni(0, 25), uni(0, 25), uni(0.8, 2.0), uni(3, 8)));
    const std::size_t fi = rng() % map.size();
    std::size_t ti = rng() % map.size();
    while (ti == fi) ti = rng() % map.size();
    const TreeEntry& fromT = map[fi];
    const TreeEntry& toT = map[ti];
    const Point3 from = sprayPoint(fromT, map, cfg, nullptr);
    const Path3 p = planGlide(from, fromT.id, toT, map, cfg, nullptr);
    climbs += p.climbed ? 1 : 0;
    ASSERT_GE(p.pts.size(), 2u);
    EXPECT_DOUBLE_EQ(p.pts.front().z, from.z);
    const Point3 pt = sprayPoint(toT, map, cfg, nullptr);
    EXPECT_DOUBLE_EQ(p.pts.back().x, pt.x);
    EXPECT_DOUBLE_EQ(p.pts.back().y, pt.y);
    EXPECT_DOUBLE_EQ(p.pts.back().z, pt.z);
    for (std::size_t i = 0; i + 1 < p.pts.size(); ++i) {
      const Point3 &a = p.pts[i], &b = p.pts[i + 1];
      const double len = std::hypot(b.x - a.x, b.y - a.y);
      if (len < 1e-12) {  // vertical leg: both ends at this xy
        const double req = requirement(map, cfg, fromT.id, toT.id, a.x, a.y);
        EXPECT_GE(std::min(a.z, b.z) + 1e-9, req) << "iter " << iter << " vertical leg " << i;
        continue;
      }
      const int steps = static_cast<int>(std::ceil(len / 0.05));
      for (int s = 0; s <= steps; ++s) {
        const double d = std::min(s * 0.05, len), t = d / len;
        const double x = a.x + t * (b.x - a.x), y = a.y + t * (b.y - a.y), z = a.z + t * (b.z - a.z);
        const double req = requirement(map, cfg, fromT.id, toT.id, x, y);
        ASSERT_GE(z + 1e-9, req) << "iter " << iter << " leg " << i << " s " << d;
      }
    }
  }
  EXPECT_GT(climbs, 10);  // the random maps exercise the detour as well as the straight case
}

TEST(Glide, SprayPointRaisedOverOverlappingNeighbour) {
  const Config cfg;
  // N's outline contains T's centre (T is the half hidden under N's crown)
  const std::vector<TreeEntry> map = {tree(1, 0, 0, 1.0, 4.0), tree(2, 0.5, 0, 2.0, 6.0), tree(3, 12, 0, 1.0, 4.0)};
  Counters c;
  const Point3 p = sprayPoint(map[0], map, cfg, &c);
  EXPECT_DOUBLE_EQ(p.z, 8.0);
  EXPECT_EQ(c["spray_point_raised"], 1u);
  // a far tree does not raise it, and a null counter is fine
  EXPECT_DOUBLE_EQ(sprayPoint(map[2], map, cfg, nullptr).z, 6.5);

  Counters g;
  const Point3 from = sprayPoint(map[2], map, cfg, nullptr);
  const Path3 path = planGlide(from, 3, map[0], map, cfg, &g);
  ASSERT_GE(path.pts.size(), 2u);
  EXPECT_DOUBLE_EQ(path.pts.back().x, p.x);
  EXPECT_DOUBLE_EQ(path.pts.back().y, p.y);
  EXPECT_DOUBLE_EQ(path.pts.back().z, p.z);
  EXPECT_EQ(g["spray_point_raised"], 1u);
}

TEST(Glide, PathTimeAndCarrot) {
  const Config cfg;  // max_speed_xy 4, max_speed_z 1.5, carrot_lead_s 0.5
  Path3 path;
  path.pts = {{0, 0, 10}, {8, 0, 10}, {8, 0, 7}};
  EXPECT_DOUBLE_EQ(pathTime(path, cfg), 8.0 / 4.0 + 3.0 / 1.5);
  const Point3 c = carrot(path, {0, 0, 10}, cfg);
  EXPECT_NEAR(c.x, 2.0, 1e-12);
  EXPECT_NEAR(c.z, 10.0, 1e-12);
  // off the path: projected first, then advanced
  const Point3 d = carrot(path, {4, 1.5, 10.2}, cfg);
  EXPECT_NEAR(d.x, 6.0, 1e-12);
  EXPECT_NEAR(d.y, 0.0, 1e-12);
  // the lead carries over a corner: 1 m before the corner (0.25 s) + 0.25 s of the descent (0.375 m)
  const Point3 e = carrot(path, {7, 0, 10}, cfg);
  EXPECT_NEAR(e.x, 8.0, 1e-12);
  EXPECT_NEAR(e.z, 10.0 - 0.375, 1e-12);
  // clamped at the end
  const Point3 f = carrot(path, {8, 0, 7.1}, cfg);
  EXPECT_NEAR(f.x, 8.0, 1e-12);
  EXPECT_NEAR(f.z, 7.0, 1e-12);
  // degenerate paths
  EXPECT_NEAR(carrot(Path3{}, {1, 2, 3}, cfg).z, 3.0, 1e-12);
  Path3 one;
  one.pts = {{5, 5, 5}};
  EXPECT_NEAR(carrot(one, {1, 2, 3}, cfg).x, 5.0, 1e-12);
  EXPECT_DOUBLE_EQ(pathTime(one, cfg), 0.0);
}

TEST(Glide, ReplanWhenHeightMoves) {
  const Config cfg;
  std::vector<TreeEntry> map = {tree(7, 0, 0, 1.0, 5.4)};
  EXPECT_TRUE(needsReplan({{7, 5.0}}, map, cfg));
  map[0].height = 5.2;
  EXPECT_FALSE(needsReplan({{7, 5.0}}, map, cfg));
  map[0].height = 4.6;
  EXPECT_TRUE(needsReplan({{7, 5.0}}, map, cfg));
  // a planned tree that left the map
  EXPECT_TRUE(needsReplan({{8, 5.0}}, map, cfg));
  EXPECT_FALSE(needsReplan({}, map, cfg));
}
