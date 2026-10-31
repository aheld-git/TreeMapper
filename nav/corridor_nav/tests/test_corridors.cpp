#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "corridor_nav/corridors.hpp"
#include "treemapper/geometry.hpp"

using namespace corridor_nav;
using treemapper::Point2;
using treemapper::Polygon2;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

CameraFov fov(double deg) { return CameraFov{deg * kDeg, deg * kDeg}; }

Polygon2 rect(double w, double h, double rot = 0.0) {
  Polygon2 p{{0, 0}, {w, 0}, {w, h}, {0, h}};
  for (auto& q : p) {
    const double x = q.x, y = q.y;
    q = {x * std::cos(rot) - y * std::sin(rot), x * std::sin(rot) + y * std::cos(rot)};
  }
  return p;
}

double dist(const Point2& a, const Point2& b) { return std::hypot(a.x - b.x, a.y - b.y); }

}  // namespace

TEST(CorridorPlan, MaxWidthMatchesFormula) {
  Config c;  // z_scan 15, top 7, radius 2.5, margin 1
  EXPECT_NEAR(maxCorridorWidth(fov(70.0), c), 2 * 8 * std::tan(35.0 * kDeg) - 5 - 1, 1e-9);
  EXPECT_NEAR(maxCorridorWidth(fov(70.0), c), 5.2, 0.01);
}

TEST(CorridorPlan, MaxWidthUsesTheSmallerFov) {
  Config c;
  EXPECT_NEAR(maxCorridorWidth(CameraFov{90.0 * kDeg, 70.0 * kDeg}, c), maxCorridorWidth(fov(70.0), c), 1e-12);
  EXPECT_NEAR(maxCorridorWidth(CameraFov{70.0 * kDeg, 90.0 * kDeg}, c), maxCorridorWidth(fov(70.0), c), 1e-12);
}

TEST(CorridorPlan, HeadingIsLongAxis) {
  const double h = minAreaRectHeading(rect(60, 20, 30.0 * kDeg));
  EXPECT_GE(h, 0.0);
  EXPECT_LT(h, kPi);
  EXPECT_NEAR(std::remainder(h - 30.0 * kDeg, kPi), 0.0, 1e-9);
  // a tall rectangle: long axis along y
  EXPECT_NEAR(minAreaRectHeading(rect(10, 40)), kPi / 2, 1e-9);
}

TEST(CorridorPlan, TilesFieldAndAllEnterOnTheSameSide) {
  Field f;
  f.polygon = rect(60, 20);
  f.heading_rad = 0.0;
  Config c;
  c.corridor_width = 5.0;
  Counters n;
  const CorridorPlan plan = planCorridors(f, fov(90.0), c, &n);
  ASSERT_TRUE(plan.ok) << plan.error;
  EXPECT_NEAR(plan.width, 5.0, 1e-12);
  ASSERT_EQ(plan.corridors.size(), 4u);
  double covered = 0.0;
  for (std::size_t k = 0; k < plan.corridors.size(); ++k) {
    const Corridor& cor = plan.corridors[k];
    EXPECT_EQ(cor.index, static_cast<int>(k));
    EXPECT_NEAR(treemapper::polygonArea(cor.strip), 60.0 * 5.0, 1e-9);  // CCW, 60 x 5
    covered += treemapper::convexIntersectionArea(cor.strip, f.polygon);
    EXPECT_NEAR(dist(cor.entry, cor.exit), 60.0, 1e-9);
    // every corridor is scanned along +x (the field heading); entries line up on the same side
    EXPECT_DOUBLE_EQ(cor.heading_rad, 0.0);
    EXPECT_NEAR(cor.exit.x - cor.entry.x, 60.0, 1e-9);
    if (k > 0) {
      EXPECT_NEAR(dist(cor.entry, plan.corridors[k - 1].entry), 5.0, 1e-9);
    }
  }
  EXPECT_NEAR(covered, 1200.0, 1e-6);
  EXPECT_TRUE(n.empty());
}

TEST(CorridorPlan, DerivesHeadingWhenAbsent) {
  Field f;
  f.polygon = rect(60, 20, 30.0 * kDeg);
  Config c;
  c.corridor_width = 5.0;
  const CorridorPlan plan = planCorridors(f, fov(90.0), c, nullptr);
  ASSERT_TRUE(plan.ok) << plan.error;
  ASSERT_EQ(plan.corridors.size(), 4u);
  EXPECT_NEAR(std::remainder(plan.corridors[0].heading_rad - 30.0 * kDeg, 2 * kPi), 0.0, 1e-9);
  EXPECT_NEAR(dist(plan.corridors[0].entry, plan.corridors[0].exit), 60.0, 1e-9);
}

TEST(CorridorPlan, DefaultWidthIsWMaxAndStripsStayWithinIt) {
  Field f;
  f.polygon = rect(60, 20);
  f.heading_rad = 0.0;
  Config c;  // corridor_width 0 -> the widest allowed
  const CorridorPlan plan = planCorridors(f, fov(70.0), c, nullptr);
  ASSERT_TRUE(plan.ok) << plan.error;
  EXPECT_NEAR(plan.width, plan.w_max, 1e-12);
  ASSERT_EQ(plan.corridors.size(), 4u);  // ceil(20 / 5.2)
  for (const Corridor& cor : plan.corridors) EXPECT_NEAR(treemapper::polygonArea(cor.strip), 60.0 * 5.0, 1e-9);
}

TEST(CorridorPlan, ClampsWidthAndCounts) {
  Field f;
  f.polygon = rect(60, 20);
  f.heading_rad = 0.0;
  Config c;
  c.corridor_width = 50.0;
  Counters n;
  const CorridorPlan plan = planCorridors(f, fov(70.0), c, &n);
  ASSERT_TRUE(plan.ok) << plan.error;
  EXPECT_DOUBLE_EQ(plan.width, plan.w_max);
  EXPECT_EQ(n["corridor_width_clamped"], 1u);
}

TEST(CorridorPlan, RefusesWhenCoverageImpossible) {
  Field f;
  f.polygon = rect(60, 20);
  f.heading_rad = 0.0;
  Config c;
  c.z_scan = 8.0;  // (8 - 7) * 2 tan(20 deg) - 6 < 0
  const CorridorPlan plan = planCorridors(f, fov(40.0), c, nullptr);
  EXPECT_FALSE(plan.ok);
  EXPECT_EQ(plan.error, "coverage impossible");
  EXPECT_TRUE(plan.corridors.empty());
}

TEST(CorridorPlan, RefusesANonFiniteFov) {
  Field f;
  f.polygon = rect(60, 20);
  f.heading_rad = 0.0;
  Config c;
  c.corridor_width = 5.0;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const CameraFov& bad : {CameraFov{nan, nan}, CameraFov{90.0 * kDeg, nan}}) {
    const CorridorPlan plan = planCorridors(f, bad, c, nullptr);
    EXPECT_FALSE(plan.ok);
    EXPECT_EQ(plan.error, "coverage impossible");
    EXPECT_TRUE(plan.corridors.empty());
  }
}

TEST(CorridorPlan, RefusesAnInfiniteWidth) {
  Field f;
  f.polygon = rect(60, 20);
  f.heading_rad = 0.0;
  Config c;  // built directly, bypassing configFrom / sanitized
  c.z_scan = std::numeric_limits<double>::infinity();
  const CorridorPlan plan = planCorridors(f, fov(90.0), c, nullptr);
  EXPECT_FALSE(plan.ok);
  EXPECT_EQ(plan.error, "coverage impossible");
  EXPECT_TRUE(plan.corridors.empty());
}

TEST(CorridorPlan, RefusesAnInvalidField) {
  Config c;
  c.corridor_width = 5.0;
  Field two;
  two.polygon = {{0, 0}, {10, 0}};
  EXPECT_EQ(planCorridors(two, fov(90.0), c, nullptr).error, "field invalid");
  Field flat;
  flat.polygon = {{0, 0}, {10, 0}, {20, 0}};
  const CorridorPlan plan = planCorridors(flat, fov(90.0), c, nullptr);
  EXPECT_FALSE(plan.ok);
  EXPECT_EQ(plan.error, "field invalid");
}

TEST(CorridorPlan, DropsShortStripsAndCounts) {
  // 60 x 10 with a 2 m tall tab sticking up: the top strip's centre line crosses only the 3 m wide tab
  Field f;
  f.polygon = {{0, 0}, {60, 0}, {60, 10}, {30, 10}, {30, 15}, {27, 15}, {27, 10}, {0, 10}};
  f.heading_rad = 0.0;
  Config c;
  c.corridor_width = 5.0;
  Counters n;
  const CorridorPlan plan = planCorridors(f, fov(90.0), c, &n);
  ASSERT_TRUE(plan.ok) << plan.error;
  // extent_v 15 -> 3 strips of 5: centre lines y = 2.5, 7.5, 12.5; the last one is 3 m long (< 5)
  EXPECT_EQ(plan.corridors.size(), 2u);
  EXPECT_EQ(n["corridors_dropped_short"], 1u);
}

TEST(CorridorPlan, LShapedFieldSplitsStripIntoTwoCorridors) {
  Field f;
  f.polygon = {{0, 0}, {40, 0}, {40, 10}, {10, 10}, {10, 30}, {0, 30}};
  f.heading_rad = kPi / 2;  // u along y
  Config c;
  c.corridor_width = 5.0;
  const CorridorPlan plan = planCorridors(f, fov(90.0), c, nullptr);
  ASSERT_TRUE(plan.ok) << plan.error;
  int in_foot = 0, in_leg = 0;
  for (const Corridor& cor : plan.corridors) {
    const Point2 mid{(cor.entry.x + cor.exit.x) / 2, (cor.entry.y + cor.exit.y) / 2};
    EXPECT_TRUE(treemapper::pointInPolygon(f.polygon, mid)) << "corridor " << cor.index;
    const double x = cor.entry.x;  // the scan line is vertical
    EXPECT_NEAR(cor.exit.x, x, 1e-9);
    if (x > 10.0) {
      ++in_foot;
      // the strip over the foot must not run up the leg: its u-range stays within y in [0, 10]
      EXPECT_GE(std::min(cor.entry.y, cor.exit.y), -1e-9);
      EXPECT_LE(std::max(cor.entry.y, cor.exit.y), 10.0 + 1e-9);
    } else {
      ++in_leg;
      EXPECT_NEAR(std::fabs(cor.exit.y - cor.entry.y), 30.0, 1e-9);
    }
  }
  EXPECT_EQ(in_foot, 6);  // x centres 12.5 ... 37.5
  EXPECT_EQ(in_leg, 2);   // x centres 2.5, 7.5
}
