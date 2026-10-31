#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numeric>

#include "corridor_nav/ordering.hpp"

using namespace corridor_nav;
using treemapper::TreeId;

namespace {

double euclid(const Point3& a, const Point3& b) { return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)); }

double pathCost(const std::vector<Stop>& s, const Point3& start, const Point3& end, const CostFn& cost) {
  double c = 0.0;
  Point3 prev = start;
  for (const Stop& x : s) {
    c += cost(prev, x.p);
    prev = x.p;
  }
  return c + cost(prev, end);
}

std::vector<TreeId> ids(const std::vector<Stop>& s) {
  std::vector<TreeId> v;
  for (const Stop& x : s) v.push_back(x.id);
  return v;
}

Stop stop(TreeId id, double x, double y) { return Stop{id, Point3{x, y, 0.0}}; }

std::function<double()> neverCalledClock() {
  return [] { return 0.0; };
}

}  // namespace

TEST(Ordering, StartsAtExitEndsAtEntry) {
  std::vector<Stop> s;
  const double us[] = {5, 20, 0, 25, 10, 15};
  TreeId id = 0;
  for (double u : us) s.push_back(stop(id++, u, 0));
  const auto r = orderTargets(s, Point3{30, 0, 0}, Point3{-5, 0, 0}, 0.0, euclid, 5.0, neverCalledClock());
  EXPECT_EQ(ids(r), (std::vector<TreeId>{3, 1, 5, 4, 0, 2}));  // u = 25, 20, 15, 10, 5, 0
}

TEST(Ordering, AxialSortFollowsTheHeading) {
  // heading +y: descending y
  std::vector<Stop> s{stop(1, 0, 2), stop(2, 0, 9), stop(3, 0, 5)};
  const auto r = orderTargets(s, Point3{0, 12, 0}, Point3{0, -2, 0}, 1.5707963267948966, euclid, 5.0, neverCalledClock());
  EXPECT_EQ(ids(r), (std::vector<TreeId>{2, 3, 1}));
}

TEST(Ordering, EmptyAndSingle) {
  EXPECT_TRUE(orderTargets({}, Point3{}, Point3{}, 0.0, euclid, 5.0, neverCalledClock()).empty());
  const auto r = orderTargets({stop(4, 1, 1)}, Point3{}, Point3{}, 0.0, euclid, 5.0, neverCalledClock());
  EXPECT_EQ(ids(r), std::vector<TreeId>{4});
}

TEST(Ordering, TwoOptFixesStaggeredRows) {
  // two rows at v = +-1.5, staggered by 2 m; the axial sort zigzags between them
  std::vector<Stop> s;
  for (int i = 0; i < 4; ++i) {
    s.push_back(stop(10 + i, 4.0 * i, 1.5));
    s.push_back(stop(20 + i, 4.0 * i + 2.0, -1.5));
  }
  const Point3 start{18, 0, 0}, end{-4, 0, 0};
  std::vector<Stop> axial = s;
  std::stable_sort(axial.begin(), axial.end(), [](const Stop& a, const Stop& b) { return a.p.x > b.p.x; });
  const double axial_cost = pathCost(axial, start, end, euclid);

  const auto r = orderTargets(s, start, end, 0.0, euclid, 1e9, [] { return 0.0; });
  const double c = pathCost(r, start, end, euclid);
  EXPECT_LE(c, axial_cost + 1e-9);

  // a deliberately bad order: rows interleaved front to back
  std::vector<Stop> bad = s;
  std::sort(bad.begin(), bad.end(), [](const Stop& a, const Stop& b) { return a.id % 10 == b.id % 10 ? a.id < b.id : a.id % 10 < b.id % 10; });
  EXPECT_LE(c, pathCost(bad, start, end, euclid));

  auto sorted_ids = ids(r);
  std::sort(sorted_ids.begin(), sorted_ids.end());
  auto want = ids(s);
  std::sort(want.begin(), want.end());
  EXPECT_EQ(sorted_ids, want);
}

TEST(Ordering, TwoOptBeatsAxialSortWhenRowsAreFarApart) {
  // rows 16 m apart with interleaved u: the axial sort crosses three times, the optimum once
  const std::vector<Stop> s{stop(1, 10, 8), stop(2, 9, -8), stop(3, 8, 8), stop(4, 7, -8)};
  const Point3 start{30, 0, 0}, end{-5, 0, 0};
  const auto r = orderTargets(s, start, end, 0.0, euclid, 1e9, [] { return 0.0; });
  EXPECT_EQ(ids(r), (std::vector<TreeId>{1, 3, 2, 4}));
  EXPECT_LT(pathCost(r, start, end, euclid), pathCost(s, start, end, euclid) - 20.0);
}

TEST(Ordering, FourStopCaseEqualsBruteForce) {
  const std::vector<Stop> s{stop(1, 9, 1.5), stop(2, 7, -1.5), stop(3, 3, 1.5), stop(4, 1, -1.5)};
  const Point3 start{12, 0, 0}, end{-2, 0, 0};
  const auto r = orderTargets(s, start, end, 0.0, euclid, 1e9, [] { return 0.0; });
  std::vector<std::size_t> perm(s.size());
  std::iota(perm.begin(), perm.end(), 0);
  double best = 1e18;
  do {
    std::vector<Stop> p;
    for (std::size_t k : perm) p.push_back(s[k]);
    best = std::min(best, pathCost(p, start, end, euclid));
  } while (std::next_permutation(perm.begin(), perm.end()));
  EXPECT_NEAR(pathCost(r, start, end, euclid), best, 1e-9);
}

TEST(Ordering, AsymmetricCostIsHonoured) {
  // moving towards +x is 10x dearer than towards -x, so the descending-u sweep is already optimal and must not be reversed
  const CostFn asym = [](const Point3& a, const Point3& b) { return (b.x > a.x ? 10.0 : 1.0) * euclid(a, b); };
  std::vector<Stop> s{stop(1, 4, 0), stop(2, 8, 3), stop(3, 12, 0), stop(4, 16, 3)};
  const Point3 start{20, 0, 0}, end{0, 0, 0};
  const auto r = orderTargets(s, start, end, 0.0, asym, 1e9, [] { return 0.0; });
  EXPECT_LE(pathCost(r, start, end, asym), pathCost({s[3], s[2], s[1], s[0]}, start, end, asym) + 1e-9);
}

TEST(Ordering, EqualCostTiesKeepTheCurrentOrder) {
  // all stops coincide: every order costs the same, the (stable) axial order must survive
  std::vector<Stop> s{stop(5, 3, 0), stop(6, 3, 0), stop(7, 3, 0)};
  const auto r = orderTargets(s, Point3{9, 0, 0}, Point3{-9, 0, 0}, 0.0, euclid, 1e9, [] { return 0.0; });
  EXPECT_EQ(ids(r), (std::vector<TreeId>{5, 6, 7}));
}

TEST(Ordering, RespectsBudget) {
  // rows 16 m apart: the unlimited search needs several passes, the 5 ms budget must cut it after the first
  const std::vector<Stop> s{stop(1, 10, 8), stop(2, 9, -8), stop(3, 8, 8), stop(4, 7, -8)};
  const Point3 start{30, 0, 0}, end{-5, 0, 0};
  auto run = [&](double budget, int* calls) {
    double t = 0.0;
    *calls = 0;
    return orderTargets(s, start, end, 0.0, euclid, budget, [&] {
      ++*calls;
      const double v = t;
      t += 10.0;
      return v;
    });
  };
  int limited_calls = 0, full_calls = 0;
  const auto limited = run(5.0, &limited_calls);
  EXPECT_EQ(limited_calls, 2);  // t0, then one pass ending at 10 ms > 5 ms
  auto a = ids(limited), b = ids(s);
  std::sort(a.begin(), a.end());
  std::sort(b.begin(), b.end());
  EXPECT_EQ(a, b);

  const auto full = run(1e9, &full_calls);
  EXPECT_GT(full_calls, 2);
  EXPECT_LE(pathCost(full, start, end, euclid), pathCost(limited, start, end, euclid) + 1e-9);
}

TEST(Ordering, UnlimitedBudgetStopsWhenNoPassImproves) {
  std::vector<Stop> s{stop(1, 10, 0), stop(2, 5, 0)};  // already optimal
  int calls = 0;
  orderTargets(s, Point3{12, 0, 0}, Point3{0, 0, 0}, 0.0, euclid, 1e9, [&] { ++calls; return 0.0; });
  EXPECT_EQ(calls, 2);  // t0 and one non-improving pass
}
