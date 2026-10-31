#include "corridor_nav/navigator_support.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "treemapper/geometry.hpp"

namespace corridor_nav {
namespace support {

namespace {
using treemapper::kNoTree;
using treemapper::Point2;
using treemapper::TreeEntry;
using treemapper::TreeId;

constexpr double kZeroLength = 1e-9;        ///< m, consecutive path points closer than this are merged
constexpr double kHeightSampleStep = 0.25;  ///< m, xy sampling of a path when collecting the trees near it
constexpr double kPairCostNs = 5.0;         ///< ns charged per stop pair and 2-opt pass (deterministic order budget)
constexpr double kInf = std::numeric_limits<double>::infinity();

/// Axis-aligned xy bounding box, empty until the first add().
struct Box {
  double x0{kInf}, y0{kInf}, x1{-kInf}, y1{-kInf};
  void add(double x, double y) {
    x0 = std::min(x0, x);
    y0 = std::min(y0, y);
    x1 = std::max(x1, x);
    y1 = std::max(y1, y);
  }
  bool overlaps(const Box& o, double grow) const {
    return x0 - grow <= o.x1 && o.x0 <= x1 + grow && y0 - grow <= o.y1 && o.y0 <= y1 + grow;
  }
};

Box outlineBox(const TreeEntry& t) {
  Box b;
  for (const Point2& p : t.outline) b.add(p.x, p.y);
  return b;
}
}  // namespace

bool usable(const TreeEntry& t) {
  if (!std::isfinite(t.height) || !std::isfinite(t.centre.x) || !std::isfinite(t.centre.y)) return false;
  if (t.outline.size() < 3) return false;
  return std::all_of(t.outline.begin(), t.outline.end(),
                     [](const Point2& p) { return std::isfinite(p.x) && std::isfinite(p.y); });
}

double yawOf(const treemapper::DronePose& p) {
  return std::atan2(2.0 * (p.qw * p.qz + p.qx * p.qy), 1.0 - 2.0 * (p.qy * p.qy + p.qz * p.qz));
}

double distXY(const Point3& a, const Point3& b) { return std::hypot(a.x - b.x, a.y - b.y); }

double dist3(const Point3& a, const Point3& b) { return std::hypot(distXY(a, b), a.z - b.z); }

Path3 makePath(const std::vector<Point3>& pts) {
  Path3 path;
  for (const Point3& p : pts) {
    if (path.pts.empty() || dist3(path.pts.back(), p) >= kZeroLength) path.pts.push_back(p);
  }
  if (path.pts.size() == 1) path.pts.push_back(path.pts.front());
  return path;
}

const TreeEntry* findTree(const std::vector<TreeEntry>& map, TreeId id) {
  const auto it = std::find_if(map.begin(), map.end(), [id](const TreeEntry& t) { return t.id == id; });
  return it == map.end() ? nullptr : &*it;
}

std::vector<Point3> transitPoints(const Point3& from, const Point3& to, double z_scan_abs) {
  return {from, {from.x, from.y, std::max(from.z, z_scan_abs)}, {to.x, to.y, z_scan_abs}};
}

std::vector<Stop> orderStops(const std::vector<TreeId>& ids, const std::vector<TreeEntry>& map, const Point3& start,
                             const Point3& end, double z_scan_abs, const Config& cfg) {
  std::vector<Stop> stops;
  Box box;
  box.add(start.x, start.y);
  box.add(end.x, end.y);
  for (TreeId id : ids) {
    const TreeEntry* tr = findTree(map, id);
    if (!tr || tr->sprayed) continue;
    stops.push_back({id, sprayPoint(*tr, map, cfg, nullptr)});
    box.add(stops.back().p.x, stops.back().p.y);
  }
  if (stops.empty()) return {};

  // Only trees within reach of the stops' bounding box can constrain a glide between them.
  const double reach = cfg.drone_radius + cfg.lateral_margin;
  std::vector<TreeEntry> local;
  for (const TreeEntry& t : map) {
    if (outlineBox(t).overlaps(box, reach)) local.push_back(t);
  }
  const auto stopTree = [&](const Point3& p) -> const TreeEntry* {
    for (const Stop& s : stops) {
      if (s.p.x == p.x && s.p.y == p.y && s.p.z == p.z) return findTree(map, s.id);
    }
    return nullptr;
  };
  // cost = glide time; counters are not touched (only the glide actually flown is counted)
  const CostFn cost = [&](const Point3& a, const Point3& b) {
    const TreeEntry* to = stopTree(b);
    if (!to) return pathTime(makePath(transitPoints(a, b, z_scan_abs)), cfg);   // b = the end: climb out, then level
    const TreeEntry* from = stopTree(a);
    return pathTime(planGlide(a, from ? from->id : kNoTree, *to, local, cfg, nullptr), cfg);
  };
  // Deterministic budget: each 2-opt pass is charged n^2 pair evaluations, not wall time, so the same inputs always
  // give the same order.
  const double n = static_cast<double>(stops.size());
  const double pass_ms = n * n * kPairCostNs * 1e-6;
  double clock_ms = -pass_ms;
  const auto now_ms = [&]() { return clock_ms += pass_ms; };

  // orderTargets sorts by descending axial coordinate along the heading: point it from `end` towards `start`
  const double heading = std::atan2(start.y - end.y, start.x - end.x);
  return orderTargets(stops, start, end, heading, cost, cfg.order_budget_ms, now_ms);
}

std::map<TreeId, double> heightsNear(const Path3& path, TreeId target, const std::vector<TreeEntry>& map,
                                     const Config& cfg) {
  std::map<TreeId, double> out;
  if (const TreeEntry* t = findTree(map, target)) out[target] = t->height;
  std::vector<Point2> samples;
  Box box;
  for (std::size_t i = 0; i + 1 < path.pts.size(); ++i) {
    const Point3 &a = path.pts[i], &b = path.pts[i + 1];
    const int n = std::max(1, static_cast<int>(std::ceil(distXY(a, b) / kHeightSampleStep)));
    for (int j = 0; j <= n; ++j) {
      const double f = static_cast<double>(j) / n;
      samples.push_back({a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)});
      box.add(samples.back().x, samples.back().y);
    }
  }
  // half a sample step of slack so a tree between two samples is not missed
  const double reach = cfg.drone_radius + cfg.lateral_margin + kHeightSampleStep / 2;
  for (const TreeEntry& k : map) {
    if (!outlineBox(k).overlaps(box, reach)) continue;
    for (const Point2& s : samples) {
      if (distanceToPolygon(k.outline, s) <= reach) {
        out[k.id] = k.height;
        break;
      }
    }
  }
  return out;
}

}  // namespace support
}  // namespace corridor_nav
