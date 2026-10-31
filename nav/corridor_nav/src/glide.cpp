#include "corridor_nav/glide.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "treemapper/geometry.hpp"

namespace corridor_nav {

namespace {

constexpr double kSampleStep = 0.05;   ///< m, spacing of the requirement samples along the xy segment
constexpr double kZeroLength = 1e-9;   ///< m, legs shorter than this are dropped

void count(Counters* counters, const char* name) {
  if (counters) ++(*counters)[name];
}

double segmentDistance(const treemapper::Point2& p, const treemapper::Point2& a, const treemapper::Point2& b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double u = len2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
  u = std::clamp(u, 0.0, 1.0);
  return std::hypot(p.x - (a.x + u * dx), p.y - (a.y + u * dy));
}

/// A tree that constrains the path, with its requirement and an xy box (outline grown by the reach) for quick rejection.
struct Obstacle {
  const treemapper::TreeEntry* tree{nullptr};
  double req{0.0};
  double min_x{0.0}, max_x{0.0}, min_y{0.0}, max_y{0.0};
};

class Requirements {
 public:
  Requirements(const std::vector<treemapper::TreeEntry>& map, treemapper::TreeId from_id, treemapper::TreeId to_id,
               const Config& config)
      : reach_(config.drone_radius + config.lateral_margin) {
    for (const treemapper::TreeEntry& k : map) {
      if (k.outline.empty()) continue;
      Obstacle o;
      o.tree = &k;
      o.req = (k.id == to_id || k.id == from_id) ? k.height + (config.spray_clearance - config.descent_slack)
                                                 : k.height + config.transit_clearance;
      o.min_x = o.min_y = std::numeric_limits<double>::infinity();
      o.max_x = o.max_y = -std::numeric_limits<double>::infinity();
      for (const treemapper::Point2& v : k.outline) {
        o.min_x = std::min(o.min_x, v.x);
        o.max_x = std::max(o.max_x, v.x);
        o.min_y = std::min(o.min_y, v.y);
        o.max_y = std::max(o.max_y, v.y);
      }
      obstacles_.push_back(o);
    }
  }

  /// Highest requirement at xy; -infinity where no tree is within the reach.
  double at(double x, double y) const {
    double req = -std::numeric_limits<double>::infinity();
    for (const Obstacle& o : obstacles_) {
      if (o.req <= req) continue;
      if (x < o.min_x - reach_ || x > o.max_x + reach_ || y < o.min_y - reach_ || y > o.max_y + reach_) continue;
      if (distanceToPolygon(o.tree->outline, {x, y}) <= reach_) req = o.req;
    }
    return req;
  }

 private:
  double reach_;
  std::vector<Obstacle> obstacles_;
};

double segmentTime(const Point3& a, const Point3& b, const Config& config) {
  const double dxy = std::hypot(b.x - a.x, b.y - a.y);
  const double dz = std::fabs(b.z - a.z);
  return std::max(dxy > 0.0 ? dxy / config.max_speed_xy : 0.0, dz > 0.0 ? dz / config.max_speed_z : 0.0);
}

}  // namespace

double distanceToPolygon(const treemapper::Polygon2& poly, const treemapper::Point2& p) {
  if (poly.empty()) return std::numeric_limits<double>::infinity();
  if (treemapper::pointInPolygon(poly, p)) return 0.0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < poly.size(); ++i) {
    best = std::min(best, segmentDistance(p, poly[i], poly[(i + 1) % poly.size()]));
  }
  return best;
}

Point3 sprayPoint(const treemapper::TreeEntry& target, const std::vector<treemapper::TreeEntry>& map,
                  const Config& config, Counters* counters) {
  Point3 p{target.centre.x, target.centre.y, target.height + config.spray_clearance};
  const double reach = config.drone_radius + config.lateral_margin;
  bool raised = false;
  for (const treemapper::TreeEntry& k : map) {
    if (k.id == target.id) continue;
    // the neighbour is also the `from` tree of a glide into this one, whose requirement is the larger of the two
    const double need = k.height + std::max(config.transit_clearance, config.spray_clearance - config.descent_slack);
    if (need > p.z && distanceToPolygon(k.outline, target.centre) <= reach) {
      p.z = need;
      raised = true;
    }
  }
  if (raised) count(counters, "spray_point_raised");
  return p;
}

Path3 planGlide(const Point3& from, treemapper::TreeId from_id, const treemapper::TreeEntry& to,
                const std::vector<treemapper::TreeEntry>& map, const Config& config, Counters* counters) {
  const Point3 target = sprayPoint(to, map, config, counters);
  const Requirements req(map, from_id, to.id, config);

  // Walk the straight segment; remember the highest requirement it dips below.
  const double len = std::hypot(target.x - from.x, target.y - from.y);
  const int steps = static_cast<int>(std::ceil(len / kSampleStep));
  double violated = -std::numeric_limits<double>::infinity();
  if (steps == 0) {  // vertical segment: the xy is constant, both ends count
    const double r = req.at(from.x, from.y);
    if (std::min(from.z, target.z) < r) violated = r;
  } else {
    for (int i = 0; i <= steps; ++i) {
      const double s = std::min(i * kSampleStep, len), t = s / len;
      const double x = from.x + t * (target.x - from.x), y = from.y + t * (target.y - from.y);
      const double z = from.z + t * (target.z - from.z);
      const double r = req.at(x, y);
      if (z < r) violated = std::max(violated, r);
    }
  }

  Path3 path;
  if (!(violated > -std::numeric_limits<double>::infinity())) {
    path.pts = {from, target};
  } else {
    const double z_up = std::max({violated, from.z, target.z});
    const Point3 corners[] = {from, {from.x, from.y, z_up}, {target.x, target.y, z_up}, target};
    for (const Point3& c : corners) {
      if (!path.pts.empty()) {
        const Point3& q = path.pts.back();
        if (std::hypot(std::hypot(c.x - q.x, c.y - q.y), c.z - q.z) < kZeroLength) continue;
      }
      path.pts.push_back(c);
    }
    path.climbed = true;
    count(counters, "glide_climbs_inserted");
  }
  if (path.pts.size() == 1) path.pts.push_back(path.pts.front());  // from == target: keep a two-point path
  return path;
}

double pathTime(const Path3& path, const Config& config) {
  double t = 0.0;
  for (std::size_t i = 0; i + 1 < path.pts.size(); ++i) t += segmentTime(path.pts[i], path.pts[i + 1], config);
  return t;
}

Point3 carrot(const Path3& path, const Point3& pos, const Config& config) {
  if (path.pts.empty()) return pos;
  if (path.pts.size() == 1) return path.pts.front();

  // closest point on the polyline (the earliest wins a tie): segment index and fraction
  std::size_t seg = 0;
  double u_best = 0.0, d_best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.pts.size(); ++i) {
    const Point3 &a = path.pts[i], &b = path.pts[i + 1];
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    const double len2 = dx * dx + dy * dy + dz * dz;
    double u = len2 > 0.0 ? ((pos.x - a.x) * dx + (pos.y - a.y) * dy + (pos.z - a.z) * dz) / len2 : 0.0;
    u = std::clamp(u, 0.0, 1.0);
    const double d = std::sqrt(std::pow(pos.x - (a.x + u * dx), 2) + std::pow(pos.y - (a.y + u * dy), 2) +
                               std::pow(pos.z - (a.z + u * dz), 2));
    if (d < d_best) {
      d_best = d;
      seg = i;
      u_best = u;
    }
  }

  // advance carrot_lead_s of travel time; within a segment the time is linear in the fraction
  double budget = std::max(0.0, config.carrot_lead_s);
  double u = u_best;
  for (std::size_t i = seg; i + 1 < path.pts.size(); ++i, u = 0.0) {
    const Point3 &a = path.pts[i], &b = path.pts[i + 1];
    const double t_seg = segmentTime(a, b, config);
    const double t_left = (1.0 - u) * t_seg;
    if (t_seg > 0.0 && budget < t_left) {
      const double f = u + budget / t_seg;
      return {a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), a.z + f * (b.z - a.z)};
    }
    budget -= t_left;
  }
  return path.pts.back();
}

bool needsReplan(const std::map<treemapper::TreeId, double>& planned_heights,
                 const std::vector<treemapper::TreeEntry>& map, const Config& config) {
  for (const auto& [id, planned] : planned_heights) {
    const auto it = std::find_if(map.begin(), map.end(), [id = id](const treemapper::TreeEntry& t) { return t.id == id; });
    if (it == map.end()) return true;
    if (!(std::fabs(it->height - planned) <= config.replan_dz)) return true;
  }
  return false;
}

}  // namespace corridor_nav
