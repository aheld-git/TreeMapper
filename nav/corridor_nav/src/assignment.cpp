#include "corridor_nav/assignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "treemapper/geometry.hpp"

namespace corridor_nav {

namespace {
using treemapper::Point2;
using treemapper::Polygon2;
using treemapper::TreeEntry;
using treemapper::TreeId;

void count(Counters* counters, const char* name) {
  if (counters) ++(*counters)[name];
}

double distToSegment(const Point2& p, const Point2& a, const Point2& b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double t = len2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

/// 0 when `p` is inside the polygon, else the distance to its boundary.
double distToPolygon(const Point2& p, const Polygon2& poly) {
  if (poly.empty()) return std::numeric_limits<double>::infinity();
  if (treemapper::pointInPolygon(poly, p)) return 0.0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < poly.size(); ++i) {
    best = std::min(best, distToSegment(p, poly[i], poly[(i + 1) % poly.size()]));
  }
  return best;
}

/// Index (Corridor::index) of the corridor whose strip holds `p`, else of the nearest strip; -1 if no corridors.
/// Ties go to the lower position in `corridors`.
int corridorOf(const Point2& p, const std::vector<Corridor>& corridors) {
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (const Corridor& c : corridors) {
    const double d = distToPolygon(p, c.strip);
    if (d < best_d) {
      best_d = d;
      best = c.index;
    }
  }
  return best;
}

bool contains(const std::vector<TreeId>& v, TreeId id) { return std::find(v.begin(), v.end(), id) != v.end(); }

}  // namespace

std::vector<TreeId> updateAssignment(const std::vector<TreeEntry>& map, const std::vector<Corridor>& corridors,
                                     const Polygon2& field, int just_locked, Assignment* assignment,
                                     Counters* counters) {
  std::vector<TreeId> assigned;
  if (!assignment) return assigned;
  for (const TreeEntry& t : map) {
    if (t.sprayed) continue;
    if (assignment->corridor_of.count(t.id) || assignment->outside_field.count(t.id) ||
        contains(assignment->cleanup, t.id)) {
      continue;
    }
    if (!treemapper::pointInPolygon(field, t.centre)) {
      assignment->outside_field.insert(t.id);
      count(counters, "trees_outside_field");
      continue;
    }
    const int k = corridorOf(t.centre, corridors);
    if (k < 0) continue;
    if (k == just_locked) {
      assignment->corridor_of[t.id] = k;
      assigned.push_back(t.id);
    } else if (k < just_locked) {
      assignment->cleanup.push_back(t.id);
      count(counters, "late_trees");
    }
  }
  return assigned;
}

TreeId reresolveId(const Point2& old_centre, const std::vector<TreeEntry>& map, double radius) {
  TreeId in_outline = treemapper::kNoTree;
  double in_outline_d = std::numeric_limits<double>::infinity();
  TreeId nearest = treemapper::kNoTree;
  double nearest_d = std::numeric_limits<double>::infinity();
  for (const TreeEntry& t : map) {
    if (t.sprayed) continue;
    const double d = std::hypot(t.centre.x - old_centre.x, t.centre.y - old_centre.y);
    if (t.outline.size() >= 3 && treemapper::pointInPolygon(t.outline, old_centre) && d < in_outline_d) {
      in_outline_d = d;
      in_outline = t.id;
    }
    if (d <= radius && d < nearest_d) {
      nearest_d = d;
      nearest = t.id;
    }
  }
  return in_outline != treemapper::kNoTree ? in_outline : nearest;
}

}  // namespace corridor_nav
