#include "corridor_nav/corridors.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "treemapper/geometry.hpp"

namespace corridor_nav {

namespace {
using treemapper::Point2;
using treemapper::Polygon2;

constexpr double kPi = 3.14159265358979323846;

void count(Counters* counters, const char* name) {
  if (counters) ++(*counters)[name];
}

/// Rotation by -heading (world -> heading frame: u along the heading, v across) and back.
struct Frame {
  double c, s;
  explicit Frame(double heading) : c(std::cos(heading)), s(std::sin(heading)) {}
  Point2 toLocal(const Point2& p) const { return {c * p.x + s * p.y, -s * p.x + c * p.y}; }
  Point2 toWorld(double u, double v) const { return {c * u - s * v, s * u + c * v}; }
};

/// u-intervals where the line v = vc lies inside the polygon (even-odd; half-open edge rule so a vertex on the
/// line is crossed exactly once or twice, never miscounted).
std::vector<std::pair<double, double>> insideIntervals(const Polygon2& local, double vc) {
  std::vector<double> us;
  for (std::size_t i = 0; i < local.size(); ++i) {
    const Point2& a = local[i];
    const Point2& b = local[(i + 1) % local.size()];
    if ((a.y <= vc) == (b.y <= vc)) continue;
    us.push_back(a.x + (vc - a.y) / (b.y - a.y) * (b.x - a.x));
  }
  std::sort(us.begin(), us.end());
  std::vector<std::pair<double, double>> out;
  for (std::size_t i = 0; i + 1 < us.size(); i += 2) out.emplace_back(us[i], us[i + 1]);
  return out;
}
}  // namespace

double minAreaRectHeading(const Polygon2& field) {
  const Polygon2 hull = treemapper::convexHull(field);
  if (hull.size() < 3) return 0.0;
  double best_area = std::numeric_limits<double>::infinity(), best = 0.0;
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const Point2& a = hull[i];
    const Point2& b = hull[(i + 1) % hull.size()];
    const double theta = std::atan2(b.y - a.y, b.x - a.x);
    const Frame f(theta);
    double umin = std::numeric_limits<double>::infinity(), umax = -umin, vmin = umin, vmax = -umin;
    for (const Point2& p : hull) {
      const Point2 q = f.toLocal(p);
      umin = std::min(umin, q.x);
      umax = std::max(umax, q.x);
      vmin = std::min(vmin, q.y);
      vmax = std::max(vmax, q.y);
    }
    const double du = umax - umin, dv = vmax - vmin, area = du * dv;
    if (area < best_area) {
      best_area = area;
      best = du >= dv ? theta : theta + kPi / 2;
    }
  }
  best = std::fmod(best, kPi);
  if (best < 0.0) best += kPi;
  return best >= kPi ? 0.0 : best;  // a tiny negative angle shifted by pi can round to exactly pi
}

double maxCorridorWidth(const CameraFov& fov, const Config& config) {
  // std::min would silently drop a NaN second argument: a non-finite fov must stay non-finite (fail closed)
  if (!std::isfinite(fov.hfov_rad) || !std::isfinite(fov.vfov_rad)) return std::numeric_limits<double>::quiet_NaN();
  const double fov_min = std::min(fov.hfov_rad, fov.vfov_rad);
  return 2.0 * (config.z_scan - config.expected_canopy_top_max) * std::tan(fov_min / 2.0) -
         2.0 * config.expected_canopy_radius_max - config.w_margin;
}

CorridorPlan planCorridors(const Field& field, const CameraFov& fov, const Config& config, Counters* counters) {
  CorridorPlan plan;
  plan.w_max = maxCorridorWidth(fov, config);
  if (field.polygon.size() < 3 || std::fabs(treemapper::polygonArea(field.polygon)) <= 0.0) {
    plan.error = "field invalid";
    return plan;
  }
  if (!(plan.w_max >= config.corridor_width_min)) {  // also catches a non-finite w_max (NaN fov)
    plan.error = "coverage impossible";
    return plan;
  }
  if (config.corridor_width == 0.0) {
    plan.width = plan.w_max;
  } else if (config.corridor_width > plan.w_max) {
    plan.width = plan.w_max;
    count(counters, "corridor_width_clamped");
  } else {
    plan.width = config.corridor_width;
  }

  const double heading = field.heading_rad ? *field.heading_rad : minAreaRectHeading(field.polygon);
  const Frame frame(heading);
  Polygon2 local;
  local.reserve(field.polygon.size());
  double vmin = std::numeric_limits<double>::infinity(), vmax = -vmin;
  for (const Point2& p : field.polygon) {
    local.push_back(frame.toLocal(p));
    vmin = std::min(vmin, local.back().y);
    vmax = std::max(vmax, local.back().y);
  }

  // equal strips no wider than the chosen width (the epsilon keeps 20 / 5 at 4 strips, not 5)
  const double extent = vmax - vmin;
  const int n = std::max(1, static_cast<int>(std::ceil(extent / plan.width - 1e-9)));
  const double strip_w = extent / n;
  for (int j = 0; j < n; ++j) {
    const double vc = vmin + (j + 0.5) * strip_w;
    for (const auto& [u0, u1] : insideIntervals(local, vc)) {
      if (u1 - u0 <= config.corridor_length_min) {
        count(counters, "corridors_dropped_short");
        continue;
      }
      Corridor c;
      c.index = static_cast<int>(plan.corridors.size());
      c.strip = {frame.toWorld(u0, vc - strip_w / 2), frame.toWorld(u1, vc - strip_w / 2),
                 frame.toWorld(u1, vc + strip_w / 2), frame.toWorld(u0, vc + strip_w / 2)};
      // every corridor is scanned out along the heading and sprayed back to its entry, so the entries line up
      c.entry = frame.toWorld(u0, vc);
      c.exit = frame.toWorld(u1, vc);
      c.heading_rad = heading;
      plan.corridors.push_back(std::move(c));
    }
  }
  plan.ok = true;
  return plan;
}

}  // namespace corridor_nav
