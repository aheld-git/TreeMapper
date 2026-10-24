#include "treemapper/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace treemapper {

namespace {
void rotate(double qx, double qy, double qz, double qw, double x, double y, double z, double* o) {
  const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  qx /= n; qy /= n; qz /= n; qw /= n;
  // v' = v + 2 w (q x v) + 2 q x (q x v)
  const double cx = qy * z - qz * y, cy = qz * x - qx * z, cz = qx * y - qy * x;
  const double ccx = qy * cz - qz * cy, ccy = qz * cx - qx * cz, ccz = qx * cy - qy * cx;
  o[0] = x + 2.0 * (qw * cx + ccx);
  o[1] = y + 2.0 * (qw * cy + ccy);
  o[2] = z + 2.0 * (qw * cz + ccz);
}
double cross(const Point2& o, const Point2& a, const Point2& b) {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}
}  // namespace

Vec3 Transform::apply(const Vec3& p) const {
  double o[3];
  rotate(qx, qy, qz, qw, p.x, p.y, p.z, o);
  return {o[0] + tx, o[1] + ty, o[2] + tz};
}

Transform Transform::compose(const Transform& b) const {
  Transform r;
  // q = qa * qb
  r.qw = qw * b.qw - qx * b.qx - qy * b.qy - qz * b.qz;
  r.qx = qw * b.qx + qx * b.qw + qy * b.qz - qz * b.qy;
  r.qy = qw * b.qy - qx * b.qz + qy * b.qw + qz * b.qx;
  r.qz = qw * b.qz + qx * b.qy - qy * b.qx + qz * b.qw;
  const Vec3 t = apply({b.tx, b.ty, b.tz});
  r.tx = t.x; r.ty = t.y; r.tz = t.z;
  return r;
}

bool backprojectOptical(const StereoCalibration& cal, double u, double v, double d, Vec3* out) {
  if (!(d > 0.0) || !std::isfinite(d)) return false;
  const double Z = cal.fx * cal.baseline / d;
  out->x = (u - cal.cx) * Z / cal.fx;
  out->y = (v - cal.cy) * Z / cal.fy;
  out->z = Z;
  return true;
}

Polygon2 convexHull(std::vector<Point2> p) {
  std::sort(p.begin(), p.end(), [](const Point2& a, const Point2& b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  p.erase(std::unique(p.begin(), p.end(), [](const Point2& a, const Point2& b) {
            return a.x == b.x && a.y == b.y;
          }), p.end());
  if (p.size() < 3) return p;
  Polygon2 h(2 * p.size());
  std::size_t k = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0) --k;
    h[k++] = p[i];
  }
  for (std::size_t i = p.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(h[k - 2], h[k - 1], p[i - 1]) <= 0) --k;
    h[k++] = p[i - 1];
  }
  h.resize(k - 1);
  return h;
}

double polygonArea(const Polygon2& poly) {
  double a = 0.0;
  for (std::size_t i = 0, n = poly.size(); i < n; ++i) {
    const auto& p = poly[i];
    const auto& q = poly[(i + 1) % n];
    a += p.x * q.y - q.x * p.y;
  }
  return 0.5 * a;
}

Point2 polygonCentroid(const Polygon2& poly) {
  const std::size_t n = poly.size();
  if (n == 0) return {std::nan(""), std::nan("")};
  double a = 0.0, cx = 0.0, cy = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const auto& p = poly[i];
    const auto& q = poly[(i + 1) % n];
    const double c = p.x * q.y - q.x * p.y;
    a += c; cx += (p.x + q.x) * c; cy += (p.y + q.y) * c;
  }
  if (std::fabs(a) < 1e-12) {
    Point2 m;
    for (const auto& p : poly) { m.x += p.x; m.y += p.y; }
    return {m.x / n, m.y / n};
  }
  return {cx / (3.0 * a), cy / (3.0 * a)};
}

bool pointInPolygon(const Polygon2& poly, const Point2& p) {
  bool in = false;
  for (std::size_t i = 0, j = poly.size() - 1, n = poly.size(); i < n; j = i++) {
    const auto& a = poly[i];
    const auto& b = poly[j];
    if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
  }
  return in;
}

double convexIntersectionArea(const Polygon2& a, const Polygon2& b) {
  if (a.size() < 3 || b.size() < 3) return 0.0;
  Polygon2 out = a;
  for (std::size_t i = 0, n = b.size(); i < n && !out.empty(); ++i) {
    const Point2 e0 = b[i], e1 = b[(i + 1) % n];
    Polygon2 in;
    in.swap(out);
    for (std::size_t k = 0, m = in.size(); k < m; ++k) {
      const Point2 p = in[k], q = in[(k + 1) % m];
      const double cp = cross(e0, e1, p), cq = cross(e0, e1, q);
      if (cp >= 0) out.push_back(p);
      if ((cp >= 0) != (cq >= 0)) {
        const double t = cp / (cp - cq);
        out.push_back({p.x + t * (q.x - p.x), p.y + t * (q.y - p.y)});
      }
    }
  }
  return out.size() < 3 ? 0.0 : std::fabs(polygonArea(out));
}

}  // namespace treemapper
