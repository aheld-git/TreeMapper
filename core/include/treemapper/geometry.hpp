// Small geometry toolbox for the core (no dependencies): 3D vectors and rotations, stereo
// back-projection, 2D convex hulls and polygon measures.
#pragma once

#include <array>
#include <vector>

#include "treemapper/types.hpp"

namespace treemapper {

struct Vec3 {
  double x{0.0}, y{0.0}, z{0.0};
};

/// Rigid transform: rotation quaternion (x, y, z, w) + translation. Applies p -> R p + t.
struct Transform {
  double tx{0.0}, ty{0.0}, tz{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
  Vec3 apply(const Vec3& p) const;
  /// this ∘ other (apply `other` first).
  Transform compose(const Transform& other) const;
};

/// Rectified stereo camera of the frame stream (left image). Disparity in full-resolution pixels.
struct StereoCalibration {
  int width{0}, height{0};
  double fx{0.0}, fy{0.0}, cx{0.0}, cy{0.0};
  double baseline{0.0};  ///< metres
  int stride{1};         ///< disparity grid stride (grid (r, c) = pixel (r*stride + stride/2, c*stride + stride/2))
  Transform body_cam;    ///< optical camera frame -> body
};

/// Pixel (u, v) with disparity d (> 0) -> optical-frame point. Returns false for d <= 0 / non-finite.
bool backprojectOptical(const StereoCalibration& cal, double u, double v, double d, Vec3* out);

/// Convex hull (Andrew's monotone chain), counter-clockwise, no repeated closing vertex.
/// Fewer than 3 distinct non-collinear points -> result with < 3 vertices.
Polygon2 convexHull(std::vector<Point2> pts);
/// Signed area (CCW positive).
double polygonArea(const Polygon2& poly);
/// Area-weighted centroid; vertex mean for degenerate polygons.
Point2 polygonCentroid(const Polygon2& poly);
/// Point-in-polygon (even-odd ray casting).
bool pointInPolygon(const Polygon2& poly, const Point2& p);
/// Area of the intersection of two CONVEX CCW polygons (Sutherland-Hodgman clipping).
double convexIntersectionArea(const Polygon2& a, const Polygon2& b);

}  // namespace treemapper
