#include "corridor_nav/ordering.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace corridor_nav {

namespace {
constexpr double kMinGain = 1e-9;   ///< a reversal must shorten the path by more than this
}  // namespace

std::vector<Stop> orderTargets(std::vector<Stop> stops, const Point3& start, const Point3& end, double heading_rad,
                               const CostFn& cost, double budget_ms, const std::function<double()>& now_ms) {
  const double ch = std::cos(heading_rad), sh = std::sin(heading_rad);
  std::stable_sort(stops.begin(), stops.end(), [&](const Stop& a, const Stop& b) {
    return a.p.x * ch + a.p.y * sh > b.p.x * ch + b.p.y * sh;
  });
  const std::size_t n = stops.size();
  if (n < 2) return stops;

  // Cost matrix over nodes 0 = start, 1..n = stops, n+1 = end; `cost` is evaluated once per pair.
  std::vector<Point3> pts;
  pts.reserve(n + 2);
  pts.push_back(start);
  for (const Stop& s : stops) pts.push_back(s.p);
  pts.push_back(end);
  const std::size_t m = n + 2;
  std::vector<double> c(m * m, 0.0);
  for (std::size_t a = 0; a < m; ++a) {
    for (std::size_t b = 0; b < m; ++b) {
      if (a != b) c[a * m + b] = cost(pts[a], pts[b]);
    }
  }
  auto C = [&](std::size_t a, std::size_t b) { return c[a * m + b]; };

  // order[k] = node index (1..n) of the k-th stop; path = start, order..., end.
  std::vector<std::size_t> order(n);
  for (std::size_t i = 0; i < n; ++i) order[i] = i + 1;

  const double t0 = now_ms();
  while (true) {
    bool improved = false;
    // fwd[k] = cost of the stop-to-stop edges up to stop k, rev[k] = the same edges traversed backwards.
    std::vector<double> fwd(n, 0.0), rev(n, 0.0);
    auto rebuild = [&]() {
      for (std::size_t k = 1; k < n; ++k) {
        fwd[k] = fwd[k - 1] + C(order[k - 1], order[k]);
        rev[k] = rev[k - 1] + C(order[k], order[k - 1]);
      }
    };
    rebuild();
    for (std::size_t i = 0; i + 1 < n; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {
        const std::size_t prev = i == 0 ? 0 : order[i - 1];
        const std::size_t next = j + 1 == n ? n + 1 : order[j + 1];
        const double before = C(prev, order[i]) + (fwd[j] - fwd[i]) + C(order[j], next);
        const double after = C(prev, order[j]) + (rev[j] - rev[i]) + C(order[i], next);
        if (after < before - kMinGain) {
          std::reverse(order.begin() + static_cast<std::ptrdiff_t>(i), order.begin() + static_cast<std::ptrdiff_t>(j) + 1);
          rebuild();
          improved = true;
        }
      }
    }
    const double elapsed = now_ms() - t0;
    if (!improved || elapsed > budget_ms) break;
  }

  std::vector<Stop> out;
  out.reserve(n);
  for (std::size_t k : order) out.push_back(stops[k - 1]);
  return out;
}

}  // namespace corridor_nav
