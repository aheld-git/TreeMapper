// corridor_nav: spray order. Axial sort along the field heading, then 2-opt on the open path start -> stops -> end
// under a time budget. Pure function, no ROS; the clock is injected so the budget is testable.
#pragma once

#include <functional>
#include <vector>

#include "corridor_nav/types.hpp"
#include "treemapper/types.hpp"

namespace corridor_nav {

struct Stop {
  treemapper::TreeId id{treemapper::kNoTree};
  Point3 p;
};

using CostFn = std::function<double(const Point3&, const Point3&)>;

/// Orders `stops` for one spray pass: initial order by axial coordinate along `heading_rad`, descending (the return
/// direction; ties keep the input order), then 2-opt (segment reversal) on the open path start -> stops -> end until
/// no reversal improves it or the budget is spent. `now_ms` is called once before the first pass and once after
/// each full pass; the loop stops when a pass improved nothing or elapsed > `budget_ms`. `cost` may be asymmetric.
/// Equal-cost alternatives never replace the current order. The result is a permutation of `stops`.
std::vector<Stop> orderTargets(std::vector<Stop> stops, const Point3& start, const Point3& end, double heading_rad,
                               const CostFn& cost, double budget_ms, const std::function<double()>& now_ms);

}  // namespace corridor_nav
