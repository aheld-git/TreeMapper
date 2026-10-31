// corridor_nav: tree assignment. A tree belongs to the corridor whose strip holds its centre; it is assigned when
// that corridor locks and never moves afterwards. Trees that appear later in an already locked corridor go to the
// cleanup list. Pure functions, no ROS.
#pragma once

#include <map>
#include <set>
#include <vector>

#include "corridor_nav/types.hpp"
#include "treemapper/types.hpp"

namespace corridor_nav {

struct Assignment {
  std::map<treemapper::TreeId, int> corridor_of;   ///< permanent
  std::vector<treemapper::TreeId> cleanup;         ///< late trees, in the order they were found
  std::set<treemapper::TreeId> outside_field;      ///< centre outside the field polygon; never targeted
};

/// Corridors 0..locked_upto are locked (-1 = none). With `lock_now` (the LOCK event of corridor `locked_upto`) every
/// published, unsprayed, unassigned tree whose centre lies in that corridor's strip (nearest strip when the centre is
/// inside the field but in none, e.g. boundary rounding) is assigned to it; the ids are returned in map order.
/// Every other unassigned, unsprayed tree whose corridor is <= locked_upto goes to `cleanup` once (in the lock event
/// that means < locked_upto). A centre outside `field` goes to `outside_field` once. Sprayed trees are ignored, and
/// assigned / cleanup / outside trees are not processed again. Call with lock_now=true at each LOCK and lock_now=false
/// on every other tick. Counts: late_trees, trees_outside_field (once per tree). `counters` may be null.
std::vector<treemapper::TreeId> updateAssignment(const std::vector<treemapper::TreeEntry>& map,
                                                 const std::vector<Corridor>& corridors,
                                                 const treemapper::Polygon2& field, int locked_upto,
                                                 bool lock_now, Assignment* assignment, Counters* counters);

/// The map id that now stands for a tree last seen at `old_centre` (after a merge or split changed the ids): the
/// unsprayed tree whose outline contains the point (nearest centre if several), else the nearest unsprayed centre
/// within `radius`, else kNoTree.
treemapper::TreeId reresolveId(const treemapper::Point2& old_centre, const std::vector<treemapper::TreeEntry>& map,
                               double radius);

}  // namespace corridor_nav
