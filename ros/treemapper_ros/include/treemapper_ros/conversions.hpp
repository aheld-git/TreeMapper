// Conversions between the ROS messages (treemapper_msgs, geometry_msgs, visualization_msgs) and the
// ROS-free core types. No node state; unit-tested in test/test_conversions.cpp.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/Transform.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <treemapper_msgs/Detection2DArray.h>
#include <treemapper_msgs/DropCounters.h>
#include <treemapper_msgs/StageTiming.h>
#include <treemapper_msgs/TreeEntry.h>
#include <treemapper_msgs/TreeMap.h>
#include <visualization_msgs/MarkerArray.h>

#include "treemapper/geometry.hpp"
#include "treemapper/types.hpp"
#include "treemapper_ros/node_support.hpp"

namespace treemapper_ros {

/// Contract entry -> message. `meta` (may be null) fills the diagnostics fields; confidence and
/// observations are not exposed by the Mapper interface and are published as 0 ("not provided").
treemapper_msgs::TreeEntry toMsg(const treemapper::TreeEntry& e, const TreeMeta* meta);
/// Message -> contract entry (used by tools and tests; diagnostics fields are dropped).
treemapper::TreeEntry fromMsg(const treemapper_msgs::TreeEntry& m);

treemapper_msgs::TreeMap toMapMsg(const std_msgs::Header& header, const std::vector<treemapper::TreeEntry>& trees,
                                  const TreeMetaTracker* tracker);

/// treemapper_msgs/Detection2DArray -> core boxes, each coordinate multiplied by `scale` (detector image ->
/// calibrated image pixels; 1.0 when they are the same image).
std::vector<treemapper::Detection2D> fromMsg(const treemapper_msgs::Detection2DArray& m, double scale);
void scaleBoxes(std::vector<treemapper::Detection2D>* boxes, double scale);

treemapper::Transform fromMsg(const geometry_msgs::Transform& t);
/// world <- body transform -> drone pose at time t (z = world z, the height above the take-off ground).
treemapper::DronePose toDronePose(const geometry_msgs::Transform& world_body, double t);

treemapper_msgs::DropCounters toCountersMsg(const std_msgs::Header& h, const std::string& stage,
                                            const std::map<std::string, std::uint64_t>& counts);
treemapper_msgs::StageTiming toTimingMsg(const std_msgs::Header& h, const std::map<std::string, double>& ms);

/// RViz view of the map: per tree an outline LINE_STRIP at the canopy height, a centre sphere and a text
/// label "id  h" (namespaces outline / centre / label); colour: sprayed = green, hop target = red,
/// locked = orange, else blue. The array starts with a DELETEALL so merged-away trees disappear.
visualization_msgs::MarkerArray toMarkers(const std_msgs::Header& h, const std::vector<treemapper::TreeEntry>& trees,
                                          const TreeMetaTracker* tracker, treemapper::TreeId hop_target);

}  // namespace treemapper_ros
