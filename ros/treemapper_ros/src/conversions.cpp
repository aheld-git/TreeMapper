#include "treemapper_ros/conversions.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace treemapper_ros {

using treemapper::Detection2D;
using treemapper::TreeEntry;
using treemapper::TreeId;

treemapper_msgs::TreeEntry toMsg(const TreeEntry& e, const TreeMeta* meta) {
  treemapper_msgs::TreeEntry m;
  m.id = e.id;
  m.outline.reserve(e.outline.size());
  for (const auto& p : e.outline) {
    geometry_msgs::Point q;
    q.x = p.x;
    q.y = p.y;
    q.z = 0.0;
    m.outline.push_back(q);
  }
  m.centre.x = e.centre.x;
  m.centre.y = e.centre.y;
  m.centre.z = 0.0;
  m.height = e.height;
  m.sprayed = e.sprayed;
  m.confidence = 0.0f;  // reserved, always 0 (TreeEntry.msg)
  m.observations = static_cast<std::uint32_t>(std::max(0, e.observations));
  if (meta) {
    m.locked = meta->locked;
    m.first_published = ros::Time(meta->first_published);
    m.last_updated = ros::Time(meta->last_updated);
  }
  return m;
}

TreeEntry fromMsg(const treemapper_msgs::TreeEntry& m) {
  TreeEntry e;
  e.id = m.id;
  for (const auto& p : m.outline) e.outline.push_back({p.x, p.y});
  e.centre = {m.centre.x, m.centre.y};
  e.height = m.height;
  e.sprayed = m.sprayed;
  return e;
}

treemapper_msgs::TreeMap toMapMsg(const std_msgs::Header& header, const std::vector<TreeEntry>& trees,
                                  const TreeMetaTracker* tracker) {
  treemapper_msgs::TreeMap m;
  m.header = header;
  m.trees.reserve(trees.size());
  for (const auto& e : trees) m.trees.push_back(toMsg(e, tracker ? tracker->find(e.id) : nullptr));
  return m;
}

void scaleBoxes(std::vector<Detection2D>* boxes, double scale) {
  if (scale == 1.0) return;
  for (auto& b : *boxes) {
    b.x *= scale;
    b.y *= scale;
    b.width *= scale;
    b.height *= scale;
  }
}

std::vector<Detection2D> fromMsg(const treemapper_msgs::Detection2DArray& m, double scale) {
  std::vector<Detection2D> out;
  out.reserve(m.detections.size());
  for (const auto& d : m.detections) {
    Detection2D b;
    b.x = d.x;
    b.y = d.y;
    b.width = d.width;
    b.height = d.height;
    b.score = d.score;
    b.label = d.label;
    out.push_back(b);
  }
  scaleBoxes(&out, scale);
  return out;
}

treemapper::Transform fromMsg(const geometry_msgs::Transform& t) {
  treemapper::Transform o;
  o.tx = t.translation.x;
  o.ty = t.translation.y;
  o.tz = t.translation.z;
  o.qx = t.rotation.x;
  o.qy = t.rotation.y;
  o.qz = t.rotation.z;
  o.qw = t.rotation.w;
  return o;
}

treemapper::DronePose toDronePose(const geometry_msgs::Transform& wb, double t) {
  treemapper::DronePose p;
  p.t = t;
  p.x = wb.translation.x;
  p.y = wb.translation.y;
  p.z = wb.translation.z;
  p.qx = wb.rotation.x;
  p.qy = wb.rotation.y;
  p.qz = wb.rotation.z;
  p.qw = wb.rotation.w;
  const double n = std::sqrt(p.qx * p.qx + p.qy * p.qy + p.qz * p.qz + p.qw * p.qw);
  p.valid = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(n) && n > 1e-6;
  return p;
}

treemapper_msgs::DropCounters toCountersMsg(const std_msgs::Header& h, const std::string& stage,
                                            const std::map<std::string, std::uint64_t>& counts) {
  treemapper_msgs::DropCounters m;
  m.header = h;
  m.stage = stage;
  for (const auto& kv : counts) {
    m.reasons.push_back(kv.first);
    m.counts.push_back(kv.second);
  }
  return m;
}

treemapper_msgs::StageTiming toTimingMsg(const std_msgs::Header& h, const std::map<std::string, double>& ms) {
  treemapper_msgs::StageTiming m;
  m.header = h;
  for (const auto& kv : ms) {
    m.stages.push_back(kv.first);
    m.ms.push_back(kv.second);
  }
  return m;
}

visualization_msgs::MarkerArray toMarkers(const std_msgs::Header& h, const std::vector<TreeEntry>& trees,
                                          const TreeMetaTracker* tracker, TreeId hop_target) {
  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker clear;
  clear.header = h;
  clear.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(clear);
  for (const auto& e : trees) {
    const TreeMeta* meta = tracker ? tracker->find(e.id) : nullptr;
    std_msgs::ColorRGBA c;
    c.a = 1.0f;
    if (e.sprayed) { c.r = 0.1f; c.g = 0.8f; c.b = 0.1f; }
    else if (e.id == hop_target) { c.r = 0.95f; c.g = 0.1f; c.b = 0.1f; }
    else if (meta && meta->locked) { c.r = 1.0f; c.g = 0.55f; c.b = 0.0f; }
    else { c.r = 0.2f; c.g = 0.5f; c.b = 1.0f; }
    const double z = std::isfinite(e.height) ? e.height : 0.0;

    visualization_msgs::Marker ln;
    ln.header = h;
    ln.ns = "outline";
    ln.id = e.id;
    ln.type = visualization_msgs::Marker::LINE_STRIP;
    ln.action = visualization_msgs::Marker::ADD;
    ln.pose.orientation.w = 1.0;
    ln.scale.x = 0.08;
    ln.color = c;
    for (std::size_t k = 0; k <= e.outline.size() && !e.outline.empty(); ++k) {
      const auto& p = e.outline[k % e.outline.size()];
      geometry_msgs::Point q;
      q.x = p.x;
      q.y = p.y;
      q.z = z;
      ln.points.push_back(q);
    }
    arr.markers.push_back(ln);

    visualization_msgs::Marker ce;
    ce.header = h;
    ce.ns = "centre";
    ce.id = e.id;
    ce.type = visualization_msgs::Marker::SPHERE;
    ce.action = visualization_msgs::Marker::ADD;
    ce.pose.position.x = e.centre.x;
    ce.pose.position.y = e.centre.y;
    ce.pose.position.z = z;
    ce.pose.orientation.w = 1.0;
    ce.scale.x = ce.scale.y = ce.scale.z = 0.3;
    ce.color = c;
    arr.markers.push_back(ce);

    visualization_msgs::Marker tx;
    tx.header = h;
    tx.ns = "label";
    tx.id = e.id;
    tx.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    tx.action = visualization_msgs::Marker::ADD;
    tx.pose.position.x = e.centre.x;
    tx.pose.position.y = e.centre.y;
    tx.pose.position.z = z + 0.8;
    tx.pose.orientation.w = 1.0;
    tx.scale.z = 0.6;
    tx.color = c;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%d  %.1f m%s", e.id, e.height, e.sprayed ? "  sprayed" : "");
    tx.text = buf;
    arr.markers.push_back(tx);
  }
  return arr;
}

}  // namespace treemapper_ros
