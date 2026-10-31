#include "corridor_nav_ros/conversions.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace corridor_nav_ros {

namespace {
bool positiveFinite(double v) { return std::isfinite(v) && v > 0.0; }
double quatNorm(double x, double y, double z, double w) { return std::sqrt(x * x + y * y + z * z + w * w); }
}  // namespace

std::vector<treemapper::TreeEntry> fromMsg(const treemapper_msgs::TreeMap& m) {
  std::vector<treemapper::TreeEntry> out;
  out.reserve(m.trees.size());
  for (const auto& t : m.trees) {
    treemapper::TreeEntry e;
    e.id = t.id;
    e.outline.reserve(t.outline.size());
    for (const auto& p : t.outline) e.outline.push_back({p.x, p.y});
    e.centre = {t.centre.x, t.centre.y};
    e.height = t.height;
    e.sprayed = t.sprayed;
    out.push_back(std::move(e));
  }
  return out;
}

corridor_nav_msgs::NavStatus toMsg(const corridor_nav::Status& status, const corridor_nav::Counters& counters,
                                   const ros::Time& stamp) {
  corridor_nav_msgs::NavStatus m;
  m.header.stamp = stamp;
  m.state = corridor_nav::statusName(status);
  m.corridor = status.corridor;
  m.corridors_total = status.corridors_total;
  m.target_id = status.target;
  m.trees_sprayed = status.sprayed;
  m.trees_skipped = status.skipped;
  m.trees_remaining = status.remaining;
  m.clearance_m = status.clearance_m;
  m.reasons.reserve(counters.size());
  m.counts.reserve(counters.size());
  for (const auto& kv : counters) {   // std::map: sorted by name
    m.reasons.push_back(kv.first);
    m.counts.push_back(kv.second);
  }
  m.warnings = status.warnings;
  return m;
}

geometry_msgs::PoseStamped toSetpoint(const corridor_nav::Outputs& out, const std::string& frame,
                                      const ros::Time& stamp) {
  geometry_msgs::PoseStamped m;
  m.header.stamp = stamp;
  m.header.frame_id = frame;
  m.pose.position.x = out.setpoint.x;
  m.pose.position.y = out.setpoint.y;
  m.pose.position.z = out.setpoint.z;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = std::sin(0.5 * out.yaw);
  m.pose.orientation.w = std::cos(0.5 * out.yaw);
  return m;
}

std::optional<corridor_nav::CameraFov> fovFrom(const sensor_msgs::CameraInfo& left) {
  const double fx = left.K[0], fy = left.K[4];
  if (left.width == 0 || left.height == 0 || !positiveFinite(fx) || !positiveFinite(fy)) return std::nullopt;
  corridor_nav::CameraFov fov;
  fov.hfov_rad = 2.0 * std::atan(static_cast<double>(left.width) / (2.0 * fx));
  fov.vfov_rad = 2.0 * std::atan(static_cast<double>(left.height) / (2.0 * fy));
  return fov;
}

std::optional<corridor_nav::StereoIntrinsics> stereoFrom(const sensor_msgs::CameraInfo& left,
                                                         const sensor_msgs::CameraInfo& right) {
  const double fx = left.K[0];
  if (!positiveFinite(fx) || right.P[0] == 0.0 || !std::isfinite(right.P[0])) return std::nullopt;
  const double baseline = -right.P[3] / right.P[0];
  if (!positiveFinite(baseline)) return std::nullopt;
  corridor_nav::StereoIntrinsics s;
  s.fx = fx;
  s.baseline = baseline;
  return s;
}

treemapper::DronePose toDronePose(const geometry_msgs::TransformStamped& wb) {
  treemapper::DronePose p;
  const auto& t = wb.transform;
  p.t = wb.header.stamp.toSec();
  p.x = t.translation.x;
  p.y = t.translation.y;
  p.z = t.translation.z;
  p.qx = t.rotation.x;
  p.qy = t.rotation.y;
  p.qz = t.rotation.z;
  p.qw = t.rotation.w;
  const double n = quatNorm(p.qx, p.qy, p.qz, p.qw);
  p.valid = std::isfinite(p.t) && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
            std::isfinite(n) && n > 1e-6;
  return p;
}

void rollPitch(const treemapper::DronePose& p, double* roll, double* pitch) {
  *roll = 0.0;
  *pitch = 0.0;
  const double n = quatNorm(p.qx, p.qy, p.qz, p.qw);
  if (!std::isfinite(n) || n <= 1e-6) return;
  const double x = p.qx / n, y = p.qy / n, z = p.qz / n, w = p.qw / n;
  *roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  *pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (w * y - z * x))));
}

bool disparityView(const sensor_msgs::Image& img, corridor_nav::DisparityView* view) {
  if (img.encoding != "32FC1" || img.is_bigendian != 0 || img.width == 0 || img.height == 0) return false;
  const std::size_t row = static_cast<std::size_t>(img.width) * sizeof(float);
  if (img.step != row || img.data.size() < row * img.height) return false;
  view->width = static_cast<int>(img.width);
  view->height = static_cast<int>(img.height);
  view->data = reinterpret_cast<const float*>(img.data.data());
  view->stamp = img.header.stamp.toSec();
  return true;
}

}  // namespace corridor_nav_ros
