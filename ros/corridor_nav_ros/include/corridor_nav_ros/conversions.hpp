// Conversions between ROS messages and the ROS-free corridor_nav types. No node state; unit-tested in
// test/test_conversions.cpp.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <corridor_nav_msgs/NavStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <treemapper_msgs/TreeMap.h>

#include "corridor_nav/guard.hpp"
#include "corridor_nav/navigator.hpp"
#include "treemapper/types.hpp"

namespace corridor_nav_ros {

/// The map message's trees as contract entries (outline, centre, height, sprayed; the diagnostics fields are dropped).
std::vector<treemapper::TreeEntry> fromMsg(const treemapper_msgs::TreeMap& m);

/// NavStatus of the navigator. `counters` become reasons / counts, in the (sorted) order of the map. clearance_m is
/// NaN when invalid (Status already says so); state is statusName(status), i.e. CLEANUP while cleaning up.
corridor_nav_msgs::NavStatus toMsg(const corridor_nav::Status& status, const corridor_nav::Counters& counters,
                                   const ros::Time& stamp);

/// Carrot point + yaw as a PoseStamped in `frame`; orientation = rotation by `yaw` about z.
geometry_msgs::PoseStamped toSetpoint(const corridor_nav::Outputs& out, const std::string& frame,
                                      const ros::Time& stamp);

/// Full field of view of a camera_info: hfov = 2 atan(width / (2 K[0])), vfov = 2 atan(height / (2 K[4])).
/// nullopt when the size or the focal lengths are not positive and finite.
std::optional<corridor_nav::CameraFov> fovFrom(const sensor_msgs::CameraInfo& left);

/// Yaw, in the body frame, of the optical camera's x-axis (image columns increase along it): rotate (1, 0, 0) by the
/// optical -> body quaternion (normalised) and return atan2(y, x). NaN when the quaternion is not finite or has no
/// usable length, or when the rotated axis is near vertical (its xy norm < 0.1: no across-track direction).
double imageXAxisBodyYaw(const geometry_msgs::Quaternion& optical_to_body);

/// fx = K[0] of the left info, baseline = -P_right[3] / P_right[0]. nullopt when either is not positive and finite.
std::optional<corridor_nav::StereoIntrinsics> stereoFrom(const sensor_msgs::CameraInfo& left,
                                                         const sensor_msgs::CameraInfo& right);

/// world <- body transform -> drone pose; pose.t = the transform's stamp, valid = all finite and a usable quaternion.
treemapper::DronePose toDronePose(const geometry_msgs::TransformStamped& world_body);

/// Roll and pitch (rad) of a pose's quaternion (ZYX convention). Both 0 for an unusable quaternion.
void rollPitch(const treemapper::DronePose& p, double* roll, double* pitch);

/// Read-only view of a 32FC1, little-endian, tightly packed (step == 4 * width) disparity image; `view->data`
/// points into `img`, which must outlive it. view->stamp = the header stamp. False (view untouched) otherwise.
bool disparityView(const sensor_msgs::Image& img, corridor_nav::DisparityView* view);

}  // namespace corridor_nav_ros
