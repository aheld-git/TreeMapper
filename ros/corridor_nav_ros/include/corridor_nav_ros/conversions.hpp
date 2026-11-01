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

/// The setpoint the node publishes for a navigator step: toSetpoint(out), or nullopt while out.setpoint_valid is false
/// (no pose yet: the setpoint would be the origin, not the drone's position).
std::optional<geometry_msgs::PoseStamped> setpointToPublish(const corridor_nav::Outputs& out, const std::string& frame,
                                                            const ros::Time& stamp);

/// Full field of view of a camera_info: hfov = 2 atan(width / (2 K[0])), vfov = 2 atan(height / (2 K[4])).
/// nullopt when the size or the focal lengths are not positive and finite.
std::optional<corridor_nav::CameraFov> fovFrom(const sensor_msgs::CameraInfo& left);

/// Yaw, in the body frame, of the optical camera's x-axis (image columns increase along it): rotate (1, 0, 0) by the
/// optical -> body quaternion (normalised) and return atan2(y, x). NaN when the quaternion is not finite or has no
/// usable length, or when the rotated axis is near vertical (its xy norm < 0.1: no across-track direction).
double imageXAxisBodyYaw(const geometry_msgs::Quaternion& optical_to_body);

/// ROS-convention camera frame -> body (x = optical axis, y left, z up) to optical -> body: the quaternion times the
/// fixed optical -> ROS camera rotation q = (-0.5, 0.5, -0.5, 0.5), the same one treemapper_ros appends
/// (rosCameraToOptical() in treemapper_ros/src/input_sync.cpp, camera_frame_is_optical: false). Not linkable from
/// here without the whole node library, hence repeated. A non-finite or zero quaternion stays unusable (NaN follows).
geometry_msgs::Quaternion opticalMountFromRosCamera(const geometry_msgs::Quaternion& ros_camera_to_body);

struct BaselineResult {
  bool ok{false};
  double baseline_m{0.0};
  std::string form;    ///< which P[3] convention matched
  std::string error;   ///< why it was rejected
};

/// Stereo baseline from the RIGHT camera's projection matrix: P[3] = Tx, P[0] = fx.
/// Accepted (mutually exclusive within the plausible range 0.01..1 m):
///   "rep104_m"  Tx = -fx * B, B in metres (REP-104)
///   "neg_mm"    Tx = -fx * B, B in millimetres (the field driver and the Isaac recording, sim/specs/camera.json)
///   "pos_mm"    Tx = +fx * B, B in millimetres
/// Anything else is rejected (fail closed: no clearance without a trusted baseline).
/// A copy of treemapper_ros' deduceBaseline() (ros/treemapper_ros/src/input_sync.cpp); not linkable from here without
/// the whole node library, hence repeated (as opticalMountFromRosCamera).
BaselineResult deduceBaseline(double right_P3, double right_fx);

/// Stereo intrinsics for the nadir clearance, or why there are none.
struct StereoCalibration {
  std::optional<corridor_nav::StereoIntrinsics> stereo;   ///< nullopt = rejected: the clearance stays invalid
  std::string form;    ///< deduceBaseline's convention when accepted
  std::string error;   ///< the reason when rejected
};

/// fx = P[0] of the left info (the rectified focal length the disparity is in), baseline = deduceBaseline(P_right[3],
/// P_right[0]). Rejected when the left P[0] is not positive and finite or no baseline convention fits.
StereoCalibration stereoFrom(const sensor_msgs::CameraInfo& left, const sensor_msgs::CameraInfo& right);

enum class DisparityIssue { kNone, kFormat, kSizeMismatch };

/// Nadir clearance (corridor_nav::nadirClearance) of a disparity image. Invalid when the image is not a packed
/// little-endian 32FC1 (*issue = kFormat) or its size differs from the left camera_info the calibration belongs to
/// (*issue = kSizeMismatch); else *issue = kNone. `issue` may be null.
corridor_nav::Clearance clearanceFrom(const sensor_msgs::Image& disparity, const sensor_msgs::CameraInfo& left,
                                      const corridor_nav::StereoIntrinsics& stereo, double roll, double pitch,
                                      const corridor_nav::Config& cfg, DisparityIssue* issue);

/// world <- body transform -> drone pose; pose.t = the transform's stamp, valid = all finite and a usable quaternion.
treemapper::DronePose toDronePose(const geometry_msgs::TransformStamped& world_body);

/// Roll and pitch (rad) of a pose's quaternion (ZYX convention). Both 0 for an unusable quaternion.
void rollPitch(const treemapper::DronePose& p, double* roll, double* pitch);

/// Read-only view of a 32FC1, little-endian, tightly packed (step == 4 * width) disparity image; `view->data`
/// points into `img`, which must outlive it. view->stamp = the header stamp. False (view untouched) otherwise.
bool disparityView(const sensor_msgs::Image& img, corridor_nav::DisparityView* view);

}  // namespace corridor_nav_ros
