#include <gtest/gtest.h>

#include <cmath>

#include "corridor_nav_ros/conversions.hpp"

using namespace corridor_nav_ros;

namespace {
treemapper::TreeEntry tree(int id, double cx, double cy, double h, bool sprayed) {
  treemapper::TreeEntry e;
  e.id = id;
  e.outline = {{cx - 1, cy - 1}, {cx + 1, cy - 1}, {cx + 1, cy + 1}, {cx - 1, cy + 1}};
  e.centre = {cx, cy};
  e.height = h;
  e.sprayed = sprayed;
  return e;
}

treemapper_msgs::TreeEntry entryMsg(const treemapper::TreeEntry& e) {
  treemapper_msgs::TreeEntry m;
  m.id = e.id;
  for (const auto& p : e.outline) {
    geometry_msgs::Point q;
    q.x = p.x;
    q.y = p.y;
    m.outline.push_back(q);
  }
  m.centre.x = e.centre.x;
  m.centre.y = e.centre.y;
  m.height = e.height;
  m.sprayed = e.sprayed;
  m.observations = 7;
  m.locked = true;
  return m;
}
}  // namespace

TEST(Conversions, TreeMapRoundTrip) {
  const auto a = tree(3, 4.0, 5.0, 6.5, false), b = tree(9, -2.0, 8.0, 3.25, true);
  treemapper_msgs::TreeMap m;
  m.header.stamp = ros::Time(12.5);
  m.trees = {entryMsg(a), entryMsg(b)};
  const auto out = fromMsg(m);
  ASSERT_EQ(out.size(), 2u);
  for (int k = 0; k < 2; ++k) {
    const auto& want = k == 0 ? a : b;
    EXPECT_EQ(out[k].id, want.id);
    ASSERT_EQ(out[k].outline.size(), want.outline.size());
    for (std::size_t j = 0; j < want.outline.size(); ++j) {
      EXPECT_DOUBLE_EQ(out[k].outline[j].x, want.outline[j].x);
      EXPECT_DOUBLE_EQ(out[k].outline[j].y, want.outline[j].y);
    }
    EXPECT_DOUBLE_EQ(out[k].centre.x, want.centre.x);
    EXPECT_DOUBLE_EQ(out[k].centre.y, want.centre.y);
    EXPECT_DOUBLE_EQ(out[k].height, want.height);
    EXPECT_EQ(out[k].sprayed, want.sprayed);
  }
  EXPECT_TRUE(fromMsg(treemapper_msgs::TreeMap()).empty());
}

TEST(Conversions, SetpointYaw) {
  corridor_nav::Outputs o;
  o.setpoint = {1.0, 2.0, 3.0};
  o.yaw = 1.0;
  const auto sp = toSetpoint(o, "world", ros::Time(5.0));
  EXPECT_EQ(sp.header.frame_id, "world");
  EXPECT_DOUBLE_EQ(sp.header.stamp.toSec(), 5.0);
  EXPECT_DOUBLE_EQ(sp.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(sp.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(sp.pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(sp.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(sp.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(sp.pose.orientation.z, std::sin(0.5));
  EXPECT_DOUBLE_EQ(sp.pose.orientation.w, std::cos(0.5));
}

// Final review I1: the node publishes the navigator's setpoint only when it is valid (the navigator has had a pose).
TEST(Conversions, SetpointPublishedOnlyWhenValid) {
  corridor_nav::Outputs o;
  o.setpoint = {1.0, 2.0, 3.0};
  o.yaw = 1.0;
  EXPECT_FALSE(setpointToPublish(o, "world", ros::Time(5.0)));   // default: not valid
  o.setpoint_valid = true;
  const auto sp = setpointToPublish(o, "world", ros::Time(5.0));
  ASSERT_TRUE(sp);
  EXPECT_EQ(sp->header.frame_id, "world");
  EXPECT_DOUBLE_EQ(sp->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(sp->pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(sp->pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(sp->pose.orientation.z, std::sin(0.5));

  // the node's path: a navigator created without a pose (field file with home) publishes no (0, 0, 0) setpoint
  corridor_nav::Field field;
  field.polygon = {{0, 0}, {30, 0}, {30, 24}, {0, 24}};
  field.home = {0.0, -2.0};
  corridor_nav::Navigator nav(corridor_nav::Config(), field, corridor_nav::CameraFov{1.5, 1.5});
  corridor_nav::Inputs in;
  in.t = 3.0;   // no pose (TF lookup failed: valid = false)
  EXPECT_FALSE(setpointToPublish(nav.step(in), "world", ros::Time(3.0)));
  in.t = 3.1;
  in.pose.valid = true;
  in.pose.t = 3.1;
  in.pose.x = 4.0;
  in.pose.y = -1.0;
  in.pose.z = 0.2;
  in.pose.qw = 1.0;
  const auto first = setpointToPublish(nav.step(in), "world", ros::Time(3.1));
  ASSERT_TRUE(first);
  EXPECT_DOUBLE_EQ(first->pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(first->pose.position.y, -1.0);
  EXPECT_DOUBLE_EQ(first->pose.position.z, 0.2);
}

TEST(Conversions, StatusCarriesCounters) {
  corridor_nav::Status s;
  s.state = corridor_nav::NavState::kGlide;
  s.cleanup = true;
  s.corridor = 2;
  s.corridors_total = 5;
  s.target = 17;
  s.sprayed = 4;
  s.skipped = 1;
  s.remaining = 9;
  s.clearance_m = NAN;
  s.warnings = {"w1", "w2"};
  corridor_nav::Counters c;
  c["zeta"] = 3;
  c["alpha"] = 1;
  c["mid"] = 2;
  const auto m = toMsg(s, c, ros::Time(7.0));
  EXPECT_DOUBLE_EQ(m.header.stamp.toSec(), 7.0);
  EXPECT_EQ(m.state, "CLEANUP");
  EXPECT_EQ(m.corridor, 2);
  EXPECT_EQ(m.corridors_total, 5);
  EXPECT_EQ(m.target_id, 17);
  EXPECT_EQ(m.trees_sprayed, 4u);
  EXPECT_EQ(m.trees_skipped, 1u);
  EXPECT_EQ(m.trees_remaining, 9u);
  EXPECT_TRUE(std::isnan(m.clearance_m));
  ASSERT_EQ(m.reasons.size(), 3u);
  ASSERT_EQ(m.counts.size(), m.reasons.size());
  EXPECT_EQ(m.reasons[0], "alpha");
  EXPECT_EQ(m.reasons[1], "mid");
  EXPECT_EQ(m.reasons[2], "zeta");
  EXPECT_EQ(m.counts[0], 1u);
  EXPECT_EQ(m.counts[1], 2u);
  EXPECT_EQ(m.counts[2], 3u);
  EXPECT_EQ(m.warnings.size(), 2u);

  s.cleanup = false;
  s.clearance_m = 4.5;
  const auto m2 = toMsg(s, {}, ros::Time(8.0));
  EXPECT_EQ(m2.state, "GLIDE");
  EXPECT_DOUBLE_EQ(m2.clearance_m, 4.5);
  EXPECT_TRUE(m2.reasons.empty());
}

namespace {
sensor_msgs::CameraInfo info(unsigned w, unsigned h, double fx, double fy, double p0, double p3) {
  sensor_msgs::CameraInfo i;
  i.width = w;
  i.height = h;
  i.K[0] = fx;
  i.K[4] = fy;
  i.P[0] = p0;
  i.P[3] = p3;
  return i;
}
}  // namespace

TEST(Conversions, FovFromCameraInfo) {
  const auto fov = fovFrom(info(640, 400, 320.0, 200.0, 0, 0));
  ASSERT_TRUE(fov.has_value());
  EXPECT_NEAR(fov->hfov_rad, 2.0 * std::atan(1.0), 1e-12);  // width / (2 fx) = 1
  EXPECT_NEAR(fov->vfov_rad, 2.0 * std::atan(1.0), 1e-12);
  EXPECT_FALSE(fovFrom(info(0, 400, 320, 200, 0, 0)).has_value());
  EXPECT_FALSE(fovFrom(info(640, 400, 0, 200, 0, 0)).has_value());
  EXPECT_FALSE(fovFrom(info(640, 400, 320, NAN, 0, 0)).has_value());
}

namespace {
geometry_msgs::Quaternion quat(double x, double y, double z, double w) {
  geometry_msgs::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}
}  // namespace

TEST(Conversions, ImageXAxisBodyYaw) {
  // identity: the image x-axis is the body x-axis
  EXPECT_NEAR(imageXAxisBodyYaw(quat(0, 0, 0, 1)), 0.0, 1e-12);
  // 90 deg about body z: x -> body y
  EXPECT_NEAR(imageXAxisBodyYaw(quat(0, 0, std::sin(M_PI / 4), std::cos(M_PI / 4))), M_PI / 2, 1e-12);
  // The Isaac camera mount (treemapper_sim.launch, optical -> body), q = (qx, qy, qz, qw) =
  // (-0.7071065073, 0.7071070551, 1.3156e-7, 5.1934e-7). Rotating (1, 0, 0) by q:
  //   x' = 1 - 2 (qy^2 + qz^2) = 1 - 2 (0.50000039 + 1.7e-14) = -7.7e-7
  //   y' = 2 (qx qy + qw qz)   = 2 (-0.49999999999971 + 6.8e-14) = -0.9999999999993
  //   z' = 2 (qx qz - qw qy)   = -9.2e-7
  // so the image x-axis points along body -y: atan2(-1.0, -7.7e-7) = -pi/2 - 7.7e-7 = -1.5707971 (hand value
  // -1.5707963 = -pi/2 to 1e-6). Corridor yaw = heading + pi/2 - phi = heading + pi.
  const auto isaac = quat(-0.7071065073147165, 0.7071070550580697, 1.315609245066774e-07, 5.193386764157588e-07);
  EXPECT_NEAR(imageXAxisBodyYaw(isaac), -M_PI / 2, 1e-5);
  EXPECT_NEAR(imageXAxisBodyYaw(isaac), -1.5707971014, 1e-8);
  // x-axis turned vertical (90 deg about body y): the xy part is 0 -> NaN
  EXPECT_TRUE(std::isnan(imageXAxisBodyYaw(quat(0, std::sin(M_PI / 4), 0, std::cos(M_PI / 4)))));
  // tilted 80 deg: xy norm cos(80 deg) = 0.17 >= 0.1 is still usable
  EXPECT_NEAR(imageXAxisBodyYaw(quat(0, std::sin(40 * M_PI / 180), 0, std::cos(40 * M_PI / 180))), 0.0, 1e-9);
  // invalid quaternions
  EXPECT_TRUE(std::isnan(imageXAxisBodyYaw(quat(0, 0, 0, 0))));
  EXPECT_TRUE(std::isnan(imageXAxisBodyYaw(quat(NAN, 0, 0, 1))));
  EXPECT_TRUE(std::isnan(imageXAxisBodyYaw(quat(0, 0, 0, INFINITY))));
}

TEST(Conversions, RosCameraMountToOptical) {
  // The real drone: body <- sensor_down is a ROS camera frame (x = optical axis, y left, z up) pitched Ry(82 deg),
  // q = (0, sin 41, 0, cos 41) (treemapper_real.launch). Optical -> ROS camera is the fixed rotation
  // q_r = (-0.5, 0.5, -0.5, 0.5) of treemapper_ros (rosCameraToOptical()), which sends the optical x-axis (image
  // columns, right) to ROS (0, -1, 0): x' = 1 - 2 (0.25 + 0.25) = 0, y' = 2 (qx qy + qw qz) = 2 (-0.25 - 0.25) = -1,
  // z' = 2 (qx qz - qw qy) = 2 (0.25 - 0.25) = 0. The pitch is a rotation about y, which leaves (0, -1, 0) alone, so
  // the image x-axis is body -y: phi = atan2(-1, 0) = -pi/2. Fed as if it were optical, the same quaternion gives
  // phi = 0 (x-axis rotated about y stays in the xz plane: (cos 82, 0, -sin 82) -> atan2(0, 0.139) = 0): the wrong
  // axis across track.
  const double a = 82.0 * M_PI / 180.0;
  const auto real = quat(0.0, std::sin(a / 2), 0.0, std::cos(a / 2));
  EXPECT_NEAR(imageXAxisBodyYaw(opticalMountFromRosCamera(real)), -M_PI / 2, 1e-12);
  EXPECT_NEAR(imageXAxisBodyYaw(real), 0.0, 1e-12);   // what the node would get by mistake
  // the launch file's literal quaternion
  EXPECT_NEAR(imageXAxisBodyYaw(opticalMountFromRosCamera(quat(0.0, 0.6560590289905073, 0.0, 0.7547095802227720))),
              -M_PI / 2, 1e-9);
  // identity ROS camera frame (x forward, y left, z up): image x-axis = body -y
  EXPECT_NEAR(imageXAxisBodyYaw(opticalMountFromRosCamera(quat(0, 0, 0, 1))), -M_PI / 2, 1e-12);
  // yawed 90 deg about body z: the image x-axis (body -y) turns to body +x
  EXPECT_NEAR(imageXAxisBodyYaw(opticalMountFromRosCamera(quat(0, 0, std::sin(M_PI / 4), std::cos(M_PI / 4)))), 0.0,
              1e-12);
  // vertical-axis gate on the composed rotation: rolled 90 deg about body x the image x-axis is vertical -> NaN;
  // rolled 80 deg its xy norm is sin(10 deg) = 0.17 >= 0.1 -> still usable
  EXPECT_TRUE(std::isnan(
      imageXAxisBodyYaw(opticalMountFromRosCamera(quat(std::sin(M_PI / 4), 0, 0, std::cos(M_PI / 4))))));
  EXPECT_NEAR(imageXAxisBodyYaw(opticalMountFromRosCamera(quat(std::sin(40 * M_PI / 180), 0, 0, std::cos(40 * M_PI / 180)))),
              -M_PI / 2, 1e-9);
  // an invalid quaternion stays invalid
  EXPECT_TRUE(std::isnan(imageXAxisBodyYaw(opticalMountFromRosCamera(quat(0, 0, 0, 0)))));
  EXPECT_TRUE(std::isnan(imageXAxisBodyYaw(opticalMountFromRosCamera(quat(NAN, 0, 0, 1)))));
}

TEST(Conversions, DeduceBaselineConventions) {
  // the table of treemapper_ros' test (test_input_sync.cpp, Calibration.BaselineConventions): same port, same answers
  const double fx = 454.53;
  auto r = deduceBaseline(-fx * 0.16, fx);
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.form, "rep104_m");
  EXPECT_NEAR(r.baseline_m, 0.16, 1e-12);
  r = deduceBaseline(-72724.796875, 454.5299987792969);   // the Isaac recording's right P[3] (mm)
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.form, "neg_mm");
  EXPECT_NEAR(r.baseline_m, 0.16, 1e-6);
  r = deduceBaseline(fx * 120.0, fx);
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.form, "pos_mm");
  EXPECT_NEAR(r.baseline_m, 0.12, 1e-12);
  EXPECT_FALSE(deduceBaseline(0.0, fx).ok);           // a left/mono P (Tx = 0): fail closed
  EXPECT_FALSE(deduceBaseline(fx * 0.16, fx).ok);     // +fx*B metres is not a known convention
  EXPECT_FALSE(deduceBaseline(-fx * 5.0, fx).ok);     // 5 m baseline: implausible
  EXPECT_FALSE(deduceBaseline(-fx * 0.16, 0.0).ok);
  EXPECT_FALSE(deduceBaseline(std::nan(""), fx).ok);
  EXPECT_FALSE(deduceBaseline(0.0, fx).error.empty());
}

TEST(Conversions, StereoFromCameraInfo) {
  // sim/specs/camera.json, exactly: the right P[3] is in millimetres (-fx * 160 mm)
  sensor_msgs::CameraInfo left = info(640, 400, 454.53265380859375, 454.53265380859375, 454.53265380859375, 0.0);
  sensor_msgs::CameraInfo right = info(640, 400, 455.0869445800781, 455.0869445800781, 454.5299987792969, -72724.796875);
  auto st = stereoFrom(left, right);
  ASSERT_TRUE(st.stereo.has_value()) << st.error;
  EXPECT_EQ(st.form, "neg_mm");
  EXPECT_NEAR(st.stereo->baseline, 0.16, 1e-6);
  EXPECT_DOUBLE_EQ(st.stereo->fx, 454.53265380859375);

  // REP-104 metres still works; fx is the left P[0] (rectified), not K[0]
  left = info(640, 400, 360.0, 360.0, 350.0, 0.0);
  right = info(640, 400, 350.0, 350.0, 350.0, -35.0);   // baseline 0.1 m
  st = stereoFrom(left, right);
  ASSERT_TRUE(st.stereo.has_value()) << st.error;
  EXPECT_EQ(st.form, "rep104_m");
  EXPECT_DOUBLE_EQ(st.stereo->fx, 350.0);
  EXPECT_NEAR(st.stereo->baseline, 0.1, 1e-12);

  // nonsense is rejected with a reason: clearance invalid, the core fails closed
  for (const auto& bad : {info(640, 400, 350, 350, 350.0, 0.0),        // zero baseline (a left P)
                          info(640, 400, 350, 350, 0.0, -35.0),        // P[0] = 0
                          info(640, 400, 350, 350, 350.0, 35.0),       // +fx*B metres
                          info(640, 400, 350, 350, 350.0, -350.0 * 5), // 5 m
                          info(640, 400, 350, 350, 350.0, NAN)}) {
    st = stereoFrom(left, bad);
    EXPECT_FALSE(st.stereo.has_value()) << "P[0] " << bad.P[0] << " P[3] " << bad.P[3];
    EXPECT_FALSE(st.error.empty());
  }
  st = stereoFrom(info(640, 400, 350, 350, 0.0, 0.0), right);   // left P[0] = 0 (uncalibrated)
  EXPECT_FALSE(st.stereo.has_value());
  EXPECT_FALSE(st.error.empty());
}

namespace {
sensor_msgs::Image constantDisparity(unsigned w, unsigned h, float d) {
  sensor_msgs::Image img;
  img.header.stamp = ros::Time(4.0);
  img.width = w;
  img.height = h;
  img.encoding = "32FC1";
  img.step = 4 * w;
  img.data.resize(static_cast<std::size_t>(img.step) * h);
  float* f = reinterpret_cast<float*>(img.data.data());
  for (std::size_t k = 0; k < static_cast<std::size_t>(w) * h; ++k) f[k] = d;
  return img;
}
}  // namespace

TEST(Conversions, ClearanceChecksTheDisparitySizeAgainstTheLeftInfo) {
  const corridor_nav::Config cfg;
  const auto left = info(640, 400, 400.0, 400.0, 400.0, 0.0);
  corridor_nav::StereoIntrinsics stereo;
  stereo.fx = 400.0;
  stereo.baseline = 0.12;
  DisparityIssue issue = DisparityIssue::kNone;
  // 48 px of disparity at fx * B = 48: Z = 1 m
  auto c = clearanceFrom(constantDisparity(640, 400, 48.0f), left, stereo, 0.0, 0.0, cfg, &issue);
  EXPECT_EQ(issue, DisparityIssue::kNone);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.metres, 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(c.stamp, 4.0);
  // a full-resolution disparity against the half-resolution info: the calibration does not fit it
  c = clearanceFrom(constantDisparity(1280, 800, 48.0f), left, stereo, 0.0, 0.0, cfg, &issue);
  EXPECT_EQ(issue, DisparityIssue::kSizeMismatch);
  EXPECT_FALSE(c.valid);
  EXPECT_TRUE(std::isnan(c.metres));
  c = clearanceFrom(constantDisparity(640, 399, 48.0f), left, stereo, 0.0, 0.0, cfg, &issue);
  EXPECT_EQ(issue, DisparityIssue::kSizeMismatch);
  EXPECT_FALSE(c.valid);
  // not a packed 32FC1
  auto bad = constantDisparity(640, 400, 48.0f);
  bad.encoding = "16UC1";
  c = clearanceFrom(bad, left, stereo, 0.0, 0.0, cfg, &issue);
  EXPECT_EQ(issue, DisparityIssue::kFormat);
  EXPECT_FALSE(c.valid);
}

TEST(Conversions, PoseFromTransformAndRollPitch) {
  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time(3.5);
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 3.0;
  const double roll = 0.2, pitch = -0.1, yaw = 0.7;
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2),
               cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  t.transform.rotation.w = cr * cp * cy + sr * sp * sy;
  t.transform.rotation.x = sr * cp * cy - cr * sp * sy;
  t.transform.rotation.y = cr * sp * cy + sr * cp * sy;
  t.transform.rotation.z = cr * cp * sy - sr * sp * cy;
  const auto p = toDronePose(t);
  EXPECT_TRUE(p.valid);
  EXPECT_DOUBLE_EQ(p.t, 3.5);
  EXPECT_DOUBLE_EQ(p.z, 3.0);
  double r = 9, q = 9;
  rollPitch(p, &r, &q);
  EXPECT_NEAR(r, roll, 1e-12);
  EXPECT_NEAR(q, pitch, 1e-12);

  geometry_msgs::TransformStamped bad;   // all-zero quaternion
  EXPECT_FALSE(toDronePose(bad).valid);
  bad.transform.rotation.w = 1.0;
  bad.transform.translation.x = NAN;
  EXPECT_FALSE(toDronePose(bad).valid);
}

TEST(Conversions, DisparityView) {
  sensor_msgs::Image img;
  img.header.stamp = ros::Time(9.25);
  img.width = 4;
  img.height = 2;
  img.encoding = "32FC1";
  img.step = 16;
  img.data.resize(32);
  float* f = reinterpret_cast<float*>(img.data.data());
  for (int k = 0; k < 8; ++k) f[k] = static_cast<float>(k + 1);
  corridor_nav::DisparityView v;
  ASSERT_TRUE(disparityView(img, &v));
  EXPECT_EQ(v.width, 4);
  EXPECT_EQ(v.height, 2);
  EXPECT_DOUBLE_EQ(v.stamp, 9.25);
  EXPECT_FLOAT_EQ(v.data[7], 8.0f);

  auto wrong = img;
  wrong.encoding = "16UC1";
  EXPECT_FALSE(disparityView(wrong, &v));
  wrong = img;
  wrong.step = 20;   // padded rows
  EXPECT_FALSE(disparityView(wrong, &v));
  wrong = img;
  wrong.data.resize(16);   // short
  EXPECT_FALSE(disparityView(wrong, &v));
  wrong = img;
  wrong.is_bigendian = 1;
  EXPECT_FALSE(disparityView(wrong, &v));
  wrong = img;
  wrong.width = 0;
  EXPECT_FALSE(disparityView(wrong, &v));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
