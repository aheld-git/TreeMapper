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

TEST(Conversions, StereoFromCameraInfo) {
  const auto left = info(640, 400, 350.0, 350.0, 350.0, 0.0);
  const auto right = info(640, 400, 350.0, 350.0, 350.0, -35.0);   // baseline 0.1 m
  const auto st = stereoFrom(left, right);
  ASSERT_TRUE(st.has_value());
  EXPECT_DOUBLE_EQ(st->fx, 350.0);
  EXPECT_NEAR(st->baseline, 0.1, 1e-12);
  EXPECT_FALSE(stereoFrom(left, info(640, 400, 350, 350, 350.0, 0.0)).has_value());    // zero baseline
  EXPECT_FALSE(stereoFrom(left, info(640, 400, 350, 350, 0.0, -35.0)).has_value());    // P[0] = 0
  EXPECT_FALSE(stereoFrom(info(640, 400, 0, 350, 0, 0), right).has_value());
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
