#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "corridor_nav/guard.hpp"

using namespace corridor_nav;

namespace {

constexpr double kFx = 500.0, kBaseline = 0.12;
const float kNaN = std::numeric_limits<float>::quiet_NaN();

float dispFor(double z) { return static_cast<float>(kFx * kBaseline / z); }

struct Img {
  int w{100}, h{100};
  std::vector<float> d;
  explicit Img(float fill = kNaN) : d(100 * 100, fill) {}
  float& at(int x, int y) { return d[static_cast<std::size_t>(y) * w + x]; }
  DisparityView view(double stamp = 1.0) const { return DisparityView{w, h, d.data(), stamp}; }
};

// The central 20 % x 20 % window of a 100x100 image: columns/rows 40..59.
void fillWindow(Img& img, float v) {
  for (int y = 40; y < 60; ++y)
    for (int x = 40; x < 60; ++x) img.at(x, y) = v;
}

const StereoIntrinsics kStereo{kFx, kBaseline};

}  // namespace

TEST(Guard, ClearanceFromDisparity) {
  Config cfg;
  Img img;
  fillWindow(img, dispFor(3.0));
  Clearance c = nadirClearance(img.view(7.5), kStereo, 0.0, 0.0, cfg);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.metres, 3.0, 1e-4);
  EXPECT_DOUBLE_EQ(c.stamp, 7.5);
  c = nadirClearance(img.view(), kStereo, 0.0, 0.3, cfg);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.metres, 3.0 * std::cos(0.3), 1e-4);
  c = nadirClearance(img.view(), kStereo, 0.2, 0.3, cfg);
  EXPECT_NEAR(c.metres, 3.0 * std::cos(0.2) * std::cos(0.3), 1e-4);
}

TEST(Guard, OnlyTheCentralWindowCounts) {
  Config cfg;
  Img img(dispFor(0.5));            // everything outside the window is very near
  fillWindow(img, dispFor(3.0));
  const Clearance c = nadirClearance(img.view(), kStereo, 0.0, 0.0, cfg);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.metres, 3.0, 1e-4);
}

TEST(Guard, PercentileIgnoresFewNearOutliers) {
  Config cfg;
  Img img;
  fillWindow(img, dispFor(3.0));
  for (int i = 0; i < 20; ++i) img.at(40 + i, 40) = dispFor(0.5);   // 20 of 400 pixels = 5 %
  const Clearance c = nadirClearance(img.view(), kStereo, 0.0, 0.0, cfg);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.metres, 3.0, 1e-3);
  // A larger near cluster (15 %) does pull the 10th percentile down.
  for (int i = 0; i < 20; ++i)
    for (int j = 1; j < 3; ++j) img.at(40 + i, 40 + j) = dispFor(0.5);
  const Clearance c2 = nadirClearance(img.view(), kStereo, 0.0, 0.0, cfg);
  ASSERT_TRUE(c2.valid);
  EXPECT_NEAR(c2.metres, 0.5, 1e-3);
}

TEST(Guard, InvalidWhenTooFewPixels) {
  Config cfg;
  Img img;
  // 25 % of the window valid (top 5 rows), the rest NaN.
  for (int y = 40; y < 45; ++y)
    for (int x = 40; x < 60; ++x) img.at(x, y) = dispFor(3.0);
  Clearance c = nadirClearance(img.view(), kStereo, 0.0, 0.0, cfg);
  EXPECT_FALSE(c.valid);
  EXPECT_TRUE(std::isnan(c.metres));
  // 40 % valid is enough.
  for (int y = 45; y < 48; ++y)
    for (int x = 40; x < 60; ++x) img.at(x, y) = dispFor(3.0);
  c = nadirClearance(img.view(), kStereo, 0.0, 0.0, cfg);
  EXPECT_TRUE(c.valid);
}

TEST(Guard, InvalidPixelsAndBadInputs) {
  Config cfg;
  Img img;
  fillWindow(img, dispFor(3.0));
  // Zero, negative and infinite disparities are invalid pixels, not ranges.
  for (int x = 40; x < 60; ++x) {
    img.at(x, 40) = 0.0f;
    img.at(x, 41) = -4.0f;
    img.at(x, 42) = std::numeric_limits<float>::infinity();
  }
  Clearance c = nadirClearance(img.view(), kStereo, 0.0, 0.0, cfg);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.metres, 3.0, 1e-3);
  EXPECT_FALSE(nadirClearance(img.view(), StereoIntrinsics{0.0, kBaseline}, 0.0, 0.0, cfg).valid);
  EXPECT_FALSE(nadirClearance(img.view(), StereoIntrinsics{kFx, 0.0}, 0.0, 0.0, cfg).valid);
  EXPECT_FALSE(nadirClearance(DisparityView{0, 0, nullptr, 1.0}, kStereo, 0.0, 0.0, cfg).valid);
  EXPECT_FALSE(nadirClearance(DisparityView{100, 100, nullptr, 1.0}, kStereo, 0.0, 0.0, cfg).valid);
}

TEST(Guard, ClimbHysteresisAndBlind) {
  Config cfg;  // min_clearance 1.5, hysteresis 0.5, max age 0.5
  const double now = 10.0;
  auto clr = [&](double metres, double age = 0.1) {
    Clearance c;
    c.valid = true;
    c.metres = metres;
    c.stamp = now - age;
    return c;
  };
  EXPECT_EQ(guardDecision(clr(1.2), now, false, cfg), GuardAction::kClimb);
  EXPECT_EQ(guardDecision(clr(1.8), now, true, cfg), GuardAction::kClimb);    // climbing: needs >= 2.0
  EXPECT_EQ(guardDecision(clr(1.8), now, false, cfg), GuardAction::kNone);    // not climbing: 1.8 is fine
  EXPECT_EQ(guardDecision(clr(2.0), now, true, cfg), GuardAction::kNone);     // boundary releases
  EXPECT_EQ(guardDecision(clr(2.1), now, true, cfg), GuardAction::kNone);
  EXPECT_EQ(guardDecision(clr(1.5), now, false, cfg), GuardAction::kNone);    // boundary is not a trigger
  EXPECT_EQ(guardDecision(clr(3.0, 0.6), now, false, cfg), GuardAction::kNoDescent);   // stale
  EXPECT_EQ(guardDecision(clr(1.0, 0.6), now, false, cfg), GuardAction::kNoDescent);   // stale beats near
  Clearance blind;                                                                     // never valid
  blind.stamp = now;
  EXPECT_EQ(guardDecision(blind, now, false, cfg), GuardAction::kNoDescent);
  EXPECT_EQ(guardDecision(blind, now, true, cfg), GuardAction::kNoDescent);
}
