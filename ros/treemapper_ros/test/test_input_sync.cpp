// Unit tests of the ROS-free input stage: disparity sampling, calibration checks, detection decoding and the
// detection <-> disparity pairing (same rules as tools/treemapper_tools/bag_frames.py).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "treemapper_ros/input_sync.hpp"

using namespace treemapper_ros;
using treemapper::Detection2D;

namespace {

std::vector<std::uint8_t> floatImage(int w, int h, float fill) {
  std::vector<std::uint8_t> d(static_cast<std::size_t>(w) * h * 4);
  for (int k = 0; k < w * h; ++k) std::memcpy(&d[static_cast<std::size_t>(k) * 4], &fill, 4);
  return d;
}
void setPx(std::vector<std::uint8_t>* d, int w, int u, int v, float val) {
  std::memcpy(&(*d)[(static_cast<std::size_t>(v) * w + u) * 4], &val, 4);
}
DisparityGrid grid1(float v) {
  DisparityGrid g;
  g.grid_w = g.grid_h = 1;
  g.data = {v};
  return g;
}
std::vector<Detection2D> oneBox() {
  Detection2D b;
  b.x = 1; b.y = 2; b.width = 3; b.height = 4; b.score = 0.5;
  return {b};
}
constexpr StampNs kMs = 1000000;

}  // namespace

// ------------------------------------------------------------------------------------------ disparity

TEST(Disparity, GridSizeMatchesPythonRange) {
  EXPECT_EQ(gridSize(640, 4), 160);  // len(range(2, 640, 4))
  EXPECT_EQ(gridSize(400, 4), 100);
  EXPECT_EQ(gridSize(10, 4), 2);     // range(2, 10, 4) = [2, 6]
}

TEST(Disparity, SamplesStrideGridWithHalfStrideOffsetAndInvalidates) {
  const int w = 16, h = 8, s = 4;
  auto img = floatImage(w, h, 10.0f);
  setPx(&img, w, 2, 2, 33.0f);                                         // grid (0, 0)
  setPx(&img, w, 6, 2, -1.0f);                                         // grid (0, 1): negative -> NaN
  setPx(&img, w, 10, 2, std::numeric_limits<float>::infinity());       // grid (0, 2): inf -> NaN
  setPx(&img, w, 14, 6, 0.0f);                                         // grid (1, 3): zero -> NaN
  DisparityGrid g;
  ASSERT_EQ(sampleDisparity("32FC1", w, h, 4 * w, false, img.data(), img.size(), s, w, h, 0.0, 0.0, &g), DisparityStatus::kOk);
  ASSERT_EQ(g.grid_w, 4);
  ASSERT_EQ(g.grid_h, 2);
  EXPECT_FLOAT_EQ(g.data[0], 33.0f);
  EXPECT_TRUE(std::isnan(g.data[1]));
  EXPECT_TRUE(std::isnan(g.data[2]));
  EXPECT_FLOAT_EQ(g.data[3], 10.0f);
  EXPECT_TRUE(std::isnan(g.data[7]));
  EXPECT_FLOAT_EQ(g.data[4], 10.0f);
}

TEST(Disparity, RejectsMalformedAndMismatchedImages) {
  const int w = 16, h = 8;
  auto img = floatImage(w, h, 5.0f);
  DisparityGrid g;
  // the Isaac recording's malformed twin: step = width, 1 byte per pixel worth of data
  EXPECT_EQ(sampleDisparity("32FC1", w, h, w, false, img.data(), static_cast<std::size_t>(w) * h, 4, 0, 0, 0.0, 0.0, &g),
            DisparityStatus::kMalformed);
  EXPECT_EQ(sampleDisparity("16UC1", w, h, 4 * w, false, img.data(), img.size(), 4, 0, 0, 0.0, 0.0, &g), DisparityStatus::kMalformed);
  EXPECT_EQ(sampleDisparity("32FC1", w, h, 4 * w, true, img.data(), img.size(), 4, 0, 0, 0.0, 0.0, &g), DisparityStatus::kMalformed);
  EXPECT_EQ(sampleDisparity("32FC1", w, h, 4 * w, false, img.data(), img.size() - 4, 4, 0, 0, 0.0, 0.0, &g), DisparityStatus::kMalformed);
  EXPECT_EQ(sampleDisparity("32FC1", w, h, 4 * w, false, img.data(), img.size(), 4, 2 * w, h, 0.0, 0.0, &g),
            DisparityStatus::kResolutionMismatch);
}

TEST(Disparity, PrincipalPointOffsetIsSubtractedBeforeTheValidityTest) {
  const int w = 8, h = 4;
  auto img = floatImage(w, h, 10.0f);
  setPx(&img, w, 6, 2, 1.5f);  // grid (0, 1): 1.5 - 2 < 0 -> invalid
  DisparityGrid g;
  ASSERT_EQ(sampleDisparity("32FC1", w, h, 4 * w, false, img.data(), img.size(), 4, 0, 0, 2.0, 0.0, &g), DisparityStatus::kOk);
  EXPECT_FLOAT_EQ(g.data[0], 8.0f);
  EXPECT_TRUE(std::isnan(g.data[1]));
}

TEST(Disparity, QuantumReproducesTheFramesBinCache) {
  const int w = 4, h = 4;
  auto img = floatImage(w, h, 12.3456789f);
  setPx(&img, w, 2, 2, 1e-5f);  // tiny but valid -> clipped up to one quantum, as frames.py does
  DisparityGrid g;
  ASSERT_EQ(sampleDisparity("32FC1", w, h, 4 * w, false, img.data(), img.size(), 4, 0, 0, 0.0, 1.0 / 256, &g), DisparityStatus::kOk);
  ASSERT_EQ(g.data.size(), 1u);
  EXPECT_FLOAT_EQ(g.data[0], 1.0f / 256);
  auto img2 = floatImage(w, h, 12.3456789f);
  ASSERT_EQ(sampleDisparity("32FC1", w, h, 4 * w, false, img2.data(), img2.size(), 4, 0, 0, 0.0, 1.0 / 256, &g), DisparityStatus::kOk);
  EXPECT_FLOAT_EQ(g.data[0], std::nearbyint(12.3456789f * 256.0f) / 256.0f);
}

// ------------------------------------------------------------------------------------------ calibration

TEST(Calibration, BaselineConventions) {
  const double fx = 454.53;
  auto r = deduceBaseline(-fx * 0.16, fx);
  EXPECT_TRUE(r.ok); EXPECT_EQ(r.form, "rep104_m"); EXPECT_NEAR(r.baseline_m, 0.16, 1e-12);
  r = deduceBaseline(-72724.796875, 454.5299987792969);  // the Isaac recording's right P[3] (mm)
  EXPECT_TRUE(r.ok); EXPECT_EQ(r.form, "neg_mm"); EXPECT_NEAR(r.baseline_m, 0.16, 1e-6);
  r = deduceBaseline(fx * 120.0, fx);
  EXPECT_TRUE(r.ok); EXPECT_EQ(r.form, "pos_mm"); EXPECT_NEAR(r.baseline_m, 0.12, 1e-12);
  EXPECT_FALSE(deduceBaseline(0.0, fx).ok);           // a left/mono P (Tx = 0): fail closed
  EXPECT_FALSE(deduceBaseline(fx * 0.16, fx).ok);     // +fx*B metres is not a known convention
  EXPECT_FALSE(deduceBaseline(-fx * 5.0, fx).ok);     // 5 m baseline: implausible
  EXPECT_FALSE(deduceBaseline(-fx * 0.16, 0.0).ok);
  EXPECT_FALSE(deduceBaseline(std::nan(""), fx).ok);
}

TEST(Calibration, BuildsFromCameraInfoAndRejectsUncalibrated) {
  const double L[12] = {454.53, 0, 346.7, 0, 0, 454.53, 209.7, 0, 0, 0, 1, 0};
  const double R[12] = {454.53, 0, 346.7, -72724.8, 0, 454.53, 209.7, 0, 0, 0, 1, 0};
  treemapper::StereoCalibration c;
  std::string err, form;
  ASSERT_TRUE(makeCalibration(640, 400, L, R, 4, treemapper::Transform{}, &c, &err, &form)) << err;
  EXPECT_EQ(c.width, 640); EXPECT_EQ(c.height, 400); EXPECT_EQ(c.stride, 4);
  EXPECT_DOUBLE_EQ(c.fx, 454.53); EXPECT_DOUBLE_EQ(c.cx, 346.7); EXPECT_DOUBLE_EQ(c.cy, 209.7);
  EXPECT_NEAR(c.baseline, 0.16, 1e-4);
  EXPECT_EQ(form, "neg_mm");
  const double Z[12] = {0};
  EXPECT_FALSE(makeCalibration(640, 400, Z, R, 4, treemapper::Transform{}, &c, &err));
  EXPECT_NE(err.find("uncalibrated"), std::string::npos);
  EXPECT_FALSE(makeCalibration(0, 400, L, R, 4, treemapper::Transform{}, &c, &err));
  EXPECT_FALSE(makeCalibration(640, 400, L, L, 4, treemapper::Transform{}, &c, &err));  // right P without Tx
}

TEST(Calibration, RosCameraToOpticalMapsTheOpticalAxisToX) {
  const auto T = rosCameraToOptical();  // optical -> ROS camera frame
  auto z = T.apply({0, 0, 1});          // optical axis
  EXPECT_NEAR(z.x, 1, 1e-12); EXPECT_NEAR(z.y, 0, 1e-12); EXPECT_NEAR(z.z, 0, 1e-12);
  auto x = T.apply({1, 0, 0});          // image right -> -y (right of a forward-looking ROS frame)
  EXPECT_NEAR(x.y, -1, 1e-12);
  auto y = T.apply({0, 1, 0});          // image down -> -z
  EXPECT_NEAR(y.z, -1, 1e-12);
}

// ------------------------------------------------------------------------------------------ detections

TEST(Detections, DecodesSpotSprayingDetectionResult) {
  std::vector<std::uint8_t> b;
  auto put = [&](const void* p, std::size_t n) { const auto* c = static_cast<const std::uint8_t*>(p); b.insert(b.end(), c, c + n); };
  const std::uint32_t seq = 7, sec = 123, nsec = 456000000, len = 4, n = 2;
  put(&seq, 4); put(&sec, 4); put(&nsec, 4); put(&len, 4); put("body", 4); put(&n, 4);
  const std::int32_t b0[5] = {10, 20, 30, 40, 0};
  const float s0 = 0.9f;
  const std::int32_t b1[5] = {-5, 0, 7, 8, 2};
  const float s1 = 0.25f;
  put(b0, 20); put(&s0, 4); put(b1, 20); put(&s1, 4);
  StampNs st = 0;
  std::vector<Detection2D> boxes;
  std::string err;
  ASSERT_TRUE(parseSpotDetectionResult(b.data(), b.size(), &st, &boxes, &err)) << err;
  EXPECT_EQ(st, 123456000000LL);
  ASSERT_EQ(boxes.size(), 2u);
  EXPECT_EQ(boxes[0].x, 10); EXPECT_EQ(boxes[0].y, 20); EXPECT_EQ(boxes[0].width, 30); EXPECT_EQ(boxes[0].height, 40);
  EXPECT_FLOAT_EQ(boxes[0].score, 0.9f);
  EXPECT_EQ(boxes[1].x, -5); EXPECT_EQ(boxes[1].label, 2); EXPECT_FLOAT_EQ(boxes[1].score, 0.25f);
  // truncated / inconsistent messages are rejected, never half-read
  EXPECT_FALSE(parseSpotDetectionResult(b.data(), b.size() - 1, &st, &boxes, &err));
  EXPECT_FALSE(parseSpotDetectionResult(b.data(), 10, &st, &boxes, &err));
  b.push_back(0);
  EXPECT_FALSE(parseSpotDetectionResult(b.data(), b.size(), &st, &boxes, &err));
}

// ------------------------------------------------------------------------------------------ pairing

TEST(FrameSync, ExactPairingEitherOrder) {
  FrameSync s;
  auto out = s.addDisparity(1.00, 100 * kMs, grid1(5.f));
  EXPECT_TRUE(out.empty());
  out = s.addDetections(1.01, 100 * kMs, oneBox(), "SCAN", 3);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].pairing, SyncedFrame::Pairing::kExact);
  EXPECT_TRUE(out[0].has_disparity);
  EXPECT_EQ(out[0].stamp, 100 * kMs);
  EXPECT_EQ(out[0].phase, "SCAN");
  EXPECT_EQ(out[0].hop_target, 3);
  EXPECT_DOUBLE_EQ(out[0].t, 1.01);
  // detection first, disparity later: "late"
  out = s.addDetections(1.10, 200 * kMs, oneBox(), "SCAN", -1);
  EXPECT_TRUE(out.empty());
  out = s.addDisparity(1.12, 200 * kMs, grid1(6.f));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].pairing, SyncedFrame::Pairing::kLate);
  EXPECT_FLOAT_EQ(out[0].disparity.data[0], 6.f);
  EXPECT_EQ(s.counters().at("frames_paired"), 1u);
  EXPECT_EQ(s.counters().at("frames_paired_late"), 1u);
}

TEST(FrameSync, DuplicateDisparityStampIsDroppedAndCounted) {
  FrameSync s;
  s.addDisparity(1.0, 100 * kMs, grid1(5.f));
  s.addDisparity(1.0, 100 * kMs, grid1(99.f));  // the recording publishes every stamp twice
  auto out = s.addDetections(1.0, 100 * kMs, oneBox(), "", -1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].disparity.data[0], 5.f);  // the first valid copy wins
  EXPECT_EQ(s.counters().at("disparity_duplicate"), 1u);
}

TEST(FrameSync, AfterTheWaitNearestWithinToleranceElseNoDisparity) {
  FrameSync s;  // wait 0.5 s, tolerance 20 ms
  s.addDisparity(1.0, 1000 * kMs, grid1(5.f));
  EXPECT_TRUE(s.addDetections(1.0, 1015 * kMs, oneBox(), "SCAN", -1).empty());   // 15 ms off: waits first
  EXPECT_TRUE(s.addDetections(1.0, 1100 * kMs, oneBox(), "SCAN", -1).empty());   // 100 ms off
  EXPECT_TRUE(s.poll(1.4).empty());                                               // not yet waited 0.5 s
  auto out = s.poll(1.6);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].pairing, SyncedFrame::Pairing::kNearest);
  EXPECT_EQ(out[0].stamp, 1000 * kMs);  // takes the disparity stamp: pose and depth stay consistent
  EXPECT_TRUE(out[0].has_disparity);
  EXPECT_EQ(out[1].pairing, SyncedFrame::Pairing::kNone);
  EXPECT_FALSE(out[1].has_disparity);
  EXPECT_EQ(out[1].boxes.size(), 1u);  // the frame still carries its boxes
  EXPECT_EQ(s.counters().at("frames_paired_nearest"), 1u);
  EXPECT_EQ(s.counters().at("frames_without_disparity"), 1u);
}

TEST(FrameSync, ReleasesFramesInDetectionReceiveOrder) {
  FrameSync::Params p;
  p.early_resolve = false;  // this stream delivers disparity out of stamp order
  FrameSync s(p);
  EXPECT_TRUE(s.addDetections(1.00, 100 * kMs, oneBox(), "SCAN", -1).empty());   // waits for its disparity
  s.addDisparity(1.01, 150 * kMs, grid1(1.f));
  EXPECT_TRUE(s.addDetections(1.02, 150 * kMs, oneBox(), "SCAN", -1).empty());   // complete, but held behind #1
  EXPECT_EQ(s.pending(), 2u);
  auto out = s.addDisparity(1.03, 100 * kMs, grid1(2.f));                          // completes #1
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].stamp, 100 * kMs);
  EXPECT_EQ(out[1].stamp, 150 * kMs);
  EXPECT_EQ(s.pending(), 0u);
}

TEST(FrameSync, CacheIsBoundedAndFlushResolvesEverything) {
  FrameSync::Params p;
  p.cache = 3;
  FrameSync s(p);
  for (int k = 0; k < 10; ++k) s.addDisparity(1.0, k * 100 * kMs, grid1(static_cast<float>(k)));
  // stamp 0 was evicted: a detection for it can no longer pair exactly or by nearest
  EXPECT_TRUE(s.addDetections(1.0, 0, oneBox(), "", -1).empty());
  auto out = s.flush();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].has_disparity);
  auto ex = s.addDetections(1.0, 900 * kMs, oneBox(), "", -1);
  ASSERT_EQ(ex.size(), 1u);
  EXPECT_FLOAT_EQ(ex[0].disparity.data[0], 9.f);
}

TEST(FrameSync, EarlyResolveOnceANewerDisparityArrivedSameResultNoStall) {
  for (bool early : {false, true}) {
    FrameSync::Params p;
    p.early_resolve = early;
    FrameSync s(p);
    s.addDisparity(1.00, 1000 * kMs, grid1(5.f));
    EXPECT_TRUE(s.addDetections(1.01, 1015 * kMs, oneBox(), "SCAN", -1).empty());  // exact one never comes
    auto out = s.addDisparity(1.05, 1030 * kMs, grid1(6.f));                         // within tol: could be nearer
    EXPECT_TRUE(out.empty());
    out = s.addDisparity(1.08, 1060 * kMs, grid1(7.f));                              // > 1015 + 20 ms
    if (!early) {
      EXPECT_TRUE(out.empty());
      out = s.poll(1.6);
    }
    ASSERT_EQ(out.size(), 1u) << early;
    EXPECT_EQ(out[0].pairing, SyncedFrame::Pairing::kNearest);
    EXPECT_EQ(out[0].stamp, 1000 * kMs);  // tie (15 ms each side) -> the older one, like numpy argmin over arrival order
    EXPECT_FLOAT_EQ(out[0].disparity.data[0], 5.f);
  }
}
