#include "treemapper_ros/input_sync.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace treemapper_ros {

using treemapper::Detection2D;
using treemapper::StereoCalibration;
using treemapper::Transform;

// ------------------------------------------------------------------------------------------ disparity

int gridSize(int pixels, int stride) {
  if (pixels <= 0 || stride <= 0) return 0;
  const int o = stride / 2;
  return pixels > o ? (pixels - o + stride - 1) / stride : 0;
}

DisparityStatus sampleDisparity(const std::string& encoding, std::uint32_t width, std::uint32_t height,
                                std::uint32_t step, bool bigendian, const std::uint8_t* data, std::size_t size,
                                int stride, int expected_w, int expected_h, double quantum_px, DisparityGrid* out) {
  if (encoding != "32FC1" || bigendian || width == 0 || height == 0 || step != 4u * width ||
      size != static_cast<std::size_t>(step) * height || data == nullptr || stride <= 0) {
    return DisparityStatus::kMalformed;
  }
  if ((expected_w > 0 && static_cast<int>(width) != expected_w) ||
      (expected_h > 0 && static_cast<int>(height) != expected_h)) {
    return DisparityStatus::kResolutionMismatch;
  }
  const int o = stride / 2;
  out->grid_w = gridSize(static_cast<int>(width), stride);
  out->grid_h = gridSize(static_cast<int>(height), stride);
  out->data.assign(static_cast<std::size_t>(out->grid_w) * out->grid_h, std::numeric_limits<float>::quiet_NaN());
  const float q = static_cast<float>(quantum_px);
  for (int r = 0; r < out->grid_h; ++r) {
    const std::uint8_t* row = data + static_cast<std::size_t>(r * stride + o) * step;
    for (int c = 0; c < out->grid_w; ++c) {
      float d;
      std::memcpy(&d, row + static_cast<std::size_t>(c * stride + o) * 4u, sizeof(float));
      if (!std::isfinite(d) || d <= 0.0f) continue;
      if (q > 0.0f) {  // frames.bin: clip(round(d / q), 1, 65535) * q
        float k = std::nearbyint(d / q);
        k = std::min(std::max(k, 1.0f), 65535.0f);
        d = k * q;
      }
      out->data[static_cast<std::size_t>(r) * out->grid_w + c] = d;
    }
  }
  return DisparityStatus::kOk;
}

// ------------------------------------------------------------------------------------------ calibration

BaselineResult deduceBaseline(double right_P3, double right_fx) {
  BaselineResult r;
  constexpr double kMin = 0.01, kMax = 1.0;
  if (!std::isfinite(right_P3) || !std::isfinite(right_fx) || right_fx <= 0.0) {
    r.error = "right camera P[0] (fx) must be finite and > 0";
    return r;
  }
  const double b = -right_P3 / right_fx;  // REP-104: Tx = -fx * B
  auto plausible = [&](double v) { return v >= kMin && v <= kMax; };
  if (plausible(b)) {
    r.ok = true; r.baseline_m = b; r.form = "rep104_m";
  } else if (plausible(b / 1000.0)) {
    r.ok = true; r.baseline_m = b / 1000.0; r.form = "neg_mm";
  } else if (plausible(-b / 1000.0)) {
    r.ok = true; r.baseline_m = -b / 1000.0; r.form = "pos_mm";
  } else {
    r.error = "right camera P[3] = " + std::to_string(right_P3) + " gives no baseline in [0.01, 1] m under any "
              "known convention (-fx*B m, -fx*B mm, +fx*B mm)";
  }
  return r;
}

bool makeCalibration(int width, int height, const double left_P[12], const double right_P[12], int stride,
                     const Transform& body_cam, StereoCalibration* out, std::string* error,
                     std::string* baseline_form) {
  auto fail = [&](const std::string& m) { if (error) *error = m; return false; };
  if (width <= 0 || height <= 0) return fail("camera_info has no image size");
  if (stride <= 0) return fail("disparity_stride must be > 0");
  bool all_zero = true;
  for (int k = 0; k < 12; ++k) all_zero = all_zero && left_P[k] == 0.0;
  if (all_zero) return fail("left camera_info P is all zero (uncalibrated camera)");
  const double fx = left_P[0], fy = left_P[5], cx = left_P[2], cy = left_P[6];
  if (!(fx > 0.0) || !(fy > 0.0) || !std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) ||
      !std::isfinite(cy)) {
    return fail("left camera_info P has a non-positive or non-finite focal length / principal point");
  }
  const BaselineResult b = deduceBaseline(right_P[3], right_P[0]);
  if (!b.ok) return fail(b.error);
  out->width = width;
  out->height = height;
  out->fx = fx; out->fy = fy; out->cx = cx; out->cy = cy;
  out->baseline = b.baseline_m;
  out->stride = stride;
  out->body_cam = body_cam;
  if (baseline_form) *baseline_form = b.form;
  return true;
}

Transform rosCameraToOptical() {
  Transform t;
  t.qx = -0.5; t.qy = 0.5; t.qz = -0.5; t.qw = 0.5;
  return t;
}

// ------------------------------------------------------------------------------------------ detections

bool parseSpotDetectionResult(const std::uint8_t* data, std::size_t size, StampNs* stamp,
                              std::vector<Detection2D>* boxes, std::string* error) {
  std::size_t pos = 0;
  auto fail = [&](const char* m) { if (error) *error = m; return false; };
  auto u32 = [&](std::uint32_t* v) {
    if (pos + 4 > size) return false;
    std::memcpy(v, data + pos, 4);
    pos += 4;
    return true;
  };
  std::uint32_t seq = 0, sec = 0, nsec = 0, len = 0, n = 0;
  if (!data || !u32(&seq) || !u32(&sec) || !u32(&nsec) || !u32(&len)) return fail("truncated header");
  if (pos + len > size) return fail("truncated frame_id");
  pos += len;
  if (!u32(&n)) return fail("truncated box count");
  if (static_cast<std::uint64_t>(n) * 24u != size - pos) return fail("box array size does not match the message size");
  boxes->clear();
  boxes->reserve(n);
  for (std::uint32_t k = 0; k < n; ++k) {
    std::int32_t v[5];
    float score;
    std::memcpy(v, data + pos, 20);
    std::memcpy(&score, data + pos + 20, 4);
    pos += 24;
    Detection2D b;
    b.x = v[0]; b.y = v[1]; b.width = v[2]; b.height = v[3]; b.label = v[4];
    b.score = score;
    boxes->push_back(b);
  }
  *stamp = static_cast<StampNs>(sec) * 1000000000LL + nsec;
  return true;
}

// ------------------------------------------------------------------------------------------ pairing

std::vector<SyncedFrame> FrameSync::addDisparity(double t, StampNs stamp, DisparityGrid grid) {
  ++counters_["disparity_msgs_valid"];
  if (disp_.count(stamp)) {
    ++counters_["disparity_duplicate"];
    return poll(t);
  }
  disp_order_.push_back(stamp);
  auto& slot = disp_[stamp];
  slot = std::move(grid);
  while (disp_.size() > p_.cache && !disp_order_.empty()) {
    disp_.erase(disp_order_.front());
    disp_order_.pop_front();
  }
  if (!have_disp_ || stamp > newest_disp_) newest_disp_ = stamp;
  have_disp_ = true;
  for (auto& p : pending_) {
    if (!p.done && p.stamp == stamp) {
      p.frame.stamp = stamp;
      p.frame.pairing = SyncedFrame::Pairing::kLate;
      p.frame.has_disparity = true;
      p.frame.disparity = slot;
      p.done = true;
      ++counters_["frames_paired_late"];
    }
  }
  if (p_.early_resolve) {
    const StampNs tol = fromSec(p_.nearest_tol_s);
    for (auto& p : pending_) {
      if (!p.done && newest_disp_ > p.stamp + tol) {
        resolveWaiting(p);
        ++counters_["frames_resolved_early"];
      }
    }
  }
  auto out = release();
  auto more = poll(t);
  out.insert(out.end(), std::make_move_iterator(more.begin()), std::make_move_iterator(more.end()));
  return out;
}

std::vector<SyncedFrame> FrameSync::addDetections(double t, StampNs stamp, std::vector<Detection2D> boxes,
                                                  const std::string& phase, treemapper::TreeId hop_target) {
  ++counters_["detection_msgs"];
  Pending p;
  p.t = t;
  p.stamp = stamp;
  p.phase = phase;
  p.hop_target = hop_target;
  p.frame.t = t;
  p.frame.stamp = stamp;
  p.frame.phase = phase;
  p.frame.hop_target = hop_target;
  p.frame.boxes = std::move(boxes);
  auto it = disp_.find(stamp);
  if (it != disp_.end()) {
    p.frame.pairing = SyncedFrame::Pairing::kExact;
    p.frame.has_disparity = true;
    p.frame.disparity = it->second;
    p.done = true;
    ++counters_["frames_paired"];
  }
  pending_.push_back(std::move(p));
  auto out = release();
  auto more = poll(t);
  out.insert(out.end(), std::make_move_iterator(more.begin()), std::make_move_iterator(more.end()));
  return out;
}

void FrameSync::resolveWaiting(Pending& p) {
  // nearest cached disparity within the tolerance (the frame takes the disparity stamp)
  StampNs best = 0;
  double bd = std::numeric_limits<double>::infinity();
  for (const auto& kv : disp_) {
    const double d = std::fabs(toSec(kv.first - p.stamp));
    if (d < bd) { bd = d; best = kv.first; }
  }
  if (bd <= p_.nearest_tol_s) {
    p.frame.stamp = best;
    p.frame.pairing = SyncedFrame::Pairing::kNearest;
    p.frame.has_disparity = true;
    p.frame.disparity = disp_.at(best);
    ++counters_["frames_paired_nearest"];
  } else {
    p.frame.pairing = SyncedFrame::Pairing::kNone;
    p.frame.has_disparity = false;
    ++counters_["frames_without_disparity"];
  }
  p.done = true;
}

std::vector<SyncedFrame> FrameSync::poll(double now) {
  for (auto& p : pending_) {
    if (!p.done && now - p.t > p_.wait_s) resolveWaiting(p);
  }
  return release();
}

std::vector<SyncedFrame> FrameSync::flush() {
  for (auto& p : pending_) {
    if (!p.done) resolveWaiting(p);
  }
  return release();
}

std::vector<SyncedFrame> FrameSync::release() {
  std::vector<SyncedFrame> out;
  while (!pending_.empty() && pending_.front().done) {
    out.push_back(std::move(pending_.front().frame));
    pending_.pop_front();
  }
  return out;
}

}  // namespace treemapper_ros
