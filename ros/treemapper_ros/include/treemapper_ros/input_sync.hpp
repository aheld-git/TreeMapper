// ROS-free input stage of treemapper_node: disparity sampling, stereo calibration from CameraInfo
// numbers, and the detection <-> disparity pairing. Everything here is plain C++17 so it is unit-tested
// without a ROS graph (test/test_input_sync.cpp) and behaves exactly like the offline bag reader
// (tools/treemapper_tools/bag_frames.py):
//
//  * disparity: 32FC1 only, step == 4 * width, data size == step * height; anything else is malformed
//    and counted (`disparity_rejected_malformed`); a second valid copy of a stamp is dropped
//    (`disparity_duplicate`); the grid is sampled at (r*s + s/2, c*s + s/2); invalid (<= 0, non-finite)
//    -> NaN.
//  * pairing: a detection is paired with the disparity of the SAME stamp. A detection that waited longer
//    than `wait_s` (receive clock) gets the nearest cached disparity within `nearest_tol_s` (the frame then
//    takes the disparity stamp, so pose and depth stay consistent), else it becomes a frame without
//    disparity. Frames are released in detection-receive order, as the offline reader sorts them.
#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "treemapper/geometry.hpp"
#include "treemapper/types.hpp"

namespace treemapper_ros {

/// Header stamp in integer nanoseconds (exact equality, no float rounding).
using StampNs = std::int64_t;
inline double toSec(StampNs s) { return static_cast<double>(s) * 1e-9; }
inline StampNs fromSec(double s) { return static_cast<StampNs>(s * 1e9 + (s >= 0 ? 0.5 : -0.5)); }

// ------------------------------------------------------------------------------------------ disparity

struct DisparityGrid {
  int grid_w{0}, grid_h{0};
  std::vector<float> data;  ///< row-major grid_h x grid_w, NaN = invalid
};

enum class DisparityStatus { kOk, kMalformed, kResolutionMismatch };

/// Grid size of a `width` x `height` image sampled with `stride` (= len(range(stride/2, width, stride))).
int gridSize(int pixels, int stride);

/// Samples a disparity image onto the stride grid. `expected_w/h` = the calibrated image size (0 = do
/// not check). `offset_px` is subtracted from every raw value first (d - (cx_left - cx_right) when the
/// driver's disparity is measured between unequal principal points, as on the real drone; 0 in Isaac);
/// a value that is then <= 0 or non-finite is invalid (NaN). `quantum_px` > 0 rounds every valid value like
/// the frames.bin cache does (1/256 px), so a live run can be compared with an offline replay; 0 keeps full
/// float precision.
DisparityStatus sampleDisparity(const std::string& encoding, std::uint32_t width, std::uint32_t height,
                                std::uint32_t step, bool bigendian, const std::uint8_t* data, std::size_t size,
                                int stride, int expected_w, int expected_h, double offset_px, double quantum_px,
                                DisparityGrid* out);

// ------------------------------------------------------------------------------------------ calibration

struct BaselineResult {
  bool ok{false};
  double baseline_m{0.0};
  std::string form;   ///< which P[3] convention matched
  std::string error;  ///< why it was rejected
};

/// Stereo baseline from the RIGHT camera's projection matrix: P[3] = Tx, P[0] = fx.
/// Accepted (mutually exclusive within the plausible range 0.01..1 m):
///   "rep104_m"  Tx = -fx * B, B in metres (REP-104)
///   "neg_mm"    Tx = -fx * B, B in millimetres (the field driver and the Isaac recording)
///   "pos_mm"    Tx = +fx * B, B in millimetres
/// Anything else is rejected (fail closed: no depth without a trusted baseline).
BaselineResult deduceBaseline(double right_P3, double right_fx);

/// Builds the mapper calibration from the two CameraInfo P matrices (row-major 3x4) and image size.
/// Rejects an uncalibrated (all-zero / non-positive focal length) left camera and an implausible baseline.
/// `body_cam` = optical camera frame -> body. Returns false and sets `error` on rejection.
bool makeCalibration(int width, int height, const double left_P[12], const double right_P[12], int stride,
                     const treemapper::Transform& body_cam, treemapper::StereoCalibration* out, std::string* error,
                     std::string* baseline_form = nullptr);

/// ROS body-convention camera frame (x forward = optical axis, y left, z up) -> optical frame (z forward,
/// x right, y down): the fixed rotation q = (-0.5, 0.5, -0.5, 0.5). Compose with it when the mount TF
/// ends in a non-optical camera frame (`camera_frame_is_optical: false`).
treemapper::Transform rosCameraToOptical();

// ------------------------------------------------------------------------------------------ detections

/// The existing detector's message. Its layout (ROS 1 serialisation, little endian):
///   Header {uint32 seq, time stamp (uint32 sec, uint32 nsec), string frame_id (uint32 len + bytes)},
///   uint32 n, n x BoundingBox {int32 x, int32 y, int32 width, int32 height, int32 label, float32 score}.
/// The node subscribes with topic_tools::ShapeShifter and decodes it here, so the mapper does not depend
/// on spot_spraying_msgs (which is not part of this workspace). The md5 is checked by the caller.
constexpr const char* kSpotDetectionResultType = "spot_spraying_msgs/DetectionResult";
constexpr const char* kSpotDetectionResultMd5 = "e4a10b4f716e7b82b6d65613ad662a6a";
bool parseSpotDetectionResult(const std::uint8_t* data, std::size_t size, StampNs* stamp,
                              std::vector<treemapper::Detection2D>* boxes, std::string* error);

// ------------------------------------------------------------------------------------------ pairing

struct SyncedFrame {
  enum class Pairing { kExact, kLate, kNearest, kNone };
  double t{0.0};               ///< receive time of the detection message
  StampNs stamp{0};            ///< stamp of the frame (the disparity stamp for kNearest)
  std::vector<treemapper::Detection2D> boxes;
  std::string phase;           ///< phase string in force when the detections arrived
  treemapper::TreeId hop_target{treemapper::kNoTree};  ///< hop target in force when they arrived
  Pairing pairing{Pairing::kNone};
  bool has_disparity{false};
  DisparityGrid disparity;
};

class FrameSync {
 public:
  struct Params {
    double wait_s{0.5};          ///< how long a detection waits for its exact-stamp disparity
    double nearest_tol_s{0.02};  ///< fallback pairing tolerance
    std::size_t cache{400};      ///< disparity frames kept (by stamp, oldest evicted)
    /// Disparity stamps arrive in increasing order (one publisher): once a disparity newer than a waiting
    /// detection's stamp + nearest_tol_s has arrived, neither its exact match nor a closer neighbour can still
    /// come, so it is resolved at once instead of after wait_s. Same pairing result as the offline reader,
    /// without stalling every later frame for wait_s (frames are released in order).
    bool early_resolve{true};
  };
  FrameSync() : FrameSync(Params{}) {}
  explicit FrameSync(Params p) : p_(p) {}

  /// A valid sampled disparity frame arrived (receive time `t`). Returns the frames released by it.
  std::vector<SyncedFrame> addDisparity(double t, StampNs stamp, DisparityGrid grid);
  /// A detection message arrived (receive time `t`). Returns the frames released by it.
  std::vector<SyncedFrame> addDetections(double t, StampNs stamp, std::vector<treemapper::Detection2D> boxes,
                                         const std::string& phase, treemapper::TreeId hop_target);
  /// Resolves detections that waited longer than wait_s at receive time `now`. Returns released frames.
  std::vector<SyncedFrame> poll(double now);
  /// Resolves every waiting detection now (shutdown / end of a replay).
  std::vector<SyncedFrame> flush();

  std::size_t pending() const { return pending_.size(); }
  /// Cumulative event / drop counters (names as in the offline reader's meta counts).
  const std::map<std::string, std::uint64_t>& counters() const { return counters_; }

 private:
  struct Pending {
    double t;
    StampNs stamp;
    std::vector<treemapper::Detection2D> boxes;
    std::string phase;
    treemapper::TreeId hop_target;
    bool done{false};
    SyncedFrame frame;
  };
  void resolveWaiting(Pending& p);
  std::vector<SyncedFrame> release();

  Params p_;
  std::map<StampNs, DisparityGrid> disp_;   ///< cache by stamp
  std::deque<StampNs> disp_order_;          ///< insertion order for eviction
  StampNs newest_disp_{0};
  bool have_disp_{false};
  std::deque<Pending> pending_;             ///< in detection-receive order (released in that order)
  std::map<std::string, std::uint64_t> counters_;
};

}  // namespace treemapper_ros
