// Reader of the frame stream file (frames.bin, format: tools/treemapper_tools/frames.py).
// Offline tooling only (replay runner); the ROS nodes receive the same FrameInput from topics.
#pragma once

#include <cstdio>
#include <string>

#include "treemapper/frame.hpp"
#include "treemapper/geometry.hpp"

namespace treemapper {

/// One recorded frame: the mapper input plus navigation's side of the replay.
struct RecordedFrame {
  FrameInput input;            ///< input.hop_target is left kNoTree: the navigation emulator resolves it
  std::string phase_name;      ///< as sent by navigation ("" = none)
  int nav_target_id{-1};       ///< hop target in the RECORDED navigation's ID space (-1 = none)
  double target_x{0.0}, target_y{0.0};  ///< descent point for that target (NaN = unknown)
};

class FrameReader {
 public:
  explicit FrameReader(const std::string& path);  ///< throws std::runtime_error on a bad file
  ~FrameReader();
  FrameReader(const FrameReader&) = delete;
  FrameReader& operator=(const FrameReader&) = delete;
  const StereoCalibration& calibration() const { return cal_; }
  const std::string& headerJson() const { return header_; }
  /// Next frame; false at end of file.
  bool next(RecordedFrame* out);

 private:
  std::FILE* f_{nullptr};
  StereoCalibration cal_;
  std::string header_;
  int gw_{0}, gh_{0};
};

}  // namespace treemapper
