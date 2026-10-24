#include "treemapper/frame_io.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace treemapper {

namespace {
const char* kPhases[] = {"", "IDLE", "SCAN_APPROACH", "SCAN", "PLANNING", "HOPPING", "SPRAYING", "RETURN", "ERROR"};
template <typename T>
bool rd(std::FILE* f, T* v, std::size_t n = 1) { return std::fread(v, sizeof(T), n, f) == n; }
}  // namespace

FrameReader::FrameReader(const std::string& path) {
  f_ = std::fopen(path.c_str(), "rb");
  if (!f_) throw std::runtime_error("cannot open " + path);
  char magic[4];
  std::uint32_t n = 0;
  if (!rd(f_, magic, 4) || std::memcmp(magic, "TMF1", 4) != 0 || !rd(f_, &n)) {
    throw std::runtime_error(path + ": not a TMF1 frame file");
  }
  header_.resize(n);
  if (n && !rd(f_, &header_[0], n)) throw std::runtime_error(path + ": truncated header");
  double c[15];
  if (!rd(f_, c, 15)) throw std::runtime_error(path + ": truncated calibration");
  cal_.width = static_cast<int>(c[0]);
  cal_.height = static_cast<int>(c[1]);
  cal_.fx = c[2]; cal_.fy = c[3]; cal_.cx = c[4]; cal_.cy = c[5];
  cal_.baseline = c[6];
  cal_.stride = static_cast<int>(c[7]);
  cal_.body_cam = {c[8], c[9], c[10], c[11], c[12], c[13], c[14]};
  const int s = cal_.stride, o = s / 2;
  gw_ = (cal_.width - o + s - 1) / s;
  gh_ = (cal_.height - o + s - 1) / s;
}

FrameReader::~FrameReader() {
  if (f_) std::fclose(f_);
}

bool FrameReader::next(RecordedFrame* out) {
#pragma pack(push, 1)
  struct Rec {
    double t, stamp, pose[7];
    std::uint8_t valid;
    std::int8_t phase;
    std::int32_t target;
    double txy[2];
    std::uint16_t nb;
    std::uint8_t has;
  } r;
#pragma pack(pop)
  static_assert(sizeof(Rec) == 8 * 9 + 1 + 1 + 4 + 16 + 2 + 1, "record layout");
  if (!rd(f_, &r)) return false;
  FrameInput& in = out->input;
  in = FrameInput{};
  in.t = r.t;
  in.stamp = r.stamp;
  in.pose.t = r.stamp;
  in.pose.x = r.pose[0]; in.pose.y = r.pose[1]; in.pose.z = r.pose[2];
  in.pose.qx = r.pose[3]; in.pose.qy = r.pose[4]; in.pose.qz = r.pose[5]; in.pose.qw = r.pose[6];
  in.pose.valid = r.valid != 0;
  out->phase_name = (r.phase >= 0 && r.phase < 9) ? kPhases[r.phase] : "";
  in.phase = parseMissionPhase(out->phase_name);
  out->nav_target_id = r.target;
  out->target_x = r.txy[0];
  out->target_y = r.txy[1];
  std::vector<float> b(static_cast<std::size_t>(r.nb) * 6);
  if (r.nb && !rd(f_, b.data(), b.size())) throw std::runtime_error("truncated boxes");
  for (int i = 0; i < r.nb; ++i) {
    Detection2D d;
    d.x = b[i * 6]; d.y = b[i * 6 + 1]; d.width = b[i * 6 + 2]; d.height = b[i * 6 + 3];
    d.score = b[i * 6 + 4]; d.label = static_cast<int>(b[i * 6 + 5]);
    in.boxes.push_back(d);
  }
  if (r.has) {
    std::vector<std::uint16_t> q(static_cast<std::size_t>(gw_ * gh_));
    if (!rd(f_, q.data(), q.size())) throw std::runtime_error("truncated disparity");
    in.grid_w = gw_;
    in.grid_h = gh_;
    in.disparity.resize(q.size());
    for (std::size_t k = 0; k < q.size(); ++k) in.disparity[k] = q[k] ? q[k] / 256.0f : std::nanf("");
  }
  return true;
}

}  // namespace treemapper
