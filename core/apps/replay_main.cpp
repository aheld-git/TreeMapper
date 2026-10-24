// treemapper_replay: drive a mapper with a recorded/synthetic frame stream and write the scorer's
// map stream (contract JSONL) at a steady rate.
//
//   treemapper_replay --frames F.bin --out-map map.jsonl --out-targets targets.json
//                     [--out-counters counters.json] [--rate 5] [--system simple]
//
// Navigation emulator (replay only, not part of the mapper): the recorded hop target belongs to the
// recorded navigation's ID space, so each target is resolved to THIS mapper's ID through its descent
// point (published tree whose outline contains it, else the nearest centre within 3 m), and the tree
// is marked sprayed when the SPRAYING phase for it ends. The resolved target timeline is written to
// --out-targets so the scorer judges the descent rules against the IDs this mapper actually received.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "treemapper/frame_io.hpp"
#include "treemapper/simple_mapper.hpp"

using namespace treemapper;

namespace {

void writeSnapshot(std::FILE* f, double t, const std::vector<TreeEntry>& trees,
                   const std::map<std::string, double>& stage_ms) {
  std::fprintf(f, "{\"t\":%.6f,\"frame\":\"world\",\"trees\":[", t);
  for (std::size_t i = 0; i < trees.size(); ++i) {
    const auto& e = trees[i];
    std::fprintf(f, "%s{\"id\":%d,\"outline\":[", i ? "," : "", e.id);
    for (std::size_t k = 0; k < e.outline.size(); ++k) {
      std::fprintf(f, "%s[%.4f,%.4f]", k ? "," : "", e.outline[k].x, e.outline[k].y);
    }
    std::fprintf(f, "],\"centre\":[%.4f,%.4f],", e.centre.x, e.centre.y);
    if (std::isfinite(e.height)) std::fprintf(f, "\"height\":%.4f,", e.height);
    else std::fprintf(f, "\"height\":null,");
    std::fprintf(f, "\"sprayed\":%s}", e.sprayed ? "true" : "false");
  }
  std::fprintf(f, "]");
  if (!stage_ms.empty()) {
    std::fprintf(f, ",\"stage_ms\":{");
    bool first = true;
    for (const auto& kv : stage_ms) {
      std::fprintf(f, "%s\"%s\":%.4f", first ? "" : ",", kv.first.c_str(), kv.second);
      first = false;
    }
    std::fprintf(f, "}");
  }
  std::fprintf(f, "}\n");
}

TreeId resolveTarget(const std::vector<TreeEntry>& trees, double x, double y) {
  if (!std::isfinite(x) || !std::isfinite(y)) return kNoTree;
  TreeId best = kNoTree;
  double bd = 3.0;
  for (const auto& e : trees) {
    if (pointInPolygon(e.outline, {x, y})) return e.id;
    const double d = std::hypot(e.centre.x - x, e.centre.y - y);
    if (d < bd) { bd = d; best = e.id; }
  }
  return best;
}

}  // namespace

int main(int argc, char** argv) {
  std::map<std::string, std::string> a;
  for (int i = 1; i + 1 < argc; i += 2) a[argv[i]] = argv[i + 1];
  if (!a.count("--frames") || !a.count("--out-map") || !a.count("--out-targets")) {
    std::cerr << "usage: treemapper_replay --frames F.bin --out-map M.jsonl --out-targets T.json "
                 "[--out-counters C.json] [--rate 5] [--system simple]\n";
    return 2;
  }
  const double rate = a.count("--rate") ? std::stod(a["--rate"]) : 5.0;
  const std::string system = a.count("--system") ? a["--system"] : "simple";
  if (system != "simple") { std::cerr << "unknown --system " << system << "\n"; return 2; }

  FrameReader reader(a["--frames"]);
  SimpleMapper mapper(SimpleMapperConfig{}, reader.calibration());
  std::FILE* out = std::fopen(a["--out-map"].c_str(), "w");
  if (!out) { std::cerr << "cannot write " << a["--out-map"] << "\n"; return 1; }

  std::vector<std::pair<double, TreeId>> targets;
  RecordedFrame fr;
  bool first = true;
  double next_snap = 0.0, last_t = 0.0, last_snap_t = -1e18;
  std::string prev_phase;
  int nav_id = -1;
  double tx = std::nan(""), ty = std::nan("");
  TreeId resolved = kNoTree;
  long n_frames = 0, n_snaps = 0;
  while (reader.next(&fr)) {
    const double t = fr.input.t;
    if (first) {
      next_snap = t;
      targets.push_back({t, kNoTree});
      first = false;
    }
    // On a phase change, record the map as it stood after the last frame of the previous phase, stamped
    // with that frame's time. Without it, a tree published by the last SCAN frame first appears in the
    // next steady-rate snapshot, which already falls in PLANNING. That would be a cadence artefact, not a
    // tree created after scanning.
    if (!prev_phase.empty() && fr.phase_name != prev_phase && last_t > last_snap_t) {
      writeSnapshot(out, last_t, mapper.publishedTrees(), mapper.takeStageTiming());
      ++n_snaps;
      last_snap_t = last_t;
    }
    while (next_snap <= t) {  // snapshots reflect every frame received strictly before them
      if (next_snap > last_snap_t) {
        writeSnapshot(out, next_snap, mapper.publishedTrees(), mapper.takeStageTiming());
        ++n_snaps;
        last_snap_t = next_snap;
      }
      next_snap += 1.0 / rate;
    }
    // --- navigation emulator
    if (prev_phase == "SPRAYING" && fr.phase_name != "SPRAYING" && resolved != kNoTree) {
      mapper.markSprayed(resolved);  // navigation reports the tree done when spraying ends
    }
    prev_phase = fr.phase_name;
    const bool target_changed = fr.nav_target_id != nav_id ||
                                (std::isfinite(fr.target_x) != std::isfinite(tx)) ||
                                (std::isfinite(fr.target_x) && (fr.target_x != tx || fr.target_y != ty));
    if (target_changed) {
      nav_id = fr.nav_target_id;
      tx = fr.target_x;
      ty = fr.target_y;
      const TreeId r = nav_id < 0 ? kNoTree : resolveTarget(mapper.publishedTrees(), tx, ty);
      if (r != resolved) {
        resolved = r;
        targets.push_back({t, resolved});
      }
    } else if (nav_id >= 0 && resolved == kNoTree) {
      const TreeId r = resolveTarget(mapper.publishedTrees(), tx, ty);
      if (r != kNoTree) {
        resolved = r;
        targets.push_back({t, resolved});
      }
    }
    fr.input.hop_target = resolved;
    mapper.process(fr.input);
    last_t = t;
    ++n_frames;
  }
  if (last_t > last_snap_t) {
    writeSnapshot(out, last_t, mapper.publishedTrees(), mapper.takeStageTiming());
    ++n_snaps;
  }
  std::fclose(out);

  std::ofstream tf(a["--out-targets"]);
  tf << "[";
  for (std::size_t i = 0; i < targets.size(); ++i) {
    tf << (i ? "," : "") << "[" << std::to_string(targets[i].first) << "," << targets[i].second << "]";
  }
  tf << "]\n";
  if (a.count("--out-counters")) {
    std::ofstream cf(a["--out-counters"]);
    cf << "{";
    bool f1 = true;
    for (const auto& kv : mapper.counters()) {
      cf << (f1 ? "" : ",") << "\"" << kv.first << "\":" << kv.second;
      f1 = false;
    }
    cf << "}\n";
  }
  std::cerr << "treemapper_replay: " << n_frames << " frames, " << n_snaps << " snapshots, "
            << mapper.publishedTrees().size() << " trees in final map\n";
  return 0;
}
