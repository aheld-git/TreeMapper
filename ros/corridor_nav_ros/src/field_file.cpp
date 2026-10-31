#include "corridor_nav_ros/field_file.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace corridor_nav_ros {

namespace {
[[noreturn]] void fail(const std::string& path, const std::string& what) {
  throw std::invalid_argument(path + ": " + what);
}

double finiteScalar(const std::string& path, const YAML::Node& n, const std::string& what) {
  double v = 0.0;
  try {
    if (!n.IsScalar()) fail(path, what);
    v = n.as<double>();
  } catch (const YAML::Exception&) {
    fail(path, what);
  }
  if (!std::isfinite(v)) fail(path, what);
  return v;
}

treemapper::Point2 point(const std::string& path, const YAML::Node& n, const std::string& key, const std::string& at) {
  if (!n.IsSequence() || n.size() != 2) fail(path, key + ": " + at + " must be [x, y]");
  return {finiteScalar(path, n[0], key + ": " + at + " has a non-numeric or non-finite x"),
          finiteScalar(path, n[1], key + ": " + at + " has a non-numeric or non-finite y")};
}

std::string trim(const std::string& s) {
  const auto a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
  return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}
}  // namespace

corridor_nav::Field loadFieldFile(const std::string& path, bool* has_home) {
  if (has_home) *has_home = false;
  if (!std::ifstream(path)) fail(path, "cannot open field file");
  YAML::Node y;
  try {
    y = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    fail(path, std::string("not valid YAML: ") + e.what());
  }
  if (!y.IsMap()) fail(path, "expected a mapping with the key 'polygon'");
  for (auto it = y.begin(); it != y.end(); ++it) {
    const std::string k = it->first.as<std::string>("");
    if (k != "polygon" && k != "heading_rad" && k != "home") fail(path, "unknown key '" + k + "'");
  }

  corridor_nav::Field f;
  const YAML::Node poly = y["polygon"];
  if (!poly || !poly.IsSequence()) fail(path, "polygon: missing, expected [[x, y], ...]");
  if (poly.size() < 3) fail(path, "polygon: needs at least 3 vertices, got " + std::to_string(poly.size()));
  for (std::size_t k = 0; k < poly.size(); ++k) {
    f.polygon.push_back(point(path, poly[k], "polygon", "vertex " + std::to_string(k)));
  }
  double twice_area = 0.0;
  for (std::size_t k = 0; k < f.polygon.size(); ++k) {
    const auto& p = f.polygon[k];
    const auto& q = f.polygon[(k + 1) % f.polygon.size()];
    twice_area += p.x * q.y - q.x * p.y;
  }
  if (!(std::fabs(twice_area) > 1e-9)) fail(path, "polygon: has zero area");
  if (twice_area < 0.0) std::reverse(f.polygon.begin(), f.polygon.end());   // Field::polygon is counter-clockwise

  if (y["heading_rad"]) f.heading_rad = finiteScalar(path, y["heading_rad"], "heading_rad: expected a finite number");
  if (y["home"]) {
    f.home = point(path, y["home"], "home", "home");
    if (has_home) *has_home = true;
  }
  return f;
}

treemapper::MapperConfig readCfgFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open config file '" + path + "'");
  treemapper::MapperConfig c;
  std::string line;
  int n = 0;
  while (std::getline(in, line)) {
    ++n;
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    line = trim(line);
    if (line.empty()) continue;
    const auto eq = line.find('=');
    const std::string key = eq == std::string::npos ? std::string() : trim(line.substr(0, eq));
    if (key.empty()) {
      throw std::invalid_argument(path + ":" + std::to_string(n) + ": expected 'key = value', got '" + line + "'");
    }
    if (c.count(key)) throw std::invalid_argument(path + ":" + std::to_string(n) + ": duplicate key '" + key + "'");
    c[key] = trim(line.substr(eq + 1));
  }
  return c;
}

}  // namespace corridor_nav_ros
