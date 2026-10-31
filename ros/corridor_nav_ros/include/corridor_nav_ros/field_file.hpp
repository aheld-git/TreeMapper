// The field file (YAML) and the key = value config file reader of the node.
#pragma once

#include <string>

#include "corridor_nav/types.hpp"
#include "treemapper/mapper.hpp"  // MapperConfig

namespace corridor_nav_ros {

/// Field file: `polygon: [[x, y], ...]` (>= 3 finite vertices, map frame; reordered counter-clockwise), optional
/// `heading_rad` (finite), optional `home: [x, y]`. Without `home` the field's home stays (0, 0) and
/// `*has_home` (if given) is false: the caller decides (the node uses the first pose it sees).
/// Throws std::invalid_argument naming the file and the problem (missing file, bad YAML, bad entry).
corridor_nav::Field loadFieldFile(const std::string& path, bool* has_home = nullptr);

/// `key = value` lines, '#' starts a comment (the format of config/corridor_nav.cfg). Throws std::runtime_error
/// when the file cannot be opened and std::invalid_argument on a malformed or duplicate line.
treemapper::MapperConfig readCfgFile(const std::string& path);

}  // namespace corridor_nav_ros
