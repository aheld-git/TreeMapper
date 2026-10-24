// Registry of mapping systems, keyed by name. The only place that knows every system is
// src/mapper_registry.cpp, where each system has exactly one registration line.
//
//   auto m = treemapper::createMapper("simple", {}, calibration);   // throws std::invalid_argument
//
// How to add a system: see PROGRESS.md, "How to add a candidate".
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "treemapper/geometry.hpp"
#include "treemapper/mapper.hpp"

namespace treemapper {

/// Factory signature: config in, ready mapper out. Throws std::invalid_argument on a bad config.
using MapperFactory = std::unique_ptr<Mapper> (*)(const MapperConfig&, const StereoCalibration&);

/// Names of all registered systems, sorted.
std::vector<std::string> registeredMappers();

/// Creates the system `name`. Throws std::invalid_argument (message lists the registered names) for an
/// unknown name, and whatever the factory throws for a bad config.
std::unique_ptr<Mapper> createMapper(const std::string& name, const MapperConfig& config,
                                     const StereoCalibration& calibration);

/// Parses "key=value;key=value" (the `--config` text of treemapper_replay). Empty text -> empty config.
/// Throws std::invalid_argument on a pair without '='.
MapperConfig parseMapperConfig(const std::string& text);

}  // namespace treemapper
