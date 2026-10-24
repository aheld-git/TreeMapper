// The ROS parameter file and the core configuration agree (single source of truth = config/<mapper>.cfg):
//  * config/treemapper.yaml names a registered mapper and points at <repo>/config/<mapper>.cfg (through the
//    package's config/core symlink), with no overrides;
//  * the node's .cfg reader builds exactly the key/value set the core's own config test reads, and the mapper
//    accepts it through the registry (unknown keys would throw);
//  * no launch file sets a mapper key directly (they may only redirect mapper_config_file).
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>

#include <limits.h>
#include <stdlib.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "treemapper/mapper_registry.hpp"
#include "treemapper_ros/node_support.hpp"

using namespace treemapper_ros;

namespace {
std::string realPath(const std::string& p) {
  char buf[PATH_MAX];
  return ::realpath(p.c_str(), buf) ? std::string(buf) : std::string();
}
std::string readFile(const std::string& p) {
  std::ifstream in(p);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}
treemapper::StereoCalibration someCalibration() {
  treemapper::StereoCalibration c;
  c.width = 640; c.height = 400; c.fx = c.fy = 454.5; c.cx = 346.7; c.cy = 209.7; c.baseline = 0.16; c.stride = 4;
  return c;
}
const std::string kPkg = TM_ROS_PKG_DIR;
const std::string kRepo = realPath(std::string(TM_CORE_DIR) + "/..");
}  // namespace

TEST(ConfigAgreement, YamlPointsAtTheCoreConfigOfARegisteredMapper) {
  const YAML::Node y = YAML::LoadFile(kPkg + "/config/treemapper.yaml");
  const std::string mapper = y["mapper"].as<std::string>();
  const auto names = treemapper::registeredMappers();
  EXPECT_NE(std::find(names.begin(), names.end(), mapper), names.end()) << mapper << " is not registered";
  const std::string file = y["mapper_config_file"].as<std::string>();
  ASSERT_FALSE(file.empty());
  const std::string resolved = realPath(file[0] == '/' ? file : kPkg + "/" + file);
  ASSERT_FALSE(resolved.empty()) << file << " does not resolve";
  EXPECT_EQ(resolved, realPath(kRepo + "/config/" + mapper + ".cfg"));
  ASSERT_TRUE(y["mapper_overrides"]);
  EXPECT_EQ(y["mapper_overrides"].size(), 0u) << "the flight configuration must equal config/" << mapper << ".cfg";
}

TEST(ConfigAgreement, NodeReaderBuildsTheCoreFileAndTheMapperAcceptsIt) {
  const YAML::Node y = YAML::LoadFile(kPkg + "/config/treemapper.yaml");
  const std::string mapper = y["mapper"].as<std::string>();
  const auto cfg = readCfgFile(kRepo + "/config/" + mapper + ".cfg");
  ASSERT_FALSE(cfg.empty());
  // every non-comment line of the file is one key: the reader neither drops nor invents keys
  std::ifstream in(kRepo + "/config/" + mapper + ".cfg");
  std::string line;
  std::size_t lines = 0;
  while (std::getline(in, line)) {
    const auto h = line.find('#');
    if (h != std::string::npos) line = line.substr(0, h);
    if (line.find('=') != std::string::npos) ++lines;
  }
  EXPECT_EQ(cfg.size(), lines);
  EXPECT_NO_THROW(treemapper::createMapper(mapper, cfg, someCalibration()));
  // an override of an unknown key must fail loudly, not run the defaults
  EXPECT_THROW(treemapper::createMapper(mapper, applyOverrides(cfg, {{"no_such_key", "1"}}), someCalibration()),
               std::invalid_argument);
}

TEST(ConfigAgreement, EveryShippedCoreConfigParsesAndIsAccepted) {
  for (const auto& name : treemapper::registeredMappers()) {
    const std::string p = kRepo + "/config/" + name + ".cfg";
    if (!std::ifstream(p)) continue;  // e.g. simple has no file (built-in defaults only)
    treemapper::MapperConfig cfg;
    ASSERT_NO_THROW(cfg = readCfgFile(p)) << p;
    EXPECT_NO_THROW(treemapper::createMapper(name, cfg, someCalibration())) << p;
  }
}

TEST(ConfigAgreement, LaunchFilesDoNotSetMapperKeys) {
  for (const char* f : {"treemapper_sim.launch", "treemapper_real.launch", "replay_isaac.launch"}) {
    const std::string s = readFile(kPkg + "/launch/" + f);
    ASSERT_FALSE(s.empty()) << f;
    EXPECT_EQ(s.find("mapper_overrides"), std::string::npos) << f;
    EXPECT_EQ(s.find("name=\"mapper\""), std::string::npos) << f;
  }
}

TEST(CfgReader, CommentsBlankLinesAndErrors) {
  const std::string p = "/tmp/treemapper_ros_test_cfg_" + std::to_string(::getpid()) + ".cfg";
  {
    std::ofstream o(p);
    o << "# comment\n\n a = 1   # trailing comment\nb=two\n";
  }
  const auto c = readCfgFile(p);
  ASSERT_EQ(c.size(), 2u);
  EXPECT_EQ(c.at("a"), "1");
  EXPECT_EQ(c.at("b"), "two");
  {
    std::ofstream o(p);
    o << "a = 1\na = 2\n";
  }
  EXPECT_THROW(readCfgFile(p), std::invalid_argument);
  {
    std::ofstream o(p);
    o << "just words\n";
  }
  EXPECT_THROW(readCfgFile(p), std::invalid_argument);
  std::remove(p.c_str());
  EXPECT_THROW(readCfgFile("/nonexistent/x.cfg"), std::runtime_error);
}
