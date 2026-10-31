#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "corridor_nav/config.hpp"
#include "corridor_nav_ros/field_file.hpp"

using namespace corridor_nav_ros;

namespace {
std::string writeTmp(const std::string& name, const std::string& body) {
  const std::string path = ::testing::TempDir() + name;
  std::ofstream(path) << body;
  return path;
}

// Throws std::invalid_argument whose message contains `what`.
void expectRejects(const std::string& path, const std::string& what) {
  try {
    loadFieldFile(path);
    FAIL() << "expected std::invalid_argument mentioning '" << what << "'";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find(what), std::string::npos) << e.what();
  }
}
}  // namespace

TEST(FieldFile, ParsesAndRejects) {
  const auto ok = writeTmp("ff_ok.yaml",
                           "polygon: [[0, 0], [30, 0], [30, 20], [0, 20]]\nheading_rad: 0.5\nhome: [1.5, -2]\n");
  bool has_home = false;
  const auto f = loadFieldFile(ok, &has_home);
  ASSERT_EQ(f.polygon.size(), 4u);
  ASSERT_TRUE(f.heading_rad.has_value());
  EXPECT_DOUBLE_EQ(*f.heading_rad, 0.5);
  EXPECT_TRUE(has_home);
  EXPECT_DOUBLE_EQ(f.home.x, 1.5);
  EXPECT_DOUBLE_EQ(f.home.y, -2.0);
  EXPECT_DOUBLE_EQ(f.polygon[2].x, 30.0);

  expectRejects(writeTmp("ff_two.yaml", "polygon: [[0, 0], [1, 1]]\n"), "polygon");
  expectRejects(writeTmp("ff_nopoly.yaml", "heading_rad: 1.0\n"), "polygon");
  expectRejects(writeTmp("ff_badpt.yaml", "polygon: [[0, 0], [1, 0], [1]]\n"), "polygon");
  expectRejects(writeTmp("ff_nan.yaml", "polygon: [[0, 0], [1, 0], [.nan, 1]]\n"), "polygon");
  expectRejects(writeTmp("ff_flat.yaml", "polygon: [[0, 0], [1, 0], [2, 0]]\n"), "polygon");
  expectRejects(writeTmp("ff_head.yaml", "polygon: [[0, 0], [1, 0], [1, 1]]\nheading_rad: .inf\n"), "heading_rad");
  expectRejects(writeTmp("ff_home.yaml", "polygon: [[0, 0], [1, 0], [1, 1]]\nhome: [1]\n"), "home");
  expectRejects(writeTmp("ff_yaml.yaml", "polygon: [[0, 0\n"), "ff_yaml.yaml");
  expectRejects(::testing::TempDir() + "does_not_exist.yaml", "does_not_exist.yaml");
}

TEST(FieldFile, OptionalKeysAndWinding) {
  // clockwise input is stored counter-clockwise; no heading, no home
  const auto path = writeTmp("ff_cw.yaml", "polygon: [[0, 0], [0, 10], [10, 10], [10, 0]]\n");
  bool has_home = true;
  const auto f = loadFieldFile(path, &has_home);
  EXPECT_FALSE(f.heading_rad.has_value());
  EXPECT_FALSE(has_home);
  ASSERT_EQ(f.polygon.size(), 4u);
  double a = 0.0;
  for (std::size_t k = 0; k < f.polygon.size(); ++k) {
    const auto& p = f.polygon[k];
    const auto& q = f.polygon[(k + 1) % f.polygon.size()];
    a += p.x * q.y - q.x * p.y;
  }
  EXPECT_GT(a, 0.0);
}

TEST(CfgFile, ShippedFileLoadsThroughTheConfigSymlink) {
  // config/core -> <repo>/config: the node's default ~config_file resolves, and the navigator accepts it.
  const auto cfg = readCfgFile(std::string(CN_ROS_PKG_DIR) + "/config/core/corridor_nav.cfg");
  EXPECT_FALSE(cfg.empty());
  EXPECT_NO_THROW(corridor_nav::configFrom(cfg));
}

TEST(CfgFile, RejectsMalformedAndDuplicateLines) {
  EXPECT_THROW(readCfgFile(::testing::TempDir() + "nope.cfg"), std::runtime_error);
  EXPECT_THROW(readCfgFile(writeTmp("c_bad.cfg", "z_scan 15\n")), std::invalid_argument);
  EXPECT_THROW(readCfgFile(writeTmp("c_dup.cfg", "z_scan = 15\nz_scan = 16\n")), std::invalid_argument);
  const auto c = readCfgFile(writeTmp("c_ok.cfg", "# c\n\nz_scan = 15 # trailing\nstrategy=field_first\n"));
  EXPECT_EQ(c.at("z_scan"), "15");
  EXPECT_EQ(c.at("strategy"), "field_first");
}
