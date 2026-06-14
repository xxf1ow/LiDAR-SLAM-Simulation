#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <string>
#include "gicp_localization/prior_map.hpp"

using namespace gicp_localization;

static std::string writeTempPcd() {
  std::string path = std::string(std::tmpnam(nullptr)) + ".pcd";
  std::ofstream ofs(path);
  ofs << "# .PCD v0.7 - Point Cloud Data file format\n"
         "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         "WIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA ascii\n"
         "1 2 3\n4 5 6\n7 8 9\n";
  ofs.close();
  return path;
}

TEST(PriorMap, LoadsPcdPoints) {
  std::string path = writeTempPcd();
  auto pts = loadPriorMapPcd(path);
  ASSERT_EQ(pts.size(), 3u);
  EXPECT_NEAR(pts[0].x(), 1.0f, 1e-5);
  EXPECT_NEAR(pts[0].y(), 2.0f, 1e-5);
  EXPECT_NEAR(pts[0].z(), 3.0f, 1e-5);
  EXPECT_NEAR(pts[0].w(), 1.0f, 1e-5);
  std::remove(path.c_str());
}

TEST(PriorMap, ThrowsOnMissingFile) {
  EXPECT_THROW(loadPriorMapPcd("/nonexistent/path/to/map.pcd"), std::runtime_error);
}

// Gz gpu_lidar 对无回波射线返回 inf/nan,这些点会让 small_gicp 的 voxelgrid_sampling
// 体素坐标越界刷屏(floor(inf)→垃圾整数),并干扰 RViz 渲染。loadPriorMapPcd 必须丢弃它们。
static std::string writeTempPcdWithNonFinite() {
  std::string path = std::string(std::tmpnam(nullptr)) + ".pcd";
  std::ofstream ofs(path);
  ofs << "# .PCD v0.7 - Point Cloud Data file format\n"
         "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         "WIDTH 4\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 4\nDATA ascii\n"
         "1 2 3\nnan 0 0\n4 5 6\n0 nan 0\n";
  ofs.close();
  return path;
}

TEST(PriorMap, DropsNonFinitePoints) {
  std::string path = writeTempPcdWithNonFinite();
  auto pts = loadPriorMapPcd(path);
  ASSERT_EQ(pts.size(), 2u);  // 只保留两个有限点
  EXPECT_NEAR(pts[0].x(), 1.0f, 1e-5);
  EXPECT_NEAR(pts[1].x(), 4.0f, 1e-5);
  std::remove(path.c_str());
}
