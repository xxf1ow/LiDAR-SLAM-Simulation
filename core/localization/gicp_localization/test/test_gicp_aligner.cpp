#include <gtest/gtest.h>
#include <vector>
#include <Eigen/Geometry>
#include "gicp_localization/gicp_aligner.hpp"

using namespace gicp_localization;

static std::vector<Eigen::Vector4f> makeGrid() {
  std::vector<Eigen::Vector4f> pts;
  for (int i = 0; i < 10; ++i)
    for (int j = 0; j < 10; ++j)
      for (int k = 0; k < 10; ++k)
        pts.emplace_back(i * 0.1f, j * 0.1f, k * 0.1f, 1.0f);
  return pts;
}

TEST(GicpAligner, RecoversKnownTranslation) {
  GicpParams p;
  p.map_voxel_size = 0.05;
  p.scan_voxel_size = 0.05;
  p.max_corr_dist = 1.0;
  p.num_neighbors = 10;
  p.num_threads = 1;
  p.max_iterations = 50;

  auto target = makeGrid();
  // 平移量必须与栅格间距(0.1)不可公度，否则规则点阵平移整数个周期会产生
  // 混叠：能量在"位移 0"与"位移 -shift"两处出现对称极小，GICP 从单位阵初值
  // 收敛到中点而非真解。取 0.03 使真解唯一、可干净恢复。
  const float shift = 0.03f;
  std::vector<Eigen::Vector4f> source;
  for (auto pt : target) { pt.x() += shift; source.push_back(pt); }

  GicpAligner aligner(p);
  aligner.setMap(target);
  ASSERT_TRUE(aligner.hasMap());

  // source = target + shift·x，配到 target 需 -shift·x
  auto out = aligner.align(source, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(out.converged);
  EXPECT_NEAR(out.T_map_odom.translation().x(), -shift, 0.01);
  EXPECT_GT(out.fitness, 0.9);
  EXPECT_GT(out.num_inliers, 0u);
}
