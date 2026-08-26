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
        pts.emplace_back(i * 0.3f, j * 0.3f, k * 0.3f, 1.0f);
  return pts;
}

GicpParams syntheticParams() {
  return GicpParams{
      0.23,
      0.07,
      0.83,
      11,
      2,
      13,
  };
}

TEST(GicpAligner, RecoversKnownTranslation) {
  GicpParams p = syntheticParams();

  auto target = makeGrid();
  // 平移量必须与合成栅格间距(0.3)不可公度，否则规则点阵平移整数个周期会产生
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
  EXPECT_NEAR(out.T_target_source.translation().x(), -shift, 0.01);
  EXPECT_GT(out.fitness, 0.9);
  EXPECT_GT(out.num_inliers, 0u);
}
