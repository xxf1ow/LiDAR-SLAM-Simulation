#include <gtest/gtest.h>
#include <cmath>
#include "gicp_localization/pose_math.hpp"

using namespace gicp_localization;

TEST(PoseMath, ParamTranslationOnly) {
  auto T = poseParamToIsometry(1, 2, 3, 0, 0, 0);
  EXPECT_NEAR(T.translation().x(), 1.0, 1e-9);
  EXPECT_NEAR(T.translation().y(), 2.0, 1e-9);
  EXPECT_NEAR(T.translation().z(), 3.0, 1e-9);
  EXPECT_TRUE(T.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(PoseMath, ParamYaw90MapsXtoY) {
  auto T = poseParamToIsometry(0, 0, 0, M_PI / 2, 0, 0);
  Eigen::Vector3d x = T.rotation() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(x.x(), 0.0, 1e-9);
  EXPECT_NEAR(x.y(), 1.0, 1e-9);
}

TEST(PoseMath, MapToOdomRoundTrip) {
  auto T_map_base = poseParamToIsometry(5, -2, 0, 0.3, 0, 0);
  auto T_odom_base = poseParamToIsometry(1, 1, 0, 0.1, 0, 0);
  auto T_map_odom = mapToOdomFromBase(T_map_base, T_odom_base);
  auto back = composeMapToBase(T_map_odom, T_odom_base);
  EXPECT_TRUE(back.isApprox(T_map_base, 1e-9));
}

TEST(PoseMath, MapToOdomIdentityOdom) {
  auto T_map_base = poseParamToIsometry(5, -2, 0, 0.3, 0, 0);
  auto T_map_odom = mapToOdomFromBase(T_map_base, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(T_map_odom.isApprox(T_map_base, 1e-9));
}

TEST(PoseMath, Fitness) {
  EXPECT_NEAR(computeFitness(8, 10), 0.8, 1e-9);
  EXPECT_NEAR(computeFitness(0, 0), 0.0, 1e-9);
}

TEST(PoseMath, CorrectionDelta) {
  auto a = Eigen::Isometry3d::Identity();
  auto b = poseParamToIsometry(0.3, 0, 0, 0, 0, 0);
  auto d = correctionDelta(a, b);
  EXPECT_NEAR(d.translation, 0.3, 1e-9);
  EXPECT_NEAR(d.rotation, 0.0, 1e-9);
}

TEST(PoseMath, AcceptGate) {
  EXPECT_TRUE(accept(0.85, 0.8, true));
  EXPECT_FALSE(accept(0.70, 0.8, true));
  EXPECT_FALSE(accept(0.90, 0.8, false));
}
