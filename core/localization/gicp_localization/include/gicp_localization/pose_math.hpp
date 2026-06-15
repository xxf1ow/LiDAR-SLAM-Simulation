#pragma once
#include <cstddef>
#include <Eigen/Geometry>

namespace gicp_localization {

struct CorrectionDelta {
  double translation;  // 米
  double rotation;     // 弧度
};

// 欧拉角约定 yaw(Z)·pitch(Y)·roll(X)
Eigen::Isometry3d poseParamToIsometry(double x, double y, double z,
                                      double yaw, double pitch, double roll);

// T_map_odom = T_map_base · T_odom_base^{-1}
Eigen::Isometry3d mapToOdomFromBase(const Eigen::Isometry3d& T_map_base,
                                    const Eigen::Isometry3d& T_odom_base);

// T_map_base = T_map_odom · T_odom_base
Eigen::Isometry3d composeMapToBase(const Eigen::Isometry3d& T_map_odom,
                                   const Eigen::Isometry3d& T_odom_base);

double computeFitness(std::size_t num_inliers, std::size_t num_source);

CorrectionDelta correctionDelta(const Eigen::Isometry3d& prev,
                                const Eigen::Isometry3d& cur);

// 仅按 fitness 判定(对齐 spec/参照实现); converged 仅作诊断, 不参与门限
bool accept(double fitness, double threshold);

}  // namespace gicp_localization
