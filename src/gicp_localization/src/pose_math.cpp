#include "gicp_localization/pose_math.hpp"

namespace gicp_localization {

Eigen::Isometry3d poseParamToIsometry(double x, double y, double z,
                                      double yaw, double pitch, double roll) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  T.linear() = R;
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

Eigen::Isometry3d mapToOdomFromBase(const Eigen::Isometry3d& T_map_base,
                                    const Eigen::Isometry3d& T_odom_base) {
  return T_map_base * T_odom_base.inverse();
}

Eigen::Isometry3d composeMapToBase(const Eigen::Isometry3d& T_map_odom,
                                   const Eigen::Isometry3d& T_odom_base) {
  return T_map_odom * T_odom_base;
}

double computeFitness(std::size_t num_inliers, std::size_t num_source) {
  if (num_source == 0) return 0.0;
  return static_cast<double>(num_inliers) / static_cast<double>(num_source);
}

CorrectionDelta correctionDelta(const Eigen::Isometry3d& prev,
                                const Eigen::Isometry3d& cur) {
  Eigen::Isometry3d rel = prev.inverse() * cur;
  CorrectionDelta d;
  d.translation = rel.translation().norm();
  d.rotation = Eigen::AngleAxisd(rel.rotation()).angle();
  return d;
}

bool accept(double fitness, double threshold, bool converged) {
  return converged && fitness >= threshold;
}

}  // namespace gicp_localization
