#pragma once
#include <cstddef>
#include <memory>
#include <vector>
#include <Eigen/Geometry>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>

namespace gicp_localization {

struct GicpParams {
  double map_voxel_size = 0.4;
  double scan_voxel_size = 0.1;
  double max_corr_dist = 1.0;
  int num_neighbors = 20;
  int num_threads = 4;
  int max_iterations = 20;
};

struct AlignOutcome {
  Eigen::Isometry3d T_map_odom = Eigen::Isometry3d::Identity();
  double fitness = 0.0;
  double mean_residual = 0.0;
  std::size_t num_inliers = 0;
  std::size_t num_source = 0;
  int iterations = 0;
  bool converged = false;
};

class GicpAligner {
public:
  explicit GicpAligner(const GicpParams& params);
  void setMap(const std::vector<Eigen::Vector4f>& map_points);
  bool hasMap() const { return static_cast<bool>(target_); }
  AlignOutcome align(const std::vector<Eigen::Vector4f>& scan_points,
                     const Eigen::Isometry3d& seed) const;

private:
  GicpParams params_;
  small_gicp::PointCloud::Ptr target_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree_;
};

}  // namespace gicp_localization
