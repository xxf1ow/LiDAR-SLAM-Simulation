#include "gicp_localization/gicp_aligner.hpp"
#include "gicp_localization/pose_math.hpp"

#include <tuple>
#include <small_gicp/registration/registration_helper.hpp>

namespace gicp_localization {

GicpAligner::GicpAligner(const GicpParams& params) : params_(params) {}

void GicpAligner::setMap(const std::vector<Eigen::Vector4f>& map_points) {
  std::tie(target_, target_tree_) = small_gicp::preprocess_points(
      map_points, params_.map_voxel_size, params_.num_neighbors, params_.num_threads);
}

AlignOutcome GicpAligner::align(const std::vector<Eigen::Vector4f>& scan_points,
                                const Eigen::Isometry3d& seed) const {
  auto preprocessed = small_gicp::preprocess_points(
      scan_points, params_.scan_voxel_size, params_.num_neighbors, params_.num_threads);
  const auto& source = preprocessed.first;

  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.max_correspondence_distance = params_.max_corr_dist;
  setting.num_threads = params_.num_threads;
  setting.max_iterations = params_.max_iterations;

  auto result = small_gicp::align(*target_, *source, *target_tree_, seed, setting);

  AlignOutcome out;
  out.T_map_odom = result.T_target_source;
  out.num_inliers = result.num_inliers;
  out.num_source = source->size();
  out.fitness = computeFitness(result.num_inliers, source->size());
  out.mean_residual = result.num_inliers > 0
      ? result.error / static_cast<double>(result.num_inliers) : 0.0;
  out.iterations = static_cast<int>(result.iterations);
  out.converged = result.converged;
  return out;
}

}  // namespace gicp_localization
