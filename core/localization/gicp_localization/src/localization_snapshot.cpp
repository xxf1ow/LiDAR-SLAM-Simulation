#include "gicp_localization/localization_snapshot.hpp"

#include <utility>

#include <tf2_eigen/tf2_eigen.hpp>

#include "gicp_localization/pose_math.hpp"

namespace gicp_localization {

tf2_msgs::msg::TFMessage make_localization_snapshot(
    const Eigen::Isometry3d &map_to_odom,
    const Eigen::Isometry3d &odom_to_base,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &map_frame,
    const std::string &odom_frame,
    const std::string &base_frame)
{
  auto map_to_odom_message = tf2::eigenToTransform(map_to_odom);
  map_to_odom_message.header.stamp = stamp;
  map_to_odom_message.header.frame_id = map_frame;
  map_to_odom_message.child_frame_id = odom_frame;

  auto map_to_base_message = tf2::eigenToTransform(
      composeMapToBase(map_to_odom, odom_to_base));
  map_to_base_message.header.stamp = stamp;
  map_to_base_message.header.frame_id = map_frame;
  map_to_base_message.child_frame_id = base_frame;

  tf2_msgs::msg::TFMessage snapshot;
  snapshot.transforms.reserve(2);
  snapshot.transforms.push_back(std::move(map_to_odom_message));
  snapshot.transforms.push_back(std::move(map_to_base_message));
  return snapshot;
}

}  // namespace gicp_localization
