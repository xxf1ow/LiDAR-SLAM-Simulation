#ifndef GICP_LOCALIZATION_LOCALIZATION_SNAPSHOT_HPP_
#define GICP_LOCALIZATION_LOCALIZATION_SNAPSHOT_HPP_

#include <string>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace gicp_localization {

tf2_msgs::msg::TFMessage make_localization_snapshot(
    const Eigen::Isometry3d &map_to_odom,
    const Eigen::Isometry3d &odom_to_base,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &map_frame,
    const std::string &odom_frame,
    const std::string &base_frame);

}  // namespace gicp_localization

#endif  // GICP_LOCALIZATION_LOCALIZATION_SNAPSHOT_HPP_
