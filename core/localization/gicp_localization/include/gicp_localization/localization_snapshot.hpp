#ifndef XX_LOCALIZATION_SNAPSHOT_H_
#define XX_LOCALIZATION_SNAPSHOT_H_

#include <string>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace gicp_localization {

/**
 * Returns exactly two transforms with `stamp`: `map_frame` to `odom_frame`
 * from `map_to_odom`, followed by `map_frame` to `base_frame` composed from
 * `map_to_odom * odom_to_base`.
 */
tf2_msgs::msg::TFMessage make_localization_snapshot(
    const Eigen::Isometry3d &map_to_odom,
    const Eigen::Isometry3d &odom_to_base,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &map_frame,
    const std::string &odom_frame,
    const std::string &base_frame);

}  // namespace gicp_localization

#endif  // XX_LOCALIZATION_SNAPSHOT_H_
