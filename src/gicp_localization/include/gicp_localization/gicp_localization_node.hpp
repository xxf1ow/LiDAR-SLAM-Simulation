#pragma once
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>

#include "gicp_localization/gicp_aligner.hpp"
#include "gicp_localization/diagnostics.hpp"

namespace gicp_localization {

class GicpLocalizationNode : public rclcpp::Node {
public:
  GicpLocalizationNode();

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void initialPoseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void gicpTimerCb();
  void tfTimerCb();

  // 参数
  std::string map_frame_, odom_frame_, base_frame_;
  double fitness_threshold_{0.8};
  int min_scan_points_{100};

  std::unique_ptr<GicpAligner> aligner_;
  std::unique_ptr<Diagnostics> diag_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr loc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;  // latched, 供 RViz 查看
  rclcpp::TimerBase::SharedPtr gicp_timer_, tf_timer_;
  rclcpp::CallbackGroup::SharedPtr slow_group_, fast_group_;

  // 共享状态（mtx_ 保护）
  std::mutex mtx_;
  Eigen::Isometry3d T_map_odom_{Eigen::Isometry3d::Identity()};
  std::optional<std::vector<Eigen::Vector4f>> latest_scan_;
  std::optional<Eigen::Isometry3d> latest_odom_;     // T_odom_base
  std::optional<Eigen::Isometry3d> pending_init_base_;  // /initialpose 给的 T_map_base
  std::size_t rejected_{0};
};

}  // namespace gicp_localization
