#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lidar_target_tracking/scan_bridge.h"
#include "lidar_target_tracking/tracking_snapshot.h"
#include "reference/lidar_tracker.hpp"

namespace lidar_target_tracking {
namespace {

class TrackerNode final : public rclcpp::Node {
 public:
  TrackerNode();

 private:
  void on_target(const geometry_msgs::msg::PointStamped::SharedPtr message);
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  SharedState state_;
  LidarTracker tracker_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  uint16_t ring_;
  size_t columns_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

TrackerNode::TrackerNode()
    : Node("lidar_target_tracker"),
      tracker_(state_),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_),
      ring_(static_cast<uint16_t>(declare_parameter<int>("ring", 7))),
      columns_(static_cast<size_t>(declare_parameter<int>("columns", 1800))) {
  state_pub_ = create_publisher<std_msgs::msg::String>(
      "/tracking/state", rclcpp::QoS(10).durability_volatile());
  target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/tracking/target", 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr message) { on_target(message); });
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) { on_cloud(cloud); });
}

void TrackerNode::on_target(const geometry_msgs::msg::PointStamped::SharedPtr message) {
  if (message->header.frame_id != "base_footprint" ||
      !std::isfinite(message->point.x) || !std::isfinite(message->point.y)) {
    RCLCPP_ERROR(get_logger(), "tracking target requires finite x/y in base_footprint");
    return;
  }
  state_.setTarget(message->point.x, message->point.y);
  state_.active.store(true);
}

void TrackerNode::on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
        "base_footprint", cloud->header.frame_id, rclcpp::Time(cloud->header.stamp),
        rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException &error) {
    RCLCPP_ERROR(get_logger(), "tracking cloud transform unavailable: %s", error.what());
    return;
  }

  auto frame = bridge_scan(*cloud, transform, ring_, columns_);
  if (!frame) {
    RCLCPP_ERROR(get_logger(), "tracking input fields or transform invalid");
    return;
  }
  tracker_.processScan(std::make_shared<sensor_msgs::msg::LaserScan>(frame->scan));
  double target_x;
  double target_y;
  state_.getTarget(target_x, target_y);
  std_msgs::msg::String message;
  message.data = make_tracking_snapshot(*frame, state_.active.load(), target_x, target_y);
  state_pub_->publish(message);
}

}  // namespace
}  // namespace lidar_target_tracking

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_target_tracking::TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
