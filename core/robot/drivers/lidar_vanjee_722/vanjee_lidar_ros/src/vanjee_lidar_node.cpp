#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vanjee_driver/api/lidar_driver.hpp>

#include "vanjee_lidar_ros/driver_config.hpp"
#include "vanjee_lidar_ros/message_conversion.hpp"

namespace vanjee_lidar_ros {

class VanjeeLidarNode final : public rclcpp::Node {
public:
  VanjeeLidarNode() : Node("vanjee_lidar") {
    read_parameters();

    std::string error;
    if (!make_driver_param(config_, driver_param_, error)) {
      throw std::runtime_error(error);
    }

    point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.point_cloud_topic, rclcpp::SensorDataQoS());
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        config_.imu_topic, rclcpp::SensorDataQoS());

    driver_.regPointCloudCallback(
        [] { return std::make_shared<VendorPointCloud>(); },
        [this](std::shared_ptr<VendorPointCloud> cloud) {
          publish_cloud(std::move(cloud));
        });
    driver_.regImuPacketCallback(
        [] { return std::make_shared<vanjee::lidar::ImuPacket>(); },
        [this](std::shared_ptr<vanjee::lidar::ImuPacket> imu) {
          publish_imu(std::move(imu));
        });
    driver_.regExceptionCallback(
        [this](const vanjee::lidar::Error &error_value) {
          RCLCPP_WARN(get_logger(), "%s", error_value.toString().c_str());
        });

    if (!driver_.init(driver_param_)) {
      throw std::runtime_error("vanjee_driver init failed");
    }
    if (!driver_.start()) {
      driver_.stop();
      throw std::runtime_error("vanjee_driver start failed");
    }
    started_ = true;
    RCLCPP_INFO(get_logger(), "Vanjee %s: %s -> %s, topics %s and %s",
                config_.lidar_type.c_str(), config_.lidar_address.c_str(),
                config_.host_address.c_str(), config_.point_cloud_topic.c_str(),
                config_.imu_topic.c_str());
  }

  ~VanjeeLidarNode() override {
    stopping_.store(true);
    if (started_) {
      driver_.stop();
      started_ = false;
    }
  }

private:
  void read_parameters() {
    config_.lidar_type =
        declare_parameter<std::string>("lidar_type", config_.lidar_type);
    config_.host_address =
        declare_parameter<std::string>("host_address", config_.host_address);
    config_.lidar_address =
        declare_parameter<std::string>("lidar_address", config_.lidar_address);
    const int64_t host_msop_port =
        declare_parameter<int64_t>("host_msop_port", config_.host_msop_port);
    if (host_msop_port < 1 || host_msop_port > 65535) {
      throw std::runtime_error("host_msop_port must be in 1..65535");
    }
    config_.host_msop_port = static_cast<uint16_t>(host_msop_port);
    const int64_t lidar_msop_port =
        declare_parameter<int64_t>("lidar_msop_port", config_.lidar_msop_port);
    if (lidar_msop_port < 1 || lidar_msop_port > 65535) {
      throw std::runtime_error("lidar_msop_port must be in 1..65535");
    }
    config_.lidar_msop_port = static_cast<uint16_t>(lidar_msop_port);
    config_.start_angle = static_cast<float>(
        declare_parameter<double>("start_angle", config_.start_angle));
    config_.end_angle = static_cast<float>(
        declare_parameter<double>("end_angle", config_.end_angle));
    config_.min_distance = static_cast<float>(
        declare_parameter<double>("min_distance", config_.min_distance));
    config_.max_distance = static_cast<float>(
        declare_parameter<double>("max_distance", config_.max_distance));
    config_.wait_for_difop =
        declare_parameter<bool>("wait_for_difop", config_.wait_for_difop);
    config_.config_from_file =
        declare_parameter<bool>("config_from_file", config_.config_from_file);
    config_.use_lidar_clock =
        declare_parameter<bool>("use_lidar_clock", config_.use_lidar_clock);
    config_.ts_first_point =
        declare_parameter<bool>("ts_first_point", config_.ts_first_point);
    config_.dense_points =
        declare_parameter<bool>("dense_points", config_.dense_points);
    config_.lidar_frame =
        declare_parameter<std::string>("lidar_frame", config_.lidar_frame);
    config_.imu_frame =
        declare_parameter<std::string>("imu_frame", config_.imu_frame);
    config_.point_cloud_topic = declare_parameter<std::string>(
        "point_cloud_topic", config_.point_cloud_topic);
    config_.imu_topic =
        declare_parameter<std::string>("imu_topic", config_.imu_topic);
  }

  void publish_cloud(std::shared_ptr<VendorPointCloud> cloud) {
    if (stopping_.load() || !cloud) {
      return;
    }
    sensor_msgs::msg::PointCloud2 output;
    std::string error;
    if (!to_point_cloud2(*cloud, config_.lidar_frame, output, error)) {
      RCLCPP_ERROR(get_logger(), "Dropping point cloud: %s", error.c_str());
      return;
    }
    point_cloud_publisher_->publish(std::move(output));
  }

  void publish_imu(std::shared_ptr<vanjee::lidar::ImuPacket> imu) {
    if (stopping_.load() || !imu) {
      return;
    }
    sensor_msgs::msg::Imu output;
    std::string error;
    if (!to_imu(*imu, config_.imu_frame, output, error)) {
      RCLCPP_ERROR(get_logger(), "Dropping IMU sample: %s", error.c_str());
      return;
    }
    imu_publisher_->publish(std::move(output));
  }

  DriverConfig config_;
  vanjee::lidar::WJDriverParam driver_param_;
  vanjee::lidar::LidarDriver<VendorPointCloud> driver_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  std::atomic<bool> stopping_{false};
  bool started_{false};
};

} // namespace vanjee_lidar_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<vanjee_lidar_ros::VanjeeLidarNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("vanjee_lidar"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
