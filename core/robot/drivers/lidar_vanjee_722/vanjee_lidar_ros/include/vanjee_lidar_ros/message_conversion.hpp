#pragma once

#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vanjee_driver/msg/imu_packet.hpp>
#include <vanjee_driver/msg/point_cloud_msg.hpp>

namespace vanjee_lidar_ros {

using VendorPointCloud =
    vanjee::lidar::PointCloudT<vanjee::lidar::PointXYZIRT>;

bool seconds_to_stamp(
    double seconds, builtin_interfaces::msg::Time &stamp, std::string &error);
bool to_point_cloud2(
    const VendorPointCloud &input,
    const std::string &frame_id,
    sensor_msgs::msg::PointCloud2 &output,
    std::string &error);
bool to_imu(
    const vanjee::lidar::ImuPacket &input,
    const std::string &frame_id,
    sensor_msgs::msg::Imu &output,
    std::string &error);

}  // namespace vanjee_lidar_ros
