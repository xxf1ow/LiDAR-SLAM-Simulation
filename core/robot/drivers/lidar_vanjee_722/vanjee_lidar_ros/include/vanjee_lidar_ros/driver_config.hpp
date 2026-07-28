#pragma once

#include <cstdint>
#include <string>

#include <vanjee_driver/driver/driver_param.hpp>

namespace vanjee_lidar_ros {

struct DriverConfig {
  std::string lidar_type{"vanjee_722"};
  std::string host_address{"192.168.2.88"};
  std::string lidar_address{"192.168.2.86"};
  uint16_t host_msop_port{3001};
  uint16_t lidar_msop_port{3333};
  float start_angle{0.0F};
  float end_angle{360.0F};
  float min_distance{0.05F};
  float max_distance{70.0F};
  bool wait_for_difop{true};
  bool config_from_file{false};
  bool use_lidar_clock{false};
  bool ts_first_point{true};
  bool dense_points{false};
  std::string lidar_frame{"velodyne"};
  std::string imu_frame{"imu_link"};
  std::string point_cloud_topic{"/points_raw"};
  std::string imu_topic{"/imu/data"};
};

bool parse_lidar_type(
    const std::string &name, vanjee::lidar::LidarType &type);
bool make_driver_param(
    const DriverConfig &config,
    vanjee::lidar::WJDriverParam &param,
    std::string &error);

}  // namespace vanjee_lidar_ros
