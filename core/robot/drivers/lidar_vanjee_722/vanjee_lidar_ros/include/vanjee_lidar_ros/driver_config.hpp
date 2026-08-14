#pragma once

#include <cstdint>
#include <string>

#include <vanjee_driver/driver/driver_param.hpp>

namespace vanjee_lidar_ros {

struct DriverConfig {
  std::string lidar_type{};
  std::string host_address{};
  std::string lidar_address{};
  uint16_t host_msop_port{};
  uint16_t lidar_msop_port{};
  float start_angle{};
  float end_angle{};
  float min_distance{};
  float max_distance{};
  bool wait_for_difop{};
  bool config_from_file{};
  bool use_lidar_clock{};
  bool ts_first_point{};
  bool dense_points{};
  std::string lidar_frame{};
  std::string imu_frame{};
  std::string point_cloud_topic{};
  std::string imu_topic{};
};

bool parse_lidar_type(
    const std::string &name, vanjee::lidar::LidarType &type);
bool make_driver_param(
    const DriverConfig &config,
    vanjee::lidar::WJDriverParam &param,
    std::string &error);
bool configure_calibration_paths(
    const DriverConfig &config,
    const std::string &home_directory,
    vanjee::lidar::WJDriverParam &param,
    std::string &error);

}  // namespace vanjee_lidar_ros
