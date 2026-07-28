#include "vanjee_lidar_ros/driver_config.hpp"

#include <array>
#include <cctype>
#include <string>
#include <utility>

namespace vanjee_lidar_ros {
namespace {

using Entry = std::pair<const char *, vanjee::lidar::LidarType>;

constexpr std::array<Entry, 17> kModels{{
    {"vanjee_716mini", vanjee::lidar::LidarType::vanjee_716mini},
    {"vanjee_718h", vanjee::lidar::LidarType::vanjee_718h},
    {"vanjee_719", vanjee::lidar::LidarType::vanjee_719},
    {"vanjee_719c", vanjee::lidar::LidarType::vanjee_719c},
    {"vanjee_719e", vanjee::lidar::LidarType::vanjee_719e},
    {"vanjee_720", vanjee::lidar::LidarType::vanjee_720_16},
    {"vanjee_720_16", vanjee::lidar::LidarType::vanjee_720_16},
    {"vanjee_720_32", vanjee::lidar::LidarType::vanjee_720_32},
    {"vanjee_721", vanjee::lidar::LidarType::vanjee_721},
    {"vanjee_722", vanjee::lidar::LidarType::vanjee_722},
    {"vanjee_722f", vanjee::lidar::LidarType::vanjee_722f},
    {"vanjee_722h", vanjee::lidar::LidarType::vanjee_722h},
    {"vanjee_722z", vanjee::lidar::LidarType::vanjee_722z},
    {"vanjee_733", vanjee::lidar::LidarType::vanjee_733},
    {"vanjee_738", vanjee::lidar::LidarType::vanjee_738},
    {"vanjee_750", vanjee::lidar::LidarType::vanjee_750},
    {"vanjee_760", vanjee::lidar::LidarType::vanjee_760},
}};

bool valid_ipv4(const std::string &value) {
  std::size_t start = 0;
  for (int part = 0; part < 4; ++part) {
    const std::size_t end =
        part == 3 ? value.size() : value.find('.', start);
    if (end == std::string::npos || end == start || end - start > 3) {
      return false;
    }
    int octet = 0;
    for (std::size_t i = start; i < end; ++i) {
      const unsigned char c = static_cast<unsigned char>(value[i]);
      if (!std::isdigit(c)) {
        return false;
      }
      octet = octet * 10 + (value[i] - '0');
    }
    if (octet > 255) {
      return false;
    }
    start = end + 1;
  }
  return start == value.size() + 1;
}

}  // namespace

bool parse_lidar_type(
    const std::string &name, vanjee::lidar::LidarType &type) {
  for (const auto &entry : kModels) {
    if (name == entry.first) {
      type = entry.second;
      return true;
    }
  }
  return false;
}

bool make_driver_param(
    const DriverConfig &config,
    vanjee::lidar::WJDriverParam &param,
    std::string &error) {
  vanjee::lidar::LidarType type{};
  if (!parse_lidar_type(config.lidar_type, type)) {
    error = "unsupported lidar_type: " + config.lidar_type;
    return false;
  }
  if (!valid_ipv4(config.host_address)) {
    error = "invalid host_address: " + config.host_address;
    return false;
  }
  if (!valid_ipv4(config.lidar_address)) {
    error = "invalid lidar_address: " + config.lidar_address;
    return false;
  }
  if (config.host_msop_port == 0 || config.lidar_msop_port == 0) {
    error = "MSOP ports must be non-zero";
    return false;
  }
  if (config.min_distance < 0.0F) {
    error = "min_distance must be non-negative";
    return false;
  }
  if (config.max_distance <= config.min_distance) {
    error = "max_distance must be greater than min_distance";
    return false;
  }
  if (config.lidar_frame.empty() || config.imu_frame.empty()) {
    error = "frame names must be non-empty";
    return false;
  }
  if (config.point_cloud_topic.empty() || config.imu_topic.empty()) {
    error = "topic names must be non-empty";
    return false;
  }

  param = vanjee::lidar::WJDriverParam{};
  param.lidar_type = type;
  param.input_type = vanjee::lidar::InputType::ONLINE_LIDAR;
  param.input_param.connect_type = 1;
  param.input_param.host_address = config.host_address;
  param.input_param.lidar_address = config.lidar_address;
  param.input_param.group_address = "0.0.0.0";
  param.input_param.host_msop_port = config.host_msop_port;
  param.input_param.lidar_msop_port = config.lidar_msop_port;

  param.decoder_param.start_angle = config.start_angle;
  param.decoder_param.end_angle = config.end_angle;
  param.decoder_param.min_distance = config.min_distance;
  param.decoder_param.max_distance = config.max_distance;
  param.decoder_param.wait_for_difop = config.wait_for_difop;
  param.decoder_param.config_from_file = config.config_from_file;
  param.decoder_param.use_lidar_clock = config.use_lidar_clock;
  param.decoder_param.ts_first_point = config.ts_first_point;
  param.decoder_param.dense_points = config.dense_points;
  param.decoder_param.use_offset_timestamp = true;
  param.decoder_param.publish_mode = 0;
  param.decoder_param.point_cloud_enable = true;
  param.decoder_param.imu_enable = 1;
  param.decoder_param.imu_orientation_enable = true;
  error.clear();
  return true;
}

}  // namespace vanjee_lidar_ros
