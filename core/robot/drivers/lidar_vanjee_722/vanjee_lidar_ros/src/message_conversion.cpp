#include "vanjee_lidar_ros/message_conversion.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

namespace vanjee_lidar_ros {
namespace {

sensor_msgs::msg::PointField field(
    const char *name, uint32_t offset, uint8_t datatype) {
  sensor_msgs::msg::PointField value;
  value.name = name;
  value.offset = offset;
  value.datatype = datatype;
  value.count = 1;
  return value;
}

template <typename T>
void write_at(std::vector<uint8_t> &data, std::size_t offset, const T &value) {
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

}  // namespace

bool seconds_to_stamp(
    double seconds, builtin_interfaces::msg::Time &stamp, std::string &error) {
  if (!std::isfinite(seconds) || seconds < 0.0) {
    error = "timestamp must be finite and non-negative";
    return false;
  }
  double integral = 0.0;
  const double fractional = std::modf(seconds, &integral);
  if (integral > std::numeric_limits<int32_t>::max()) {
    error = "timestamp seconds exceed ROS Time range";
    return false;
  }
  int64_t sec = static_cast<int64_t>(integral);
  int64_t nanosec = std::llround(fractional * 1000000000.0);
  if (nanosec == 1000000000LL) {
    ++sec;
    nanosec = 0;
  }
  if (sec > std::numeric_limits<int32_t>::max()) {
    error = "timestamp seconds exceed ROS Time range";
    return false;
  }
  stamp.sec = static_cast<int32_t>(sec);
  stamp.nanosec = static_cast<uint32_t>(nanosec);
  error.clear();
  return true;
}

bool to_point_cloud2(
    const VendorPointCloud &input,
    const std::string &frame_id,
    sensor_msgs::msg::PointCloud2 &output,
    std::string &error) {
  if (frame_id.empty()) {
    error = "point cloud frame_id must be non-empty";
    return false;
  }
  constexpr uint32_t point_step = 24;
  const uint64_t row_step =
      static_cast<uint64_t>(input.width) * point_step;
  if (row_step > std::numeric_limits<uint32_t>::max()) {
    error = "point cloud row_step exceeds uint32 range";
    return false;
  }
  const uint64_t data_size = row_step * input.height;
  if (data_size > output.data.max_size()) {
    error = "point cloud data size exceeds platform range";
    return false;
  }
  const uint64_t expected =
      static_cast<uint64_t>(input.height) * static_cast<uint64_t>(input.width);
  if (expected != input.points.size()) {
    error = "point count does not match height * width";
    return false;
  }
  if (!seconds_to_stamp(input.timestamp, output.header.stamp, error)) {
    return false;
  }

  output.header.frame_id = frame_id;
  output.height = input.height;
  output.width = input.width;
  output.fields = {
      field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
      field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
      field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
      field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
      field("ring", 16, sensor_msgs::msg::PointField::UINT16),
      field("time", 20, sensor_msgs::msg::PointField::FLOAT32),
  };
  output.is_bigendian = false;
  output.point_step = point_step;
  output.row_step = static_cast<uint32_t>(row_step);
  output.is_dense = input.is_dense;
  output.data.assign(static_cast<std::size_t>(data_size), 0);

  for (std::size_t i = 0; i < input.points.size(); ++i) {
    const auto &point = input.points[i];
    const std::size_t base = i * output.point_step;
    const float time = static_cast<float>(point.timestamp);
    write_at(output.data, base + 0, point.x);
    write_at(output.data, base + 4, point.y);
    write_at(output.data, base + 8, point.z);
    write_at(output.data, base + 12, point.intensity);
    write_at(output.data, base + 16, point.ring);
    write_at(output.data, base + 20, time);
  }
  error.clear();
  return true;
}

bool to_imu(
    const vanjee::lidar::ImuPacket &input,
    const std::string &frame_id,
    sensor_msgs::msg::Imu &output,
    std::string &error) {
  if (frame_id.empty()) {
    error = "IMU frame_id must be non-empty";
    return false;
  }
  if (!seconds_to_stamp(input.timestamp, output.header.stamp, error)) {
    return false;
  }
  output.header.frame_id = frame_id;
  output.orientation.w = input.orientation[0];
  output.orientation.x = input.orientation[1];
  output.orientation.y = input.orientation[2];
  output.orientation.z = input.orientation[3];
  output.angular_velocity.x = input.angular_voc[0];
  output.angular_velocity.y = input.angular_voc[1];
  output.angular_velocity.z = input.angular_voc[2];
  output.linear_acceleration.x = input.linear_acce[0];
  output.linear_acceleration.y = input.linear_acce[1];
  output.linear_acceleration.z = input.linear_acce[2];
  output.orientation_covariance = input.orientation_covariance;
  output.angular_velocity_covariance = input.angular_voc_covariance;
  output.linear_acceleration_covariance = input.linear_acce_covariance;
  error.clear();
  return true;
}

}  // namespace vanjee_lidar_ros
