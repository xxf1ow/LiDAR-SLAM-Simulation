#include "lidar_target_tracking/scan_bridge.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lidar_target_tracking {
namespace {

constexpr double pi = 3.14159265358979323846;

bool has_field(const sensor_msgs::msg::PointCloud2 &cloud, const char *name,
               uint8_t datatype, size_t size) {
  for (const auto &field : cloud.fields) {
    if (field.name == name) {
      return field.datatype == datatype && field.count == 1 &&
             field.offset <= cloud.point_step &&
             size <= cloud.point_step - field.offset;
    }
  }
  return false;
}

bool valid_layout(const sensor_msgs::msg::PointCloud2 &cloud) {
  if (!has_field(cloud, "x", sensor_msgs::msg::PointField::FLOAT32, sizeof(float)) ||
      !has_field(cloud, "y", sensor_msgs::msg::PointField::FLOAT32, sizeof(float)) ||
      !has_field(cloud, "z", sensor_msgs::msg::PointField::FLOAT32, sizeof(float)) ||
      !has_field(cloud, "ring", sensor_msgs::msg::PointField::UINT16, sizeof(uint16_t)) ||
      cloud.point_step == 0 || cloud.width > std::numeric_limits<size_t>::max() / cloud.point_step ||
      cloud.row_step != cloud.width * cloud.point_step ||
      (cloud.row_step != 0 &&
       cloud.height > std::numeric_limits<size_t>::max() / cloud.row_step) ||
      cloud.data.size() != static_cast<size_t>(cloud.row_step) * cloud.height) {
    return false;
  }
  return !cloud.is_bigendian;
}

}  // namespace

std::optional<ScanFrame> bridge_scan(const sensor_msgs::msg::PointCloud2 &cloud,
                                     const geometry_msgs::msg::TransformStamped &transform,
                                     uint16_t ring, size_t columns) {
  const auto &translation = transform.transform.translation;
  const auto &rotation = transform.transform.rotation;
  const double norm_squared = rotation.x * rotation.x + rotation.y * rotation.y +
                              rotation.z * rotation.z + rotation.w * rotation.w;
  if (columns == 0 || columns > static_cast<size_t>(std::numeric_limits<long long>::max() / 2) ||
      cloud.header.frame_id != "velodyne" ||
      transform.header.frame_id != "base_footprint" ||
      transform.child_frame_id != cloud.header.frame_id || !valid_layout(cloud) ||
      !std::isfinite(translation.x) || !std::isfinite(translation.y) ||
      !std::isfinite(translation.z) || !std::isfinite(norm_squared) || norm_squared == 0.0) {
    return std::nullopt;
  }

  const double qx = rotation.x / std::sqrt(norm_squared);
  const double qy = rotation.y / std::sqrt(norm_squared);
  const double qz = rotation.z / std::sqrt(norm_squared);
  const double qw = rotation.w / std::sqrt(norm_squared);
  const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double r01 = 2.0 * (qx * qy - qz * qw);
  const double r02 = 2.0 * (qx * qz + qy * qw);
  const double r10 = 2.0 * (qx * qy + qz * qw);
  const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
  const double r12 = 2.0 * (qy * qz - qx * qw);

  ScanFrame frame;
  auto &scan = frame.scan;
  scan.header.stamp = cloud.header.stamp;
  scan.header.frame_id = "base_footprint";
  scan.angle_min = static_cast<float>(-pi);
  scan.angle_increment = static_cast<float>(2.0 * pi / columns);
  scan.angle_max = static_cast<float>(-pi + (columns - 1) * (2.0 * pi / columns));
  scan.range_min = 0.0F;
  scan.range_max = std::numeric_limits<float>::max();
  scan.scan_time = 0.0F;
  scan.time_increment = 0.0F;
  scan.ranges.assign(columns, std::numeric_limits<float>::infinity());

  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<uint16_t> point_ring(cloud, "ring");
  const size_t count = static_cast<size_t>(cloud.width) * cloud.height;
  const double increment = 2.0 * pi / columns;
  for (size_t i = 0; i < count; ++i, ++x, ++y, ++z, ++point_ring) {
    if (*point_ring != ring || !std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
      continue;
    }
    const double bx = translation.x + r00 * *x + r01 * *y + r02 * *z;
    const double by = translation.y + r10 * *x + r11 * *y + r12 * *z;
    const double range = std::hypot(bx, by);
    if (!std::isfinite(range) || range > std::numeric_limits<float>::max()) {
      continue;
    }
    const double angle = std::atan2(-by, -bx);
    const size_t bin = static_cast<size_t>(std::llround((angle + pi) / increment)) % columns;
    scan.ranges[bin] = std::min(scan.ranges[bin], static_cast<float>(range));
  }

  for (size_t bin = 0; bin < columns; ++bin) {
    if (std::isfinite(scan.ranges[bin])) {
      const double angle = -pi + bin * increment;
      frame.points.push_back({-scan.ranges[bin] * std::cos(angle),
                              -scan.ranges[bin] * std::sin(angle)});
    }
  }
  return frame;
}

}  // namespace lidar_target_tracking
