#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "lidar_target_tracking/scan_bridge.h"

namespace {

sensor_msgs::msg::PointCloud2 make_cloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "velodyne";
  cloud.height = 1;
  cloud.width = 3;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16);
  modifier.resize(3);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint16_t> ring(cloud, "ring");
  for (size_t i = 0; i < 3; ++i, ++x, ++y, ++z, ++ring) {
    *x = i == 2 ? 3.0F : 0.0F;
    *y = i == 1 ? 0.0F : 1.0F;
    *z = 0.0F;
    *ring = i == 2 ? 25 : 26;
  }
  return cloud;
}

geometry_msgs::msg::TransformStamped make_transform() {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_footprint";
  transform.child_frame_id = "velodyne";
  transform.transform.translation.x = 2.0;
  transform.transform.rotation.z = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);
  return transform;
}

TEST(ScanBridge, SelectsRawRingAndKeepsNearestTransformedRange) {
  const auto result = lidar_target_tracking::bridge_scan(make_cloud(), make_transform(), 26, 1200);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->scan.header.frame_id, "base_footprint");
  EXPECT_EQ(result->scan.ranges.size(), 1200U);
  EXPECT_FLOAT_EQ(result->scan.angle_min, -3.14159265358979323846F);
  EXPECT_FLOAT_EQ(result->scan.ranges.at(0), 1.0F);
  EXPECT_TRUE(std::isinf(result->scan.ranges.at(1)));
  ASSERT_EQ(result->points.size(), 1U);
  EXPECT_NEAR(result->points.at(0).at(0), 1.0, 0.01);
  EXPECT_NEAR(result->points.at(0).at(1), 0.0, 0.01);
}

TEST(ScanBridge, RejectsMalformedRingType) {
  auto cloud = make_cloud();
  cloud.fields.at(3).datatype = sensor_msgs::msg::PointField::FLOAT32;
  EXPECT_FALSE(lidar_target_tracking::bridge_scan(cloud, make_transform(), 26, 1200).has_value());
}

TEST(ScanBridge, EmptySelectedRingIsValid) {
  const auto result = lidar_target_tracking::bridge_scan(make_cloud(), make_transform(), 7, 1200);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->points.empty());
  for (float range : result->scan.ranges) EXPECT_TRUE(std::isinf(range));
}

}  // namespace
