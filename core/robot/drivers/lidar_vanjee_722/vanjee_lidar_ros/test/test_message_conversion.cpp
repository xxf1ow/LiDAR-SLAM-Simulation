#include <cstring>
#include <limits>

#include <gtest/gtest.h>

#include "vanjee_lidar_ros/message_conversion.hpp"

namespace vlr = vanjee_lidar_ros;

template <typename T>
T read_at(const sensor_msgs::msg::PointCloud2 &msg, std::size_t offset) {
  T value{};
  std::memcpy(&value, msg.data.data() + offset, sizeof(T));
  return value;
}

TEST(MessageConversion, CarriesRoundedNanoseconds) {
  builtin_interfaces::msg::Time stamp;
  std::string error;
  ASSERT_TRUE(vlr::seconds_to_stamp(10.9999999996, stamp, error)) << error;
  EXPECT_EQ(stamp.sec, 11);
  EXPECT_EQ(stamp.nanosec, 0U);
  EXPECT_FALSE(vlr::seconds_to_stamp(
      std::numeric_limits<double>::quiet_NaN(), stamp, error));
}

TEST(MessageConversion, BuildsAlignedPointCloudContract) {
  vlr::VendorPointCloud cloud;
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_dense = true;
  cloud.timestamp = 100.25;
  cloud.points.resize(2);
  cloud.points[0].x = 1.0F;
  cloud.points[0].y = 2.0F;
  cloud.points[0].z = 3.0F;
  cloud.points[0].intensity = 4.0F;
  cloud.points[0].ring = 0;
  cloud.points[0].timestamp = 0.0;
  cloud.points[1].x = 5.0F;
  cloud.points[1].ring = 31;
  cloud.points[1].timestamp = 0.099F;

  sensor_msgs::msg::PointCloud2 output;
  std::string error;
  ASSERT_TRUE(vlr::to_point_cloud2(cloud, "velodyne", output, error)) << error;
  ASSERT_EQ(output.fields.size(), 6U);
  EXPECT_EQ(output.fields[4].name, "ring");
  EXPECT_EQ(output.fields[4].offset, 16U);
  EXPECT_EQ(output.fields[4].datatype, sensor_msgs::msg::PointField::UINT16);
  EXPECT_EQ(output.fields[5].name, "time");
  EXPECT_EQ(output.fields[5].offset, 20U);
  EXPECT_EQ(output.fields[5].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(output.point_step, 24U);
  EXPECT_EQ(output.row_step, 48U);
  EXPECT_EQ(output.header.frame_id, "velodyne");
  EXPECT_EQ(read_at<float>(output, 0), 1.0F);
  EXPECT_EQ(read_at<uint16_t>(output, 24 + 16), 31U);
  EXPECT_EQ(output.data[18], 0U);
  EXPECT_EQ(output.data[19], 0U);
  EXPECT_FLOAT_EQ(read_at<float>(output, 24 + 20), 0.099F);
}

TEST(MessageConversion, RejectsInconsistentCloudDimensions) {
  vlr::VendorPointCloud cloud;
  cloud.height = 32;
  cloud.width = 100;
  cloud.timestamp = 1.0;
  cloud.points.resize(10);
  sensor_msgs::msg::PointCloud2 output;
  std::string error;
  EXPECT_FALSE(vlr::to_point_cloud2(cloud, "velodyne", output, error));
  EXPECT_EQ(error, "point count does not match height * width");
}

TEST(MessageConversion, MapsVendorImuOrderAndCovariances) {
  vanjee::lidar::ImuPacket imu;
  imu.timestamp = 12.5;
  imu.orientation = {1.0, 0.1, 0.2, 0.3};
  imu.angular_voc = {0.4, 0.5, 0.6};
  imu.linear_acce = {1.0, 2.0, 9.8};
  imu.orientation_covariance.fill(0.0);
  imu.angular_voc_covariance.fill(0.0);
  imu.linear_acce_covariance.fill(0.0);
  imu.angular_voc_covariance[0] = 0.7;

  sensor_msgs::msg::Imu output;
  std::string error;
  ASSERT_TRUE(vlr::to_imu(imu, "imu_link", output, error)) << error;
  EXPECT_EQ(output.header.frame_id, "imu_link");
  EXPECT_DOUBLE_EQ(output.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(output.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(output.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(output.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(output.angular_velocity.x, 0.4);
  EXPECT_DOUBLE_EQ(output.linear_acceleration.z, 9.8);
  EXPECT_DOUBLE_EQ(output.angular_velocity_covariance[0], 0.7);
}
