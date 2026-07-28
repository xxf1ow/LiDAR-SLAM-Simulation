#include <cstring>
#include <cstdint>
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
  ASSERT_TRUE(vlr::seconds_to_stamp(
      static_cast<double>(std::numeric_limits<int32_t>::max()), stamp, error))
      << error;
  EXPECT_EQ(stamp.sec, std::numeric_limits<int32_t>::max());
  EXPECT_EQ(stamp.nanosec, 0U);
  EXPECT_FALSE(vlr::seconds_to_stamp(
      static_cast<double>(std::numeric_limits<int32_t>::max()) + 1.0,
      stamp,
      error));
  EXPECT_EQ(error, "timestamp seconds exceed ROS Time range");
  EXPECT_FALSE(vlr::seconds_to_stamp(
      std::numeric_limits<double>::quiet_NaN(), stamp, error));
  EXPECT_EQ(error, "timestamp must be finite and non-negative");
  for (const double seconds : {
           -0.1,
           std::numeric_limits<double>::infinity(),
           -std::numeric_limits<double>::infinity(),
       }) {
    EXPECT_FALSE(vlr::seconds_to_stamp(seconds, stamp, error));
    EXPECT_EQ(error, "timestamp must be finite and non-negative");
  }
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
  EXPECT_EQ(output.fields[0].name, "x");
  EXPECT_EQ(output.fields[0].offset, 0U);
  EXPECT_EQ(output.fields[0].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(output.fields[0].count, 1U);
  EXPECT_EQ(output.fields[1].name, "y");
  EXPECT_EQ(output.fields[1].offset, 4U);
  EXPECT_EQ(output.fields[1].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(output.fields[1].count, 1U);
  EXPECT_EQ(output.fields[2].name, "z");
  EXPECT_EQ(output.fields[2].offset, 8U);
  EXPECT_EQ(output.fields[2].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(output.fields[2].count, 1U);
  EXPECT_EQ(output.fields[3].name, "intensity");
  EXPECT_EQ(output.fields[3].offset, 12U);
  EXPECT_EQ(output.fields[3].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(output.fields[3].count, 1U);
  EXPECT_EQ(output.fields[4].name, "ring");
  EXPECT_EQ(output.fields[4].offset, 16U);
  EXPECT_EQ(output.fields[4].datatype, sensor_msgs::msg::PointField::UINT16);
  EXPECT_EQ(output.fields[4].count, 1U);
  EXPECT_EQ(output.fields[5].name, "time");
  EXPECT_EQ(output.fields[5].offset, 20U);
  EXPECT_EQ(output.fields[5].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(output.fields[5].count, 1U);
  EXPECT_EQ(output.header.stamp.sec, 100);
  EXPECT_EQ(output.header.stamp.nanosec, 250000000U);
  EXPECT_EQ(output.height, 1U);
  EXPECT_EQ(output.width, 2U);
  EXPECT_EQ(output.point_step, 24U);
  EXPECT_EQ(output.row_step, 48U);
  EXPECT_EQ(output.header.frame_id, "velodyne");
  EXPECT_FALSE(output.is_bigendian);
  EXPECT_TRUE(output.is_dense);
  EXPECT_EQ(read_at<float>(output, 0), 1.0F);
  EXPECT_EQ(read_at<uint16_t>(output, 24 + 16), 31U);
  EXPECT_EQ(output.data[18], 0U);
  EXPECT_EQ(output.data[19], 0U);
  EXPECT_FLOAT_EQ(read_at<float>(output, 24 + 20), 0.099F);
}

TEST(MessageConversion, RejectsPointCloudRowStepOverflow) {
  vlr::VendorPointCloud cloud;
  cloud.height = 0;
  cloud.width = std::numeric_limits<uint32_t>::max() / 24U + 1U;
  cloud.timestamp = 1.0;

  sensor_msgs::msg::PointCloud2 output;
  std::string error;
  EXPECT_FALSE(vlr::to_point_cloud2(cloud, "velodyne", output, error));
  EXPECT_EQ(error, "point cloud row_step exceeds uint32 range");
}

TEST(MessageConversion, RejectsPointCloudDataBeyondVectorCapacity) {
  vlr::VendorPointCloud cloud;
  cloud.height = std::numeric_limits<uint32_t>::max();
  cloud.width = std::numeric_limits<uint32_t>::max() / 24U;
  cloud.timestamp = 1.0;

  sensor_msgs::msg::PointCloud2 output;
  std::string error;
  EXPECT_FALSE(vlr::to_point_cloud2(cloud, "velodyne", output, error));
  EXPECT_EQ(error, "point cloud data size exceeds platform range");
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
