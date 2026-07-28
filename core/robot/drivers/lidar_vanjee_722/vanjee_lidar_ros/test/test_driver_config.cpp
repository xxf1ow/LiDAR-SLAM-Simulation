#include <gtest/gtest.h>

#include "vanjee_lidar_ros/driver_config.hpp"

namespace vlr = vanjee_lidar_ros;
namespace vjl = vanjee::lidar;

TEST(DriverConfig, ParsesInstalledAndAlternateModels) {
  vjl::LidarType type{};
  ASSERT_TRUE(vlr::parse_lidar_type("vanjee_722", type));
  EXPECT_EQ(type, vjl::LidarType::vanjee_722);
  ASSERT_TRUE(vlr::parse_lidar_type("vanjee_720_32", type));
  EXPECT_EQ(type, vjl::LidarType::vanjee_720_32);
  EXPECT_FALSE(vlr::parse_lidar_type("vanjee_722x", type));
}

TEST(DriverConfig, BuildsOnline722Parameters) {
  vlr::DriverConfig config;
  vjl::WJDriverParam param;
  std::string error;
  ASSERT_TRUE(vlr::make_driver_param(config, param, error)) << error;
  EXPECT_EQ(param.lidar_type, vjl::LidarType::vanjee_722);
  EXPECT_EQ(param.input_type, vjl::InputType::ONLINE_LIDAR);
  EXPECT_EQ(param.input_param.connect_type, 1);
  EXPECT_EQ(param.input_param.host_address, "192.168.2.88");
  EXPECT_EQ(param.input_param.lidar_address, "192.168.2.86");
  EXPECT_EQ(param.input_param.host_msop_port, 3001);
  EXPECT_EQ(param.input_param.lidar_msop_port, 3333);
  EXPECT_TRUE(param.decoder_param.point_cloud_enable);
  EXPECT_EQ(param.decoder_param.imu_enable, 1);
  EXPECT_TRUE(param.decoder_param.ts_first_point);
  EXPECT_FALSE(param.decoder_param.use_lidar_clock);
}

TEST(DriverConfig, RejectsInvalidInputsWithoutVendorExit) {
  vlr::DriverConfig config;
  vjl::WJDriverParam param;
  std::string error;

  config.lidar_type = "unknown";
  EXPECT_FALSE(vlr::make_driver_param(config, param, error));
  EXPECT_EQ(error, "unsupported lidar_type: unknown");

  config = vlr::DriverConfig{};
  config.host_address = "192.168.2.999";
  EXPECT_FALSE(vlr::make_driver_param(config, param, error));
  EXPECT_EQ(error, "invalid host_address: 192.168.2.999");

  config = vlr::DriverConfig{};
  config.max_distance = config.min_distance;
  EXPECT_FALSE(vlr::make_driver_param(config, param, error));
  EXPECT_EQ(error, "max_distance must be greater than min_distance");
}
