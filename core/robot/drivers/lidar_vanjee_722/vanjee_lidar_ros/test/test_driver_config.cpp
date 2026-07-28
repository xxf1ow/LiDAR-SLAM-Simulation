#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <utility>

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

TEST(DriverConfig, RejectsNonFiniteDistanceLimits) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const std::array<std::pair<float, float>, 3> limits{{
      {nan, 70.0F},
      {0.05F, nan},
      {0.05F, infinity},
  }};

  for (const auto &[min_distance, max_distance] : limits) {
    vlr::DriverConfig config;
    config.min_distance = min_distance;
    config.max_distance = max_distance;
    vjl::WJDriverParam param;
    std::string error;

    EXPECT_FALSE(vlr::make_driver_param(config, param, error));
    EXPECT_EQ(error, "distance limits must be finite");
  }
}

TEST(DriverConfig, RejectsZeroMsopPorts) {
  for (const bool host_port : {true, false}) {
    vlr::DriverConfig config;
    if (host_port) {
      config.host_msop_port = 0;
    } else {
      config.lidar_msop_port = 0;
    }
    vjl::WJDriverParam param;
    std::string error;

    EXPECT_FALSE(vlr::make_driver_param(config, param, error));
    EXPECT_EQ(error, "MSOP ports must be non-zero");
  }
}

TEST(DriverConfig, RejectsNegativeMinimumDistance) {
  vlr::DriverConfig config;
  config.min_distance = -0.01F;
  vjl::WJDriverParam param;
  std::string error;

  EXPECT_FALSE(vlr::make_driver_param(config, param, error));
  EXPECT_EQ(error, "min_distance must be non-negative");
}

TEST(DriverConfig, RejectsEmptyFrameNames) {
  for (const bool lidar_frame : {true, false}) {
    vlr::DriverConfig config;
    if (lidar_frame) {
      config.lidar_frame.clear();
    } else {
      config.imu_frame.clear();
    }
    vjl::WJDriverParam param;
    std::string error;

    EXPECT_FALSE(vlr::make_driver_param(config, param, error));
    EXPECT_EQ(error, "frame names must be non-empty");
  }
}

TEST(DriverConfig, RejectsEmptyTopicNames) {
  for (const bool point_cloud_topic : {true, false}) {
    vlr::DriverConfig config;
    if (point_cloud_topic) {
      config.point_cloud_topic.clear();
    } else {
      config.imu_topic.clear();
    }
    vjl::WJDriverParam param;
    std::string error;

    EXPECT_FALSE(vlr::make_driver_param(config, param, error));
    EXPECT_EQ(error, "topic names must be non-empty");
  }
}
