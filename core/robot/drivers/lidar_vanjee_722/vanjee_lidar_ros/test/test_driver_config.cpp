#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <utility>

#include "vanjee_lidar_ros/driver_config.hpp"

namespace vlr = vanjee_lidar_ros;
namespace vjl = vanjee::lidar;

namespace vanjee_lidar_ros::detail {
bool valid_scan_angle(double angle);
}

namespace {

vlr::DriverConfig syntheticConfig() {
  vlr::DriverConfig config{};
  config.lidar_type = "vanjee_722";
  config.host_address = "198.51.100.10";
  config.lidar_address = "198.51.100.11";
  config.host_msop_port = 4101;
  config.lidar_msop_port = 4102;
  config.start_angle = 5.0F;
  config.end_angle = 355.0F;
  config.min_distance = 0.25F;
  config.max_distance = 123.5F;
  config.wait_for_difop = false;
  config.config_from_file = true;
  config.use_lidar_clock = true;
  config.ts_first_point = false;
  config.dense_points = true;
  config.lidar_frame = "synthetic_lidar";
  config.imu_frame = "synthetic_imu";
  config.point_cloud_topic = "/synthetic/points";
  config.imu_topic = "/synthetic/imu";
  return config;
}

}  // namespace

TEST(DriverConfig, ParsesInstalledAndAlternateModels) {
  vjl::LidarType type{};
  ASSERT_TRUE(vlr::parse_lidar_type("vanjee_722", type));
  EXPECT_EQ(type, vjl::LidarType::vanjee_722);
  ASSERT_TRUE(vlr::parse_lidar_type("vanjee_720_32", type));
  EXPECT_EQ(type, vjl::LidarType::vanjee_720_32);
  EXPECT_FALSE(vlr::parse_lidar_type("vanjee_722x", type));
}

TEST(DriverConfig, RejectsFactoryUnsupported738Model) {
  vjl::LidarType type{};
  EXPECT_FALSE(vlr::parse_lidar_type("vanjee_738", type));

  vlr::DriverConfig config = syntheticConfig();
  config.lidar_type = "vanjee_738";
  vjl::WJDriverParam param;
  std::string error;
  EXPECT_FALSE(vlr::make_driver_param(config, param, error));
  EXPECT_EQ(error, "unsupported lidar_type: vanjee_738");
}

TEST(DriverConfig, BuildsOnline722Parameters) {
  vlr::DriverConfig config = syntheticConfig();
  vjl::WJDriverParam param;
  std::string error;
  ASSERT_TRUE(vlr::make_driver_param(config, param, error)) << error;
  EXPECT_EQ(param.lidar_type, vjl::LidarType::vanjee_722);
  EXPECT_EQ(param.input_type, vjl::InputType::ONLINE_LIDAR);
  EXPECT_EQ(param.input_param.connect_type, 1);
  EXPECT_EQ(param.input_param.host_address, "198.51.100.10");
  EXPECT_EQ(param.input_param.lidar_address, "198.51.100.11");
  EXPECT_EQ(param.input_param.host_msop_port, 4101);
  EXPECT_EQ(param.input_param.lidar_msop_port, 4102);
  EXPECT_TRUE(param.decoder_param.point_cloud_enable);
  EXPECT_EQ(param.decoder_param.imu_enable, 1);
  EXPECT_FALSE(param.decoder_param.ts_first_point);
  EXPECT_TRUE(param.decoder_param.use_lidar_clock);
}

TEST(DriverConfig, BuildsModelScopedCalibrationPathsInMapDirectory) {
  namespace fs = std::filesystem;
  const fs::path home =
      fs::temp_directory_path() / "vanjee_lidar_calibration_path_test";
  fs::remove_all(home);

  vlr::DriverConfig config = syntheticConfig();
  vjl::WJDriverParam param_722;
  std::string error;
  ASSERT_TRUE(vlr::configure_calibration_paths(
      config, home.string(), param_722, error)) << error;

  const fs::path directory =
      home / "result" / "lidar_calibration" / "198.51.100.11";
  EXPECT_TRUE(fs::is_directory(directory));
  EXPECT_EQ(fs::path(param_722.decoder_param.angle_path_ver),
            directory / "vanjee_722_vertical_angles.csv");
  EXPECT_EQ(fs::path(param_722.decoder_param.angle_path_hor),
            directory / "vanjee_722_horizontal_angles.csv");
  EXPECT_EQ(fs::path(param_722.decoder_param.imu_param_path),
            directory / "vanjee_722_imu_params.csv");

  config.lidar_type = "vanjee_720_32";
  vjl::WJDriverParam param_720;
  ASSERT_TRUE(vlr::configure_calibration_paths(
      config, home.string(), param_720, error)) << error;
  EXPECT_NE(param_722.decoder_param.angle_path_ver,
            param_720.decoder_param.angle_path_ver);
  EXPECT_EQ(fs::path(param_720.decoder_param.angle_path_ver),
            directory / "vanjee_720_32_vertical_angles.csv");

  fs::remove_all(home);
}

TEST(DriverConfig, RejectsMissingHomeForCalibrationPaths) {
  vlr::DriverConfig config = syntheticConfig();
  vjl::WJDriverParam param;
  std::string error;

  EXPECT_FALSE(vlr::configure_calibration_paths(config, "", param, error));
  EXPECT_EQ(error, "HOME is not set");
}

TEST(DriverConfig, RejectsInvalidInputsWithoutVendorExit) {
  vlr::DriverConfig config = syntheticConfig();
  vjl::WJDriverParam param;
  std::string error;

  config.lidar_type = "unknown";
  EXPECT_FALSE(vlr::make_driver_param(config, param, error));
  EXPECT_EQ(error, "unsupported lidar_type: unknown");

  config = syntheticConfig();
  config.host_address = "192.168.2.999";
  EXPECT_FALSE(vlr::make_driver_param(config, param, error));
  EXPECT_EQ(error, "invalid host_address: 192.168.2.999");

  config = syntheticConfig();
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
    vlr::DriverConfig config = syntheticConfig();
    config.min_distance = min_distance;
    config.max_distance = max_distance;
    vjl::WJDriverParam param;
    std::string error;

    EXPECT_FALSE(vlr::make_driver_param(config, param, error));
    EXPECT_EQ(error, "distance limits must be finite");
  }
}

TEST(DriverConfig, RejectsInvalidScanAngles) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const std::array<float, 5> invalid_angles{{
      nan,
      infinity,
      -infinity,
      -0.01F,
      360.01F,
  }};

  for (const float angle : invalid_angles) {
    vlr::DriverConfig config = syntheticConfig();
    config.start_angle = angle;
    vjl::WJDriverParam param;
    std::string error;
    EXPECT_FALSE(vlr::make_driver_param(config, param, error));
    EXPECT_EQ(error, "start_angle must be finite and in [0, 360]");

    config = syntheticConfig();
    config.end_angle = angle;
    EXPECT_FALSE(vlr::make_driver_param(config, param, error));
    EXPECT_EQ(error, "end_angle must be finite and in [0, 360]");
  }
}

TEST(DriverConfig, AcceptsSupportedScanAngleSemantics) {
  const std::array<std::pair<float, float>, 3> angle_ranges{{
      {0.0F, 0.0F},
      {0.0F, 360.0F},
      {350.0F, 10.0F},
  }};

  for (const auto &[start_angle, end_angle] : angle_ranges) {
    vlr::DriverConfig config = syntheticConfig();
    config.start_angle = start_angle;
    config.end_angle = end_angle;
    vjl::WJDriverParam param;
    std::string error;
    ASSERT_TRUE(vlr::make_driver_param(config, param, error)) << error;
    EXPECT_EQ(param.decoder_param.start_angle, start_angle);
    EXPECT_EQ(param.decoder_param.end_angle, end_angle);
  }
}

TEST(DriverConfig, ValidatesDoubleScanAnglesBeforeNarrowing) {
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 5> invalid_angles{{
      std::numeric_limits<double>::quiet_NaN(),
      infinity,
      -infinity,
      std::nextafter(0.0, -infinity),
      std::nextafter(360.0, infinity),
  }};
  for (const double angle : invalid_angles) {
    EXPECT_FALSE(vlr::detail::valid_scan_angle(angle));
  }

  for (const double angle : {0.0, 10.0, 350.0, 360.0}) {
    EXPECT_TRUE(vlr::detail::valid_scan_angle(angle));
  }
}

TEST(DriverConfig, RejectsZeroMsopPorts) {
  for (const bool host_port : {true, false}) {
    vlr::DriverConfig config = syntheticConfig();
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
  vlr::DriverConfig config = syntheticConfig();
  config.min_distance = -0.01F;
  vjl::WJDriverParam param;
  std::string error;

  EXPECT_FALSE(vlr::make_driver_param(config, param, error));
  EXPECT_EQ(error, "min_distance must be non-negative");
}

TEST(DriverConfig, RejectsEmptyFrameNames) {
  for (const bool lidar_frame : {true, false}) {
    vlr::DriverConfig config = syntheticConfig();
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
    vlr::DriverConfig config = syntheticConfig();
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
