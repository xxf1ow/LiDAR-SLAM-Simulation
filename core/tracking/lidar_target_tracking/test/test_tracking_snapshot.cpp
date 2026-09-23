#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <memory>

#include "lidar_target_tracking/scan_bridge.h"
#include "lidar_target_tracking/tracking_snapshot.h"
#include "lidar_tracker.hpp"

TEST(TrackingSnapshot, BridgesCloudThroughReferenceTrackerToJson) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "velodyne";
  cloud.header.stamp.sec = 5;
  cloud.header.stamp.nanosec = 42;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16);
  modifier.resize(1);
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "x") = 1.25F;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "y") = 0.0F;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "z") = 0.0F;
  *sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "ring") = 26;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_footprint";
  transform.child_frame_id = "velodyne";
  transform.transform.rotation.w = 1.0;
  auto frame = lidar_target_tracking::bridge_scan(cloud, transform, 26, 4);
  ASSERT_TRUE(frame.has_value());

  SharedState state;
  state.setTarget(1.0, 0.0);
  state.active.store(true);
  LidarTracker tracker(state);
  tracker.processScan(std::make_shared<sensor_msgs::msg::LaserScan>(frame->scan));
  double target_x = 0.0;
  double target_y = 0.0;
  state.getTarget(target_x, target_y);
  const auto json = nlohmann::json::parse(
      lidar_target_tracking::make_tracking_snapshot(*frame, state.active.load(), target_x, target_y));

  EXPECT_EQ(json.at("frame_id"), "base_footprint");
  EXPECT_EQ(json.at("stamp"), (nlohmann::json{{"sec", 5}, {"nanosec", 42}}));
  EXPECT_EQ(json.at("active"), true);
  ASSERT_EQ(json.at("points").size(), 1U);
  EXPECT_NEAR(json.at("points").at(0).at(0).get<double>(), 1.25, 1e-6);
  EXPECT_NEAR(json.at("points").at(0).at(1).get<double>(), 0.0, 1e-6);
  EXPECT_NEAR(json.at("target").at("x").get<double>(), 1.25, 1e-6);
  EXPECT_NEAR(json.at("target").at("y").get<double>(), 0.0, 1e-6);
}

TEST(TrackingSnapshot, SelectedTargetAndPoints) {
  lidar_target_tracking::ScanFrame frame;
  frame.scan.header.frame_id = "base_footprint";
  frame.scan.header.stamp.sec = 5;
  frame.scan.header.stamp.nanosec = 42;
  frame.points = {{{1.1, -0.2}}};

  const auto json = nlohmann::json::parse(
      lidar_target_tracking::make_tracking_snapshot(frame, true, 1.1, -0.2));
  EXPECT_EQ(json.size(), 5U);
  EXPECT_EQ(json.at("frame_id"), "base_footprint");
  EXPECT_EQ(json.at("stamp"), (nlohmann::json{{"sec", 5}, {"nanosec", 42}}));
  EXPECT_EQ(json.at("active"), true);
  EXPECT_EQ(json.at("target"), (nlohmann::json{{"x", 1.1}, {"y", -0.2}}));
  EXPECT_EQ(json.at("points"), (nlohmann::json::array({{1.1, -0.2}})));
  EXPECT_FALSE(json.contains("lost"));
  EXPECT_FALSE(json.contains("prediction"));
}

TEST(TrackingSnapshot, InactiveRetainsPointsWithoutTarget) {
  lidar_target_tracking::ScanFrame frame;
  frame.scan.header.frame_id = "base_footprint";
  frame.points = {{{1.1, -0.2}}};

  const auto json = nlohmann::json::parse(
      lidar_target_tracking::make_tracking_snapshot(frame, false, 1.1, -0.2));
  EXPECT_EQ(json.at("active"), false);
  EXPECT_TRUE(json.at("target").is_null());
  EXPECT_EQ(json.at("points"), (nlohmann::json::array({{1.1, -0.2}})));
  EXPECT_FALSE(json.contains("lost"));
  EXPECT_FALSE(json.contains("prediction"));
}
