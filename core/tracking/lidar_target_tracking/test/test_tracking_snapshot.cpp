#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "lidar_target_tracking/scan_bridge.h"
#include "lidar_target_tracking/tracking_snapshot.h"

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
