#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "lidar_tracker.hpp"

TEST(ReferenceTracking, UpdatesTargetFromNegativeCosineScanConvention) {
  SharedState state;
  state.setTarget(1.0, 0.0);
  state.active.store(true);
  LidarTracker tracker(state);

  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->angle_min = -3.14159265358979323846F;
  scan->angle_increment = 1.57079632679489661923F;
  const float infinity = std::numeric_limits<float>::infinity();
  scan->ranges = {1.1F, infinity, infinity, infinity};
  tracker.processScan(scan);

  double x = 0.0;
  double y = 0.0;
  state.getTarget(x, y);
  EXPECT_NEAR(x, 1.1, 1e-6);
  EXPECT_NEAR(y, 0.0, 1e-6);

  scan->ranges = {infinity, infinity, infinity, infinity};
  tracker.processScan(scan);
  state.getTarget(x, y);
  EXPECT_NEAR(x, 1.1, 1e-6);
  EXPECT_NEAR(y, 0.0, 1e-6);
}
