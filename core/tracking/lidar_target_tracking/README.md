# LiDAR target tracking reference

This ROS 2 package installs the original `SharedState`, `KalmanFilter2D`, and `LidarTracker` headers in the global namespace. The headers are copied byte-for-byte from [6-robot/jie_deamon commit `9403185c0cf99a0b415ddd4f3b4215b44dd0809c`](https://github.com/6-robot/jie_deamon/tree/9403185c0cf99a0b415ddd4f3b4215b44dd0809c), from `include/common_types.hpp`, `include/kalman_filter.hpp`, and `include/lidar_tracker.hpp`. The source package declares MIT in its `package.xml`.

`LidarTracker::processScan` consumes `sensor_msgs::msg::LaserScan`. It applies the reference `(-range*cos(angle), -range*sin(angle))` convention, selects points in the target search radius, and keeps the last target position when no point matches. Kalman filtering and OpenCV display are disabled at construction. The tracker computes velocity internally; callers must leave its velocity callback unset when using it for observation only.

The `reference_tracking` CMake interface target exports the headers and OpenCV link dependency. This package contains no executable or scan conversion; ROS input and output adapters belong outside the copied headers. Build and test it from `core/` after sourcing ROS 2 Humble:

```sh
colcon build --packages-select lidar_target_tracking
source install/setup.bash
colcon test --packages-select lidar_target_tracking
colcon test-result --all --verbose
```

The reference implementation retains its original search radius, obstacle calculation, velocity logic, and default filtering behavior. A missing target observation leaves the last position in `SharedState`; it does not establish current target presence.
