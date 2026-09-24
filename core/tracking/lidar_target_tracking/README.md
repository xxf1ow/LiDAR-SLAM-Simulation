# LiDAR target tracking reference

This ROS 2 package installs the original `SharedState`, `KalmanFilter2D`, and `LidarTracker` headers in the global namespace. The headers are copied byte-for-byte from [6-robot/jie_deamon commit `9403185c0cf99a0b415ddd4f3b4215b44dd0809c`](https://github.com/6-robot/jie_deamon/tree/9403185c0cf99a0b415ddd4f3b4215b44dd0809c), from `include/common_types.hpp`, `include/kalman_filter.hpp`, and `include/lidar_tracker.hpp`. The source package declares MIT in its `package.xml`.

`LidarTracker::processScan` consumes `sensor_msgs::msg::LaserScan`. It applies the reference `(-range*cos(angle), -range*sin(angle))` convention, selects points in the target search radius, and keeps the last target position when no point matches. Kalman filtering and OpenCV display are disabled at construction. The tracker computes velocity internally; callers must leave its velocity callback unset when using it for observation only.

The `reference_tracking` CMake interface target exports the headers and OpenCV link dependency. `tracker_node` subscribes to `/points_raw` with sensor-data QoS and accepts a finite `/tracking/target` `PointStamped` only in `base_footprint`. Configure `ring` and `columns` as 26 and 1200 for Vanjee 722, or 7 and 1800 for simulation. The node converts that raw ring with a stamped transform into `base_footprint`, passes the scan to the reference tracker, and publishes a volatile `/tracking/state` `String` for every converted cloud. Invalid cloud fields or transforms skip that frame with a ROS error.

The state JSON has `frame_id`, `stamp` (`sec` and `nanosec`), `active`, `target`, and `points`. `points` contains `[x,y]` pairs even before selection. `target` is null until a target is selected, then reports the reference tracker's last output; a missed observation leaves that value in place. The node does not publish the reference's computed velocity or send navigation goals. Build and test it from `core/` after sourcing ROS 2 Humble:

```sh
colcon build --packages-select lidar_target_tracking
source install/setup.bash
colcon test --packages-select lidar_target_tracking
colcon test-result --all --verbose
```

The reference implementation retains its original search radius, obstacle calculation, velocity logic, and default filtering behavior. A missing target observation leaves the last position in `SharedState`; it does not establish current target presence.
