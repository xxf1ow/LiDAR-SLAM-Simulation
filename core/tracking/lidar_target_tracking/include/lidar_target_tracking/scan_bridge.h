#ifndef XX_SCAN_BRIDGE_H_
#define XX_SCAN_BRIDGE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_target_tracking {

struct ScanFrame {
  sensor_msgs::msg::LaserScan scan;
  std::vector<std::array<double, 2>> points;
};

// Converts one raw ring into a base_footprint scan and meter-valued display points
// at populated bin centers. Empty rings yield infinite ranges; malformed fields,
// layout, or transform yield no frame. The cloud and transform are borrowed for
// this call only.
[[nodiscard]] std::optional<ScanFrame> bridge_scan(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const geometry_msgs::msg::TransformStamped &transform,
    uint16_t ring, size_t columns);

}  // namespace lidar_target_tracking

#endif  // XX_SCAN_BRIDGE_H_
