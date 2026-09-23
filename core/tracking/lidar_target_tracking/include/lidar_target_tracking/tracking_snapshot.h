#ifndef XX_TRACKING_SNAPSHOT_H_
#define XX_TRACKING_SNAPSHOT_H_

#include <string>

#include "lidar_target_tracking/scan_bridge.h"

namespace lidar_target_tracking {

// Encodes the converted frame and the reference tracker's last target. An
// inactive selection has a null target; points remain visible.
[[nodiscard]] std::string make_tracking_snapshot(
    const ScanFrame &frame, bool active, double target_x, double target_y);

}  // namespace lidar_target_tracking

#endif  // XX_TRACKING_SNAPSHOT_H_
