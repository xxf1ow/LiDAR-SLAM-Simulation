#include "lidar_target_tracking/tracking_snapshot.h"

#include <nlohmann/json.hpp>

namespace lidar_target_tracking {

std::string make_tracking_snapshot(const ScanFrame &frame, bool active,
                                   double target_x, double target_y) {
  nlohmann::json points = nlohmann::json::array();
  for (const auto &point : frame.points) {
    points.push_back({point[0], point[1]});
  }
  nlohmann::json target = nullptr;
  if (active) {
    target = {{"x", target_x}, {"y", target_y}};
  }
  return nlohmann::json{{"frame_id", frame.scan.header.frame_id},
                        {"stamp", {{"sec", frame.scan.header.stamp.sec},
                                   {"nanosec", frame.scan.header.stamp.nanosec}}},
                        {"active", active},
                        {"target", target},
                        {"points", points}}.dump();
}

}  // namespace lidar_target_tracking
