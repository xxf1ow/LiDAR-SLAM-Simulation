#ifndef ROBOT_WEB_UI_MAP_SNAPSHOT_H_
#define ROBOT_WEB_UI_MAP_SNAPSHOT_H_

#include "robot_web_ui/web_types.h"

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <tl/expected.hpp>

namespace robot_web_ui
{
/** Loads a trinary Nav2 map and returns its first immutable binary revision. */
[[nodiscard]] tl::expected<GridSnapshotPtr, std::string> load_nav2_pgm(const std::filesystem::path &yaml_path);

/** Reuses an equal grid snapshot, creates a new immutable revision, or returns null for mismatched data. */
[[nodiscard]] GridSnapshotPtr update_grid_snapshot(
    GridSnapshotPtr current, const GridInfo &info, const std::vector<uint8_t> &data);

/** Converts signed ROS occupancy values before creating an immutable grid revision or returning null. */
[[nodiscard]] GridSnapshotPtr update_grid_snapshot(
    GridSnapshotPtr current, const GridInfo &info, const std::vector<int8_t> &data);

/** Reuses an equal path snapshot or encodes points as little-endian float32 pairs. */
[[nodiscard]] PathSnapshotPtr update_path_snapshot(
    PathSnapshotPtr current, const std::string &frame_id, const std::vector<std::pair<double, double>> &points);
} // namespace robot_web_ui

#endif  // ROBOT_WEB_UI_MAP_SNAPSHOT_H_
