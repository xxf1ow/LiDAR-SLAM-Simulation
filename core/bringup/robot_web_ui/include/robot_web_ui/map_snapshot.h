#ifndef XX_MAP_SNAPSHOT_H_
#define XX_MAP_SNAPSHOT_H_

#include "robot_web_ui/web_types.h"

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <tl/expected.hpp>

namespace robot_web_ui
{
/**
 * Loads a trinary Nav2 YAML/P5 PGM map and returns revision 1 in the `map`
 * frame. A relative image path resolves from the YAML directory. Invalid input
 * and known YAML or file-read failures return a diagnostic string.
 */
[[nodiscard]] tl::expected<GridSnapshotPtr, std::string> load_nav2_pgm(const std::filesystem::path &yaml_path);

/**
 * Reuses an exactly equal snapshot or creates the next immutable revision.
 * Callers provide validated finite grid geometry with positive resolution and
 * occupancy bytes in 0 through 100 or 255 for unknown. Returns null when either
 * dimension is zero or `data` does not contain exactly one byte per cell.
 */
[[nodiscard]] GridSnapshotPtr update_grid_snapshot(
    GridSnapshotPtr current, const GridInfo &info, const std::vector<uint8_t> &data);

/**
 * Converts validated ROS occupancy values (-1 or 0 through 100) to bytes, then
 * applies the unsigned-grid contract. Callers must enforce that value range.
 */
[[nodiscard]] GridSnapshotPtr update_grid_snapshot(
    GridSnapshotPtr current, const GridInfo &info, const std::vector<int8_t> &data);

/**
 * Reuses an exactly equal path or encodes each point as a little-endian float32
 * x/y pair in caller order. Callers provide a consumer-valid frame and
 * coordinates that remain finite when represented as float32. Empty paths are
 * valid snapshots.
 */
[[nodiscard]] PathSnapshotPtr update_path_snapshot(
    PathSnapshotPtr current, const std::string &frame_id, const std::vector<std::pair<double, double>> &points);
} // namespace robot_web_ui

#endif  // XX_MAP_SNAPSHOT_H_
