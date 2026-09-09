#ifndef XX_NAVIGATION_REQUEST_H_
#define XX_NAVIGATION_REQUEST_H_

#include "robot_web_ui/web_types.h"

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace robot_web_ui
{
enum class PoseErrorKind {
    invalid_request = 1,
    map_revision_conflict,
};

struct PoseError {
    PoseErrorKind kind;
    std::string message;
};

/**
 * Validates an object containing exactly finite numeric `x`, `y`, and `yaw`
 * fields plus a positive integer `map_revision`. The revision must equal
 * `current_revision`. `map` must have nonzero dimensions, finite origin values,
 * and a finite positive resolution. Bounds are checked in the rotated grid's
 * local coordinates, including each lower edge and excluding each upper edge.
 * The returned yaw is normalized to the principal interval [-pi, pi]. Invalid
 * fields or geometry return `invalid_request`; a stale revision returns
 * `map_revision_conflict`.
 */
[[nodiscard]] tl::expected<NavigationPose, PoseError> parse_navigation_pose(
    const nlohmann::json &payload, const GridInfo &map, uint64_t current_revision);
} // namespace robot_web_ui

#endif  // XX_NAVIGATION_REQUEST_H_
