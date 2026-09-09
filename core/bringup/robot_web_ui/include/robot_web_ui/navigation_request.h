#ifndef ROBOT_WEB_UI_NAVIGATION_REQUEST_H_
#define ROBOT_WEB_UI_NAVIGATION_REQUEST_H_

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

/** Returns a validation error or a map-revision conflict for an unusable browser pose. */
[[nodiscard]] tl::expected<NavigationPose, PoseError> parse_navigation_pose(
    const nlohmann::json &payload, const GridInfo &map, uint64_t current_revision);
} // namespace robot_web_ui

#endif  // ROBOT_WEB_UI_NAVIGATION_REQUEST_H_
