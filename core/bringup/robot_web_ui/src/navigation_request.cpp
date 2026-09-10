#include "robot_web_ui/navigation_request.h"

#include <cmath>
#include <utility>

namespace robot_web_ui
{
namespace
{
tl::unexpected<PoseError> invalid_request(std::string message)
{
    return tl::make_unexpected(PoseError{PoseErrorKind::invalid_request, std::move(message)});
}

bool finite_number(const nlohmann::json &value, double *number)
{
    if (!value.is_number())
        return false;
    *number = value.get<double>();
    return std::isfinite(*number);
}
} // namespace

tl::expected<NavigationPose, PoseError> parse_navigation_pose(
    const nlohmann::json &payload, const GridInfo &map, uint64_t current_revision)
{
    if (!payload.is_object() || payload.size() != 4 || !payload.contains("x") ||
        !payload.contains("y") || !payload.contains("yaw") || !payload.contains("map_revision")) {
        return invalid_request("pose request keys must be x, y, yaw, map_revision");
    }

    const nlohmann::json &revision_value = payload.at("map_revision");
    if (!revision_value.is_number_integer() || revision_value.get<int64_t>() <= 0)
        return invalid_request("map_revision must be a positive integer");
    const uint64_t revision = static_cast<uint64_t>(revision_value.get<int64_t>());
    if (revision != current_revision) {
        return tl::make_unexpected(
            PoseError{PoseErrorKind::map_revision_conflict, "map revision changed"});
    }

    if (map.width == 0 || map.height == 0 || !std::isfinite(map.resolution) || map.resolution <= 0.0 ||
        !std::isfinite(map.origin_x) || !std::isfinite(map.origin_y) || !std::isfinite(map.origin_yaw)) {
        return invalid_request("invalid map geometry");
    }

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (!finite_number(payload.at("x"), &x) || !finite_number(payload.at("y"), &y) ||
        !finite_number(payload.at("yaw"), &yaw)) {
        return invalid_request("pose coordinates must be finite numbers");
    }

    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    const double dx = x - map.origin_x;
    const double dy = y - map.origin_y;
    const double cosine = std::cos(map.origin_yaw);
    const double sine = std::sin(map.origin_yaw);
    const double local_x = dx * cosine + dy * sine;
    const double local_y = -dx * sine + dy * cosine;
    if (local_x < 0.0 || local_x >= static_cast<double>(map.width) * map.resolution || local_y < 0.0 ||
        local_y >= static_cast<double>(map.height) * map.resolution) {
        return invalid_request("pose is outside the static map");
    }

    return NavigationPose{x, y, yaw};
}
} // namespace robot_web_ui
