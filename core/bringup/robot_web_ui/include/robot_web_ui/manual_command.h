#ifndef ROBOT_WEB_UI_MANUAL_COMMAND_H_
#define ROBOT_WEB_UI_MANUAL_COMMAND_H_

#include <string>

#include <tl/expected.hpp>

namespace robot_web_ui
{
struct CommandValues {
    /** Linear velocity in meters per second. */
    double linear_x;
    /** Angular velocity in radians per second. */
    double angular_z;
};

/** Returns the scaled velocity command, or a validation error for an invalid input. */
[[nodiscard]] tl::expected<CommandValues, std::string> command_values(
    const std::string &direction, double speed_percent, double max_linear, double max_angular);
} // namespace robot_web_ui

#endif  // ROBOT_WEB_UI_MANUAL_COMMAND_H_
