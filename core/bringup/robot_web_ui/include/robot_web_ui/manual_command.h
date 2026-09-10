#ifndef XX_MANUAL_COMMAND_H_
#define XX_MANUAL_COMMAND_H_

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

/**
 * Returns a command for `forward`, `backward`, `left`, `right`, or `stop`.
 * `speed_percent` must be finite and within 0 through 100 inclusive.
 * `max_linear` and `max_angular` must be finite and are expressed in meters per
 * second and radians per second. Invalid input returns a validation message.
 */
[[nodiscard]] tl::expected<CommandValues, std::string> command_values(
    const std::string &direction, double speed_percent, double max_linear, double max_angular);
} // namespace robot_web_ui

#endif  // XX_MANUAL_COMMAND_H_
