#include "robot_web_ui/manual_command.h"

#include <cmath>

namespace robot_web_ui
{
tl::expected<CommandValues, std::string> command_values(
    const std::string &direction, double speed_percent, double max_linear, double max_angular)
{
    if (!std::isfinite(speed_percent) || !std::isfinite(max_linear) || !std::isfinite(max_angular))
        return tl::make_unexpected("speed values must be finite numbers");
    if (speed_percent < 0.0 || speed_percent > 100.0)
        return tl::make_unexpected("speed_percent must be between 0 and 100");

    const double scale = speed_percent / 100.0;
    if (direction == "forward")
        return CommandValues{max_linear * scale, 0.0};
    if (direction == "backward")
        return CommandValues{-max_linear * scale, 0.0};
    if (direction == "left")
        return CommandValues{0.0, max_angular * scale};
    if (direction == "right")
        return CommandValues{0.0, -max_angular * scale};
    if (direction == "stop")
        return CommandValues{0.0, 0.0};
    return tl::make_unexpected("unknown direction: " + direction);
}
} // namespace robot_web_ui
