#include "robot_hardware/drive_conversion.hpp"

#include <algorithm>
#include <cmath>

namespace robot_hardware
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadPerSecToRpm = 60.0 / (2.0 * kPi);
constexpr double kRawFeedbackToRadPerSec = kPi / 300.0;
}  // namespace

std::optional<MotorRpmCommand> to_motor_rpm(
  double left_rad_s, double right_rad_s, int max_motor_rpm)
{
  if (
    !std::isfinite(left_rad_s) || !std::isfinite(right_rad_s) ||
    max_motor_rpm <= 0 || max_motor_rpm > 32767)
  {
    return std::nullopt;
  }

  const auto convert = [max_motor_rpm](double rad_s) {
      const double limited_rpm = std::clamp(
        rad_s * kRadPerSecToRpm,
        -static_cast<double>(max_motor_rpm),
        static_cast<double>(max_motor_rpm));
      return static_cast<int16_t>(std::lround(limited_rpm));
    };

  return MotorRpmCommand{convert(right_rad_s), convert(left_rad_s)};
}

WheelVelocities from_motor_feedback(
  int16_t raw_channel_0, int16_t raw_channel_1)
{
  return WheelVelocities{
    -static_cast<double>(raw_channel_0) * kRawFeedbackToRadPerSec,
    static_cast<double>(raw_channel_1) * kRawFeedbackToRadPerSec};
}
}  // namespace robot_hardware
