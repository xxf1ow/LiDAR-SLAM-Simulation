#ifndef ROBOT_HARDWARE__DRIVE_CONVERSION_HPP_
#define ROBOT_HARDWARE__DRIVE_CONVERSION_HPP_

#include <cstdint>
#include <optional>

namespace robot_hardware
{
struct MotorRpmCommand
{
  int16_t right_rpm;
  int16_t left_rpm;
};

struct WheelVelocities
{
  double left_rad_s;
  double right_rad_s;
};

std::optional<MotorRpmCommand> to_motor_rpm(
  double left_rad_s, double right_rad_s, int max_motor_rpm);

WheelVelocities from_motor_feedback(
  int16_t raw_channel_0, int16_t raw_channel_1);
}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__DRIVE_CONVERSION_HPP_
