#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "robot_hardware/drive_conversion.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double rpm_to_rad_s(double rpm)
{
  return rpm * 2.0 * kPi / 60.0;
}
}  // namespace

TEST(DriveConversion, CommandsInvertPolarityAndRemainOrderedRightThenLeft)
{
  const auto command = robot_hardware::to_motor_rpm(
    rpm_to_rad_s(-20.0), rpm_to_rad_s(20.0), 256);
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->right_rpm, -20);
  EXPECT_EQ(command->left_rpm, 20);

  const auto clamped = robot_hardware::to_motor_rpm(100.0, -100.0, 256);
  ASSERT_TRUE(clamped.has_value());
  EXPECT_EQ(clamped->right_rpm, 256);
  EXPECT_EQ(clamped->left_rpm, -256);
}

TEST(DriveConversion, CommandsRoundHalfAwayFromZero)
{
  const auto command = robot_hardware::to_motor_rpm(
    rpm_to_rad_s(-2.5), rpm_to_rad_s(2.5), 256);
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->right_rpm, -3);
  EXPECT_EQ(command->left_rpm, 3);
}

TEST(DriveConversion, HugeFiniteValuesClampBeforeIntegerConversion)
{
  const auto command = robot_hardware::to_motor_rpm(
    std::numeric_limits<double>::max(),
    -std::numeric_limits<double>::max(),
    256);
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->right_rpm, 256);
  EXPECT_EQ(command->left_rpm, -256);
}

TEST(DriveConversion, RejectsNonFiniteCommandsAndInvalidLimits)
{
  EXPECT_FALSE(robot_hardware::to_motor_rpm(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 256).has_value());
  EXPECT_FALSE(robot_hardware::to_motor_rpm(
    0.0, std::numeric_limits<double>::infinity(), 256).has_value());
  EXPECT_FALSE(robot_hardware::to_motor_rpm(0.0, 0.0, 0).has_value());
  EXPECT_FALSE(robot_hardware::to_motor_rpm(0.0, 0.0, 32768).has_value());
}

TEST(DriveConversion, FeedbackUsesMeasuredOrderSignAndTenthRpmScale)
{
  const auto forward = robot_hardware::from_motor_feedback(200, -200);
  EXPECT_NEAR(forward.left_rad_s, rpm_to_rad_s(20.0), 1e-12);
  EXPECT_NEAR(forward.right_rad_s, rpm_to_rad_s(20.0), 1e-12);

  const auto left_turn = robot_hardware::from_motor_feedback(-1000, -1000);
  EXPECT_NEAR(left_turn.left_rad_s, rpm_to_rad_s(-100.0), 1e-12);
  EXPECT_NEAR(left_turn.right_rad_s, rpm_to_rad_s(100.0), 1e-12);
}

TEST(DriveConversion, ZeroFeedbackRemainsExactlyZero)
{
  const auto stopped = robot_hardware::from_motor_feedback(0, 0);
  EXPECT_DOUBLE_EQ(stopped.left_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(stopped.right_rad_s, 0.0);
}
