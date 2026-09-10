#include "robot_web_ui/manual_command.h"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace robot_web_ui
{
TEST(CommandValues, ScalesRecognizedDirections)
{
    struct Case {
        const char *direction;
        double speed_percent;
        double linear_x;
        double angular_z;
    };
    const Case cases[] = {
        {"forward", 20.0, 0.3, 0.0},
        {"backward", 100.0, -1.5, 0.0},
        {"left", 25.0, 0.0, 0.5},
        {"right", 50.0, 0.0, -1.0},
        {"stop", 100.0, 0.0, 0.0},
    };

    for (const Case &test_case : cases) {
        const auto command = command_values(test_case.direction, test_case.speed_percent, 1.5, 2.0);
        ASSERT_TRUE(command) << test_case.direction;
        EXPECT_DOUBLE_EQ(command->linear_x, test_case.linear_x);
        EXPECT_DOUBLE_EQ(command->angular_z, test_case.angular_z);
    }
}

TEST(CommandValues, RejectsInvalidDirectionAndSpeedValues)
{
    EXPECT_FALSE(command_values("forward", 101.0, 1.5, 2.0));
    EXPECT_FALSE(command_values("spin", 20.0, 1.5, 2.0));
    EXPECT_FALSE(command_values("forward", std::numeric_limits<double>::quiet_NaN(), 1.5, 2.0));
}
} // namespace robot_web_ui
