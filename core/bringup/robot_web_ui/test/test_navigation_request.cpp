#include "robot_web_ui/navigation_request.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace robot_web_ui
{
TEST(ParseNavigationPose, NormalizesPoseWithinRotatedMap)
{
    const GridInfo rotated{10, 20, 0.5, 4.0, -2.0, 0.25, "map"};
    const nlohmann::json payload{
        {"x", 4.0}, {"y", -2.0}, {"yaw", 7.0}, {"map_revision", 3}};

    const auto pose = parse_navigation_pose(payload, rotated, 3);

    ASSERT_TRUE(pose);
    EXPECT_DOUBLE_EQ(pose->x, 4.0);
    EXPECT_DOUBLE_EQ(pose->y, -2.0);
    EXPECT_NEAR(pose->yaw, std::atan2(std::sin(7.0), std::cos(7.0)), 1e-12);
}

TEST(ParseNavigationPose, DistinguishesRevisionConflictFromInvalidPose)
{
    const GridInfo rotated{10, 20, 0.5, 4.0, -2.0, 0.25, "map"};
    const nlohmann::json valid_pose{
        {"x", 4.0}, {"y", -2.0}, {"yaw", 7.0}, {"map_revision", 3}};
    const nlohmann::json outside_pose{
        {"x", 100.0}, {"y", 100.0}, {"yaw", 0.0}, {"map_revision", 3}};

    const auto stale = parse_navigation_pose(valid_pose, rotated, 2);
    EXPECT_FALSE(stale);
    EXPECT_EQ(stale.error().kind, PoseErrorKind::map_revision_conflict);
    EXPECT_FALSE(parse_navigation_pose(outside_pose, rotated, 3));
}

TEST(ParseNavigationPose, AcceptsOnlyExactFiniteTypedPayload)
{
    const GridInfo map{2, 2, 1.0, 0.0, 0.0, 0.0, "map"};
    const nlohmann::json cases[] = {
        {{"x", 0.5}, {"y", 0.5}, {"yaw", 0.0}},
        {{"x", 0.5}, {"y", 0.5}, {"yaw", 0.0}, {"map_revision", 1}, {"extra", 1}},
        {{"x", true}, {"y", 0.5}, {"yaw", 0.0}, {"map_revision", 1}},
        {{"x", 0.5}, {"y", 0.5}, {"yaw", 0.0}, {"map_revision", 1.0}},
        {{"x", 0.5}, {"y", 0.5}, {"yaw", std::numeric_limits<double>::quiet_NaN()}, {"map_revision", 1}},
    };

    for (const auto &payload : cases)
        EXPECT_FALSE(parse_navigation_pose(payload, map, 1));
}
} // namespace robot_web_ui
