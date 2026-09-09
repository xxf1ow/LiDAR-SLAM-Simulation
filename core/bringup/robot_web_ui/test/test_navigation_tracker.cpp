#include "robot_web_ui/navigation_tracker.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>

namespace robot_web_ui
{
namespace
{
constexpr std::array<uint8_t, 16> kUuidOne = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
constexpr std::array<uint8_t, 16> kUuidTwo = {2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

TEST(NavigationTracker, TracksCurrentGoalAndRejectsLateGenerationEvents)
{
    NavigationTracker tracker;
    EXPECT_EQ(tracker.state().status, NavigationStatus::idle);

    const uint64_t first_generation = tracker.reserve_goal();
    EXPECT_EQ(tracker.state().status, NavigationStatus::sending);
    ASSERT_TRUE(tracker.accept_goal(first_generation, kUuidOne));
    EXPECT_EQ(tracker.state().status, NavigationStatus::navigating);

    const GoalToken first_token{first_generation, kUuidOne};
    ASSERT_TRUE(tracker.update_path(std::make_shared<const PathSnapshot>(
        PathSnapshot{"map", std::make_shared<const BinarySnapshot>(BinarySnapshot{1, "etag", "application/octet-stream", {1, 2, 3, 4, 5, 6, 7, 8}, {}})})));
    EXPECT_TRUE(tracker.update_feedback(first_token, 4.5));
    EXPECT_EQ(tracker.state().distance_remaining, 4.5);

    GoalToken cancel_token{};
    ASSERT_TRUE(tracker.begin_cancel(&cancel_token));
    EXPECT_EQ(cancel_token, first_token);
    EXPECT_EQ(tracker.state().status, NavigationStatus::canceling);
    EXPECT_TRUE(tracker.reject_cancel(first_token, "cancel rejected"));
    EXPECT_EQ(tracker.state().status, NavigationStatus::navigating);
    EXPECT_EQ(tracker.state().message, "cancel rejected");

    ASSERT_TRUE(tracker.finish_goal(first_token, NavigationStatus::succeeded, std::nullopt));
    EXPECT_EQ(tracker.state().status, NavigationStatus::succeeded);
    EXPECT_FALSE(tracker.state().path);
    EXPECT_FALSE(tracker.state().distance_remaining);

    const uint64_t second_generation = tracker.reserve_goal();
    EXPECT_EQ(tracker.state().status, NavigationStatus::sending);
    const NavigationState before_late_events = tracker.state();
    EXPECT_FALSE(tracker.accept_goal(first_generation, kUuidOne));
    EXPECT_FALSE(tracker.update_feedback(first_token, 1.0));
    EXPECT_FALSE(tracker.reject_cancel(first_token, "late cancel"));
    EXPECT_FALSE(tracker.finish_goal(first_token, NavigationStatus::failed, "late result"));
    EXPECT_EQ(tracker.state(), before_late_events);
    EXPECT_TRUE(tracker.accept_goal(second_generation, kUuidTwo));
}

TEST(NavigationTracker, AllowsInitialPoseOnlyWhenNoGoalIsActive)
{
    NavigationTracker tracker;
    EXPECT_TRUE(tracker.initial_pose_allowed());
    const uint64_t generation = tracker.reserve_goal();
    EXPECT_FALSE(tracker.initial_pose_allowed());
    ASSERT_TRUE(tracker.accept_goal(generation, kUuidOne));
    EXPECT_FALSE(tracker.initial_pose_allowed());
    GoalToken token{};
    ASSERT_TRUE(tracker.begin_cancel(&token));
    EXPECT_FALSE(tracker.initial_pose_allowed());
    ASSERT_TRUE(tracker.finish_goal(token, NavigationStatus::canceled, std::nullopt));
    EXPECT_TRUE(tracker.initial_pose_allowed());
}

TEST(NavigationTracker, FailsCurrentSubmissionAndIgnoresStaleSubmissionFailures)
{
    NavigationTracker tracker;
    const uint64_t first_generation = tracker.reserve_goal();
    ASSERT_TRUE(tracker.update_path(std::make_shared<const PathSnapshot>()));

    ASSERT_TRUE(tracker.fail_submission(first_generation, "navigation goal rejected"));
    EXPECT_EQ(tracker.state().status, NavigationStatus::failed);
    EXPECT_EQ(tracker.state().message, "navigation goal rejected");
    EXPECT_FALSE(tracker.state().path);
    EXPECT_FALSE(tracker.state().distance_remaining);

    const uint64_t second_generation = tracker.reserve_goal();
    const NavigationState before_stale_failure = tracker.state();
    EXPECT_FALSE(tracker.fail_submission(first_generation, "navigation send failed"));
    EXPECT_EQ(tracker.state(), before_stale_failure);
    EXPECT_TRUE(tracker.accept_goal(second_generation, kUuidTwo));
}

TEST(NavigationTracker, AcceptsPathsOnlyForAnActiveOwnedGoalAndGatesDistanceOnNonemptyPath)
{
    NavigationTracker tracker;
    const PathSnapshotPtr path = std::make_shared<const PathSnapshot>();
    EXPECT_FALSE(tracker.update_path(path));

    const uint64_t generation = tracker.reserve_goal();
    ASSERT_TRUE(tracker.accept_goal(generation, kUuidOne));
    const GoalToken token{generation, kUuidOne};
    ASSERT_TRUE(tracker.update_feedback(token, 2.0));
    EXPECT_FALSE(tracker.state().distance_remaining);

    PathSnapshotPtr nonempty_path = std::make_shared<const PathSnapshot>(
        PathSnapshot{"map", std::make_shared<const BinarySnapshot>(BinarySnapshot{1, "etag", "application/octet-stream", {1, 2, 3, 4, 5, 6, 7, 8}, {}})});
    EXPECT_TRUE(tracker.update_path(nonempty_path));
    EXPECT_EQ(tracker.state().distance_remaining, 2.0);
    EXPECT_FALSE(tracker.state().phase);
    ASSERT_TRUE(tracker.finish_goal(token, NavigationStatus::succeeded, std::nullopt));
    EXPECT_FALSE(tracker.update_path(nonempty_path));
}
} // namespace
} // namespace robot_web_ui
