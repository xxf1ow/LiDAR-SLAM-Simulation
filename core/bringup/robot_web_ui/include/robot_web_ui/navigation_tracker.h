#ifndef ROBOT_WEB_UI_NAVIGATION_TRACKER_H_
#define ROBOT_WEB_UI_NAVIGATION_TRACKER_H_

#include "robot_web_ui/web_types.h"

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace robot_web_ui
{
enum class NavigationStatus {
    idle,
    sending,
    navigating,
    canceling,
    succeeded,
    canceled,
    failed,
};

/** Identifies one node-owned Nav2 goal. */
struct GoalToken {
    uint64_t generation;
    std::array<uint8_t, 16> uuid;
};

bool operator==(const GoalToken &left, const GoalToken &right);

/** Immutable current navigation projection. The phase field is always absent. */
struct NavigationState {
    NavigationStatus status;
    std::optional<double> distance_remaining;
    std::optional<std::string> message;
    std::optional<std::string> phase;
    PathSnapshotPtr path;
};

bool operator==(const NavigationState &left, const NavigationState &right);

/** Module-internal, mutex-protected navigation lifecycle state safe for short concurrent calls. */
class NavigationTracker {
public:
    NavigationTracker();

    /** Starts a new generation in sending state and clears displayed goal data. */
    [[nodiscard]] uint64_t reserve_goal();

    /** Records a Nav2 acceptance only when generation identifies the current sending goal. */
    [[nodiscard]] bool accept_goal(uint64_t generation, const std::array<uint8_t, 16> &uuid);

    /** Updates finite feedback only when the token identifies the current navigating goal. */
    [[nodiscard]] bool update_feedback(const GoalToken &token, double distance_remaining);

    /** Enters canceling state and writes the current token, or leaves token untouched on conflict. */
    [[nodiscard]] bool begin_cancel(GoalToken *token);

    /** Restores navigating after a matching cancellation rejection. */
    [[nodiscard]] bool reject_cancel(const GoalToken &token, std::string message);

    /** Records a matching terminal result and clears displayed path and distance. */
    [[nodiscard]] bool finish_goal(
        const GoalToken &token, NavigationStatus terminal, std::optional<std::string> message);

    /** Records a path only while this node has an active goal. */
    [[nodiscard]] bool update_path(PathSnapshotPtr path);

    /** Returns whether initial-pose publication does not conflict with an active goal. */
    [[nodiscard]] bool initial_pose_allowed() const;

    /** Returns a value projection of state protected by the tracker mutex. */
    [[nodiscard]] NavigationState state() const;

private:
    [[nodiscard]] bool matches_current_goal(const GoalToken &token) const;

    mutable std::mutex mutex_;
    uint64_t generation_ = 0;
    NavigationStatus status_ = NavigationStatus::idle;
    std::optional<std::array<uint8_t, 16>> uuid_;
    std::optional<double> distance_remaining_;
    std::optional<std::string> message_;
    PathSnapshotPtr path_;
};
} // namespace robot_web_ui

#endif  // ROBOT_WEB_UI_NAVIGATION_TRACKER_H_
