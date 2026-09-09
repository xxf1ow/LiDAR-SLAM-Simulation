#include "robot_web_ui/navigation_tracker.h"

#include <cmath>
#include <utility>

namespace robot_web_ui
{
namespace
{
bool is_active(NavigationStatus status)
{
    return status == NavigationStatus::sending || status == NavigationStatus::navigating ||
           status == NavigationStatus::canceling;
}

bool is_terminal(NavigationStatus status)
{
    return status == NavigationStatus::succeeded || status == NavigationStatus::canceled ||
           status == NavigationStatus::failed;
}

bool has_path_points(const PathSnapshotPtr &path)
{
    return path && path->binary && path->binary->data.size() >= 8;
}
} // namespace

bool operator==(const GoalToken &left, const GoalToken &right)
{
    return left.generation == right.generation && left.uuid == right.uuid;
}

bool operator==(const NavigationState &left, const NavigationState &right)
{
    return left.status == right.status && left.distance_remaining == right.distance_remaining &&
           left.message == right.message && left.phase == right.phase && left.path == right.path;
}

NavigationTracker::NavigationTracker() = default;

uint64_t NavigationTracker::reserve_goal()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
    status_ = NavigationStatus::sending;
    uuid_.reset();
    distance_remaining_.reset();
    message_.reset();
    path_.reset();
    return generation_;
}

bool NavigationTracker::accept_goal(uint64_t generation, const std::array<uint8_t, 16> &uuid)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ != NavigationStatus::sending || generation != generation_)
        return false;
    uuid_ = uuid;
    status_ = NavigationStatus::navigating;
    message_.reset();
    return true;
}

bool NavigationTracker::fail_submission(uint64_t generation, std::string message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ != NavigationStatus::sending || generation != generation_)
        return false;
    status_ = NavigationStatus::failed;
    uuid_.reset();
    distance_remaining_.reset();
    message_ = std::move(message);
    path_.reset();
    return true;
}

bool NavigationTracker::update_feedback(const GoalToken &token, double distance_remaining)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ != NavigationStatus::navigating || !std::isfinite(distance_remaining) || !matches_current_goal(token))
        return false;
    distance_remaining_ = distance_remaining;
    return true;
}

bool NavigationTracker::begin_cancel(GoalToken *token)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (token == nullptr || status_ != NavigationStatus::navigating || !uuid_)
        return false;
    *token = GoalToken{generation_, *uuid_};
    status_ = NavigationStatus::canceling;
    message_.reset();
    return true;
}

bool NavigationTracker::reject_cancel(const GoalToken &token, std::string message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ != NavigationStatus::canceling || !matches_current_goal(token))
        return false;
    status_ = NavigationStatus::navigating;
    message_ = std::move(message);
    return true;
}

bool NavigationTracker::finish_goal(
    const GoalToken &token, NavigationStatus terminal, std::optional<std::string> message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if ((status_ != NavigationStatus::navigating && status_ != NavigationStatus::canceling) || !is_terminal(terminal) ||
        !matches_current_goal(token))
        return false;
    status_ = terminal;
    distance_remaining_.reset();
    message_ = std::move(message);
    path_.reset();
    return true;
}

bool NavigationTracker::update_path(PathSnapshotPtr path)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_active(status_))
        return false;
    path_ = std::move(path);
    return true;
}

bool NavigationTracker::initial_pose_allowed() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return !is_active(status_);
}

NavigationState NavigationTracker::state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return {
        status_,
        has_path_points(path_) ? distance_remaining_ : std::nullopt,
        message_,
        std::nullopt,
        path_,
    };
}

bool NavigationTracker::matches_current_goal(const GoalToken &token) const
{
    return uuid_ && token.generation == generation_ && token.uuid == *uuid_;
}
} // namespace robot_web_ui
