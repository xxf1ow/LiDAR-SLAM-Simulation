#include "robot_web_ui/web_ui_node.h"
#include "robot_web_ui/http_actions.h"
#include "robot_web_ui/manual_command.h"
#include "robot_web_ui/map_snapshot.h"
#include "robot_web_ui/navigation_request.h"
#include "robot_web_ui/navigation_tracker.h"
#include "robot_web_ui/parking_point_store.h"
#include "parking_lease.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <mutex>
#include <optional>
#include <utility>

namespace robot_web_ui
{
namespace
{
using Json = nlohmann::json;
using Navigate = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
using Trigger = std_srvs::srv::Trigger;
using Pose = std::array<double, 3>;
using detail::ParkingLease;

ApiReply error_reply(int status, const std::string &message)
{
    return {status, {{"error", message}}};
}

Json optional_string(const std::optional<std::string> &value)
{
    return value ? Json(*value) : Json(nullptr);
}

std::optional<double> quaternion_yaw(const geometry_msgs::msg::Quaternion &rotation)
{
    const double norm = std::hypot(std::hypot(rotation.x, rotation.y), std::hypot(rotation.z, rotation.w));
    if (!std::isfinite(norm) || norm == 0.0)
        return std::nullopt;
    const double x = rotation.x / norm, y = rotation.y / norm, z = rotation.z / norm, w = rotation.w / norm;
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

std::optional<Pose> transform_pose(const geometry_msgs::msg::TransformStamped &transform)
{
    const auto &position = transform.transform.translation;
    const auto yaw = quaternion_yaw(transform.transform.rotation);
    if (!yaw || !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z))
        return std::nullopt;
    return Pose{position.x, position.y, *yaw};
}

std::optional<GridInfo> grid_info(const nav_msgs::msg::OccupancyGrid &message)
{
    const auto &position = message.info.origin.position;
    const auto yaw = quaternion_yaw(message.info.origin.orientation);
    if (!yaw || !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !std::isfinite(message.info.resolution) || message.info.resolution <= 0.0)
        return std::nullopt;
    return GridInfo{message.info.width, message.info.height, message.info.resolution,
                    position.x, position.y, *yaw, message.header.frame_id};
}

Json grid_state(const GridSnapshotPtr &snapshot)
{
    if (!snapshot)
        return nullptr;
    const auto &info = snapshot->info;
    return {{"width", info.width}, {"height", info.height}, {"resolution", info.resolution},
            {"origin", {info.origin_x, info.origin_y, info.origin_yaw}}, {"frame_id", info.frame_id},
            {"revision", snapshot->binary->revision}, {"etag", snapshot->binary->etag}};
}

const char *status_name(NavigationStatus status)
{
    switch (status) {
        case NavigationStatus::idle: return "idle";
        case NavigationStatus::sending: return "sending";
        case NavigationStatus::navigating: return "navigating";
        case NavigationStatus::canceling: return "canceling";
        case NavigationStatus::succeeded: return "succeeded";
        case NavigationStatus::canceled: return "canceled";
        case NavigationStatus::failed: return "failed";
    }
    return "failed";
}

geometry_msgs::msg::PoseStamped pose_message(const NavigationPose &pose, const rclcpp::Time &stamp)
{
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = "map";
    message.header.stamp = stamp;
    message.pose.position.x = pose.x;
    message.pose.position.y = pose.y;
    message.pose.orientation.z = std::sin(pose.yaw / 2.0);
    message.pose.orientation.w = std::cos(pose.yaw / 2.0);
    return message;
}

Json parking_json(size_t number, const ParkingPoint &point)
{
    return {{"number", number}, {"name", point.name}, {"x", point.x}, {"y", point.y}, {"yaw", point.yaw}};
}

ApiReply parking_error(const std::error_code &error)
{
    using Errc = ParkingPointStore::Errc;
    int status = 503;
    if (error == ParkingPointStore::make_error_code(Errc::invalid_point)) status = 400;
    if (error == ParkingPointStore::make_error_code(Errc::duplicate_name)) status = 409;
    if (error == ParkingPointStore::make_error_code(Errc::not_found)) status = 404;
    return error_reply(status, error.message());
}

struct LocalLayer {
    GridSnapshotPtr grid;
    std::optional<Pose> affine;
    bool transform_available = false;
    std::optional<std::string> error;
};
} // namespace

detail::ParkingLease::ParkingLease(std::atomic_flag &busy) : busy_(busy), acquired_(!busy.test_and_set()) {}
detail::ParkingLease::~ParkingLease()
{
    if (acquired_)
        busy_.clear();
}
bool detail::ParkingLease::acquired() const { return acquired_; }

/*******************************************************************************************************
 * @brief ROS resources and request operations
 *******************************************************************************************************/
struct WebUiNode::Impl final : HttpActions {
    explicit Impl(WebUiNode &node);
    Json navigation_state() const override;
    Json assistant_state() const override;
    BinarySnapshotPtr navigation_asset(const std::string &name) const override;
    ApiReply manual_command(const std::string &direction, double speed_percent) override;
    ApiReply takeover_manual() override;
    ApiReply resume_automatic() override;
    ApiReply publish_initial_pose(const Json &payload) override;
    ApiReply send_navigation_goal(const Json &payload) override;
    ApiReply cancel_navigation() override;
    ApiReply list_parking_points() override;
    ApiReply save_parking_point(const std::string &name) override;
    ApiReply navigate_parking_point(const std::string &name) override;
    ApiReply delete_parking_point(const std::string &name) override;
    ApiReply call_mode_service(const rclcpp::Client<Trigger>::SharedPtr &client,
                               const std::string &action, const std::string &target);
    ApiReply conflict(const std::string &message) const;
    void ingest_localization(const tf2_msgs::msg::TFMessage &message);
    void ingest_grid(const nav_msgs::msg::OccupancyGrid &message, bool local);
    void ingest_plan(const nav_msgs::msg::Path &message);

    WebUiNode &node;
    double max_linear;
    double max_angular;
    bool navigation_enabled;
    GridSnapshotPtr static_map;
    std::optional<std::string> map_error;
    std::unique_ptr<ParkingPointStore> parking;
    std::optional<std::string> parking_error_message;
    std::atomic_flag parking_busy = ATOMIC_FLAG_INIT;
    // This mutex also excludes initial-pose publication from goal reservation.
    mutable std::mutex mutex;
    std::optional<std::string> gate_mode;
    std::optional<Pose> localization;
    std::optional<std::string> localization_error;
    GridSnapshotPtr global_grid;
    LocalLayer local_layer;
    NavigationTracker tracker;
    uint64_t generation = 0;
    GoalHandle::SharedPtr goal_handle;
    std::optional<std::string> path_error;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr manual_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr localization_subscription;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_subscription;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_subscription;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription;
    rclcpp::Client<Trigger>::SharedPtr takeover_client;
    rclcpp::Client<Trigger>::SharedPtr resume_client;
    rclcpp_action::Client<Navigate>::SharedPtr navigation_client;
};

WebUiNode::Impl::Impl(WebUiNode &owner) : node(owner)
{
    max_linear = node.declare_parameter("max_linear_speed", 0.5);
    max_angular = node.declare_parameter("max_angular_speed", 1.0);
    node.declare_parameter("host", std::string("0.0.0.0"));
    node.declare_parameter("port", 8080);
    const auto map_path = node.declare_parameter("map_yaml_path", std::string());
    navigation_enabled = node.declare_parameter("navigation_sources_enabled", false);
    auto loaded = load_nav2_pgm(map_path);
    if (loaded) static_map = *loaded;
    else map_error = loaded.error();
    auto store = ParkingPointStore::create(map_path);
    if (store) parking = std::move(*store);
    else parking_error_message = store.error().message();

    const auto retained = rclcpp::QoS(1).reliable().transient_local();
    const auto current = rclcpp::QoS(1).reliable().durability_volatile();
    manual_publisher = node.create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_manual", 10);
    initial_publisher = node.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", current);
    mode_subscription = node.create_subscription<std_msgs::msg::String>(
        "/cmd_vel_gate/mode", retained, [this](std_msgs::msg::String::ConstSharedPtr message) {
            if (message->data == "manual" || message->data == "automatic") {
                std::lock_guard<std::mutex> lock(mutex);
                gate_mode = message->data;
            }
        });
    takeover_client = node.create_client<Trigger>("/cmd_vel_gate/takeover_manual");
    resume_client = node.create_client<Trigger>("/cmd_vel_gate/resume_automatic");
    if (!navigation_enabled)
        return;
    localization_subscription = node.create_subscription<tf2_msgs::msg::TFMessage>(
        "/gicp_localization/localization_snapshot", current,
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) { ingest_localization(*message); });
    global_subscription = node.create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", retained,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) { ingest_grid(*message, false); });
    local_subscription = node.create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", retained,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) { ingest_grid(*message, true); });
    plan_subscription = node.create_subscription<nav_msgs::msg::Path>(
        "/plan", current, [this](nav_msgs::msg::Path::ConstSharedPtr message) { ingest_plan(*message); });
    navigation_client = rclcpp_action::create_client<Navigate>(&node, "/navigate_to_pose");
}

void WebUiNode::Impl::ingest_localization(const tf2_msgs::msg::TFMessage &message)
{
    std::optional<Pose> affine, pose;
    bool valid = message.transforms.size() == 2;
    for (const auto &transform : message.transforms) {
        if (transform.header.frame_id != "map" || transform.header.stamp != message.transforms.front().header.stamp)
            valid = false;
        if (transform.child_frame_id == "camera_init" && !affine) affine = transform_pose(transform);
        else if (transform.child_frame_id == "body" && !pose) pose = transform_pose(transform);
        else valid = false;
    }
    std::lock_guard<std::mutex> lock(mutex);
    if (!valid || !affine || !pose) {
        localization_error = "expected complete same-stamp map localization snapshot";
        local_layer.transform_available = false;
        local_layer.error = *localization_error;
        return;
    }
    localization = pose;
    localization_error.reset();
    local_layer.affine = affine;
    local_layer.transform_available = true;
    local_layer.error.reset();
}

void WebUiNode::Impl::ingest_grid(const nav_msgs::msg::OccupancyGrid &message, bool local)
{
    const auto info = grid_info(message);
    if (!info || info->frame_id != (local ? "camera_init" : "map"))
        return;
    if (std::any_of(message.data.begin(), message.data.end(), [](int8_t value) { return value < -1 || value > 100; }))
        return;
    GridSnapshotPtr previous;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (local && !local_layer.transform_available) {
            local_layer.error = "map transform unavailable";
            return;
        }
        previous = local ? local_layer.grid : global_grid;
    }
    auto candidate = update_grid_snapshot(previous, *info, message.data);
    if (!candidate)
        return;
    std::lock_guard<std::mutex> lock(mutex);
    if (local) {
        if (local_layer.transform_available)
            local_layer.grid = std::move(candidate);
    } else {
        global_grid = std::move(candidate);
    }
}

void WebUiNode::Impl::ingest_plan(const nav_msgs::msg::Path &message)
{
    uint64_t owner;
    PathSnapshotPtr previous;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (tracker.initial_pose_allowed())
            return;
        owner = generation;
        previous = tracker.state().path;
    }
    std::vector<std::pair<double, double>> points;
    points.reserve(message.poses.size());
    bool valid = message.header.frame_id == "map";
    for (const auto &pose : message.poses) {
        if (!std::isfinite(static_cast<float>(pose.pose.position.x)) ||
            !std::isfinite(static_cast<float>(pose.pose.position.y))) {
            valid = false;
            break;
        }
        points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    auto candidate = valid ? update_path_snapshot(previous, "map", points) : nullptr;
    std::lock_guard<std::mutex> lock(mutex);
    if (owner != generation || tracker.initial_pose_allowed())
        return;
    if (!candidate) {
        path_error = "expected map path";
        return;
    }
    if (tracker.update_path(std::move(candidate)))
        path_error.reset();
}

Json WebUiNode::Impl::navigation_state() const
{
    const bool action_ready = navigation_client && navigation_client->action_server_is_ready();
    const bool initial_ready = navigation_enabled && initial_publisher->get_subscription_count() > 0;
    GridSnapshotPtr global;
    LocalLayer local;
    std::optional<Pose> pose;
    std::optional<std::string> mode, pose_error, plan_error;
    NavigationState navigation;
    {
        std::lock_guard<std::mutex> lock(mutex);
        global = global_grid;
        local = local_layer;
        pose = localization;
        mode = gate_mode;
        pose_error = localization_error;
        plan_error = path_error;
        navigation = tracker.state();
    }
    Json local_state = grid_state(local.grid);
    if (local.grid || local.error) {
        if (local_state.is_null()) local_state = Json::object();
        local_state["map_from_source"] = local.affine ? Json(*local.affine) : Json(nullptr);
        local_state["transform_available"] = local.transform_available;
        local_state["transform_error"] = optional_string(local.error);
    }
    const auto &path = navigation.path;
    return {{"map_error", optional_string(map_error)}, {"localized", pose.has_value()},
            {"localization", pose ? Json{{"frame_id", "map"}, {"x", (*pose)[0]}, {"y", (*pose)[1]}, {"yaw", (*pose)[2]}} : Json(nullptr)},
            {"localization_error", optional_string(pose_error)}, {"path_error", optional_string(plan_error)},
            {"gate_mode", optional_string(mode)},
            {"navigation", {{"initial_pose_ready", initial_ready}, {"action_server_ready", action_ready},
                            {"goal_status", status_name(navigation.status)},
                            {"cancel_available", navigation.status == NavigationStatus::navigating}, {"phase", nullptr},
                            {"distance_remaining", navigation.distance_remaining ? Json(*navigation.distance_remaining) : Json(nullptr)},
                            {"message", optional_string(navigation.message)}}},
            {"layers", {{"static", grid_state(static_map)}, {"global_costmap", grid_state(global)},
                        {"local_costmap", std::move(local_state)},
                        {"path", path ? Json{{"frame_id", path->frame_id}, {"revision", path->binary->revision},
                                              {"etag", path->binary->etag}} : Json(nullptr)}}}};
}

Json WebUiNode::Impl::assistant_state() const
{
    const auto state = navigation_state();
    Json issue = nullptr;
    if (state["layers"]["static"].is_null()) issue = "map_unavailable";
    else if (state["localized"] != true || !state["localization_error"].is_null()) issue = "localization_unavailable";
    else if (state["navigation"]["action_server_ready"] != true) issue = "navigation_unavailable";
    return {{"mode", state["gate_mode"].is_null() ? Json("unknown") : state["gate_mode"]},
            {"navigation", state["navigation"]["goal_status"]},
            {"distance_m", state["navigation"]["distance_remaining"]}, {"issue", issue}};
}

BinarySnapshotPtr WebUiNode::Impl::navigation_asset(const std::string &name) const
{
    std::lock_guard<std::mutex> lock(mutex);
    if (name == "static") return static_map ? static_map->binary : nullptr;
    if (name == "global_costmap") return global_grid ? global_grid->binary : nullptr;
    if (name == "local_costmap") return local_layer.grid ? local_layer.grid->binary : nullptr;
    if (name == "path") {
        const auto path = tracker.state().path;
        return path ? path->binary : nullptr;
    }
    return nullptr;
}

ApiReply WebUiNode::Impl::conflict(const std::string &message) const
{
    return {409, {{"error", message}, {"mode", optional_string(gate_mode)}}};
}

ApiReply WebUiNode::Impl::manual_command(const std::string &direction, double speed_percent)
{
    const auto values = command_values(direction, speed_percent, max_linear, max_angular);
    if (!values)
        return error_reply(400, values.error());
    geometry_msgs::msg::TwistStamped message;
    message.header.frame_id = "base_link";
    message.header.stamp = node.now();
    message.twist.linear.x = values->linear_x;
    message.twist.angular.z = values->angular_z;
    std::lock_guard<std::mutex> lock(mutex);
    if ((values->linear_x != 0 || values->angular_z != 0) && gate_mode != "manual")
        return conflict("manual control is not active");
    try {
        manual_publisher->publish(message);
    } catch (const rclcpp::exceptions::RCLError &error) {
        return error_reply(503, error.what());
    }
    return {200, {{"ok", true}, {"mode", optional_string(gate_mode)}}};
}

ApiReply WebUiNode::Impl::call_mode_service(const rclcpp::Client<Trigger>::SharedPtr &client,
                                          const std::string &action, const std::string &target)
{
    if (!client->service_is_ready())
        return error_reply(503, action + " service unavailable");
    auto completion = std::make_shared<std::promise<Trigger::Response::SharedPtr>>();
    auto outcome = completion->get_future();
    int64_t request_id;
    try {
        auto request = client->async_send_request(std::make_shared<Trigger::Request>(),
            [completion](rclcpp::Client<Trigger>::SharedFuture future) { completion->set_value(future.get()); });
        request_id = request.request_id;
    } catch (const rclcpp::exceptions::RCLError &error) {
        return error_reply(503, error.what());
    }
    if (outcome.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
        client->remove_pending_request(request_id);
        std::lock_guard<std::mutex> lock(mutex);
        if (gate_mode == target)
            return {200, {{"ok", true}, {"mode", target}}};
        return {202, {{"ok", false}, {"pending", true}, {"error", action + " service timed out"},
                      {"mode", optional_string(gate_mode)}}};
    }
    const auto response = outcome.get();
    if (!response || !response->success)
        return error_reply(503, response && !response->message.empty() ? response->message : action + " request rejected");
    std::lock_guard<std::mutex> lock(mutex);
    gate_mode = target;
    return {200, {{"ok", true}, {"mode", target}}};
}

ApiReply WebUiNode::Impl::takeover_manual() { return call_mode_service(takeover_client, "manual takeover", "manual"); }
ApiReply WebUiNode::Impl::resume_automatic() { return call_mode_service(resume_client, "automatic resume", "automatic"); }

ApiReply WebUiNode::Impl::publish_initial_pose(const Json &payload)
{
    if (!navigation_enabled) return error_reply(503, "navigation sources unavailable");
    if (!static_map) return error_reply(503, "static map unavailable");
    std::lock_guard<std::mutex> lock(mutex);
    const auto pose = parse_navigation_pose(payload, static_map->info, static_map->binary->revision);
    if (!pose)
        return error_reply(pose.error().kind == PoseErrorKind::map_revision_conflict ? 409 : 400, pose.error().message);
    if (initial_publisher->get_subscription_count() == 0)
        return error_reply(503, "initial pose subscriber unavailable");
    if (!tracker.initial_pose_allowed()) return conflict("navigation goal is active");
    const auto stamped = pose_message(*pose, node.now());
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header = stamped.header;
    message.pose.pose = stamped.pose;
    try {
        initial_publisher->publish(message);
    } catch (const rclcpp::exceptions::RCLError &error) {
        return error_reply(503, error.what());
    }
    return {200, {{"ok", true}}};
}

ApiReply WebUiNode::Impl::send_navigation_goal(const Json &payload)
{
    if (!navigation_enabled) return error_reply(503, "navigation sources unavailable");
    if (!static_map) return error_reply(503, "static map unavailable");
    const auto pose = parse_navigation_pose(payload, static_map->info, static_map->binary->revision);
    if (!pose)
        return error_reply(pose.error().kind == PoseErrorKind::map_revision_conflict ? 409 : 400, pose.error().message);
    if (!navigation_client->action_server_is_ready()) return error_reply(503, "navigation action server unavailable");
    Navigate::Goal goal;
    goal.pose = pose_message(*pose, node.now());
    uint64_t owner;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (gate_mode != "automatic") return conflict("automatic control is not active");
        if (!localization) return conflict("robot is not localized");
        if (!tracker.initial_pose_allowed()) return conflict("navigation goal is active");
        owner = generation = tracker.reserve_goal();
        goal_handle.reset();
        path_error.reset();
    }
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback = [this, owner](GoalHandle::SharedPtr handle) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!handle) {
            (void)tracker.fail_submission(owner, "navigation goal rejected");
            return;
        }
        if (tracker.accept_goal(owner, handle->get_goal_id()))
            goal_handle = std::move(handle);
    };
    options.feedback_callback = [this, owner](GoalHandle::SharedPtr handle, const std::shared_ptr<const Navigate::Feedback> feedback) {
        (void)tracker.update_feedback({owner, handle->get_goal_id()}, feedback->distance_remaining);
    };
    options.result_callback = [this, owner](const GoalHandle::WrappedResult &result) {
        NavigationStatus status = NavigationStatus::failed;
        std::optional<std::string> message = "navigation goal aborted";
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) { status = NavigationStatus::succeeded; message.reset(); }
        else if (result.code == rclcpp_action::ResultCode::CANCELED) { status = NavigationStatus::canceled; message.reset(); }
        std::lock_guard<std::mutex> lock(mutex);
        if (tracker.finish_goal({owner, result.goal_id}, status, message)) {
            goal_handle.reset();
            path_error.reset();
        }
    };
    try {
        (void)navigation_client->async_send_goal(goal, options);
    } catch (const rclcpp::exceptions::RCLError &error) {
        (void)tracker.fail_submission(owner, error.what());
        return error_reply(503, error.what());
    }
    return {202, {{"ok", true}, {"goal_status", "sending"}}};
}

ApiReply WebUiNode::Impl::cancel_navigation()
{
    if (!navigation_enabled) return error_reply(503, "navigation sources unavailable");
    GoalToken token;
    GoalHandle::SharedPtr handle;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!goal_handle || !tracker.begin_cancel(&token)) return conflict("navigation goal is not active");
        handle = goal_handle;
    }
    try {
        (void)navigation_client->async_cancel_goal(handle, [this, token](auto response) {
            const bool accepted = std::any_of(response->goals_canceling.begin(), response->goals_canceling.end(),
                [&token](const auto &goal) { return goal.goal_id.uuid == token.uuid; });
            if (!accepted)
                (void)tracker.reject_cancel(token, "navigation cancel rejected");
        });
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &error) {
        (void)tracker.reject_cancel(token, error.what());
        return error_reply(503, error.what());
    } catch (const rclcpp::exceptions::RCLError &error) {
        (void)tracker.reject_cancel(token, error.what());
        return error_reply(503, error.what());
    }
    return {202, {{"ok", true}, {"goal_status", "canceling"}}};
}

ApiReply WebUiNode::Impl::list_parking_points()
{
    ParkingLease lease(parking_busy);
    if (!lease.acquired()) return error_reply(503, "parking-point store busy");
    if (!parking) return error_reply(503, parking_error_message.value_or("parking-point store unavailable"));
    Json points = Json::array();
    for (const auto &point : parking->list()) points.push_back(parking_json(points.size() + 1, point));
    return {200, {{"points", points}}};
}

ApiReply WebUiNode::Impl::save_parking_point(const std::string &name)
{
    if (!navigation_enabled) return error_reply(503, "navigation sources unavailable");
    ParkingLease lease(parking_busy);
    if (!lease.acquired()) return error_reply(503, "parking-point store busy");
    if (!parking) return error_reply(503, parking_error_message.value_or("parking-point store unavailable"));
    Pose pose;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (gate_mode != "automatic") return error_reply(409, "automatic control is not active");
        if (!localization) return error_reply(503, "robot is not localized");
        pose = *localization;
    }
    const auto saved = parking->save({name, pose[0], pose[1], pose[2]});
    if (!saved) return parking_error(saved.error());
    const auto points = parking->list();
    return {201, {{"point", parking_json(points.size(), points.back())}}};
}

ApiReply WebUiNode::Impl::navigate_parking_point(const std::string &name)
{
    if (!navigation_enabled) return error_reply(503, "navigation sources unavailable");
    std::optional<ParkingPoint> point;
    {
        ParkingLease lease(parking_busy);
        if (!lease.acquired()) return error_reply(503, "parking-point store busy");
        if (!parking) return error_reply(503, parking_error_message.value_or("parking-point store unavailable"));
        const auto found = parking->get(name);
        if (!found) return parking_error(found.error());
        point = *found;
    }
    if (!static_map) return error_reply(503, "static map unavailable");
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!localization) return error_reply(503, "robot is not localized");
    }
    auto reply = send_navigation_goal({{"x", point->x}, {"y", point->y}, {"yaw", point->yaw},
                                      {"map_revision", static_map->binary->revision}});
    if (reply.status != 202) { reply.body.erase("mode"); return reply; }
    return {202, {{"name", point->name}, {"status", "accepted"}}};
}

ApiReply WebUiNode::Impl::delete_parking_point(const std::string &name)
{
    ParkingLease lease(parking_busy);
    if (!lease.acquired()) return error_reply(503, "parking-point store busy");
    if (!parking) return error_reply(503, parking_error_message.value_or("parking-point store unavailable"));
    const auto point = parking->get(name);
    if (!point) return parking_error(point.error());
    const auto erased = parking->erase(name);
    if (!erased) return parking_error(erased.error());
    return {200, {{"deleted", point->name}}};
}

WebUiNode::WebUiNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("robot_web_ui", options), impl_(std::make_unique<Impl>(*this)) {}
WebUiNode::~WebUiNode() = default;
HttpActions &WebUiNode::http_actions() { return *impl_; }
} // namespace robot_web_ui
