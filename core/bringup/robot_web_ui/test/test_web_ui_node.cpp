#include "robot_web_ui/web_ui_node.h"
#include "robot_web_ui/http_actions.h"
#include "robot_web_ui/parking_point_store.h"
#include "../src/parking_lease.h"

#include <gtest/gtest.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <functional>
#include <limits>
#include <optional>
#include <thread>

namespace robot_web_ui
{
namespace
{
using namespace std::chrono_literals;
using Navigate = nav2_msgs::action::NavigateToPose;
using ServerGoal = rclcpp_action::ServerGoalHandle<Navigate>;

class WebUiNodeTest : public testing::Test {
protected:
    static void SetUpTestSuite();
    static void TearDownTestSuite();
    void SetUp() override;
    void TearDown() override;
    void start(bool navigation = true);
    bool until(const std::function<bool()> &predicate);
    void spin_for(std::chrono::milliseconds duration);
    tf2_msgs::msg::TFMessage localization(double x = 1.0);
    nav_msgs::msg::OccupancyGrid grid(const std::string &frame);
    void publish_localization();
    void publish_mode(const std::string &mode);

    std::filesystem::path directory;
    std::shared_ptr<WebUiNode> node;
    std::shared_ptr<rclcpp::Node> peer;
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr localization_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher;
};

void WebUiNodeTest::SetUpTestSuite() { rclcpp::init(0, nullptr); }
void WebUiNodeTest::TearDownTestSuite() { rclcpp::shutdown(); }

void WebUiNodeTest::SetUp()
{
    if (!rclcpp::ok())
        rclcpp::init(0, nullptr);
    directory = std::filesystem::temp_directory_path() /
                ("web_ui_node_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(directory);
    std::ofstream(directory / "map.yaml") << "image: map.pgm\nresolution: 1.0\norigin: [0, 0, 0]\n"
                                           "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n";
    std::ofstream(directory / "map.pgm", std::ios::binary) << "P5\n2 2\n255\n\xff\xff\xff\xff";
}

void WebUiNodeTest::TearDown()
{
    executor.cancel();
    if (node)
        executor.remove_node(node);
    if (peer)
        executor.remove_node(peer);
    node.reset();
    peer.reset();
    std::filesystem::remove_all(directory);
}

void WebUiNodeTest::start(bool navigation)
{
    auto options = rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("navigation_sources_enabled", navigation),
        rclcpp::Parameter("map_yaml_path", (directory / "map.yaml").string()),
        rclcpp::Parameter("max_linear_speed", 0.8), rclcpp::Parameter("max_angular_speed", 1.6)});
    node = std::make_shared<WebUiNode>(options);
    peer = std::make_shared<rclcpp::Node>("web_ui_test_peer");
    executor.add_node(node);
    executor.add_node(peer);
    localization_publisher = peer->create_publisher<tf2_msgs::msg::TFMessage>(
        "/gicp_localization/localization_snapshot", rclcpp::QoS(1).reliable());
    mode_publisher = peer->create_publisher<std_msgs::msg::String>(
        "/cmd_vel_gate/mode", rclcpp::QoS(1).reliable().transient_local());
}

bool WebUiNodeTest::until(const std::function<bool()> &predicate)
{
    const auto end = std::chrono::steady_clock::now() + 3s;
    do {
        executor.spin_some();
        if (predicate())
            return true;
        std::this_thread::sleep_for(5ms);
    } while (std::chrono::steady_clock::now() < end);
    return false;
}

void WebUiNodeTest::spin_for(std::chrono::milliseconds duration)
{
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
        executor.spin_some();
        std::this_thread::sleep_for(5ms);
    }
}

tf2_msgs::msg::TFMessage WebUiNodeTest::localization(double x)
{
    tf2_msgs::msg::TFMessage result;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.header.stamp.sec = 42;
    transform.child_frame_id = "camera_init";
    transform.transform.translation.x = x;
    transform.transform.rotation.w = 1.0;
    result.transforms.push_back(transform);
    transform.child_frame_id = "body";
    transform.transform.translation.x = x + 0.5;
    transform.transform.translation.y = 0.5;
    transform.transform.rotation.z = std::sqrt(0.5);
    transform.transform.rotation.w = std::sqrt(0.5);
    result.transforms.push_back(transform);
    return result;
}

nav_msgs::msg::OccupancyGrid WebUiNodeTest::grid(const std::string &frame)
{
    nav_msgs::msg::OccupancyGrid result;
    result.header.frame_id = frame;
    result.info.width = 2;
    result.info.height = 2;
    result.info.resolution = 0.5;
    result.info.origin.position.x = -1.0;
    result.info.origin.orientation.w = 1.0;
    result.data = {0, 100, -1, 20};
    return result;
}

void WebUiNodeTest::publish_localization()
{
    ASSERT_TRUE(until([&] { return localization_publisher->get_subscription_count() == 1; }));
    localization_publisher->publish(localization());
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_state()["localized"] == true; }));
}

void WebUiNodeTest::publish_mode(const std::string &mode)
{
    ASSERT_TRUE(until([&] { return mode_publisher->get_subscription_count() == 1; }));
    std_msgs::msg::String message;
    message.data = mode;
    mode_publisher->publish(message);
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_state()["gate_mode"] == mode; }));
}

TEST_F(WebUiNodeTest, NavigationModeCreatesOnlyItsSpecifiedRosInterfaces)
{
    start();
    const auto subscriptions = peer->get_node_graph_interface()->get_subscriber_names_and_types_by_node("robot_web_ui", "/");
    for (const std::string topic : {"/gicp_localization/localization_snapshot", "/global_costmap/costmap",
                                   "/local_costmap/costmap", "/plan", "/cmd_vel_gate/mode"})
        EXPECT_EQ(subscriptions.count(topic), 1U) << topic;
    for (const std::string topic : {"/tf", "/tf_static", "/base_controller/odom", "/localization", "/behavior_tree_log"})
        EXPECT_EQ(subscriptions.count(topic), 0U) << topic;
    EXPECT_EQ(node->count_publishers("/cmd_vel_manual"), 1U);
    EXPECT_EQ(node->count_publishers("/initialpose"), 1U);
    const auto clients = peer->get_node_graph_interface()->get_client_names_and_types_by_node("robot_web_ui", "/");
    EXPECT_EQ(clients.count("/cmd_vel_gate/takeover_manual"), 1U);
    EXPECT_EQ(clients.count("/cmd_vel_gate/resume_automatic"), 1U);
    EXPECT_EQ(clients.count("/navigate_to_pose/_action/send_goal"), 1U);
    const auto infos = peer->get_subscriptions_info_by_topic("/local_costmap/costmap");
    ASSERT_EQ(infos.size(), 1U);
    EXPECT_EQ(infos[0].qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
    EXPECT_EQ(infos[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
}

TEST_F(WebUiNodeTest, MappingModeRetainsManualControlAndDisablesNavigation)
{
    start(false);
    const auto subscriptions = peer->get_node_graph_interface()->get_subscriber_names_and_types_by_node("robot_web_ui", "/");
    for (const std::string topic : {"/gicp_localization/localization_snapshot", "/global_costmap/costmap",
                                   "/local_costmap/costmap", "/plan", "/tf", "/tf_static", "/base_controller/odom",
                                   "/localization", "/behavior_tree_log"})
        EXPECT_EQ(subscriptions.count(topic), 0U) << topic;
    EXPECT_EQ(subscriptions.count("/cmd_vel_gate/mode"), 1U);
    const auto clients = peer->get_node_graph_interface()->get_client_names_and_types_by_node("robot_web_ui", "/");
    EXPECT_EQ(clients.count("/navigate_to_pose/_action/send_goal"), 0U);
    EXPECT_EQ(clients.count("/cmd_vel_gate/takeover_manual"), 1U);
    EXPECT_EQ(clients.count("/cmd_vel_gate/resume_automatic"), 1U);
    EXPECT_EQ(node->count_publishers("/initialpose"), 1U);
    EXPECT_EQ(node->http_actions().send_navigation_goal({}).status, 503);
    EXPECT_EQ(node->http_actions().publish_initial_pose({}).status, 503);
    EXPECT_EQ(node->http_actions().cancel_navigation().status, 503);

    std::optional<geometry_msgs::msg::TwistStamped> received;
    auto subscription = peer->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_manual", 10, [&](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) { received = *message; });
    publish_mode("automatic");
    EXPECT_EQ(node->http_actions().manual_command("forward", 50).status, 409);
    publish_mode("manual");
    EXPECT_EQ(node->http_actions().manual_command("forward", 50).status, 200);
    ASSERT_TRUE(until([&] { return received.has_value(); }));
    EXPECT_DOUBLE_EQ(received->twist.linear.x, 0.4);
    EXPECT_EQ(received->header.frame_id, "base_link");
    EXPECT_FALSE(node->http_actions().navigation_state().contains("motion"));
    EXPECT_TRUE(node->http_actions().navigation_state()["navigation"]["phase"].is_null());
}

TEST_F(WebUiNodeTest, LocalizationAndLocalAffineRemainAtomicWithoutRebuildingGrid)
{
    start();
    auto local = peer->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", rclcpp::QoS(1).reliable().transient_local());
    ASSERT_TRUE(until([&] { return local->get_subscription_count() == 1; }));
    local->publish(grid("camera_init"));
    spin_for(50ms);
    EXPECT_EQ(node->http_actions().navigation_asset("local_costmap"), nullptr);
    publish_localization();
    local->publish(grid("camera_init"));
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_asset("local_costmap") != nullptr; }));
    const auto binary = node->http_actions().navigation_asset("local_costmap");
    EXPECT_EQ(binary->data, std::vector<uint8_t>({0, 100, 255, 20}));
    localization_publisher->publish(localization(4.0));
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_state()["localization"]["x"] == 4.5; }));
    auto state = node->http_actions().navigation_state();
    EXPECT_DOUBLE_EQ(state["localization"]["yaw"], std::acos(-1.0) / 2.0);
    EXPECT_EQ(state["layers"]["local_costmap"]["map_from_source"], nlohmann::json::array({4.0, 0.0, 0.0}));
    EXPECT_EQ(state["layers"]["local_costmap"]["transform_available"], true);
    EXPECT_EQ(node->http_actions().navigation_asset("local_costmap"), binary);
    const auto valid_pose = state["localization"];
    const auto valid_affine = state["layers"]["local_costmap"]["map_from_source"];
    for (int defect = 0; defect < 6; ++defect) {
        auto bad = localization(9.0);
        if (defect == 0) bad.transforms.pop_back();
        if (defect == 1) bad.transforms[0].header.frame_id = "odom";
        if (defect == 2) bad.transforms[1].header.stamp.sec++;
        if (defect == 3) bad.transforms[1].transform.translation.z = std::numeric_limits<double>::infinity();
        if (defect == 4) bad.transforms[0].transform.rotation.w = 0.0;
        if (defect == 5) bad.transforms[0].transform.rotation.z = std::numeric_limits<double>::quiet_NaN();
        localization_publisher->publish(bad);
        spin_for(30ms);
        state = node->http_actions().navigation_state();
        EXPECT_EQ(state["localization"], valid_pose);
        EXPECT_EQ(state["layers"]["local_costmap"]["map_from_source"], valid_affine);
        EXPECT_FALSE(state["localization_error"].is_null());
        EXPECT_EQ(state["layers"]["local_costmap"]["transform_available"], false);
        EXPECT_EQ(node->http_actions().navigation_asset("local_costmap"), binary);
    }
    auto changed = grid("camera_init");
    changed.data[0] = 50;
    local->publish(changed);
    spin_for(30ms);
    EXPECT_EQ(node->http_actions().navigation_asset("local_costmap"), binary);
}

TEST_F(WebUiNodeTest, GlobalGridPreservesUnknownCellsAndIdlePlansAreIgnored)
{
    start();
    auto global = peer->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", rclcpp::QoS(1).reliable().transient_local());
    auto plan = peer->create_publisher<nav_msgs::msg::Path>("/plan", rclcpp::QoS(1).reliable());
    ASSERT_TRUE(until([&] { return global->get_subscription_count() == 1 && plan->get_subscription_count() == 1; }));
    global->publish(grid("map"));
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_asset("global_costmap") != nullptr; }));
    const auto binary = node->http_actions().navigation_asset("global_costmap");
    EXPECT_EQ(binary->data, std::vector<uint8_t>({0, 100, 255, 20}));
    EXPECT_EQ(node->http_actions().navigation_state()["layers"]["global_costmap"]["resolution"], 0.5);
    global->publish(grid("map"));
    nav_msgs::msg::Path message;
    message.header.frame_id = "map";
    message.poses.resize(1);
    message.poses[0].pose.position.x = 1.0;
    plan->publish(message);
    spin_for(50ms);
    EXPECT_EQ(node->http_actions().navigation_asset("global_costmap"), binary);
    EXPECT_EQ(node->http_actions().navigation_asset("path"), nullptr);
    auto bad = grid("map");
    bad.data[0] = -2;
    global->publish(bad);
    spin_for(30ms);
    EXPECT_EQ(node->http_actions().navigation_asset("global_costmap"), binary);
    for (const float resolution : {0.0F, -1.0F, std::numeric_limits<float>::infinity()}) {
        bad = grid("map");
        bad.info.resolution = resolution;
        global->publish(bad);
        spin_for(30ms);
        EXPECT_EQ(node->http_actions().navigation_asset("global_costmap"), binary);
    }
}

TEST_F(WebUiNodeTest, GoalLifecycleAndInitialPoseUseRealRosPeers)
{
    start();
    std::shared_ptr<ServerGoal> goal;
    auto server = rclcpp_action::create_server<Navigate>(
        peer, "/navigate_to_pose",
        [](const auto &, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
        [&](auto handle) { goal = handle; });
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> initial;
    auto initial_sub = peer->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1, [&](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose) { initial = *pose; });
    auto plan = peer->create_publisher<nav_msgs::msg::Path>("/plan", 1);
    publish_localization();
    publish_mode("automatic");
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_state()["navigation"]["action_server_ready"] == true; }));
    const nlohmann::json pose{{"x", 1.0}, {"y", 1.0}, {"yaw", 0.5}, {"map_revision", 1}};
    EXPECT_EQ(node->http_actions().publish_initial_pose(pose).status, 200);
    ASSERT_TRUE(until([&] { return initial.has_value(); }));
    EXPECT_DOUBLE_EQ(initial->pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(initial->pose.pose.orientation.z, std::sin(0.25));
    EXPECT_EQ(node->http_actions().send_navigation_goal(pose).status, 202);
    EXPECT_EQ(node->http_actions().publish_initial_pose(pose).status, 409);
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_state()["navigation"]["goal_status"] == "navigating"; }));
    ASSERT_TRUE(goal);
    EXPECT_EQ(goal->get_goal()->pose.header.frame_id, "map");
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.poses.resize(1);
    path.poses[0].pose.position.x = 1.0;
    path.poses[0].pose.position.y = 1.5;
    plan->publish(path);
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_asset("path") != nullptr; }));
    auto feedback = std::make_shared<Navigate::Feedback>();
    feedback->distance_remaining = 2.5;
    goal->publish_feedback(feedback);
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_state()["navigation"]["distance_remaining"] == 2.5; }));
    const auto valid_path = node->http_actions().navigation_asset("path");
    path.poses[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
    plan->publish(path);
    spin_for(30ms);
    EXPECT_EQ(node->http_actions().navigation_asset("path"), valid_path);
    EXPECT_FALSE(node->http_actions().navigation_state()["path_error"].is_null());
    EXPECT_EQ(node->http_actions().cancel_navigation().status, 202);
    ASSERT_TRUE(until([&] { return goal->is_canceling(); }));
    goal->canceled(std::make_shared<Navigate::Result>());
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_state()["navigation"]["goal_status"] == "canceled"; }));
    EXPECT_EQ(node->http_actions().navigation_asset("path"), nullptr);
}

TEST_F(WebUiNodeTest, ModeTimeoutDoesNotBorrowRequestStackAndManualCallsStayResponsive)
{
    start(false);
    std::promise<void> entered;
    std::promise<void> release;
    auto released = release.get_future();
    auto service = peer->create_service<std_srvs::srv::Trigger>(
        "/cmd_vel_gate/takeover_manual", [&](std_srvs::srv::Trigger::Request::SharedPtr,
                                             std_srvs::srv::Trigger::Response::SharedPtr response) {
            entered.set_value();
            released.wait();
            response->success = true;
        });
    ASSERT_TRUE(until([&] { return node->get_service_names_and_types().count("/cmd_vel_gate/takeover_manual") == 1; }));
    std::thread spinner([&] { executor.spin(); });
    auto request = std::async(std::launch::async, [&] { return node->http_actions().takeover_manual(); });
    EXPECT_EQ(entered.get_future().wait_for(3s), std::future_status::ready);
    EXPECT_EQ(request.wait_for(1500ms), std::future_status::ready);
    const auto reply = request.get();
    EXPECT_EQ(reply.status, 202);
    EXPECT_EQ(reply.body["pending"], true);
    EXPECT_EQ(node->http_actions().manual_command("stop", 0).status, 200);
    release.set_value();
    std::this_thread::sleep_for(50ms);
    executor.cancel();
    spinner.join();
}

TEST_F(WebUiNodeTest, RejectedGoalReleasesInitialPoseExclusion)
{
    start();
    auto server = rclcpp_action::create_server<Navigate>(
        peer, "/navigate_to_pose",
        [](const auto &, auto) { return rclcpp_action::GoalResponse::REJECT; },
        [](auto) { return rclcpp_action::CancelResponse::REJECT; }, [](auto) {});
    publish_localization();
    publish_mode("automatic");
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_state()["navigation"]["action_server_ready"] == true; }));
    EXPECT_EQ(node->http_actions().send_navigation_goal({{"x", 1}, {"y", 1}, {"yaw", 0}, {"map_revision", 1}}).status, 202);
    ASSERT_TRUE(until([&] { return node->http_actions().navigation_state()["navigation"]["goal_status"] == "failed"; }));
    const auto state = node->http_actions().navigation_state();
    EXPECT_FALSE(state["navigation"]["message"].is_null());
    EXPECT_FALSE(state["navigation"]["cancel_available"]);
    EXPECT_EQ(node->http_actions().cancel_navigation().status, 409);
    EXPECT_EQ(node->http_actions().send_navigation_goal({{"x", 1}, {"y", 1}, {"yaw", 0}, {"map_revision", 1}}).status, 202);
}

TEST_F(WebUiNodeTest, ParkingOperationsPersistLocalizedPoseAndReleaseAfterErrors)
{
    start();
    EXPECT_EQ(node->http_actions().save_parking_point("Dock").status, 409);
    publish_mode("automatic");
    EXPECT_EQ(node->http_actions().save_parking_point("Dock").status, 503);
    publish_localization();
    const auto saved = node->http_actions().save_parking_point("  Dock  ");
    EXPECT_EQ(saved.status, 201);
    EXPECT_EQ(saved.body["point"]["number"], 1);
    EXPECT_EQ(saved.body["point"]["name"], "Dock");
    EXPECT_EQ(saved.body["point"]["x"], 1.5);
    EXPECT_EQ(node->http_actions().save_parking_point("Dock").status, 409);
    EXPECT_EQ(node->http_actions().list_parking_points().body["points"].size(), 1U);
    auto reopened = ParkingPointStore::create(directory / "map.yaml");
    ASSERT_TRUE(reopened);
    EXPECT_EQ((*reopened)->get("Dock")->x, 1.5);
    EXPECT_EQ(node->http_actions().delete_parking_point(" missing ").status, 404);
    EXPECT_EQ(node->http_actions().delete_parking_point(" Dock ").body["deleted"], "Dock");
    EXPECT_TRUE(node->http_actions().list_parking_points().body["points"].empty());
}

TEST_F(WebUiNodeTest, ParkingLeaseRejectsConcurrentStoreAccessAndRecoversAfterRelease)
{
    auto store = ParkingPointStore::create(directory / "map.yaml");
    ASSERT_TRUE(store);
    std::atomic_flag busy = ATOMIC_FLAG_INIT;
    {
        detail::ParkingLease first(busy);
        ASSERT_TRUE(first.acquired());
        auto second = std::async(std::launch::async, [&] {
            detail::ParkingLease lease(busy);
            if (!lease.acquired())
                return false;
            return static_cast<bool>((*store)->save({"second", 0, 0, 0}));
        });
        EXPECT_EQ(second.wait_for(100ms), std::future_status::ready);
        EXPECT_FALSE(second.get());
        EXPECT_TRUE((*store)->list().empty());
        ASSERT_TRUE((*store)->save({"first", 0, 0, 0}));
    }
    detail::ParkingLease third(busy);
    ASSERT_TRUE(third.acquired());
    ASSERT_TRUE((*store)->save({"third", 1, 1, 0}));
    EXPECT_EQ((*store)->list().size(), 2U);
}

TEST_F(WebUiNodeTest, ModeServiceSuccessAndRejectionReturnTypedReplies)
{
    start(false);
    bool accept = true;
    auto service = peer->create_service<std_srvs::srv::Trigger>(
        "/cmd_vel_gate/resume_automatic", [&](std_srvs::srv::Trigger::Request::SharedPtr,
                                              std_srvs::srv::Trigger::Response::SharedPtr response) {
            response->success = accept;
            response->message = "rejected by gate";
        });
    ASSERT_TRUE(until([&] { return node->get_service_names_and_types().count("/cmd_vel_gate/resume_automatic") == 1; }));
    auto request = std::async(std::launch::async, [&] { return node->http_actions().resume_automatic(); });
    ASSERT_TRUE(until([&] { return request.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(request.get().status, 200);
    EXPECT_EQ(node->http_actions().navigation_state()["gate_mode"], "automatic");
    accept = false;
    request = std::async(std::launch::async, [&] { return node->http_actions().resume_automatic(); });
    ASSERT_TRUE(until([&] { return request.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(request.get().status, 503);
}
} // namespace
} // namespace robot_web_ui
