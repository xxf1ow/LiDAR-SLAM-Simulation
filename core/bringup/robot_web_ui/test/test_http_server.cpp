#include "robot_web_ui/http_server.h"
#include "robot_web_ui/http_actions.h"
#include "robot_web_ui/manual_command.h"
#include "robot_web_ui/navigation_request.h"

#include <gtest/gtest.h>
#include <httplib.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>

namespace robot_web_ui
{
namespace
{
using Json = nlohmann::json;
using namespace std::chrono_literals;

class FakeActions final : public HttpActions {
public:
    Json navigation_state() const override;
    Json assistant_state() const override;
    Json tracking_state() const override;
    ApiReply publish_tracking_target(const Json &payload) override;
    BinarySnapshotPtr navigation_asset(const std::string &name) const override;
    ApiReply manual_command(const std::string &direction, double speed) override;
    ApiReply takeover_manual() override;
    ApiReply resume_automatic() override;
    ApiReply publish_initial_pose(const Json &payload) override;
    ApiReply send_navigation_goal(const Json &payload) override;
    ApiReply cancel_navigation() override;
    ApiReply list_parking_points() override;
    ApiReply save_parking_point(const std::string &name) override;
    ApiReply navigate_parking_point(const std::string &name) override;
    ApiReply delete_parking_point(const std::string &name) override;

    std::atomic<int> manual_status{200};
    bool block_mode = false;
    std::promise<void> mode_started;
    std::promise<void> release_mode;
    Json points = Json::array();
    int tracking_targets = 0;
    Json last_tracking_target;
};

Json FakeActions::navigation_state() const { return {{"localized", true}, {"navigation", {{"status", "idle"}, {"phase", nullptr}}}}; }
Json FakeActions::assistant_state() const { return {{"mode", "automatic"}, {"navigation", "idle"}, {"distance_m", nullptr}, {"issue", nullptr}}; }
Json FakeActions::tracking_state() const { return {{"frame_id", "base_footprint"}, {"stamp", {{"sec", 5}, {"nanosec", 42}}}, {"active", true}, {"target", {{"x", 1.0}, {"y", -0.5}}}, {"points", Json::array({{1.0, -0.5}})}}; }
ApiReply FakeActions::publish_tracking_target(const Json &payload)
{
    ++tracking_targets;
    last_tracking_target = payload;
    return {202, {{"ok", true}}};
}
BinarySnapshotPtr FakeActions::navigation_asset(const std::string &name) const
{
    if (name == "local_costmap") return nullptr;
    if (name != "static" && name != "global_costmap" && name != "path") return nullptr;
    const std::string bytes = "gzip-" + name;
    return std::make_shared<const BinarySnapshot>(BinarySnapshot{3, "\"revision-3\"", {'r','a','w'}, {bytes.begin(), bytes.end()}});
}
ApiReply FakeActions::manual_command(const std::string &direction, double speed)
{
    auto command = command_values(direction, speed, 0.5, 1.0);
    if (!command) return {400, {{"error", command.error()}}};
    if (manual_status != 200) return {manual_status.load(), {{"error", "publisher unavailable"}}};
    return {200, {{"ok", true}, {"mode", "manual"}}};
}
ApiReply FakeActions::takeover_manual()
{
    if (block_mode) {
        mode_started.set_value();
        release_mode.get_future().wait_for(3s);
    }
    return {202, {{"ok", false}, {"pending", true}, {"error", "manual takeover service timed out"}, {"mode", "automatic"}}};
}
ApiReply FakeActions::resume_automatic() { return {200, {{"ok", true}, {"mode", "automatic"}}}; }
ApiReply FakeActions::publish_initial_pose(const Json &payload)
{
    auto pose = parse_navigation_pose(payload, GridInfo{10, 10, 1, 0, 0, 0, "map"}, 3);
    if (!pose) return {pose.error().kind == PoseErrorKind::map_revision_conflict ? 409 : 400, {{"error", pose.error().message}}};
    return {200, {{"ok", true}}};
}
ApiReply FakeActions::send_navigation_goal(const Json &payload)
{
    auto checked = publish_initial_pose(payload);
    if (checked.status != 200) return checked;
    return {202, {{"ok", true}, {"goal_status", "sending"}}};
}
ApiReply FakeActions::cancel_navigation() { return {202, {{"ok", true}, {"goal_status", "canceling"}}}; }
ApiReply FakeActions::list_parking_points() { return {200, {{"points", points}}}; }
ApiReply FakeActions::save_parking_point(const std::string &name)
{
    if (name == "unavailable") return {503, {{"error", "parking store unavailable"}}};
    if (!points.empty()) return {409, {{"error", "parking point already exists"}}};
    points.push_back({{"name", name}, {"x", 1}, {"y", 2}, {"yaw", 0}});
    return {201, {{"ok", true}, {"point", points.back()}}};
}
ApiReply FakeActions::navigate_parking_point(const std::string &name)
{
    if (points.empty() || points.back()["name"] != name) return {404, {{"error", "parking point not found"}}};
    return {202, {{"ok", true}, {"goal_status", "sending"}}};
}
ApiReply FakeActions::delete_parking_point(const std::string &name)
{
    if (points.empty() || points.back()["name"] != name) return {404, {{"error", "parking point not found"}}};
    points.clear();
    return {200, {{"ok", true}}};
}

int unused_port()
{
    const int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    EXPECT_GE(socket_fd, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    EXPECT_EQ(bind(socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
    socklen_t length = sizeof(address);
    EXPECT_EQ(getsockname(socket_fd, reinterpret_cast<sockaddr *>(&address), &length), 0);
    close(socket_fd);
    return ntohs(address.sin_port);
}

class HttpTest : public testing::Test {
protected:
    void SetUp() override;
    void TearDown() override;
    httplib::Result post(const std::string &path, const Json &body);
    Json session();
    FakeActions actions;
    int port = unused_port();
    std::filesystem::path directory;
    std::unique_ptr<HttpServer> server;
    std::unique_ptr<httplib::Client> client;
};

void HttpTest::SetUp()
{
    char name[] = "/tmp/robot-web-http-XXXXXX";
    directory = mkdtemp(name);
    std::ofstream(directory / "index.html", std::ios::binary) << "<html>robot</html>\n";
    std::ofstream(directory / "map_view.js", std::ios::binary) << "export const robot = 1;\n";
    std::ofstream(directory / "tracking_view.js", std::ios::binary) << "export const tracking = 1;\n";
    auto result = HttpServer::create({"127.0.0.1", port, directory.string()}, actions);
    ASSERT_TRUE(result) << result.error().message();
    server = std::move(*result);
    client = std::make_unique<httplib::Client>("127.0.0.1", port);
    client->set_read_timeout(2, 0);
    client->set_decompress(false);
}
void HttpTest::TearDown() { server.reset(); std::filesystem::remove_all(directory); }
httplib::Result HttpTest::post(const std::string &path, const Json &body) { return client->Post(path.c_str(), body.dump(), "application/json"); }
Json HttpTest::session()
{
    auto result = post("/api/manual-session", Json::object());
    EXPECT_TRUE(result);
    if (!result) return Json::object();
    EXPECT_EQ(result->status, 200);
    return Json::parse(result->body);
}

TEST_F(HttpTest, TrackingRoutesReturnSnapshotAndAcceptOnlyFiniteXY)
{
    auto state = client->Get("/api/tracking-state");
    ASSERT_TRUE(state);
    EXPECT_EQ(state->status, 200);
    EXPECT_EQ(Json::parse(state->body), actions.tracking_state());

    auto valid = post("/api/tracking-target", {{"x", 1.0}, {"y", -0.5}});
    ASSERT_TRUE(valid);
    EXPECT_EQ(valid->status, 202);
    EXPECT_EQ(actions.tracking_targets, 1);
    EXPECT_EQ(actions.last_tracking_target, (Json{{"x", 1.0}, {"y", -0.5}}));

    for (const std::string body : {R"({"x":"1","y":0})", R"({"x":1})", R"({"x":1,"y":0,"z":0})",
                                   R"({"x":1e999,"y":0})", R"({"x":null,"y":0})", R"([1,2])", "{"}) {
        auto invalid = client->Post("/api/tracking-target", body, "application/json");
        ASSERT_TRUE(invalid);
        EXPECT_EQ(invalid->status, 400) << body;
    }
    EXPECT_EQ(actions.tracking_targets, 1);

    auto navigation = client->Get("/api/navigation-state");
    ASSERT_TRUE(navigation);
    EXPECT_EQ(navigation->status, 200);
    auto manual = post("/api/manual-session", Json::object());
    ASSERT_TRUE(manual);
    EXPECT_EQ(manual->status, 200);
}

TEST_F(HttpTest, ServesOnlyNamedStaticAssetsAndCompactState)
{
    for (const auto &entry : {std::make_pair("/", "<html>robot</html>\n"), std::make_pair("/map_view.js", "export const robot = 1;\n"), std::make_pair("/tracking_view.js", "export const tracking = 1;\n")}) {
        auto response = client->Get(entry.first);
        ASSERT_TRUE(response);
        EXPECT_EQ(response->status, 200);
        EXPECT_EQ(response->body, entry.second);
        EXPECT_EQ(response->get_header_value("Cache-Control"), "no-store");
        EXPECT_EQ(response->get_header_value("Content-Type"), std::string(entry.first) == "/" ? "text/html; charset=utf-8" : "application/javascript; charset=utf-8");
    }
    auto state = client->Get("/api/navigation-state");
    ASSERT_TRUE(state);
    EXPECT_EQ(Json::parse(state->body), (Json{{"localized", true}, {"navigation", {{"status", "idle"}, {"phase", nullptr}}}}));
    EXPECT_EQ(state->body.find('\n'), std::string::npos);
    auto assistant = client->Get("/api/assistant-state");
    ASSERT_TRUE(assistant);
    EXPECT_EQ(Json::parse(assistant->body), (Json{{"mode", "automatic"}, {"navigation", "idle"}, {"distance_m", nullptr}, {"issue", nullptr}}));
    for (const auto *path : {"/unknown", "/index.html", "/api/map/other"}) {
        auto response = client->Get(path);
        ASSERT_TRUE(response);
        EXPECT_EQ(response->status, 404);
        EXPECT_EQ(Json::parse(response->body), (Json{{"error", "not found"}}));
    }
}

TEST_F(HttpTest, ReusesCompressedSnapshotAndRequiresExactEtagForBodyless304)
{
    for (const auto &entry : {std::make_pair("/api/map/static", "gzip-static"),
                              std::make_pair("/api/map/global-costmap", "gzip-global_costmap"),
                              std::make_pair("/api/navigation-path", "gzip-path")}) {
        const auto *path = entry.first;
        auto response = client->Get(path);
        ASSERT_TRUE(response);
        EXPECT_EQ(response->status, 200);
        EXPECT_EQ(response->body, entry.second);
        EXPECT_EQ(response->get_header_value("Content-Encoding"), "gzip");
        EXPECT_EQ(response->get_header_value("ETag"), "\"revision-3\"");
        EXPECT_EQ(response->get_header_value("Cache-Control"), "no-cache");
        auto cached = client->Get(path, {{"If-None-Match", "\"revision-3\""}});
        ASSERT_TRUE(cached);
        EXPECT_EQ(cached->status, 304);
        EXPECT_TRUE(cached->body.empty());
        EXPECT_FALSE(cached->has_header("Content-Encoding"));
        EXPECT_FALSE(cached->has_header("Content-Type"));
        auto weak = client->Get(path, {{"If-None-Match", "W/\"revision-3\""}});
        ASSERT_TRUE(weak);
        EXPECT_EQ(weak->status, 200);
    }
    auto missing = client->Get("/api/map/local-costmap");
    ASSERT_TRUE(missing);
    EXPECT_EQ(missing->status, 404);
}

TEST_F(HttpTest, SerializesManualSessionsAndDoesNotAdvanceFailedCommands)
{
    const auto first = session();
    Json command{{"session_id", first["session_id"]}, {"sequence", 1}, {"direction", "forward"}, {"speed_percent", 25}};
    auto accepted = post("/api/manual-command", command);
    ASSERT_TRUE(accepted);
    EXPECT_EQ(Json::parse(accepted->body), (Json{{"ok", true}, {"accepted", true}, {"sequence", 1}, {"last_sequence", 1}, {"mode", "manual"}}));
    auto stale = post("/api/manual-command", command);
    ASSERT_TRUE(stale);
    EXPECT_EQ(Json::parse(stale->body)["reason"], "stale_sequence");
    command["sequence"] = 2;
    actions.manual_status = 503;
    auto failed = post("/api/manual-command", command);
    ASSERT_TRUE(failed);
    EXPECT_EQ(failed->status, 503);
    EXPECT_EQ(Json::parse(failed->body), (Json{{"error", "publisher unavailable"}, {"accepted", false}, {"sequence", 2}, {"last_sequence", 1}, {"mode", "manual"}}));
    actions.manual_status = 200;
    auto retry = post("/api/manual-command", command);
    ASSERT_TRUE(retry);
    EXPECT_EQ(Json::parse(retry->body)["accepted"], true);
    auto second = session();
    EXPECT_NE(first["session_id"], second["session_id"]);
    auto inactive = post("/api/manual-command", command);
    ASSERT_TRUE(inactive);
    EXPECT_EQ(inactive->status, 409);
    EXPECT_EQ(Json::parse(inactive->body)["reason"], "inactive_session");
}

TEST_F(HttpTest, PreservesNavigationAndModeStatuses)
{
    auto pending = post("/api/takeover-manual", Json::object());
    ASSERT_TRUE(pending);
    EXPECT_EQ(pending->status, 202);
    EXPECT_EQ(Json::parse(pending->body)["pending"], true);
    auto resume = post("/api/resume-automatic", Json::object());
    ASSERT_TRUE(resume);
    EXPECT_EQ(Json::parse(resume->body), (Json{{"ok", true}, {"mode", "automatic"}}));
    Json pose{{"x", 1}, {"y", 2}, {"yaw", 0}, {"map_revision", 3}};
    auto initial = post("/api/initial-pose", pose);
    ASSERT_TRUE(initial);
    EXPECT_EQ(initial->status, 200);
    auto goal = post("/api/navigation-goal", pose);
    ASSERT_TRUE(goal);
    EXPECT_EQ(goal->status, 202);
    EXPECT_EQ(Json::parse(goal->body)["goal_status"], "sending");
    pose["map_revision"] = 2;
    auto conflict = post("/api/navigation-goal", pose);
    ASSERT_TRUE(conflict);
    EXPECT_EQ(conflict->status, 409);
    auto invalid = post("/api/navigation-goal", Json::object());
    ASSERT_TRUE(invalid);
    EXPECT_EQ(invalid->status, 400);
    auto cancel = post("/api/navigation-cancel", Json::object());
    ASSERT_TRUE(cancel);
    EXPECT_EQ(cancel->status, 202);
    EXPECT_EQ(Json::parse(cancel->body)["goal_status"], "canceling");
}

TEST_F(HttpTest, PreservesParkingStatusesAndNameValidation)
{
    auto saved = post("/api/parking-points/save", {{"name", "dock"}});
    ASSERT_TRUE(saved);
    EXPECT_EQ(saved->status, 201);
    auto listed = client->Get("/api/parking-points");
    ASSERT_TRUE(listed);
    EXPECT_EQ(Json::parse(listed->body)["points"][0]["name"], "dock");
    for (const auto &entry : {std::make_pair("save", 409), std::make_pair("navigate", 202), std::make_pair("delete", 200), std::make_pair("delete", 404)}) {
        auto response = post(std::string("/api/parking-points/") + entry.first, {{"name", "dock"}});
        ASSERT_TRUE(response);
        EXPECT_EQ(response->status, entry.second);
    }
    auto unavailable = post("/api/parking-points/save", {{"name", "unavailable"}});
    ASSERT_TRUE(unavailable);
    EXPECT_EQ(unavailable->status, 503);
    auto invalid = post("/api/parking-points/save", {{"name", 2}});
    ASSERT_TRUE(invalid);
    EXPECT_EQ(invalid->status, 400);
}

TEST_F(HttpTest, RejectsMalformedOversizedAndNonObjectBodiesWithJson400)
{
    for (const std::string body : {std::string(), std::string("{"), std::string("[]"), std::string(4097, ' ')}) {
        auto response = client->Post("/api/manual-session", body, "application/json");
        ASSERT_TRUE(response);
        EXPECT_EQ(response->status, 400);
        EXPECT_TRUE(Json::parse(response->body).contains("error"));
        EXPECT_EQ(response->get_header_value("Content-Type"), "application/json; charset=utf-8");
    }
    auto wrong_type = client->Post("/api/manual-session", "{}", "text/plain");
    ASSERT_TRUE(wrong_type);
    EXPECT_EQ(wrong_type->status, 400);
}

TEST_F(HttpTest, Accepts4096BytesAndReturnsJson400WhenBodyReadTimesOut)
{
    auto full = client->Post("/api/manual-session", "{}" + std::string(4094, ' '), "application/json");
    ASSERT_TRUE(full);
    EXPECT_EQ(full->status, 200);
    const int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_GE(socket_fd, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    ASSERT_EQ(connect(socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
    timeval timeout{2, 0};
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    const std::string request = "POST /api/manual-session HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{";
    EXPECT_EQ(send(socket_fd, request.data(), request.size(), 0), static_cast<ssize_t>(request.size()));
    std::string response;
    char buffer[1024];
    ssize_t count;
    while ((count = recv(socket_fd, buffer, sizeof(buffer), 0)) > 0) response.append(buffer, static_cast<size_t>(count));
    close(socket_fd);
    EXPECT_NE(response.find("HTTP/1.1 400"), std::string::npos);
    const auto body = response.find("\r\n\r\n");
    ASSERT_NE(body, std::string::npos);
    EXPECT_TRUE(Json::parse(response.substr(body + 4)).contains("error"));
}

TEST_F(HttpTest, ModeServiceWaitDoesNotBlockManualCommand)
{
    auto active = session();
    actions.block_mode = true;
    auto started = actions.mode_started.get_future();
    auto waiting = std::async(std::launch::async, [&] {
        httplib::Client other("127.0.0.1", port);
        return other.Post("/api/takeover-manual", "{}", "application/json");
    });
    EXPECT_EQ(started.wait_for(1s), std::future_status::ready);
    auto command = post("/api/manual-command", {{"session_id", active["session_id"]}, {"sequence", 1}, {"direction", "stop"}, {"speed_percent", 0}});
    EXPECT_EQ(waiting.wait_for(0s), std::future_status::timeout);
    actions.release_mode.set_value();
    ASSERT_TRUE(command);
    EXPECT_EQ(command->status, 200);
    EXPECT_EQ(Json::parse(command->body)["accepted"], true);
    EXPECT_TRUE(waiting.get());
}

TEST_F(HttpTest, CreationFailsSynchronouslyWhenPortIsOccupied)
{
    auto duplicate = HttpServer::create({"127.0.0.1", port, directory.string()}, actions);
    EXPECT_FALSE(duplicate);
    auto response = client->Get("/");
    ASSERT_TRUE(response);
    EXPECT_EQ(response->status, 200);
}
} // namespace
} // namespace robot_web_ui
