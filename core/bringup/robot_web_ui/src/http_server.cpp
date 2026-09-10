#include "robot_web_ui/http_server.h"
#include "robot_web_ui/http_actions.h"

#include <httplib.h>
#include <openssl/rand.h>
#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <thread>

namespace robot_web_ui
{
/*******************************************************************************************************
 * @brief HTTP representation and request validation
 *******************************************************************************************************/
namespace
{
using Json = nlohmann::json;
constexpr uint64_t manual_sequence_max = 9007199254740991ULL;

ApiReply error_reply(int status, const std::string &message)
{
    return {status, {{"error", message}}};
}

void send_json(httplib::Response &response, const ApiReply &reply)
{
    response.status = reply.status;
    response.set_header("Cache-Control", "no-store");
    try {
        response.set_content(reply.body.dump(), "application/json; charset=utf-8");
    } catch (const Json::type_error &) {
        response.status = 500;
        response.set_content("{\"error\":\"response serialization failed\"}", "application/json; charset=utf-8");
    }
}

tl::expected<Json, std::string> read_json(const httplib::Request &request)
{
    const auto type = request.get_header_value("Content-Type");
    const auto separator = type.find(';');
    if (type.substr(0, separator) != "application/json")
        return tl::make_unexpected("Content-Type must be application/json");
    if (request.body.empty() || request.body.size() > 4096)
        return tl::make_unexpected("request body must contain 1 to 4096 bytes");
    auto payload = Json::parse(request.body, nullptr, false);
    if (payload.is_discarded()) return tl::make_unexpected("request body must be valid JSON");
    if (!payload.is_object()) return tl::make_unexpected("request JSON must be an object");
    return payload;
}

void send_asset(const httplib::Request &request, httplib::Response &response, BinarySnapshotPtr snapshot)
{
    if (!snapshot) {
        send_json(response, error_reply(404, "not found"));
        return;
    }
    response.set_header("ETag", snapshot->etag);
    response.set_header("Cache-Control", "no-cache");
    if (request.get_header_value("If-None-Match") == snapshot->etag) {
        response.status = 304;
        return;
    }
    response.status = 200;
    response.set_header("Content-Encoding", "gzip");
    const auto length = snapshot->gzip_data.size();
    response.set_content_provider(length, binary_media_type,
        [snapshot = std::move(snapshot)](size_t offset, size_t count, httplib::DataSink &sink) {
            return sink.write(reinterpret_cast<const char *>(snapshot->gzip_data.data()) + offset, count);
        });
}

void send_file(httplib::Response &response, const std::filesystem::path &path, const char *content_type)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        send_json(response, error_reply(500, "web asset unavailable"));
        return;
    }
    std::string bytes{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    if (input.bad()) {
        send_json(response, error_reply(500, "web asset unavailable"));
        return;
    }
    response.set_header("Cache-Control", "no-store");
    response.set_content(bytes, content_type);
}

/*******************************************************************************************************
 * @brief Exclusive manual operation ownership without a lock during ROS publication
 *******************************************************************************************************/
class ManualLease final {
public:
    ManualLease(std::mutex &mutex, std::condition_variable &ready, bool &busy);
    ~ManualLease();
    ManualLease(const ManualLease &) = delete;
    ManualLease &operator=(const ManualLease &) = delete;
private:
    std::mutex &mutex_;
    std::condition_variable &ready_;
    bool &busy_;
};

ManualLease::ManualLease(std::mutex &mutex, std::condition_variable &ready, bool &busy)
    : mutex_(mutex), ready_(ready), busy_(busy)
{
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(lock, [this] { return !busy_; });
    busy_ = true;
}

ManualLease::~ManualLease()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
    }
    ready_.notify_one();
}
} // namespace

/*******************************************************************************************************
 * @brief HTTP routing and listener lifetime
 *******************************************************************************************************/
struct HttpServer::Impl {
    Impl(Params params, HttpActions &actions);
    ~Impl();
    ApiReply post(const std::string &path, const Json &payload);
    ApiReply manual_session(const Json &payload);
    ApiReply manual_command(const Json &payload);

    Params params;
    HttpActions &actions;
    httplib::Server server;
    std::thread listener;
    std::atomic<bool> listener_finished{false};
    std::mutex manual_mutex;
    std::condition_variable manual_ready;
    bool manual_busy = false;
    std::string session_id;
    uint64_t last_sequence = 0;
    Json manual_mode = nullptr;
};

HttpServer::Impl::Impl(Params params_value, HttpActions &actions_value)
    : params(std::move(params_value)), actions(actions_value)
{
    server.set_payload_max_length(4096);
    server.set_read_timeout(0, 500000);
    server.set_socket_options([](socket_t socket) {
        const int enabled = 1;
        setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
    });
    server.set_error_handler([](const httplib::Request &, httplib::Response &response) {
        if (!response.body.empty()) return;
        if (response.status == 413)
            send_json(response, error_reply(400, "request body must contain 1 to 4096 bytes"));
        else if (response.status == 400)
            send_json(response, error_reply(400, "request body must be valid JSON"));
        else if (response.status == 404)
            send_json(response, error_reply(404, "not found"));
    });
    server.Get("/", [this](const httplib::Request &, httplib::Response &response) {
        send_file(response, std::filesystem::path(params.web_directory) / "index.html", "text/html; charset=utf-8");
    });
    server.Get("/map_view\\.js", [this](const httplib::Request &, httplib::Response &response) {
        send_file(response, std::filesystem::path(params.web_directory) / "map_view.js", "application/javascript; charset=utf-8");
    });
    server.Get("/api/assistant-state", [this](const httplib::Request &, httplib::Response &response) {
        send_json(response, {200, actions.assistant_state()});
    });
    server.Get("/api/navigation-state", [this](const httplib::Request &, httplib::Response &response) {
        send_json(response, {200, actions.navigation_state()});
    });
    server.Get("/api/parking-points", [this](const httplib::Request &, httplib::Response &response) {
        send_json(response, actions.list_parking_points());
    });
    for (const auto &entry : {std::make_pair("/api/map/static", "static"),
                              std::make_pair("/api/map/global-costmap", "global_costmap"),
                              std::make_pair("/api/map/local-costmap", "local_costmap"),
                              std::make_pair("/api/navigation-path", "path")}) {
        server.Get(entry.first, [this, name = std::string(entry.second)](const httplib::Request &request, httplib::Response &response) {
            send_asset(request, response, actions.navigation_asset(name));
        });
    }
    for (const char *path : {"/api/manual-session", "/api/manual-command", "/api/takeover-manual",
                            "/api/resume-automatic", "/api/initial-pose", "/api/navigation-goal",
                            "/api/navigation-cancel", "/api/parking-points/save",
                            "/api/parking-points/navigate", "/api/parking-points/delete"}) {
        server.Post(path, [this](const httplib::Request &request, httplib::Response &response) {
            const auto payload = read_json(request);
            send_json(response, payload ? post(request.path, *payload) : error_reply(400, payload.error()));
        });
    }
}

HttpServer::Impl::~Impl()
{
    server.stop();
    if (listener.joinable()) listener.join();
}

ApiReply HttpServer::Impl::manual_session(const Json &payload)
{
    if (!payload.empty()) return error_reply(400, "manual session request must be {}");
    unsigned char random[24];
    if (RAND_bytes(random, sizeof(random)) != 1) return error_reply(503, "manual session unavailable");
    constexpr char hex[] = "0123456789abcdef";
    std::string token;
    for (auto byte : random) {
        token.push_back(hex[byte >> 4]);
        token.push_back(hex[byte & 15]);
    }
    ManualLease lease(manual_mutex, manual_ready, manual_busy);
    auto reply = actions.manual_command("stop", 0);
    if (reply.status != 200) return reply;
    session_id = std::move(token);
    last_sequence = 0;
    manual_mode = reply.body.value("mode", Json(nullptr));
    reply.body["session_id"] = session_id;
    return reply;
}

ApiReply HttpServer::Impl::manual_command(const Json &payload)
{
    if (payload.size() != 4 || !payload.contains("session_id") || !payload.contains("sequence") ||
        !payload.contains("direction") || !payload.contains("speed_percent"))
        return error_reply(400, "manual command fields are invalid");
    if (!payload["session_id"].is_string() || payload["session_id"].get_ref<const std::string &>().empty())
        return error_reply(400, "session_id must be a nonempty string");
    const auto &sequence_value = payload["sequence"];
    if (!sequence_value.is_number_integer() || sequence_value < 1 || sequence_value > manual_sequence_max)
        return error_reply(400, "sequence must be a positive safe integer");
    const auto sequence = sequence_value.get<uint64_t>();
    ManualLease lease(manual_mutex, manual_ready, manual_busy);
    if (session_id != payload["session_id"])
        return {409, {{"error", "inactive manual session"}, {"accepted", false}, {"reason", "inactive_session"}, {"sequence", sequence}}};
    if (sequence <= last_sequence)
        return {200, {{"ok", true}, {"accepted", false}, {"reason", "stale_sequence"},
                      {"sequence", sequence}, {"last_sequence", last_sequence}, {"mode", manual_mode}}};
    ApiReply reply;
    if (!payload["direction"].is_string())
        reply = error_reply(400, "direction must be a string");
    else if (!payload["speed_percent"].is_number())
        reply = error_reply(400, "speed_percent must be a number");
    else
        reply = actions.manual_command(payload["direction"].get<std::string>(), payload["speed_percent"].get<double>());
    const bool accepted = reply.status == 200;
    if (accepted) {
        last_sequence = sequence;
        manual_mode = reply.body.value("mode", Json(nullptr));
    }
    reply.body["accepted"] = accepted;
    reply.body["sequence"] = sequence;
    reply.body["last_sequence"] = last_sequence;
    if (!reply.body.contains("mode")) reply.body["mode"] = manual_mode;
    return reply;
}

ApiReply HttpServer::Impl::post(const std::string &path, const Json &payload)
{
    if (path == "/api/manual-session") return manual_session(payload);
    if (path == "/api/manual-command") return manual_command(payload);
    if (path == "/api/takeover-manual") return actions.takeover_manual();
    if (path == "/api/resume-automatic") return actions.resume_automatic();
    if (path == "/api/initial-pose") return actions.publish_initial_pose(payload);
    if (path == "/api/navigation-goal") return actions.send_navigation_goal(payload);
    if (path == "/api/navigation-cancel") {
        if (!payload.empty()) return error_reply(400, "navigation cancel request must be {}");
        return actions.cancel_navigation();
    }
    if (payload.size() != 1 || !payload.contains("name"))
        return error_reply(400, "parking point request must contain exactly name");
    if (!payload["name"].is_string()) return error_reply(400, "parking point name must be a string");
    const auto name = payload["name"].get<std::string>();
    if (path == "/api/parking-points/save") return actions.save_parking_point(name);
    if (path == "/api/parking-points/navigate") return actions.navigate_parking_point(name);
    return actions.delete_parking_point(name);
}

HttpServer::HttpServer(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
HttpServer::~HttpServer() = default;

HttpServer::Result<std::unique_ptr<HttpServer>> HttpServer::create(Params params, HttpActions &actions)
{
    if (params.host.empty() || params.port < 1 || params.port > 65535)
        return tl::make_unexpected(std::make_error_code(std::errc::invalid_argument));
    auto impl = std::make_unique<Impl>(std::move(params), actions);
    std::promise<bool> bound;
    try {
        impl->listener = std::thread([state = impl.get(), ready = bound.get_future()]() mutable {
            if (ready.get()) state->server.listen_after_bind();
            state->listener_finished = true;
        });
    } catch (const std::system_error &error) {
        return tl::make_unexpected(error.code());
    }
    // Allocate the thread before binding: cpp-httplib 0.10 cannot stop an unstarted listener.
    const bool listening = impl->server.bind_to_port(impl->params.host.c_str(), impl->params.port);
    const auto bind_error = std::error_code(errno ? errno : EADDRNOTAVAIL, std::generic_category());
    bound.set_value(listening);
    if (!listening) return tl::make_unexpected(bind_error);
    // stop() requires the library listener to have entered its running state.
    while (!impl->server.is_running() && !impl->listener_finished) std::this_thread::yield();
    if (impl->listener_finished)
        return tl::make_unexpected(std::make_error_code(std::errc::io_error));
    return std::unique_ptr<HttpServer>(new HttpServer(std::move(impl)));
}
} // namespace robot_web_ui
