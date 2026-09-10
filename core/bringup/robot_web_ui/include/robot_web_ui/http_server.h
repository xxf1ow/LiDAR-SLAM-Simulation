#ifndef XX_HTTP_SERVER_H_
#define XX_HTTP_SERVER_H_

#include <memory>
#include <string>
#include <system_error>
#include <tl/expected.hpp>

namespace robot_web_ui
{
class HttpActions;

/** Owns one HTTP listener and its library-managed workers. */
class HttpServer final {
public:
    struct Params {
        std::string host;
        /** TCP port in 1–65535. */
        int port;
        /** Directory containing index.html and map_view.js. */
        std::string web_directory;
    };

    template <typename T>
    using Result = tl::expected<T, std::error_code>;

    /** Binds synchronously and starts serving, or returns a socket/thread creation error.
     * Borrows actions until destruction completes. Workers call actions concurrently;
     * manual sessions and commands are serialized without holding locks during publication.
     * Request bodies are JSON objects of 1–4096 bytes with a 0.5-second socket read timeout.
     */
    [[nodiscard]] static Result<std::unique_ptr<HttpServer>> create(Params params, HttpActions &actions);
    /** Stops accepting requests and joins the listener and workers before releasing state. */
    ~HttpServer();
    HttpServer(const HttpServer &) = delete;
    HttpServer &operator=(const HttpServer &) = delete;
    HttpServer(HttpServer &&) = delete;
    HttpServer &operator=(HttpServer &&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    explicit HttpServer(std::unique_ptr<Impl> impl);
};
} // namespace robot_web_ui

#endif // XX_HTTP_SERVER_H_
