#ifndef XX_WEB_UI_NODE_H_
#define XX_WEB_UI_NODE_H_

#include <memory>
#include <rclcpp/node.hpp>

namespace robot_web_ui
{
class HttpActions;

/** Owns ROS resources; callers stop HTTP workers and the executor before destruction. */
class WebUiNode final : public rclcpp::Node {
public:
    explicit WebUiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~WebUiNode() override;
    WebUiNode(const WebUiNode &) = delete;
    WebUiNode &operator=(const WebUiNode &) = delete;

    /** Borrows concurrently callable request operations; the reference must not outlive this node. */
    [[nodiscard]] HttpActions &http_actions();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robot_web_ui

#endif // XX_WEB_UI_NODE_H_
