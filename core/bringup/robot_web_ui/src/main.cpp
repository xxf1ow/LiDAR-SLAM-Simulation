#include "robot_web_ui/http_server.h"
#include "robot_web_ui/web_ui_node.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    {
        auto node = std::make_shared<robot_web_ui::WebUiNode>();
        auto server = robot_web_ui::HttpServer::create(
            {node->get_parameter("host").as_string(), static_cast<int>(node->get_parameter("port").as_int()),
             ament_index_cpp::get_package_share_directory("robot_web_ui") + "/web"}, node->http_actions());
        if (!server) {
            RCLCPP_FATAL(node->get_logger(), "HTTP server startup failed: %s", server.error().message().c_str());
            exit_code = 1;
        } else {
            RCLCPP_INFO(node->get_logger(), "HTTP server listening on %s:%ld", node->get_parameter("host").as_string().c_str(),
                        node->get_parameter("port").as_int());
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(node);
            executor.spin();
        }
    }
    rclcpp::shutdown();
    return exit_code;
}
