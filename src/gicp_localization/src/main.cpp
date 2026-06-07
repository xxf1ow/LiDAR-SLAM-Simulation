#include <rclcpp/rclcpp.hpp>
#include "gicp_localization/gicp_localization_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gicp_localization::GicpLocalizationNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
