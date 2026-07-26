// 差速机器人真机侧 system 接口。
#ifndef ROBOT_HARDWARE__DIFF_DRIVE_SYSTEM_HPP_
#define ROBOT_HARDWARE__DIFF_DRIVE_SYSTEM_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_hardware/drive_conversion.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"

namespace robot_hardware
{
class DiffDriveSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveSystem);

  ~DiffDriveSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp::Logger get_logger() const { return *logger_; }
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  void feedback_callback(const std_msgs::msg::Int16MultiArray::SharedPtr message);
  void publish_motor(int16_t right_rpm, int16_t left_rpm);
  void publish_driver(bool enabled);
  void stop_and_disable();
  void release_io();
  bool feedback_is_fresh(std::chrono::steady_clock::time_point now) const;

  double activation_wait_sec_{5.0};
  double feedback_timeout_sec_{0.5};
  int max_motor_rpm_{256};

  rclcpp::Node::SharedPtr io_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr driver_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr feedback_sub_;

  WheelVelocities latest_feedback_{0.0, 0.0};
  std::chrono::steady_clock::time_point last_feedback_time_{};
  bool have_feedback_{false};
  bool active_{false};

  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__DIFF_DRIVE_SYSTEM_HPP_
