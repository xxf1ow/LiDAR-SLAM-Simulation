// 差速机器人真机侧 system 接口实现。
#include "robot_hardware/diff_drive_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware
{
namespace
{
double parse_positive_double(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key)
{
  const auto found = parameters.find(key);
  if (found == parameters.end()) {
    throw std::invalid_argument(key);
  }
  std::size_t parsed = 0;
  const double value = std::stod(found->second, &parsed);
  if (parsed != found->second.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(key);
  }
  return value;
}

int parse_positive_int16_limit(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key)
{
  const auto found = parameters.find(key);
  if (found == parameters.end()) {
    throw std::invalid_argument(key);
  }
  std::size_t parsed = 0;
  const int value = std::stoi(found->second, &parsed);
  if (parsed != found->second.size() || value <= 0 || value > 32767) {
    throw std::invalid_argument(key);
  }
  return value;
}

bool ends_with(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
    value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

DiffDriveSystem::~DiffDriveSystem()
{
  if (rclcpp::ok() && motor_pub_ && driver_pub_) {
    stop_and_disable();
  }
  release_io();
}

hardware_interface::CallbackReturn DiffDriveSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffDrive"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  try {
    if (info_.joints.size() != 2) {
      throw std::invalid_argument("exactly two joints are required");
    }
    if (!ends_with(info_.joints[0].name, "left_wheel_joint")) {
      throw std::invalid_argument("joint 0 must be left_wheel_joint");
    }
    if (!ends_with(info_.joints[1].name, "right_wheel_joint")) {
      throw std::invalid_argument("joint 1 must be right_wheel_joint");
    }

    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
      if (
        joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        throw std::invalid_argument("each joint requires one velocity command interface");
      }
      if (
        joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        throw std::invalid_argument(
                "each joint requires position then velocity state interfaces");
      }
    }

    activation_wait_sec_ = parse_positive_double(
      info_.hardware_parameters, "activation_wait_sec");
    feedback_timeout_sec_ = parse_positive_double(
      info_.hardware_parameters, "feedback_timeout_sec");
    max_motor_rpm_ = parse_positive_int16_limit(
      info_.hardware_parameters, "max_motor_rpm");

    hw_commands_.assign(info_.joints.size(), 0.0);
    hw_positions_.assign(info_.joints.size(), 0.0);
    hw_velocities_.assign(info_.joints.size(), 0.0);

    io_node_ = std::make_shared<rclcpp::Node>("diff_drive_system_io");
    motor_pub_ = io_node_->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/motor_speed", rclcpp::QoS(10).reliable());
    driver_pub_ = io_node_->create_publisher<std_msgs::msg::Int8>(
      "/driver", rclcpp::QoS(10).reliable());
    feedback_sub_ = io_node_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/current_speed",
      rclcpp::QoS(10).reliable(),
      std::bind(&DiffDriveSystem::feedback_callback, this, std::placeholders::_1));
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(io_node_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize DiffDriveSystem: %s", error.what());
    release_io();
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  have_feedback_ = false;
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(activation_wait_sec_);
  while (std::chrono::steady_clock::now() < deadline) {
    publish_motor(0, 0);
    publish_driver(true);
    executor_->spin_some();
    if (have_feedback_ && feedback_is_fresh(std::chrono::steady_clock::now())) {
      active_ = true;
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  stop_and_disable();
  RCLCPP_ERROR(get_logger(), "Activation timed out waiting for /current_speed");
  return hardware_interface::CallbackReturn::FAILURE;
}

hardware_interface::CallbackReturn DiffDriveSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_and_disable();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (rclcpp::ok()) {
    stop_and_disable();
  }
  release_io();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  if (rclcpp::ok()) {
    stop_and_disable();
  }
  release_io();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  executor_->spin_some();
  if (!active_) {
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    return hardware_interface::return_type::OK;
  }
  if (!have_feedback_ || !feedback_is_fresh(std::chrono::steady_clock::now())) {
    stop_and_disable();
    RCLCPP_ERROR(get_logger(), "8030D wheel feedback timed out");
    return hardware_interface::return_type::ERROR;
  }

  hw_velocities_[0] = latest_feedback_.left_rad_s;
  hw_velocities_[1] = latest_feedback_.right_rad_s;
  for (std::size_t index = 0; index < hw_positions_.size(); ++index) {
    hw_positions_[index] += period.seconds() * hw_velocities_[index];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  const auto command = to_motor_rpm(
    hw_commands_[0], hw_commands_[1], max_motor_rpm_);
  if (!command.has_value()) {
    stop_and_disable();
    RCLCPP_ERROR(get_logger(), "Rejected non-finite wheel command");
    return hardware_interface::return_type::ERROR;
  }
  publish_motor(command->right_rpm, command->left_rpm);
  return hardware_interface::return_type::OK;
}

void DiffDriveSystem::feedback_callback(
  const std_msgs::msg::Int16MultiArray::SharedPtr message)
{
  if (message->data.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "/current_speed must contain at least two channels");
    return;
  }
  latest_feedback_ = from_motor_feedback(message->data[0], message->data[1]);
  last_feedback_time_ = std::chrono::steady_clock::now();
  have_feedback_ = true;
}

void DiffDriveSystem::publish_motor(int16_t right_rpm, int16_t left_rpm)
{
  if (!motor_pub_) {
    return;
  }
  std_msgs::msg::Int16MultiArray message;
  message.data = {right_rpm, left_rpm};
  motor_pub_->publish(message);
}

void DiffDriveSystem::publish_driver(bool enabled)
{
  if (!driver_pub_) {
    return;
  }
  std_msgs::msg::Int8 message;
  message.data = enabled ? 1 : 0;
  driver_pub_->publish(message);
}

void DiffDriveSystem::stop_and_disable()
{
  active_ = false;
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  publish_motor(0, 0);
  publish_driver(false);
}

void DiffDriveSystem::release_io()
{
  if (executor_ && io_node_) {
    executor_->remove_node(io_node_);
  }
  feedback_sub_.reset();
  motor_pub_.reset();
  driver_pub_.reset();
  io_node_.reset();
  executor_.reset();
}

bool DiffDriveSystem::feedback_is_fresh(
  std::chrono::steady_clock::time_point now) const
{
  return std::chrono::duration<double>(now - last_feedback_time_).count() <=
         feedback_timeout_sec_;
}
}  // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_hardware::DiffDriveSystem, hardware_interface::SystemInterface)
