#include <atomic>
#include <chrono>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_hardware/diff_drive_system.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"

using namespace std::chrono_literals;

namespace
{
std::string test_urdf(bool swap_joints = false)
{
  const std::string left =
    R"(<joint name="left_wheel_joint">
         <command_interface name="velocity"/>
         <state_interface name="position"/>
         <state_interface name="velocity"/>
       </joint>)";
  const std::string right =
    R"(<joint name="right_wheel_joint">
         <command_interface name="velocity"/>
         <state_interface name="position"/>
         <state_interface name="velocity"/>
       </joint>)";
  return
    R"(<robot name="test"><ros2_control name="RobotSystem" type="system">
         <hardware>
           <plugin>robot_hardware/DiffDriveSystem</plugin>
           <param name="activation_wait_sec">1.0</param>
           <param name="feedback_timeout_sec">0.15</param>
           <param name="max_motor_rpm">256</param>
         </hardware>)" +
    (swap_joints ? right + left : left + right) +
    R"(</ros2_control></robot>)";
}

class Fake8030D
{
public:
  Fake8030D()
  : node_(std::make_shared<rclcpp::Node>("fake_8030d_cpp"))
  {
    feedback_pub_ = node_->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/current_speed", rclcpp::QoS(10).reliable());
    driver_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
      "/driver", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Int8::SharedPtr msg) {
        enabled_.store(msg->data == 1);
        last_driver_.store(msg->data);
      });
    motor_sub_ = node_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/motor_speed", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_ = {msg->data[0], msg->data[1]};
        }
      });
    timer_ = node_->create_wall_timer(20ms, [this]() {
      if (!enabled_.load() || !publish_feedback_.load()) {
        return;
      }
      std::vector<int16_t> command;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command = command_;
      }
      std_msgs::msg::Int16MultiArray feedback;
      feedback.data = {
        static_cast<int16_t>(-command[1] * 10),
        static_cast<int16_t>(command[0] * 10)};
      feedback_pub_->publish(feedback);
    });
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
  }

  ~Fake8030D()
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.remove_node(node_);
  }

  void set_feedback_enabled(bool enabled) {publish_feedback_.store(enabled);}
  int last_driver() const {return last_driver_.load();}

  void publish_raw_feedback(const std::vector<int16_t> & values)
  {
    std_msgs::msg::Int16MultiArray message;
    message.data = values;
    feedback_pub_->publish(message);
  }

  std::vector<int16_t> command() const
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr feedback_pub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr driver_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr motor_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> enabled_{false};
  std::atomic<bool> publish_feedback_{true};
  std::atomic<int> last_driver_{-1};
  mutable std::mutex command_mutex_;
  std::vector<int16_t> command_{0, 0};
};
}  // namespace

class DiffDriveSystemTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(DiffDriveSystemTest, RejectsSwappedJointOrder)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf(true));
  ASSERT_EQ(infos.size(), 1u);
  robot_hardware::DiffDriveSystem system;
  EXPECT_EQ(
    system.on_init(infos.front()),
    hardware_interface::CallbackReturn::ERROR);
}

TEST_F(DiffDriveSystemTest, ActivationFailsWithoutFeedback)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  robot_hardware::DiffDriveSystem system;
  ASSERT_EQ(system.on_init(infos.front()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::FAILURE);
}

TEST_F(DiffDriveSystemTest, BridgesCommandsAndMeasuredFeedback)
{
  Fake8030D fake;
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  robot_hardware::DiffDriveSystem system;
  ASSERT_EQ(
    system.on_init(infos.front()),
    hardware_interface::CallbackReturn::SUCCESS);

  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  ASSERT_EQ(commands.size(), 2u);
  ASSERT_EQ(states.size(), 4u);
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  commands[0].set_value(1.0);
  commands[1].set_value(1.0);
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::OK);

  const auto command_deadline = std::chrono::steady_clock::now() + 1s;
  while (
    fake.command() != std::vector<int16_t>({10, 10}) &&
    std::chrono::steady_clock::now() < command_deadline)
  {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(fake.command(), std::vector<int16_t>({10, 10}));

  const auto state_deadline = std::chrono::steady_clock::now() + 1s;
  while (states[1].get_value() == 0.0 && std::chrono::steady_clock::now() < state_deadline)
  {
    ASSERT_EQ(
      system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
      hardware_interface::return_type::OK);
    std::this_thread::sleep_for(10ms);
  }
  const double ten_rpm_rad_s = 10.0 * 2.0 * 3.14159265358979323846 / 60.0;
  EXPECT_NEAR(states[1].get_value(), ten_rpm_rad_s, 1e-9);
  EXPECT_NEAR(states[3].get_value(), ten_rpm_rad_s, 1e-9);
  EXPECT_GT(states[0].get_value(), 0.0);
  EXPECT_GT(states[2].get_value(), 0.0);

  ASSERT_EQ(
    system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(states[1].get_value(), 0.0);
  EXPECT_DOUBLE_EQ(states[3].get_value(), 0.0);

  commands[0].set_value(1.0);
  commands[1].set_value(1.0);
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::OK);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(fake.command(), std::vector<int16_t>({0, 0}));
}

TEST_F(DiffDriveSystemTest, ShortFeedbackDoesNotRefreshTimeout)
{
  Fake8030D fake;
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  robot_hardware::DiffDriveSystem system;
  ASSERT_EQ(system.on_init(infos.front()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  fake.set_feedback_enabled(false);
  std::this_thread::sleep_for(50ms);
  ASSERT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::OK);
  std::this_thread::sleep_for(200ms);
  fake.publish_raw_feedback({123});
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::ERROR);

  const auto stop_deadline = std::chrono::steady_clock::now() + 1s;
  while (fake.last_driver() != 0 && std::chrono::steady_clock::now() < stop_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(fake.last_driver(), 0);
  EXPECT_EQ(fake.command(), std::vector<int16_t>({0, 0}));
}

TEST_F(DiffDriveSystemTest, NonFiniteCommandStopsAndDisables)
{
  Fake8030D fake;
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  robot_hardware::DiffDriveSystem system;
  ASSERT_EQ(system.on_init(infos.front()), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  commands[1].set_value(0.0);
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::ERROR);
  const auto stop_deadline = std::chrono::steady_clock::now() + 1s;
  while (fake.last_driver() != 0 && std::chrono::steady_clock::now() < stop_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(fake.last_driver(), 0);
  EXPECT_EQ(fake.command(), std::vector<int16_t>({0, 0}));
}

TEST_F(DiffDriveSystemTest, CleanupAndShutdownReleaseIoIdempotently)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  robot_hardware::DiffDriveSystem system;
  ASSERT_EQ(system.on_init(infos.front()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_cleanup(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_shutdown(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}
