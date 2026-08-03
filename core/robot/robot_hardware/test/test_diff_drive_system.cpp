#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
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
std::string test_urdf(
  bool swap_joints = false,
  const std::string & left_prefix = "",
  const std::string & right_prefix = "")
{
  const std::string left =
    R"(<joint name=")" + left_prefix + R"(left_wheel_joint">
         <command_interface name="velocity"/>
         <state_interface name="position"/>
         <state_interface name="velocity"/>
       </joint>)";
  const std::string right =
    R"(<joint name=")" + right_prefix + R"(right_wheel_joint">
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
  explicit Fake8030D(
    std::size_t max_feedback_messages = std::numeric_limits<std::size_t>::max())
  : node_(std::make_shared<rclcpp::Node>("fake_8030d_cpp")),
    max_feedback_messages_(max_feedback_messages)
  {
    feedback_pub_ = node_->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/current_speed", rclcpp::QoS(10).reliable());
    driver_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
      "/driver", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Int8::SharedPtr msg) {
        enabled_.store(msg->data == 1);
        std::lock_guard<std::mutex> lock(state_mutex_);
        events_.push_back("driver:" + std::to_string(msg->data));
      });
    motor_sub_ = node_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/motor_speed", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          command_ = {msg->data[0], msg->data[1]};
          events_.push_back(
            "motor:" + std::to_string(msg->data[0]) + "," +
            std::to_string(msg->data[1]));
        }
      });
    timer_ = node_->create_wall_timer(20ms, [this]() {
      if (
        !enabled_.load() ||
        feedback_messages_published_ >= max_feedback_messages_)
      {
        return;
      }
      std::vector<int16_t> command;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        command = command_;
      }
      std_msgs::msg::Int16MultiArray feedback;
      feedback.data = {
        static_cast<int16_t>(-command[1] * 10),
        static_cast<int16_t>(command[0] * 10)};
      feedback_pub_->publish(feedback);
      ++feedback_messages_published_;
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

  void publish_raw_feedback(const std::vector<int16_t> & values)
  {
    std_msgs::msg::Int16MultiArray message;
    message.data = values;
    feedback_pub_->publish(message);
  }

  std::vector<int16_t> command() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return command_;
  }

  void clear_events()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    events_.clear();
  }

  std::vector<std::string> events() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return events_;
  }

  bool wait_for_event_counts(
    const std::string & first,
    std::size_t first_count,
    const std::string & second,
    std::size_t second_count,
    std::chrono::milliseconds timeout = 1s) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto snapshot = events();
      if (
        static_cast<std::size_t>(std::count(snapshot.begin(), snapshot.end(), first)) >=
        first_count &&
        static_cast<std::size_t>(std::count(snapshot.begin(), snapshot.end(), second)) >=
        second_count)
      {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  bool wait_for_events_stable(
    std::chrono::milliseconds stable_for = 50ms,
    std::chrono::milliseconds timeout = 1s) const
  {
    auto last_count = events().size();
    auto stable_since = std::chrono::steady_clock::now();
    const auto deadline = stable_since + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
      const auto current_count = events().size();
      const auto now = std::chrono::steady_clock::now();
      if (current_count != last_count) {
        last_count = current_count;
        stable_since = now;
      } else if (now - stable_since >= stable_for) {
        return true;
      }
    }
    return false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr feedback_pub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr driver_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr motor_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  const std::size_t max_feedback_messages_;
  std::size_t feedback_messages_published_{0};
  std::atomic<bool> enabled_{false};
  mutable std::mutex state_mutex_;
  std::vector<int16_t> command_{0, 0};
  std::vector<std::string> events_;
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

TEST_F(DiffDriveSystemTest, RejectsMismatchedJointPrefixes)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(
    test_urdf(false, "foo_", "bar_"));
  ASSERT_EQ(infos.size(), 1u);
  robot_hardware::DiffDriveSystem system;
  EXPECT_EQ(
    system.on_init(infos.front()),
    hardware_interface::CallbackReturn::ERROR);
}

TEST_F(DiffDriveSystemTest, AcceptsSharedJointPrefix)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(
    test_urdf(false, "front_", "front_"));
  ASSERT_EQ(infos.size(), 1u);
  robot_hardware::DiffDriveSystem system;
  EXPECT_EQ(
    system.on_init(infos.front()),
    hardware_interface::CallbackReturn::SUCCESS);
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

TEST_F(DiffDriveSystemTest, ActivationFailsPromptlyAfterRosShutdown)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  {
    robot_hardware::DiffDriveSystem system;
    ASSERT_EQ(system.on_init(infos.front()), hardware_interface::CallbackReturn::SUCCESS);
    rclcpp::shutdown();

    hardware_interface::CallbackReturn result =
      hardware_interface::CallbackReturn::ERROR;
    const auto start = std::chrono::steady_clock::now();
    EXPECT_NO_THROW(result = system.on_activate(rclcpp_lifecycle::State()));
    const auto elapsed = std::chrono::steady_clock::now() - start;

    EXPECT_EQ(result, hardware_interface::CallbackReturn::FAILURE);
    EXPECT_LT(elapsed, 500ms);
  }
  rclcpp::init(0, nullptr);
}

TEST_F(DiffDriveSystemTest, ReadAfterCleanupReturnsErrorWithoutThrowing)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  robot_hardware::DiffDriveSystem system;
  ASSERT_EQ(system.on_init(infos.front()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_cleanup(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  hardware_interface::return_type result = hardware_interface::return_type::OK;
  EXPECT_NO_THROW(
    result = system.read(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)));
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(DiffDriveSystemTest, CleanupConfigureAndActivateRebuildsIo)
{
  Fake8030D fake;
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  robot_hardware::DiffDriveSystem system;
  ASSERT_EQ(system.on_init(infos.front()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_cleanup(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
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
  ASSERT_TRUE(fake.wait_for_event_counts("motor:0,0", 1u, "driver:1", 1u));
  ASSERT_TRUE(fake.wait_for_events_stable());
  const auto activation_events = fake.events();
  EXPECT_GE(std::count(activation_events.begin(), activation_events.end(), "motor:0,0"), 1);
  EXPECT_GE(std::count(activation_events.begin(), activation_events.end(), "driver:1"), 1);

  commands[0].set_value(1.0);
  commands[1].set_value(1.0);
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::OK);

  const auto command_deadline = std::chrono::steady_clock::now() + 1s;
  while (
    fake.command() != std::vector<int16_t>({-10, -10}) &&
    std::chrono::steady_clock::now() < command_deadline)
  {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(fake.command(), std::vector<int16_t>({-10, -10}));

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
  Fake8030D fake(1u);
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  robot_hardware::DiffDriveSystem system;
  ASSERT_EQ(system.on_init(infos.front()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  ASSERT_TRUE(fake.wait_for_event_counts("motor:0,0", 1u, "driver:1", 1u));
  ASSERT_TRUE(fake.wait_for_events_stable());
  fake.clear_events();
  std::this_thread::sleep_for(200ms);
  fake.publish_raw_feedback({123});
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::ERROR);

  ASSERT_TRUE(fake.wait_for_event_counts("motor:0,0", 1u, "driver:0", 1u));
  ASSERT_TRUE(fake.wait_for_events_stable());
  const auto stop_events = fake.events();
  EXPECT_EQ(stop_events.size(), 2u);
  EXPECT_EQ(std::count(stop_events.begin(), stop_events.end(), "motor:0,0"), 1);
  EXPECT_EQ(std::count(stop_events.begin(), stop_events.end(), "driver:0"), 1);
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

  ASSERT_TRUE(fake.wait_for_event_counts("motor:0,0", 1u, "driver:1", 1u));
  ASSERT_TRUE(fake.wait_for_events_stable());
  fake.clear_events();
  commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  commands[1].set_value(0.0);
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::ERROR);
  ASSERT_TRUE(fake.wait_for_event_counts("motor:0,0", 1u, "driver:0", 1u));
  ASSERT_TRUE(fake.wait_for_events_stable());
  const auto stop_events = fake.events();
  EXPECT_EQ(stop_events.size(), 2u);
  EXPECT_EQ(std::count(stop_events.begin(), stop_events.end(), "motor:0,0"), 1);
  EXPECT_EQ(std::count(stop_events.begin(), stop_events.end(), "driver:0"), 1);
}

TEST_F(DiffDriveSystemTest, RepeatedCleanupShutdownAndDestructionReleaseIoIdempotently)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(test_urdf());
  auto system = std::make_unique<robot_hardware::DiffDriveSystem>();
  ASSERT_EQ(system->on_init(infos.front()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system->on_cleanup(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system->on_cleanup(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system->on_shutdown(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system->on_shutdown(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  system.reset();
  SUCCEED();
}
