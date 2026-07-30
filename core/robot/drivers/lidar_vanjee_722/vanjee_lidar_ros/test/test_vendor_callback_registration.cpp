#include <functional>
#include <memory>

#include <gtest/gtest.h>
#include <vanjee_driver/msg/device_ctrl_msg.hpp>
#include <vanjee_driver/msg/lidar_parameter_interface_msg.hpp>
#include <vanjee_driver/msg/scan_data_msg.hpp>

#include "../src/vendor_callback_registration.hpp"

namespace {

class CallbackProbe {
public:
  void regScanDataCallback(
      const std::function<std::shared_ptr<vanjee::lidar::ScanData>()> &get,
      const std::function<void(std::shared_ptr<vanjee::lidar::ScanData>)> &put) {
    get_scan_data = get;
    put_scan_data = put;
  }

  void regDeviceCtrlCallback(
      const std::function<std::shared_ptr<vanjee::lidar::DeviceCtrl>()> &get,
      const std::function<void(std::shared_ptr<vanjee::lidar::DeviceCtrl>)>
          &put) {
    get_device_ctrl = get;
    put_device_ctrl = put;
  }

  void regLidarParameterInterfaceCallback(
      const std::function<
          std::shared_ptr<vanjee::lidar::LidarParameterInterface>()> &get,
      const std::function<
          void(std::shared_ptr<vanjee::lidar::LidarParameterInterface>)> &put) {
    get_lidar_parameter = get;
    put_lidar_parameter = put;
  }

  std::function<std::shared_ptr<vanjee::lidar::ScanData>()> get_scan_data;
  std::function<void(std::shared_ptr<vanjee::lidar::ScanData>)> put_scan_data;
  std::function<std::shared_ptr<vanjee::lidar::DeviceCtrl>()> get_device_ctrl;
  std::function<void(std::shared_ptr<vanjee::lidar::DeviceCtrl>)> put_device_ctrl;
  std::function<std::shared_ptr<vanjee::lidar::LidarParameterInterface>()>
    get_lidar_parameter;
  std::function<void(std::shared_ptr<vanjee::lidar::LidarParameterInterface>)>
    put_lidar_parameter;
};

TEST(VendorCallbackRegistration, RegistersEveryCallbackRequiredByDriverInit) {
  CallbackProbe driver;

  vanjee_lidar_ros::detail::register_ignored_vendor_callbacks(driver);

  ASSERT_TRUE(driver.get_scan_data);
  ASSERT_TRUE(driver.put_scan_data);
  ASSERT_TRUE(driver.get_device_ctrl);
  ASSERT_TRUE(driver.put_device_ctrl);
  ASSERT_TRUE(driver.get_lidar_parameter);
  ASSERT_TRUE(driver.put_lidar_parameter);

  EXPECT_NE(driver.get_scan_data(), nullptr);
  EXPECT_NE(driver.get_device_ctrl(), nullptr);
  EXPECT_NE(driver.get_lidar_parameter(), nullptr);
  EXPECT_NO_THROW(driver.put_scan_data(driver.get_scan_data()));
  EXPECT_NO_THROW(driver.put_device_ctrl(driver.get_device_ctrl()));
  EXPECT_NO_THROW(driver.put_lidar_parameter(driver.get_lidar_parameter()));
}

} // namespace
