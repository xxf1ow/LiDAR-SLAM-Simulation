#pragma once

#include <memory>

#include <vanjee_driver/msg/device_ctrl_msg.hpp>
#include <vanjee_driver/msg/lidar_parameter_interface_msg.hpp>
#include <vanjee_driver/msg/scan_data_msg.hpp>

namespace vanjee_lidar_ros::detail {

template <typename Driver>
void register_ignored_vendor_callbacks(Driver &driver) {
  // LidarDriver::init unconditionally requests these three output buffers.
  driver.regScanDataCallback(
      [] { return std::make_shared<vanjee::lidar::ScanData>(); },
      [](std::shared_ptr<vanjee::lidar::ScanData>) {});
  driver.regDeviceCtrlCallback(
      [] { return std::make_shared<vanjee::lidar::DeviceCtrl>(); },
      [](std::shared_ptr<vanjee::lidar::DeviceCtrl>) {});
  driver.regLidarParameterInterfaceCallback(
      [] { return std::make_shared<vanjee::lidar::LidarParameterInterface>(); },
      [](std::shared_ptr<vanjee::lidar::LidarParameterInterface>) {});
}

} // namespace vanjee_lidar_ros::detail
