#include "gtest/gtest.h"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(DiffDriveSystemPlugin, LoadsByExportedName)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  auto plugin = loader.createSharedInstance("robot_hardware/DiffDriveSystem");
  ASSERT_NE(plugin, nullptr);
}
