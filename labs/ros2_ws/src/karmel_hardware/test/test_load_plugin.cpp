#include <gtest/gtest.h>

#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

// The controller_manager finds hardware plugins by the name in the URDF's <plugin> tag.
TEST(KarmelSystem, PluginIsDiscoverableAndInstantiable)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  ASSERT_TRUE(loader.isClassAvailable("karmel_hardware/KarmelSystem"));
  std::shared_ptr<hardware_interface::SystemInterface> system;
  ASSERT_NO_THROW(system = loader.createSharedInstance("karmel_hardware/KarmelSystem"));
  EXPECT_NE(system, nullptr);
}
