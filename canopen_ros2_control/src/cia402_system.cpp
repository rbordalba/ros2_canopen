#include "canopen_ros2_control/cia402_system.hpp"

namespace
{
auto const kLogger = rclcpp::get_logger("CIA402System");
}

namespace canopen_ros2_control
{
}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(canopen_ros2_control::CIA402System, hardware_interface::SystemInterface)
