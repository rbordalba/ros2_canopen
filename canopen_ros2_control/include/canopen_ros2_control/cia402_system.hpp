#ifndef CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
#define CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_

// #include "canopen_402_driver/canopen_402_driver.hpp"
#include "canopen_ros2_control/canopen_system.hpp"

namespace canopen_ros2_control
{
using namespace ros2_canopen;
using hardware_interface::return_type;

class CIA402System : public CanopenSystem
{
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
