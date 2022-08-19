// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-06-29
 *
 */
//----------------------------------------------------------------------

#include <limits>
#include <vector>

#include "canopen_ros2_control/canopen_system.hpp"

namespace
{
auto const kLogger = rclcpp::get_logger("CanopenSystem");
}

namespace canopen_ros2_control
{
return_type CanopenSystem::configure(const hardware_interface::HardwareInfo& info)
{
  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(kLogger, "bus_config: '%s'", info_.hardware_parameters["bus_config"].c_str());
  RCLCPP_INFO(kLogger, "master_config: '%s'", info_.hardware_parameters["master_config"].c_str());
  RCLCPP_INFO(kLogger, "can_interface_name: '%s'", info_.hardware_parameters["can_interface_name"].c_str());

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_manager_ = std::make_shared<DeviceContainerNode>(executor_);

  std::thread spinThread([&]() {
    if (device_manager_->init())
    {
      RCLCPP_INFO(device_manager_->get_logger(), "Initialisation successful.");
    }
    else
    {
      RCLCPP_INFO(device_manager_->get_logger(), "Initialisation failed.");
    }
  });

  executor_->add_node(device_manager_);

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> CanopenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        // TODO(anyone): insert correct interfaces
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CanopenSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        // TODO(anyone): insert correct interfaces
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

return_type CanopenSystem::start()
{
  // TODO(anyone): prepare the robot to receive commands

  return return_type::OK;
}

return_type CanopenSystem::stop()
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return return_type::OK;
}

return_type CanopenSystem::read()
{
  // TODO(anyone): read robot states

  //  RCLCPP_INFO(kLogger, "read...");

  return return_type::OK;
}

return_type CanopenSystem::write()
{
  // TODO(anyone): write robot's commands'

  return return_type::OK;
}

}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(canopen_ros2_control::CanopenSystem, hardware_interface::SystemInterface)
