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

#ifndef CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_
#define CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_

#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "canopen_core/device_container_node.hpp"
#include "canopen_proxy_driver/canopen_proxy_driver.hpp"
#include "canopen_ros2_control/visibility_control.h"

namespace canopen_ros2_control
{
using namespace ros2_canopen;
using hardware_interface::return_type;

class CanopenSystem : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  ~CanopenSystem();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type start() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type stop() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type read() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  return_type write() override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  std::shared_ptr<DeviceContainerNode> device_manager_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<rclcpp_components::ComponentManager> component_manager_;
  std::shared_ptr<rclcpp::Node> node_;

  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<std::thread> init_thread_;
  void spin();
  void initDeviceManager();
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_
