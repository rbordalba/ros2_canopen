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
 * \date    2022-08-01
 *
 */
//----------------------------------------------------------------------

#ifndef CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
#define CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_


#include "canopen_ros2_control/canopen_system.hpp"


namespace canopen_ros2_control
{

    struct MotorTriggerCommand{
        double ons_cmd{std::numeric_limits<double>::quiet_NaN()};
        double resp;
    };

    struct MotorTarget : public  MotorTriggerCommand{
        double value;
    };

    struct MotorNodeData{

        // feedback
        double actual_position;
        double actual_speed;

        // basic control
        MotorTriggerCommand init;
        MotorTriggerCommand halt;
        MotorTriggerCommand recover;

        // mode control
        MotorTriggerCommand position_mode;
        MotorTriggerCommand velocity_mode;
        MotorTriggerCommand cyclic_velocity_mode;
        MotorTriggerCommand cyclic_position_mode;
        MotorTriggerCommand torque_mode;

        // setpoint
        MotorTarget target;
    };

using namespace ros2_canopen;
class Cia402System : public CanopenSystem
{
public:

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    ~Cia402System();
    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info);

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces();

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces();

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state);

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state);

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type read();

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type write();

    // can stuff
    std::map<uint, MotorNodeData> motor_data_;

private:




};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
