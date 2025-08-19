// Copyright 2021 ros2_control Development Team
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

#include "epmc_v2_ros_hw_plugin/epmc_v2_ros_hw_plugin.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

namespace epmc_v2_ros_hw_plugin
{
  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.motor0_wheel_name = info_.hardware_parameters["motor0_wheel_name"];
    cfg_.motor1_wheel_name = info_.hardware_parameters["motor1_wheel_name"];
    cfg_.port = info_.hardware_parameters["port"];
    cfg_.cmd_vel_timeout_ms = info_.hardware_parameters["cmd_vel_timeout_ms"];

    motor0_.setup(cfg_.motor0_wheel_name);
    motor1_.setup(cfg_.motor1_wheel_name);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // epmc_v2 System has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMC_V2_HardwareInterface"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMC_V2_HardwareInterface"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMC_V2_HardwareInterface"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMC_V2_HardwareInterface"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("EPMC_V2_HardwareInterface"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> EPMC_V2_HardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(motor0_.name, hardware_interface::HW_IF_POSITION, &motor0_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(motor0_.name, hardware_interface::HW_IF_VELOCITY, &motor0_.angVel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(motor1_.name, hardware_interface::HW_IF_POSITION, &motor1_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(motor1_.name, hardware_interface::HW_IF_VELOCITY, &motor1_.angVel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EPMC_V2_HardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(motor0_.name, hardware_interface::HW_IF_VELOCITY, &motor0_.cmdAngVel));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(motor1_.name, hardware_interface::HW_IF_VELOCITY, &motor1_.cmdAngVel));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Configuring ...please wait...");
    if (epmcV2_.connected())
    {
      epmcV2_.disconnect();
    }
    epmcV2_.connect(cfg_.port);
    for (int i = 1; i <= 5; i += 1)
    { // wait to fully setup
      delay_ms(1000);
      RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "configuring controller: %d sec", (i));
    }
    epmcV2_.writeSpeed(0, 0.00);
    epmcV2_.writeSpeed(1, 0.00);

    int cmd_timeout = std::stoi(cfg_.cmd_vel_timeout_ms.c_str());
    epmcV2_.setCmdTimeout(cmd_timeout); // set motor command timeout
    epmcV2_.getCmdTimeout(cmd_timeout);
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "motor_cmd_timeout_ms: %d ms", (cmd_timeout));

    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Cleaning up ...please wait...");
    if (epmcV2_.connected())
    {
      epmcV2_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Activating ...please wait...");
    if (!epmcV2_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    epmcV2_.writeSpeed(0, 0.00);
    epmcV2_.writeSpeed(1, 0.00);

    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Successfully Activated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Deactivating ...please wait...");

    epmcV2_.writeSpeed(0, 0.00);
    epmcV2_.writeSpeed(1, 0.00);

    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Successfully Deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type EPMC_V2_HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // vel = angdistchange/period.seconds();
    if (!epmcV2_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    try
    {
      float motor0_angPos, motor1_angPos;
      float motor0_angVel, motor1_angVel;

      epmcV2_.readPos(0, motor0_angPos);
      epmcV2_.readVel(0, motor0_angVel);

      epmcV2_.readPos(1, motor1_angPos);
      epmcV2_.readVel(1, motor1_angVel);

      motor0_.angPos = (double)motor0_angPos;
      motor1_.angPos = (double)motor1_angPos;

      motor0_.angVel = (double)motor0_angVel;
      motor1_.angVel = (double)motor1_angVel;
    }
    catch (...)
    {
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type epmc_v2_ros_hw_plugin ::EPMC_V2_HardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!epmcV2_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    float motor0_cmdAngVel, motor1_cmdAngVel;

    motor0_cmdAngVel = (float)motor0_.cmdAngVel;
    motor1_cmdAngVel = (float)motor1_.cmdAngVel;

    epmcV2_.writeSpeed(0, motor0_cmdAngVel);
    epmcV2_.writeSpeed(1, motor1_cmdAngVel);

    return hardware_interface::return_type::OK;
  }

} // namespace epmc_v2_ros_hw_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(epmc_v2_ros_hw_plugin::EPMC_V2_HardwareInterface, hardware_interface::SystemInterface)
