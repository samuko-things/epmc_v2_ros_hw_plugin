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
    if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS )
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    config_.motor0_wheel_name = info_.hardware_parameters["motor0_wheel_name"];
    config_.motor1_wheel_name = info_.hardware_parameters["motor1_wheel_name"];
    config_.motor2_wheel_name = info_.hardware_parameters["motor2_wheel_name"];
    config_.motor3_wheel_name = info_.hardware_parameters["motor3_wheel_name"];
    config_.port = info_.hardware_parameters["port"];
    config_.cmd_vel_timeout_ms = info_.hardware_parameters["cmd_vel_timeout_ms"];
    config_.imu_sensor_name = info_.hardware_parameters["imu_sensor_name"];

    motor0_.setup(config_.motor0_wheel_name);
    motor1_.setup(config_.motor1_wheel_name);
    motor2_.setup(config_.motor2_wheel_name);
    motor3_.setup(config_.motor3_wheel_name);
    imu_.setup(config_.imu_sensor_name);

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

    state_interfaces.emplace_back(hardware_interface::StateInterface(motor2_.name, hardware_interface::HW_IF_POSITION, &motor2_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(motor2_.name, hardware_interface::HW_IF_VELOCITY, &motor2_.angVel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(motor3_.name, hardware_interface::HW_IF_POSITION, &motor3_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(motor3_.name, hardware_interface::HW_IF_VELOCITY, &motor3_.angVel));

    // Add IMU state interfaces
    state_interfaces.emplace_back(imu_.name, "orientation.x", &imu_.qx);
    state_interfaces.emplace_back(imu_.name, "orientation.y", &imu_.qy);
    state_interfaces.emplace_back(imu_.name, "orientation.z", &imu_.qz);
    state_interfaces.emplace_back(imu_.name, "orientation.w", &imu_.qw);

    state_interfaces.emplace_back(imu_.name, "angular_velocity.x", &imu_.gx);
    state_interfaces.emplace_back(imu_.name, "angular_velocity.y", &imu_.gy);
    state_interfaces.emplace_back(imu_.name, "angular_velocity.z", &imu_.gz);

    state_interfaces.emplace_back(imu_.name, "linear_acceleration.x", &imu_.ax);
    state_interfaces.emplace_back(imu_.name, "linear_acceleration.y", &imu_.ay);
    state_interfaces.emplace_back(imu_.name, "linear_acceleration.z", &imu_.az);

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EPMC_V2_HardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(motor0_.name, hardware_interface::HW_IF_VELOCITY, &motor0_.cmdAngVel));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(motor1_.name, hardware_interface::HW_IF_VELOCITY, &motor1_.cmdAngVel));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(motor2_.name, hardware_interface::HW_IF_VELOCITY, &motor2_.cmdAngVel));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(motor3_.name, hardware_interface::HW_IF_VELOCITY, &motor3_.cmdAngVel));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Configuring ...please wait...");
    if (epmcV2_.connected())
    {
      epmcV2_.disconnect();
    }
    epmcV2_.connect(config_.port);

    delay_ms(2000);

    epmcV2_.writeSpeed(0, 0.00);
    epmcV2_.writeSpeed(1, 0.00);
    epmcV2_.writeSpeed(2, 0.00);
    epmcV2_.writeSpeed(3, 0.00);

    int cmd_timeout = std::stoi(config_.cmd_vel_timeout_ms.c_str());
    epmcV2_.setCmdTimeout(cmd_timeout); // set motor command timeout
    cmd_timeout = epmcV2_.getCmdTimeout();

    imu_.use_imu = epmcV2_.getUseIMU();

    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "motor_cmd_timeout_ms: %d ms", (cmd_timeout));

    if(imu_.use_imu == 1){
      RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "MPU6050 IMU is available for use");
    }
    else {
      RCLCPP_WARN(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "No IMU available for use");
    }

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
    epmcV2_.writeSpeed(2, 0.00);
    epmcV2_.writeSpeed(3, 0.00);

    imu_.use_imu = epmcV2_.getUseIMU();

    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Successfully Activated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Deactivating ...please wait...");

    epmcV2_.writeSpeed(0, 0.00);
    epmcV2_.writeSpeed(1, 0.00);
    epmcV2_.writeSpeed(2, 0.00);
    epmcV2_.writeSpeed(3, 0.00);

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

      motor0_.angPos = epmcV2_.readPos(0);
      motor1_.angPos = epmcV2_.readPos(1);
      motor2_.angPos = epmcV2_.readPos(2);
      motor3_.angPos = epmcV2_.readPos(3);

      motor0_.angVel = epmcV2_.readVel(0);
      motor1_.angVel = epmcV2_.readVel(1);
      motor2_.angVel = epmcV2_.readVel(2);
      motor3_.angVel = epmcV2_.readVel(3);

      if (imu_.use_imu == 1){
        imu_.ax = epmcV2_.readAcc(0);
        imu_.ay = epmcV2_.readAcc(1);
        imu_.az = epmcV2_.readAcc(2);

        imu_.gx = epmcV2_.readGyro(0);
        imu_.gy = epmcV2_.readGyro(1);
        imu_.gz = epmcV2_.readGyro(2);
      }
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

    epmcV2_.writeSpeed(0, (float)motor0_.cmdAngVel);
    epmcV2_.writeSpeed(1, (float)motor1_.cmdAngVel);
    epmcV2_.writeSpeed(2, (float)motor2_.cmdAngVel);
    epmcV2_.writeSpeed(3, (float)motor3_.cmdAngVel);

    return hardware_interface::return_type::OK;
  }

} // namespace epmc_v2_ros_hw_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(epmc_v2_ros_hw_plugin::EPMC_V2_HardwareInterface, hardware_interface::SystemInterface)
