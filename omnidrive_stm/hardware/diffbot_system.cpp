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

#include "omnidrive_stm/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omnidrive_stm
{
hardware_interface::CallbackReturn OmniDriveStmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.wheel_0 = info_.hardware_parameters["wheel_0"];
  cfg_.wheel_1 = info_.hardware_parameters["wheel_1"];
  cfg_.wheel_2 = info_.hardware_parameters["wheel_2"];
  cfg_.wheel_3 = info_.hardware_parameters["wheel_3"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  wheel_0_.setup(cfg_.wheel_0, cfg_.enc_counts_per_rev);
  wheel_1_.setup(cfg_.wheel_1, cfg_.enc_counts_per_rev);
  wheel_2_.setup(cfg_.wheel_2, cfg_.enc_counts_per_rev);
  wheel_3_.setup(cfg_.wheel_3, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveStmHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveStmHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveStmHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveStmHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveStmHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OmniDriveStmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_0_.name, hardware_interface::HW_IF_POSITION, &wheel_0_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_0_.name, hardware_interface::HW_IF_VELOCITY, &wheel_0_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_1_.name, hardware_interface::HW_IF_POSITION, &wheel_1_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_2_.name, hardware_interface::HW_IF_POSITION, &wheel_2_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_3_.name, hardware_interface::HW_IF_POSITION, &wheel_3_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OmniDriveStmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_0_.name, hardware_interface::HW_IF_VELOCITY, &wheel_0_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn OmniDriveStmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveStmHardware"), "Activating ...please wait...");

  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  RCLCPP_INFO(rclcpp::get_logger("OmniDriveStmHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniDriveStmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveStmHardware"), "Deactivating ...please wait...");

  comms_.disconnect();

  RCLCPP_INFO(rclcpp::get_logger("OmniDriveStmHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmniDriveStmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_0_.enc, wheel_1_.enc, wheel_2_.enc, wheel_3_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_0_.pos;
  wheel_0_.pos = wheel_0_.calc_enc_angle();
  wheel_0_.vel = (wheel_0_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_1_.pos;
  wheel_1_.pos = wheel_1_.calc_enc_angle();
  wheel_1_.vel = (wheel_1_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_2_.pos;
  wheel_2_.pos = wheel_2_.calc_enc_angle();
  wheel_2_.vel = (wheel_2_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_3_.pos;
  wheel_3_.pos = wheel_3_.calc_enc_angle();
  wheel_3_.vel = (wheel_3_.pos - pos_prev) / delta_seconds;


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type omnidrive_stm ::OmniDriveStmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_0 = wheel_0_.cmd / wheel_0_.rads_per_count / cfg_.loop_rate;
  int motor_1 = wheel_1_.cmd / wheel_1_.rads_per_count / cfg_.loop_rate;
  int motor_2 = wheel_2_.cmd / wheel_2_.rads_per_count / cfg_.loop_rate;
  int motor_3 = wheel_3_.cmd / wheel_3_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_0, motor_1, motor_2, motor_3);

  return hardware_interface::return_type::OK;
}

}  // namespace omnidrive_stm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  omnidrive_stm::OmniDriveStmHardware, hardware_interface::SystemInterface)
