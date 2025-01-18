#include "robotic_arm_hw/robotic_arm_hw.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotic_arm_hw
{
hardware_interface::CallbackReturn RoboticArmHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.joint_1 = info_.hardware_parameters["joint_1"];
  cfg_.joint_2 = info_.hardware_parameters["joint_2"];
  cfg_.joint_3 = info_.hardware_parameters["joint_3"];
  cfg_.joint_4 = info_.hardware_parameters["joint_4"];

  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  

  joint_1_.setup(cfg_.joint_1, cfg_.enc_counts_per_rev);
  joint_2_.setup(cfg_.joint_2, cfg_.enc_counts_per_rev);
   joint_3_.setup(cfg_.joint_3, cfg_.enc_counts_per_rev);
   joint_4_.setup(cfg_.joint_4, cfg_.enc_counts_per_rev);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboticArmHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_1_.name, hardware_interface::HW_IF_POSITION, &joint_1_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_1_.name, hardware_interface::HW_IF_VELOCITY, &joint_1_.vel));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_2_.name, hardware_interface::HW_IF_POSITION, &joint_2_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_2_.name, hardware_interface::HW_IF_VELOCITY, &joint_2_.vel));
    
      state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_3_.name, hardware_interface::HW_IF_POSITION, &joint_3_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_3_.name, hardware_interface::HW_IF_VELOCITY, &joint_3_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_4_.name, hardware_interface::HW_IF_POSITION, &joint_4_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_4_.name, hardware_interface::HW_IF_VELOCITY, &joint_4_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboticArmHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
    joint_1_.name, hardware_interface::HW_IF_POSITION, &joint_1_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
    joint_2_.name, hardware_interface::HW_IF_POSITION, &joint_2_.cmd));
    
     command_interfaces.emplace_back(hardware_interface::CommandInterface(
    joint_3_.name, hardware_interface::HW_IF_POSITION, &joint_3_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
    joint_4_.name, hardware_interface::HW_IF_POSITION, &joint_4_.cmd));


  return command_interfaces;
}

hardware_interface::CallbackReturn RoboticArmHW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoboticArmHW"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("RoboticArmHW"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboticArmHW::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoboticArmHW"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("RoboticArmHW"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn RoboticArmHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoboticArmHW"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RoboticArmHW"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboticArmHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoboticArmHW"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("RoboticArmHW"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type RoboticArmHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Read encoder values
  int enc1, enc2,enc3,enc4;
  try {
    comms_.read_encoder_values(enc1, enc2,enc3,enc4);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RoboticArmHW"), 
      "Failed to read encoder values: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  joint_1_.enc = enc1;
  joint_2_.enc = enc2;
   joint_3_.enc = enc3;
    joint_4_.enc = enc4;

  joint_1_.pos = joint_1_.calc_enc_angle();
  joint_2_.pos = joint_2_.calc_enc_angle();
  joint_3_.pos = joint_3_.calc_enc_angle();
  joint_4_.pos = joint_4_.calc_enc_angle();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboticArmHW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }
  
  // Convert commanded positions to encoder counts
  int enc1 = static_cast<int>(joint_1_.cmd / joint_1_.rads_per_count);
  int enc2 = static_cast<int>(joint_2_.cmd / joint_2_.rads_per_count);
  int enc3 = static_cast<int>(joint_3_.cmd / joint_3_.rads_per_count);
  int enc4 = static_cast<int>(joint_4_.cmd / joint_4_.rads_per_count);

  try {
    comms_.set_motor_position(enc1, enc2, enc3, enc4);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RoboticArmHW"),
      "Failed to set motor positions: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace robotic_arm_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotic_arm_hw::RoboticArmHW, hardware_interface::SystemInterface)