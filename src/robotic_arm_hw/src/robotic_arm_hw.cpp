#include "robotic_arm_hw/robotic_arm_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <chrono>

namespace robotic_arm_hw {

RoboticArmHW::RoboticArmHW() : logger_(rclcpp::get_logger("RoboticArmHW")) {}

hardware_interface::return_type RoboticArmHW::configure(const hardware_interface::HardwareInfo& info)
{
    if (configure_default(info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "Configuring...");

    serial_port_ = info_.hardware_parameters["device"];
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
    timeout_ms_ = std::stoi(info_.hardware_parameters["timeout"]);
    counts_per_revolution_ = std::stod(info_.hardware_parameters["enc_counts_per_rev"]);
    joint_names_.resize(info_.joints.size());
    joint_positions_.resize(info_.joints.size(), 0.0);
    joint_command_positions_.resize(info_.joints.size(), 0.0);

    for (size_t i = 0; i < info_.joints.size(); i++) {
        joint_names_[i] = info_.joints[i].name;
    }
    // Set up serial connection
    try {
        serial_conn_.setPort(serial_port_);
        serial_conn_.setBaudrate(baud_rate_);
        serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms_);
        serial_conn_.setTimeout(tt);
        serial_conn_.open();
        // Test the connection
       sendMsg("\r");
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(logger_, "Error opening serial port: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    time_ = std::chrono::system_clock::now();
    RCLCPP_INFO(logger_, "Finished Configuration");
    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> RoboticArmHW::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); i++) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
       }
      return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboticArmHW::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); i++) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_command_positions_[i]));
      }
    return command_interfaces;
}

hardware_interface::return_type RoboticArmHW::start()
{
    RCLCPP_INFO(logger_, "Starting Controller...");
    // Send an empty message to the arduino to verify connection.
    if(!sendMsg("\r")){
          RCLCPP_ERROR(logger_, "Could not communicate with the Arduino when starting.");
          return hardware_interface::return_type::ERROR;
    }

    status_ = hardware_interface::status::STARTED;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboticArmHW::stop()
{
    RCLCPP_INFO(logger_, "Stopping Controller...");
    status_ = hardware_interface::status::STOPPED;
    return hardware_interface::return_type::OK;
}

void RoboticArmHW::readEncoderValues(std::vector<int>& encoder_values)
{
    std::string response;
    if(!sendMsg("e\r", response))
      {
          RCLCPP_ERROR(logger_, "Could not read encoder values, serial communication issue");
          return;
      }
    std::stringstream ss(response);
    std::string value;

    encoder_values.clear();
    while (std::getline(ss, value, ' ')) {
        encoder_values.push_back(std::atoi(value.c_str()));
    }
}

void RoboticArmHW::setMotorValues(const std::vector<double>& positions)
{
    std::stringstream ss;
    ss << "m";
    for (size_t i = 0; i < positions.size(); i++) {
         double setpoint = positions[i] * (2 * M_PI / counts_per_revolution_);
            ss << " " << (int)setpoint;
    }
    ss << "\r";
     if(!sendMsg(ss.str()))
     {
            RCLCPP_ERROR(logger_, "Could not send motor setpoints to the Arduino");
           return;
     }
}

hardware_interface::return_type RoboticArmHW::read()
{
      if (!serial_conn_.isOpen()) {
            RCLCPP_ERROR(logger_, "Serial connection is not open.");
           return hardware_interface::return_type::ERROR;
       }
         std::vector<int> encoder_values;

        readEncoderValues(encoder_values);
          if(encoder_values.size() != joint_positions_.size())
        {
            RCLCPP_ERROR(logger_, "Number of encoder values does not match number of joints, check the arduino code and hardware description.");
            return hardware_interface::return_type::ERROR;
        }
    for (size_t i = 0; i < joint_positions_.size(); i++) {
     joint_positions_[i] = encoder_values[i] * (2 * M_PI / counts_per_revolution_);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboticArmHW::write()
{
      if (!serial_conn_.isOpen()) {
            RCLCPP_ERROR(logger_, "Serial connection is not open.");
            return hardware_interface::return_type::ERROR;
        }
      setMotorValues(joint_command_positions_);
    return hardware_interface::return_type::OK;
}

bool RoboticArmHW::sendMsg(const std::string &msg_to_send, std::string &response, bool print_output)
{
    if (!serial_conn_.isOpen())
    {
        RCLCPP_ERROR(logger_, "Serial connection is not open.");
        return false;
    }
     try {
        serial_conn_.write(msg_to_send);
         response = serial_conn_.readline();
    }
    catch (serial::IOException& e) {
            RCLCPP_ERROR(logger_, "Error communicating with the serial port: %s", e.what());
            return false;
     }

    if (print_output)
    {
       RCLCPP_INFO(logger_,"Sent: %s", msg_to_send.c_str());
       RCLCPP_INFO(logger_,"Received: %s", response.c_str());
    }
     return true;
}

bool RoboticArmHW::sendMsg(const std::string &msg_to_send, bool print_output)
{
     std::string response;
     return sendMsg(msg_to_send, response, print_output);
}

}  // namespace robotic_arm_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robotic_arm_hw::RoboticArmHW, hardware_interface::SystemInterface)