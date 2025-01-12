#ifndef ROBOTIC_ARM_HW_ROBOTIC_ARM_HW_H
#define ROBOTIC_ARM_HW_ROBOTIC_ARM_HW_H

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include <chrono>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <sstream>

namespace robotic_arm_hw {

class RoboticArmHW : public hardware_interface::SystemInterface
{
public:
    RoboticArmHW();
    
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type start() override;
    hardware_interface::return_type stop() override;
    hardware_interface::return_type read() override;
    hardware_interface::return_type write() override;

private:
    rclcpp::Logger logger_;
    // Configuration parameters
    std::string serial_port_;
    int32_t baud_rate_;
    int32_t timeout_ms_;
    double counts_per_revolution_;
    // Serial Communication
    serial::Serial serial_conn_;
    // Joint State Variables
    std::vector<double> joint_positions_;
    std::vector<double> joint_command_positions_;
    std::vector<std::string> joint_names_;

    // Time Tracking
    std::chrono::time_point<std::chrono::system_clock> time_;

    std::string sendMsg(const std::string &msg_to_send, bool print_output = false);
    void readEncoderValues(std::vector<int>& encoder_values);
    void setMotorValues(const std::vector<double>& positions);
};
}

#endif