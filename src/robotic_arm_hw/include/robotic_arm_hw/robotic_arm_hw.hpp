#ifndef ROBOTIC_ARM_ARDUINO__SYSTEM_HPP_
#define ROBOTIC_ARM_ARDUINO__SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robotic_arm_hw/arduino_comms.hpp"
#include "robotic_arm_hw/joint.hpp"

namespace robotic_arm_hw
{
    class RoboticArmHW : public hardware_interface::SystemInterface
    {
        struct Config
        {
            std::string joint_1 = "";
            std::string joint_2 = "";
            std::string joint_3 = "";
            std::string joint_4 = "";
            std::string device = "";
            int baud_rate = 0;
            int timeout_ms = 0;
            int enc_counts_per_rev = 0;
        };

        public : RCLCPP_SHARED_PTR_DEFINITIONS(RoboticArmHW)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;
         hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;


        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;
    
    
       std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
       std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
       

    private:
        ArduinoComms comms_;
        Config cfg_;
        Joint joint_1_;
        Joint joint_2_;
        Joint joint_3_;
        Joint joint_4_;
    };

}

#endif