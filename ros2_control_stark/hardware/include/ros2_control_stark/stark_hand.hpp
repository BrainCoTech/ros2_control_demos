#ifndef ros2_control_stark__HAND_HPP_
#define ros2_control_stark__HAND_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_stark/stark-sdk.h"

namespace ros2_control_stark
{
class StarkHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(StarkHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  bool initialize_stark_handler();
  void cleanup_stark_handler();

private:
  // Stark SDK handle
  DeviceHandler* handle_;
  uint8_t slave_id_;

  // Configuration parameters
  std::string port_;
  uint32_t baudrate_;
  StarkHardwareType fw_type_;
  StarkProtocolType protocol_type_;
  LogLevel log_level_;

  // device information
  SkuType sku_type_;
  std::string sn_;
  std::string fw_version_;

  // Joint names
  std::vector<std::string> joint_names_;

  // Parameters for the simulated robot
  std::vector<double> hw_pos_, hw_vel_, hw_cur_;
  std::vector<uint8_t> hw_state_;

  // Store the command 
  uint8_t cmd_mode_;
  std::vector<double> cmd_pos_, cmd_vel_, cmd_cur_, cmd_pwm_;
  std::vector<double> cmd_pos_time_, cmd_pos_vel_;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace ros2_control_stark

#endif  // ros2_control_stark__HAND_HPP_
