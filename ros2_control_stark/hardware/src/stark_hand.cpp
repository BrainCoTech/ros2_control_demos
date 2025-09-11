#include "ros2_control_stark/stark_hand.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_stark
{
hardware_interface::CallbackReturn StarkHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.StarkHardware"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Initialize Stark SDK
  StarkProtocolType protocol_type_ = STARK_PROTOCOL_TYPE_MODBUS;
  LogLevel log_level_ = LOG_LEVEL_INFO;
  init_cfg(protocol_type_, log_level_);

  auto num_joints_ = info.joints.size();
  joint_names_.resize(num_joints_);
  for (size_t i = 0; i < num_joints_; i++) {
    joint_names_[i] = info.joints[i].name;
    // info.joints[i].state_interfaces.push_back("state");
    RCLCPP_INFO(get_logger(), "  Joint name: %s", joint_names_[i].c_str());
  }

  hw_pos_.assign(num_joints_, 0.0);
  hw_vel_.assign(num_joints_, 0.0);
  hw_cur_.assign(num_joints_, 0.0);
  hw_state_.assign(num_joints_, 0);

  cmd_pos_.assign(num_joints_, 0.0);
  cmd_vel_.assign(num_joints_, 0.0);
  cmd_cur_.assign(num_joints_, 0.0);
  cmd_pwm_.assign(num_joints_, 0.0);
  cmd_pos_time_.assign(num_joints_, 0.0);
  cmd_pos_vel_.assign(num_joints_, 0.0);
  // control_modes_.assign(num_joints_, ControlMode::POSITION);

  return hardware_interface::CallbackReturn::SUCCESS;
}

void StarkHardware::cleanup_stark_handler() {
  if (protocol_type_ == STARK_PROTOCOL_TYPE_MODBUS) {
    if (handle_) {
      modbus_close(handle_);
      handle_ = nullptr;
    }
  }
}

bool StarkHardware::initialize_stark_handler() {
  protocol_type_ = STARK_PROTOCOL_TYPE_MODBUS;
  if (protocol_type_ == STARK_PROTOCOL_TYPE_MODBUS) {
    // TODO: 传入参数
    port_ = "/dev/ttyUSB0"; // 适用于一只手连接一个串口

    // Revo1
    slave_id_ = 1;
    baudrate_ = 115200;
    // Revo2
    // slave_id_ = 127;
    // baudrate_ = 460800;
    handle_ = modbus_open(port_.c_str(), baudrate_);
    if (!handle_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open Modbus on %s", port_.c_str());
      return false;
    }
  // TODO: CAN/CAN-FD
  // } else if (protocol_type_ == STARK_PROTOCOL_TYPE_CAN) {
  //   handle_ = create_device_handler();  
  // } else if (protocol_type_ == STARK_PROTOCOL_TYPE_CAN_FD) {
  //   const uint8_t MASTER_ID = 1; // 主设备 ID
  //   handle_ = canfd_init(MASTER_ID);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupported protocol type");
    return false;
  }
  
  auto info = stark_get_device_info(handle_, slave_id_);
  if (info != NULL) {
    sku_type_ = info->sku_type;
    sn_ = std::string(info->serial_number);
    fw_version_ = std::string(info->firmware_version);
    fw_type_ = static_cast<StarkHardwareType>(info->hardware_type);
    RCLCPP_INFO(this->get_logger(), "Slave[%hhu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n", slave_id_,
                (uint8_t)info->sku_type, info->serial_number, info->firmware_version);
    free_device_info(info);
  }
  return true;
}

hardware_interface::CallbackReturn StarkHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_pos_.size(); i++)
  {
    hw_pos_[i] = 0.0;
    hw_vel_[i] = 0.0;
    hw_cur_[i] = 0.0;
    hw_state_[i] = 0;
  }
  cmd_mode_ = 0; // default to position mode
  if (!initialize_stark_handler()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize Stark handler");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

 hardware_interface::CallbackReturn StarkHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  cleanup_stark_handler();
  RCLCPP_INFO(get_logger(), "on_cleanup!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
StarkHardware::export_state_interfaces()
{
  // print joints name
  RCLCPP_INFO(get_logger(), "Exporting state interfaces:");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    auto joint_name = info_.joints[i].name;
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &hw_pos_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_EFFORT, &hw_cur_[i]));
    // TODO: add state interface for motor state
    // state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, "state", &hw_state_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
StarkHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    auto joint_name = info_.joints[i].name;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_POSITION, &cmd_pos_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_EFFORT, &cmd_cur_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, "pwm", &cmd_pwm_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, "position_with_time", &cmd_pos_time_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, "position_with_velocity", &cmd_pos_vel_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn StarkHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // command and state should be equal when starting
  for (uint i = 0; i < hw_pos_.size(); i++)
  {
    hw_pos_[i] = 0.0;
    hw_vel_[i] = 0.0;
    hw_cur_[i] = 0.0;
    hw_state_[i] = 0;
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StarkHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type StarkHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto motor_status = stark_get_motor_status(handle_, slave_id_);
  if (!motor_status) {
    RCLCPP_WARN(get_logger(), "Failed to get motor status");
    return hardware_interface::return_type::ERROR;
  }

  for (uint i = 0; i < hw_state_.size(); i++) {
    hw_state_[i] = motor_status->states[i];
    hw_pos_[i] = motor_status->positions[i];
    hw_vel_[i] = motor_status->speeds[i];
    hw_cur_[i] = motor_status->currents[i];
  }

  return hardware_interface::return_type::OK;
}

  hardware_interface::return_type StarkHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // RCLCPP_INFO(get_logger(), "Control Mode: %d, cmd_pos_.size=%ld", cmd_mode_, cmd_pos_.size());
    const uint16_t MAX_POS = 100; // 一代手最大位置100
    // const uint16_t MAX_POS = 1000; // 二代手最大位置1000
    switch (cmd_mode_)
    {
      case 0: { // position (with default duration)
        uint16_t positions[6] = {0};
        for (uint i = 0; i < cmd_pos_.size(); i++) {
          positions[i] = static_cast<uint16_t>(cmd_pos_[i] * MAX_POS);
          if (positions[i] > MAX_POS) positions[i] = MAX_POS;
        }
        // RCLCPP_INFO(get_logger(), "Positions: [%d, %d, %d, %d, %d, %d]", positions[0], positions[1], positions[2], positions[3], positions[4], positions[5]);
        stark_set_finger_positions(handle_, slave_id_, positions, 6);
        break;
      }

      case 1: { // velocity
        int16_t speeds[6] = {0};
        for (uint i = 0; i < cmd_vel_.size(); i++) {
          speeds[i] = static_cast<int16_t>(cmd_vel_[i] * MAX_POS);
          if (speeds[i] > MAX_POS) speeds[i] = MAX_POS;
          if (speeds[i] < -MAX_POS) speeds[i] = -MAX_POS;
        }
        stark_set_finger_speeds(handle_, slave_id_, speeds, 6);
        break;
      }

      case 2: { // current
        int16_t currents[6] = {0};
        for (uint i = 0; i < cmd_cur_.size(); i++) {
          currents[i] = static_cast<int16_t>(cmd_cur_[i] * MAX_POS);
          if (currents[i] > MAX_POS) currents[i] = MAX_POS;
          if (currents[i] < -MAX_POS) currents[i] = -MAX_POS;
        }
        stark_set_finger_currents(handle_, slave_id_, currents, 6); 
        break;
      }

      case 3: { // PWM
        int16_t pwms[6] = {0};
        for (uint i = 0; i < cmd_pwm_.size(); i++) {
          pwms[i] = static_cast<int16_t>(cmd_pwm_[i] * MAX_POS);
          if (pwms[i] > MAX_POS) pwms[i] = MAX_POS;
          if (pwms[i] < -MAX_POS) pwms[i] = -MAX_POS;
        }
        stark_set_finger_pwms(handle_, slave_id_, pwms, 6); 
        break;
      }

      case 4: { // position with time
        uint16_t positions[6] = {0};
        uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
        for (uint i = 0; i < cmd_pos_.size(); i++) {
          positions[i] = static_cast<uint16_t>(cmd_pos_[i] * MAX_POS);
          if (positions[i] > MAX_POS) positions[i] = MAX_POS;
        } 
        stark_set_finger_positions_and_durations(handle_, slave_id_, positions, durations, 6);
        break;
      }

      case 5: { // position with velocity
        uint16_t positions[6] = {0};
        uint16_t vels[6] = {0};
        for (uint i = 0; i < cmd_pos_.size(); i++) {
          positions[i] = static_cast<uint16_t>(cmd_pos_[i] * MAX_POS);
          if (positions[i] > MAX_POS) positions[i] = MAX_POS;
        } 
        for (uint i = 0; i < cmd_vel_.size(); i++) {
          vels[i] = static_cast<uint16_t>(cmd_vel_[i] * MAX_POS);
          if (vels[i] > MAX_POS) vels[i] = MAX_POS;
        }
        stark_set_finger_positions_and_speeds(handle_, slave_id_, positions, vels, 6);
        break;
      }

      default:
        RCLCPP_ERROR(get_logger(), "Invalid control mode");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

}  // namespace ros2_control_stark

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_stark::StarkHardware, hardware_interface::SystemInterface)
