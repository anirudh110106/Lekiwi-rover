#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "st_sc_servo_control_lib/SMS_STS.h"

#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace st3215_hardware_interface
{

class ST3215System : public hardware_interface::SystemInterface
{
public:

  hardware_interface::CallbackReturn on_init(
  const hardware_interface::HardwareInfo & info) override
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_name_ = info_.hardware_parameters["port_name"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  size_t n = info_.joints.size();

  position_.resize(n, 0.0);
  velocity_.resize(n, 0.0);
  cmd_position_.resize(n, 0.0);
  cmd_velocity_.resize(n, 0.0);
  last_sent_position_.resize(n, 0.0);   // ← CORRECT PLACE

  joint_ids_.resize(n);

  for (size_t i = 0; i < n; ++i)
  {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < position_.size(); ++i)
    {
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);

      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < cmd_position_.size(); ++i)
    {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);

      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]);
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn on_activate(
  const rclcpp_lifecycle::State &) override
{
  RCLCPP_INFO(rclcpp::get_logger("ST3215System"),
              "Opening serial port %s @ %d",
              port_name_.c_str(), baud_rate_);

  if (!servo_.begin(baud_rate_, port_name_.c_str()))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ST3215System"),
                 "Failed to open serial port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Enable torque
  for (auto id : joint_ids_)
  {
    servo_.EnableTorque(id, 1);
  }

  // ---- PASTE HERE ----

  double mid_rad = 2048.0 * (2.0 * M_PI / 4096.0);
  int mid_raw = 2048;

  int move_speed = 200;
  int move_acc   = 50;

  for (size_t i = 0; i < joint_ids_.size(); ++i)
  {
    position_[i] = mid_rad;
    cmd_position_[i] = mid_rad;

    servo_.WritePosEx(joint_ids_[i], mid_raw, move_speed, move_acc);

    last_sent_position_[i] = mid_rad;   // VERY IMPORTANT
  }

  // ---- END PASTE ----

  RCLCPP_INFO(rclcpp::get_logger("ST3215System"),
              "ST3215 hardware activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    for (auto id : joint_ids_)
    {
      servo_.EnableTorque(id, 0);
    }

    servo_.end();

    RCLCPP_INFO(rclcpp::get_logger("ST3215System"),
                "ST3215 hardware deactivated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

hardware_interface::return_type read(
  const rclcpp::Time &, const rclcpp::Duration &) override
{
  read_counter_++;

  // Only read every 5 cycles (at 50 Hz -> 10 Hz actual read)
  if (read_counter_ < 5)
    return hardware_interface::return_type::OK;

  read_counter_ = 0;

  for (size_t i = 0; i < joint_ids_.size(); ++i)
  {
    int raw_pos = servo_.ReadPos(joint_ids_[i]);

if (raw_pos == -1)
{
  // If one servo times out, stop reading others this cycle
  break;
}

position_[i] = raw_pos * (2.0 * M_PI / 4096.0);
    velocity_[i] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type write(
  const rclcpp::Time &, const rclcpp::Duration &) override
{
  for (size_t i = 0; i < joint_ids_.size(); ++i)
  {
    if (std::abs(cmd_position_[i] - last_sent_position_[i]) > 0.001)
    {
      int raw_pos = static_cast<int>(
        cmd_position_[i] * (4096.0 / (2.0 * M_PI)));

      servo_.WritePosEx(
        joint_ids_[i],
        raw_pos,
        300,
        50);

      last_sent_position_[i] = cmd_position_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

private:
  SMS_STS servo_;

  std::string port_name_;
  int baud_rate_;

  std::vector<int> joint_ids_;
  int read_counter_ = 0;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> cmd_position_;
  std::vector<double> cmd_velocity_;
  std::vector<double> last_sent_position_;

};

}  // namespace st3215_hardware_interface


PLUGINLIB_EXPORT_CLASS(
  st3215_hardware_interface::ST3215System,
  hardware_interface::SystemInterface)


  
