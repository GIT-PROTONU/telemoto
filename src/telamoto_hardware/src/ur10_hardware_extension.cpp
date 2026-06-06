#include "telamoto_hardware/ur10_hardware_extension.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace telamoto_hardware
{

hardware_interface::CallbackReturn
UR10HardwareExtension::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(kNumJoints, 0.0);
  hw_velocities_.assign(kNumJoints, 0.0);
  hw_commands_.assign(kNumJoints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
UR10HardwareExtension::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kNumJoints * 2);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
UR10HardwareExtension::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kNumJoints);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn
UR10HardwareExtension::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("UR10HardwareExtension"), "Activating UR10 extension...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
UR10HardwareExtension::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("UR10HardwareExtension"), "Deactivating UR10 extension.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
UR10HardwareExtension::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // State is owned by ur_robot_driver's hardware interface.
  // Add custom read logic here (e.g. extra sensors, FT data).
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
UR10HardwareExtension::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Forward commands to ur_robot_driver hardware interface.
  // Add safety checks, payload compensation, etc. here.
  return hardware_interface::return_type::OK;
}

}  // namespace telamoto_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  telamoto_hardware::UR10HardwareExtension,
  hardware_interface::SystemInterface)
