#include "bno055_hardware_interface/bno055_hardware.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "imu_bno055/bno055_i2c_driver.h"

namespace bno055_hardware_interface
{
hardware_interface::CallbackReturn BNO055HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.device = info_.hardware_parameters["device"];
  cfg_.address = std::stoi(info_.hardware_parameters["address"]);
  cfg_.frame_id = info_.hardware_parameters["frame_id"];
  
  imu_driver_ = std::make_shared<imu_bno055::BNO055I2CDriver>(cfg_.device, cfg_.address);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BNO055HardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    std::string("imu"), std::string("orientation.x"), &orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      std::string("imu"), std::string("orientation.y"), &orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      std::string("imu"), std::string("orientation.z"), &orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      std::string("imu"), std::string("orientation.w"), &orientation_[3]));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      std::string("imu"), std::string("angular_velocity.x"), &angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      std::string("imu"), std::string("angular_velocity.y"), &angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      std::string("imu"), std::string("angular_velocity.z"), &angular_velocity_[2]));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      std::string("imu"), std::string("linear_acceleration.x"), &linear_acceleration_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      std::string("imu"), std::string("linear_acceleration.y"), &linear_acceleration_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      std::string("imu"), std::string("linear_acceleration.z"), &linear_acceleration_[2]));
  return state_interfaces;
}

hardware_interface::CallbackReturn BNO055HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  imu_driver_->init();

  RCLCPP_INFO(get_logger(), "Successfully activated bno055!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO055HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BNO055HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  imu_bno055::IMURecord record;

  try {
      record = imu_driver_->read();
  } catch(const std::runtime_error& e) {
      RCLCPP_WARN(this->get_logger(), e.what());
  }

  orientation_[0] = record.fused_orientation_x;
  orientation_[1] = record.fused_orientation_y;
  orientation_[2] = record.fused_orientation_z;
  orientation_[3] = record.fused_orientation_w;

  linear_acceleration_[0] = record.fused_linear_acceleration_x / 100.0;
  linear_acceleration_[1] = record.fused_linear_acceleration_y / 100.0;
  linear_acceleration_[2] = record.fused_linear_acceleration_z / 100.0;

  angular_velocity_[0] = record.raw_angular_velocity_x / 900.0;
  angular_velocity_[1] = record.raw_angular_velocity_y / 900.0;
  angular_velocity_[2] = record.raw_angular_velocity_z / 900.0;

  return hardware_interface::return_type::OK;
}

}  // namespace bno055_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bno055_hardware_interface::BNO055HardwareInterface,
  hardware_interface::SensorInterface)
