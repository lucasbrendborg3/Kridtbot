#ifndef BNO055_HARDWARE_INTERFACE_
#define BNO055_HARDWARE_INTERFACE_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "imu_bno055/bno055_i2c_driver.h"

namespace bno055_hardware_interface
{
class BNO055HardwareInterface : public hardware_interface::SensorInterface
{

struct Config{
  std::string device = "/dev/i2c-1";
  std::string frame_id = "imu_link";
  int address = 40;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BNO055HardwareInterface);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

private:
  std::shared_ptr<imu_bno055::BNO055I2CDriver> imu_driver_;

  double orientation_[4];
  double angular_velocity_[3];
  double linear_acceleration_[3];

  Config cfg_;
};

}  // namespace bno055_hardware_interface

#endif  // BNO055_HARDWARE_INTERFACE_