#pragma once

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>


#include <Eigen/Dense>
#include <vector>
#include <string>
#include <deque>
#include <mutex>

namespace suspension_controller {

  class SuspensionController : public controller_interface::ControllerInterface{
    public:
      SuspensionController();

      // Lifecycle
      controller_interface::CallbackReturn on_init() override;
      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
      controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

      controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

      // MANDATORY INTERFACE CONFIGURATION OVERRIDES
      controller_interface::InterfaceConfiguration command_interface_configuration() const override;
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;
      
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr terrain_pub_;
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kalman_pub_;


    private:
      enum ControlMode { PID_Base, LQR, IK_Based, PID_w_Terrain, PID_w_Terrain_Square, PID_Complete, DISABLED };
      ControlMode control_mode_;

      // PID
      Eigen::Vector4d P_r_;
      Eigen::Vector3d e_prev_;
      Eigen::Vector4d e_prev_full_;
      Eigen::Vector3d e_int_;
      Eigen::Vector4d e_int_full_;
      Eigen::Matrix4d K_P_, K_I_, K_D_;

      // LQR
      Eigen::Matrix3d Q_;
      Eigen::Matrix4d R_;
      Eigen::Matrix<double, 4, 3> K_;   
      Eigen::Matrix<double, 4, 3> K0_;   
      rclcpp::Time last_update_time_;

      // Terrain
      double lambda_ = 0.0001; // Regularization weight for terrain estimation


      // IMU filter
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
      std::mutex imu_mutex_;
      double latest_pitch_, latest_roll_, latest_yaw_;
      std::deque<double> pitch_window_, roll_window_, yaw_window_;
      const size_t filter_window_size_ = 5;
      double smoothed_pitch_ = 0.0;
      double smoothed_roll_ = 0.0;
      double smoothed_yaw_ = 0.0;
      const double alpha_ = 0.1;

      bool imu_ready_ = false;

      rclcpp::Time last_time_;

      // Kalman Filter
      Eigen::Matrix4d P_cov_;                  // covariance
      std::vector<double> noise_std_;          // [theta, roll, z, yaw]


      // Core control
      void runPIDBase(const rclcpp::Duration &period);
      void runLQRControl(const rclcpp::Duration& period);
      void runIKControl(const rclcpp::Duration& period);
      void runPIDWithTerrain(const rclcpp::Duration& period);
      void runPIDWithTerrainSquare(const rclcpp::Duration& period);
      void runPIDComplete(const rclcpp::Duration& period);

      // Helpers
      Eigen::Vector4d getMeasuredChassisPose();
      Eigen::Vector4d getActuatorPositions();
      std::tuple<double, double, double, double, double, double> suspensionFK(double d_LF, double d_LR, double d_RF, double d_RR);
      Eigen::Matrix<double, 4, 4> manualJacobianFK4DOF(double d_LF, double d_LR, double d_RF, double d_RR);
      Eigen::Matrix<double, 4, 3> solveContinuousLQR(const Eigen::Matrix3d& A, const Eigen::Matrix<double, 3, 4>& B, const Eigen::Matrix3d& Q, const Eigen::Matrix4d& R);
      Eigen::Vector4d solveSuspensionIK(const Eigen::Vector3d& target_pose, const Eigen::Vector4d& d_initial);
      Eigen::Vector4d estimateTerrainLASSO(const Eigen::Matrix<double, 3, 4> &J, const Eigen::Vector3d &residual, const Eigen::Vector4d &weights, double lambda);

    };


  

} // namespace suspension_controller
