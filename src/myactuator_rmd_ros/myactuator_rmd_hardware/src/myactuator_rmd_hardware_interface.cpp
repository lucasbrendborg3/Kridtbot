#include "myactuator_rmd_hardware/myactuator_rmd_hardware_interface.hpp"

#include <atomic>
#include <chrono>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#if __has_include(<pthread.h>) && __has_include(<sched.h>)
  #include <pthread.h>
  #include <sched.h>
  #define MYACTUATOR_RMD_HARDWARE__THREAD_PRIORITY
#endif

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <myactuator_rmd/actuator_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>


#include "myactuator_rmd_hardware/conversions.hpp"
#include "myactuator_rmd_hardware/low_pass_filter.hpp"


namespace myactuator_rmd_hardware {

  using CallbackReturn = MyActuatorRmdHardwareInterface::CallbackReturn;

  MyActuatorRmdHardwareInterface::~MyActuatorRmdHardwareInterface() {
    // If the controller manager is shutdown with Ctrl + C the on_deactivate methods won't be called!
    // We therefore shut down the actuator here.
    on_cleanup(rclcpp_lifecycle::State());
    return;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_configure(rclcpp_lifecycle::State const& /*previous_state*/) {
    if (info_.hardware_parameters.find("ifname") != info_.hardware_parameters.end()) {
      ifname_ = info_.hardware_parameters["ifname"];
    } else {
      RCLCPP_FATAL(getLogger(), "Could not parse CAN interface name!");
      return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("actuator_id") != info_.hardware_parameters.end()) {
      actuator_id_ = std::stoi(info_.hardware_parameters["actuator_id"]);
    } else {
      RCLCPP_FATAL(getLogger(), "Could not parse CAN actuator id!");
      return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("torque_constant") != info_.hardware_parameters.end()) {
      torque_constant_ = std::stod(info_.hardware_parameters["torque_constant"]);
    } else {
      torque_constant_ = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(getLogger(), "Could not parse torque constant, won't be able to use torque interface!");
    }
    if (info_.hardware_parameters.find("max_velocity") != info_.hardware_parameters.end()) {
      max_velocity_ = std::stod(info_.hardware_parameters["max_velocity"]);
    } else {
      max_velocity_ = 720.0;
      RCLCPP_INFO(getLogger(), "Max velocity not set, defaulting to '%f'.", max_velocity_);
    }
    if (info_.hardware_parameters.find("velocity_alpha") != info_.hardware_parameters.end()) {
      auto const velocity_alpha {std::stod(info_.hardware_parameters["velocity_alpha"])};
      velocity_low_pass_filter_ = std::make_unique<LowPassFilter>(velocity_alpha);
      RCLCPP_INFO(getLogger(), "Using velocity low-pass filter with filter constant '%f'.", velocity_alpha);
    } else {
      velocity_low_pass_filter_ = nullptr;
      RCLCPP_INFO(getLogger(), "Not using velocity low-pass filter.");
    }
    if (info_.hardware_parameters.find("effort_alpha") != info_.hardware_parameters.end()) {
      auto const effort_alpha {std::stod(info_.hardware_parameters["effort_alpha"])};
      effort_low_pass_filter_ = std::make_unique<LowPassFilter>(effort_alpha);
      RCLCPP_INFO(getLogger(), "Using effort low-pass filter with filter constant '%f'.", effort_alpha);
    } else {
      effort_low_pass_filter_ = nullptr;
      RCLCPP_INFO(getLogger(), "Not using effort low-pass filter.");
    }
    if (info_.hardware_parameters.find("cycle_time") != info_.hardware_parameters.end()) {
      cycle_time_ = std::chrono::milliseconds(std::stol(info_.hardware_parameters["cycle_time"]));
    } else {
      cycle_time_ = std::chrono::milliseconds(2);
      RCLCPP_INFO(getLogger(), "Cycle time not set, defaulting to '%ld' ms.", cycle_time_.count());
    }
    if (info_.hardware_parameters.find("timeout") != info_.hardware_parameters.end()) {
      timeout_= std::chrono::milliseconds(std::stoi(info_.hardware_parameters["timeout"]));
    } else {
      timeout_ = std::chrono::milliseconds(0);
      RCLCPP_INFO(getLogger(), "Timeout not set, defaulting to '%ld' ms", timeout_.count());
    }
    if (timeout_ == std::chrono::milliseconds(0)) {
      RCLCPP_INFO(getLogger(), "Timeout set to 0ms, it will not be used!");
    }
    // How long to wait for the motor's reply to each request. A motor occasionally drops a request
    // (seen on one X8-PRO when another motor's request follows on the same bus); with the library's
    // default of 1 s that froze the wheel for a full second, now it only costs one cycle.
    std::chrono::milliseconds reply_timeout {10};
    if (info_.hardware_parameters.find("reply_timeout") != info_.hardware_parameters.end()) {
      reply_timeout = std::chrono::milliseconds(std::stol(info_.hardware_parameters["reply_timeout"]));
    } else {
      RCLCPP_INFO(getLogger(), "Reply timeout not set, defaulting to '%ld' ms.", reply_timeout.count());
    }
    driver_ = std::make_unique<myactuator_rmd::CanDriver>(ifname_);
    driver_->setRecvTimeout(reply_timeout);
    actuator_interface_ = std::make_unique<myactuator_rmd::ActuatorInterface>(*driver_, actuator_id_);
    if (!actuator_interface_) {
      RCLCPP_INFO(getLogger(), "Failed to create actuator interface!");
      return CallbackReturn::ERROR;
    }
    // The very first CAN read on a freshly opened socket can transiently fail (e.g. the
    // still-settling bus/controller state right after activation), so retry a few times
    // before giving up rather than crashing the whole controller manager on a one-off glitch.
    std::string motor_model {};
    constexpr int kMaxStartupAttempts {5};
    for (int attempt = 1; attempt <= kMaxStartupAttempts; ++attempt) {
      try {
        motor_model = actuator_interface_->getMotorModel();
        break;
      } catch (std::runtime_error const& e) {
        if (attempt == kMaxStartupAttempts) {
          RCLCPP_FATAL(getLogger(), "Failed to communicate with actuator after %d attempts: %s", kMaxStartupAttempts, e.what());
          return CallbackReturn::ERROR;
        }
        RCLCPP_WARN(getLogger(), "Initial communication attempt %d/%d failed (%s), retrying...", attempt, kMaxStartupAttempts, e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    RCLCPP_INFO(getLogger(), "Started actuator interface for actuator model '%s'!", motor_model.c_str());
    // Some motors (seen on an X8-PRO) ignore speed commands after power-up or reset: they reply but
    // stay idle (operating mode 0) until they have received a shutdown (motor off) command. So: send
    // motor off, then check with a 0 deg/s speed command that the motor really enters speed control,
    // and finish with motor off again so the wheel is left free until a controller takes over.
    // Nothing moves during this sequence.
    constexpr int kMaxEnableAttempts {3};
    bool speed_control_engaged {false};
    for (int attempt = 1; attempt <= kMaxEnableAttempts && !speed_control_engaged; ++attempt) {
      try {
        actuator_interface_->shutdownMotor();
        actuator_interface_->sendVelocitySetpoint(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        speed_control_engaged = (actuator_interface_->getControlMode() == myactuator_rmd::ControlMode::VELOCITY);
        actuator_interface_->shutdownMotor();
        if (!speed_control_engaged && attempt < kMaxEnableAttempts) {
          RCLCPP_WARN(getLogger(), "Actuator %u on '%s' did not enter speed control (attempt %d/%d), retrying...",
            actuator_id_, ifname_.c_str(), attempt, kMaxEnableAttempts);
        }
      } catch (std::runtime_error const& e) {
        RCLCPP_WARN(getLogger(), "Enabling speed control on actuator %u on '%s' failed (attempt %d/%d): %s",
          actuator_id_, ifname_.c_str(), attempt, kMaxEnableAttempts, e.what());
      }
      if (!speed_control_engaged) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    if (speed_control_engaged) {
      RCLCPP_INFO(getLogger(), "Actuator %u on '%s': speed control confirmed.", actuator_id_, ifname_.c_str());
    } else {
      RCLCPP_ERROR(getLogger(), "Actuator %u on '%s' did NOT enter speed control after %d attempts: this wheel will not drive! "
        "Power-cycle the motor and relaunch.", actuator_id_, ifname_.c_str(), kMaxEnableAttempts);
    }
    stop_async_thread_.store(false);
    if (!startAsyncThread(cycle_time_)) {
      RCLCPP_FATAL(getLogger(), "Failed to start async thread!");
      return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("direction") != info_.hardware_parameters.end()) {
      direction_ = std::stod(info_.hardware_parameters["direction"]);
    } else {
      direction_ = 1.0;
      RCLCPP_INFO(getLogger(), "Not using effort low-pass filter.");
    }

    return CallbackReturn::SUCCESS;
  }
      
  CallbackReturn MyActuatorRmdHardwareInterface::on_cleanup(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      // Also called from the destructor (e.g. on Ctrl+C, or during teardown after another
      // component failed to configure), so a transient CAN failure here must not be allowed
      // to escape as an uncaught exception and crash the whole controller manager.
      sendWithRetry("shut down motor", [this]() { actuator_interface_->shutdownMotor(); });
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_shutdown(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      sendWithRetry("shut down motor", [this]() { actuator_interface_->shutdownMotor(); });
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_activate(rclcpp_lifecycle::State const& /*previous_state*/) {
    RCLCPP_INFO(getLogger(), "Actuator successfully started!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_deactivate(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      // Runs during rollback of an already-active component if another one fails to configure,
      // so a transient CAN failure here must not crash the whole controller manager either.
      sendWithRetry("stop motor", [this]() { actuator_interface_->stopMotor(); });
    }
    RCLCPP_INFO(getLogger(), "Actuator successfully stopped!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_error(rclcpp_lifecycle::State const& /*previous_state*/) {
    stopAsyncThread();
    if (actuator_interface_) {
      sendWithRetry("stop motor", [this]() { actuator_interface_->stopMotor(); });
      if (sendWithRetry("reset motor", [this]() { actuator_interface_->reset(); })) {
        RCLCPP_INFO(getLogger(), "Actuator reset!");
      }
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorRmdHardwareInterface::on_init(hardware_interface::HardwareInfo const& info) {
    if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    position_state_ = 0.0;
    velocity_state_ = 0.0;
    effort_state_ = 0.0;
    position_command_ = 0.0;
    velocity_command_ = 0.0;
    effort_command_ = 0.0;

    position_interface_running_.store(false);
    velocity_interface_running_.store(false);
    effort_interface_running_.store(false);

    if (info_.joints.size() != 1) {
      RCLCPP_FATAL(getLogger(), "Expected a single joint but got %zu joints.", info_.joints.size());
      return CallbackReturn::ERROR;
    }
    hardware_interface::ComponentInfo const& joint = info_.joints.at(0);

    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu command interfaces found. 3 expected.",
        joint.name.c_str(), joint.command_interfaces.size()
      );
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
        joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY &&
        joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(),
        "Joint '%s' has unexpected command interface '%s'. Expected '%s', '%s' and '%s' ",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT
      );
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size()
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
      );
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT
      );
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MyActuatorRmdHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces {};
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &position_state_)
    );
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_VELOCITY, &velocity_state_)
    );
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_EFFORT, &effort_state_)
    );
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MyActuatorRmdHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces {};
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &position_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_VELOCITY, &velocity_command_)
    );
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints.at(0).name, hardware_interface::HW_IF_EFFORT, &effort_command_)
    );
    return command_interfaces;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::prepare_command_mode_switch(
    std::vector<std::string> const& start_interfaces, std::vector<std::string> const& stop_interfaces) {
    bool position_interface_claimed {position_interface_running_.load()};
    bool velocity_interface_claimed {velocity_interface_running_.load()};
    bool effort_interface_claimed {effort_interface_running_.load()};

    for (auto const& stop_interface: stop_interfaces) {
      if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        position_interface_claimed = false;
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_claimed = false;
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_claimed = false;
      }
    }

    for (auto const& start_interface: start_interfaces) {
      if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        if (velocity_interface_claimed || effort_interface_claimed) {
          RCLCPP_ERROR(getLogger(), "Can't claim position interface: Conflicting interface!");
          return hardware_interface::return_type::ERROR;
        }
        position_interface_claimed = true;
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        if (position_interface_claimed || effort_interface_claimed) {
          RCLCPP_ERROR(getLogger(), "Can't claim velocity interface: Conflicting interface!");
          return hardware_interface::return_type::ERROR;
        }
        velocity_interface_claimed = true;
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        if (std::isnan(torque_constant_)) {
          RCLCPP_ERROR(getLogger(), "Can't claim effort interface: Invalid torque constant!");
          return hardware_interface::return_type::ERROR;
        } else if (position_interface_claimed || velocity_interface_claimed) {
          RCLCPP_ERROR(getLogger(), "Can't claim effort interface: Conflicting interface!");
          return hardware_interface::return_type::ERROR;
        }
        effort_interface_claimed = true;
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::perform_command_mode_switch(
    std::vector<std::string> const& start_interfaces, std::vector<std::string> const& stop_interfaces) {
    for (auto const& stop_interface: stop_interfaces) {
      if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        position_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping position interface...");
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping velocity interface...");
      } else if (stop_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_running_.store(false);
        RCLCPP_INFO(getLogger(), "Stopping effort interface...");
      }
    }

    for (auto const& start_interface: start_interfaces) {
      if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_POSITION) {
        position_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Starting position interface...");
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_VELOCITY) {
        velocity_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Starting velocity interface...");
      } else if (start_interface == info_.joints.at(0).name + "/" + hardware_interface::HW_IF_EFFORT) {
        effort_interface_running_.store(true);
        RCLCPP_INFO(getLogger(), "Starting effort interface...");
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::read(rclcpp::Time const& /*time*/,
    rclcpp::Duration const& /*period*/) {
    position_state_ = direction_*async_position_state_.load();
    velocity_state_ = direction_*async_velocity_state_.load();
    effort_state_ = direction_*async_effort_state_.load();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorRmdHardwareInterface::write(rclcpp::Time const& /*time*/,
    rclcpp::Duration const& /*period*/) {
    // TODO: Make sure that all commands are finite
    if (position_interface_running_) {
      async_position_command_.store(direction_*position_command_);
    } else if (velocity_interface_running_) {
      async_velocity_command_.store(direction_*velocity_command_);
    } else if (effort_interface_running_) {
      async_effort_command_.store(direction_*effort_command_);
    }
    return hardware_interface::return_type::OK;
  }

  rclcpp::Logger MyActuatorRmdHardwareInterface::getLogger() {
    return rclcpp::get_logger("MyActuatorRmdHardwareInterface");
  }

  void MyActuatorRmdHardwareInterface::asyncThread(std::chrono::milliseconds const& cycle_time) {
    // Same transient-failure risk as the initial getMotorModel() read in on_configure(): this is
    // a real CAN write+read and must not be allowed to throw uncaught out of this thread.
    for (int attempt = 1; attempt <= 5; ++attempt) {
      try {
        actuator_interface_->setTimeout(timeout_);
        break;
      } catch (std::runtime_error const& e) {
        RCLCPP_WARN(getLogger(), "Failed to set timeout (attempt %d/5): %s", attempt, e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    std::size_t consecutive_failures {0};
    while (!stop_async_thread_) {
      auto const now {std::chrono::steady_clock::now()};
      auto const wakeup_time {now + cycle_time};
      // A single CAN read/write can transiently fail (e.g. a stray bus error frame). Skip this
      // cycle (keeping the last known state) instead of letting an uncaught exception escape this
      // thread, which would call std::terminate() and crash the whole controller manager.
      try {
        if (position_interface_running_) {
          feedback_ = actuator_interface_->sendPositionAbsoluteSetpoint(radToDeg(async_position_command_.load()), max_velocity_);
        } else if (velocity_interface_running_) {
          feedback_ = actuator_interface_->sendVelocitySetpoint(radToDeg(async_velocity_command_.load()));
        } else if (effort_interface_running_) {
          feedback_ = actuator_interface_->sendTorqueSetpoint(async_effort_command_.load(), torque_constant_);
        } else {
          feedback_ = actuator_interface_->getMotorStatus2();
        }

        double const position_state {feedback_.shaft_angle};
        double velocity_state {feedback_.shaft_speed};
        if (velocity_low_pass_filter_) {
          velocity_state = velocity_low_pass_filter_->apply(velocity_state);
        }
        double current_state {feedback_.current};
        if (effort_low_pass_filter_) {
          current_state = effort_low_pass_filter_->apply(current_state);
        }
        async_position_state_.store(degToRad(position_state));
        async_velocity_state_.store(degToRad(velocity_state));
        async_effort_state_.store(currentToTorque(current_state, torque_constant_));

        if (consecutive_failures > 0) {
          RCLCPP_INFO(getLogger(), "Recovered after %zu failed communication cycle(s).", consecutive_failures);
          consecutive_failures = 0;
        }
      } catch (std::runtime_error const& e) {
        ++consecutive_failures;
        // Log the first failure immediately, then only occasionally to avoid spamming at cycle rate.
        if (consecutive_failures == 1 || consecutive_failures % 250 == 0) {
          RCLCPP_WARN(getLogger(), "Communication cycle failed (%zu consecutive): %s", consecutive_failures, e.what());
        }
      }

      std::this_thread::sleep_until(wakeup_time);
    }
    try {
      actuator_interface_->setTimeout(std::chrono::milliseconds(0));
    } catch (std::runtime_error const& e) {
      RCLCPP_WARN(getLogger(), "Failed to clear timeout on shutdown: %s", e.what());
    }
    return;
  }

  bool MyActuatorRmdHardwareInterface::startAsyncThread(std::chrono::milliseconds const& cycle_time) {
    if (!async_thread_.joinable()) {
      async_thread_ = std::thread(&MyActuatorRmdHardwareInterface::asyncThread, this, cycle_time);
    } else {
      RCLCPP_WARN(getLogger(), "Could not start command thread, command thread already running!");
      return false;
    }

#ifdef MYACTUATOR_RMD_HARDWARE__THREAD_PRIORITY
    std::ifstream realtime_file {"/sys/kernel/realtime", std::ios::in};
    bool has_realtime {false};
    if (realtime_file.is_open()) {
      realtime_file >> has_realtime;
    }

    int policy {};
    struct ::sched_param param {};
    ::pthread_getschedparam(async_thread_.native_handle(), &policy, &param);
    if (has_realtime) {
      policy = SCHED_FIFO;
      RCLCPP_INFO(getLogger(), "Real-time system detected: Setting policy to 'SCHED_FIFO'...");
    }
    int const max_thread_priority {::sched_get_priority_max(policy)};
    if (max_thread_priority != -1) {
      param.sched_priority = max_thread_priority;
      if (::pthread_setschedparam(async_thread_.native_handle(), policy, &param) == 0) {
        RCLCPP_INFO(getLogger(), "Set thread priority '%d' and policy '%d' to async thread!",
          param.sched_priority, policy);
      } else {
        RCLCPP_WARN(getLogger(), "Failed to set thread priority '%d' and policy '%d' to async thread!",
          param.sched_priority, policy);
      }
    } else {
      RCLCPP_WARN(getLogger(), "Could not set thread priority to async thread: Failed to get max priority!");
    }
#endif  // MYACTUATOR_RMD_HARDWARE__THREAD_PRIORITY

    return true;
  }

  void MyActuatorRmdHardwareInterface::stopAsyncThread() {
    if (async_thread_.joinable()) {
      stop_async_thread_.store(true);
      async_thread_.join();
    } else {
      RCLCPP_WARN(getLogger(), "Could not stop command thread: Not running!");
    }
    return;
  }

  bool MyActuatorRmdHardwareInterface::sendWithRetry(char const* description, std::function<void()> const& command) {
    constexpr int kMaxAttempts {5};
    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
      try {
        command();
        return true;
      } catch (std::runtime_error const& e) {
        if (attempt == kMaxAttempts) {
          RCLCPP_ERROR(getLogger(), "Actuator %u on '%s': failed to %s after %d attempts: %s",
            actuator_id_, ifname_.c_str(), description, kMaxAttempts, e.what());
        }
      }
    }
    return false;
  }

}  // namespace myactuator_rmd_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(myactuator_rmd_hardware::MyActuatorRmdHardwareInterface, hardware_interface::ActuatorInterface)
