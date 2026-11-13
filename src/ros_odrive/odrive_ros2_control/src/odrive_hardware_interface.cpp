
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <mutex>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"

namespace odrive_ros2_control {

class Axis;

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    ~ODriveHardwareInterface() override;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;
    CallbackReturn on_shutdown(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);
    void set_axis_command_mode(Axis& axis);
    void hold_or_idle(Axis& axis);
    void read_estimates(double timeout_s);
    bool load_position_limits();
    void park_legs_down(const char* why);

    bool active_;
    bool trap_traj_ = false;    // position control through the ODrive's trapezoidal trajectory planner
    double traj_vel_limit_ = NAN;    // [m/s] sent to the ODrive's trajectory planner, if set
    double traj_accel_limit_ = NAN;  // [m/s^2] (acceleration and deceleration), if set
    // When no controller is using an axis (at start-up, between controllers, after the launch stops):
    // false = hold its current position, true = switch it to idle. Idle lets the rover's weight push
    // the legs down (the right front leg dropped all the way to its end stop that way).
    bool idle_when_unused_ = false;

    // Park on shutdown (Ctrl+C): drive every leg down to its lower soft limit from the calibration
    // file before the process exits. See park_legs_down().
    bool park_on_shutdown_ = false;
    double park_speed_ = 0.04;       // [m/s] software ramp speed of the parking move
    double park_timeout_ = 8.0;      // [s] give up (and hold where the legs are) after this long
    double park_max_torque_ = 0.8;   // [Nm] a leg that needs more than this is stopped and held
    std::atomic<bool> parked_{false};        // set once parking has started; write() stops sending
    std::mutex io_mutex_;                    // parking owns the CAN socket; read()/write() skip meanwhile
    bool can_open_ = false;
    rclcpp::PreShutdownCallbackHandle preshutdown_handle_;
    bool preshutdown_registered_ = false;

    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id) : can_intf_(can_intf), node_id_(node_id) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    void on_can_msg();

    SocketCanIntf* can_intf_;
    uint32_t node_id_;

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0f; // [rad]
    double vel_setpoint_ = 0.0f; // [rad/s]
    double torque_setpoint_ = 0.0f; // [Nm]

    // State (ODrives => ros2_control)
    // rclcpp::Time encoder_estimates_timestamp_;
    // uint32_t axis_error_ = 0;
    // uint8_t axis_state_ = 0;
    // uint8_t procedure_result_ = 0;
    // uint8_t trajectory_done_flag_ = 0;
    double pos_estimate_ = NAN; // [rad]
    double vel_estimate_ = NAN; // [rad/s]
    // double iq_setpoint_ = NAN;
    // double iq_measured_ = NAN;
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    uint8_t axis_state_ = 0;            // from the heartbeat
    bool have_heartbeat_ = false;
    // uint32_t active_errors_ = 0;
    // uint32_t disarm_reason_ = 0;
    // double fet_temperature_ = NAN;
    // double motor_temperature_ = NAN;
    // double bus_voltage_ = NAN;
    // double bus_current_ = NAN;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    // Position limits [m] enforced on every command (see load_position_limits())
    double min_pos_ = -std::numeric_limits<double>::infinity();
    double max_pos_ = std::numeric_limits<double>::infinity();
    bool limit_warned_ = false;
    // true when min_pos_ comes from the bottom homing in the limits file (hard_min_m + margin_mm),
    // i.e. a real lower soft limit that parking on shutdown may drive to
    bool min_from_calibration_ = false;

    // Holding the position the axis had when no controller was using it (see hold_or_idle())
    bool holding_ = false;
    double hold_pos_ = NAN; // [m]

    template <typename T>
    void send(const T& msg) const {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_intf_name_ = info_.hardware_parameters["can"];

    for (auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")));
    }

    auto const mode = info_.hardware_parameters.find("position_input_mode");
    trap_traj_ = mode != info_.hardware_parameters.end() && mode->second == "trap_traj";
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Position control input mode: %s",
        trap_traj_ ? "trapezoidal trajectory" : "passthrough");
    if (trap_traj_) {
        // Without these the ODrives' own trap_traj settings apply (their defaults capped the legs at ~5 mm/s)
        auto const vel = info_.hardware_parameters.find("traj_vel_limit");
        auto const acc = info_.hardware_parameters.find("traj_accel_limit");
        if (vel != info_.hardware_parameters.end()) { traj_vel_limit_ = std::stod(vel->second); }
        if (acc != info_.hardware_parameters.end()) { traj_accel_limit_ = std::stod(acc->second); }
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Trajectory limits: %s mm/s, %s mm/s^2",
            std::isfinite(traj_vel_limit_) ? std::to_string(traj_vel_limit_ * 1000).c_str() : "ODrive's own",
            std::isfinite(traj_accel_limit_) ? std::to_string(traj_accel_limit_ * 1000).c_str() : "ODrive's own");
    }

    auto const unused = info_.hardware_parameters.find("idle_when_unused");
    idle_when_unused_ = unused != info_.hardware_parameters.end() && unused->second == "true";
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Axes without a controller are %s",
        idle_when_unused_ ? "switched to idle" : "held at their current position");

    if (!load_position_limits()) {
        return CallbackReturn::ERROR;   // a leg isn't homed: refuse to drive the suspension
    }

    auto const param = [&](const char* name, double& value) {
        auto const it = info_.hardware_parameters.find(name);
        if (it != info_.hardware_parameters.end() && !it->second.empty()) { value = std::stod(it->second); }
    };
    auto const park = info_.hardware_parameters.find("park_on_shutdown");
    park_on_shutdown_ = park != info_.hardware_parameters.end() && park->second == "true";
    param("park_speed", park_speed_);
    param("park_timeout", park_timeout_);
    param("park_max_torque", park_max_torque_);
    if (park_speed_ <= 0.0) { park_speed_ = 0.04; }
    if (park_on_shutdown_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
            "On shutdown (Ctrl+C) the legs are driven down to their lower soft limits at %.1f mm/s", park_speed_ * 1000);
        // The controller manager normally shuts the hardware down on Ctrl+C (on_shutdown() parks).
        // This is the fallback in case it doesn't; parking only ever runs once.
        preshutdown_handle_ = rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
            [this]() { park_legs_down("ROS shutdown"); });
        preshutdown_registered_ = true;
    }

    return CallbackReturn::SUCCESS;
}

ODriveHardwareInterface::~ODriveHardwareInterface() {
    if (preshutdown_registered_) {
        rclcpp::contexts::get_global_default_context()->remove_pre_shutdown_callback(preshutdown_handle_);
    }
}

// Per-joint position limits, in metres:
//  - lower: ONLY the bottom homing by kridtbot's calibrate_suspension (hard_min_m = the bottom end stop,
//    plus margin_mm) in the YAML file given by the "limits_file" hardware parameter. There is no
//    fallback: a leg without it gets no lower limit and a warning to run calibrate_suspension.
//  - upper: the max of the joint's position command interface in the URDF. The top end stop is never
//    searched for (driving into it could unscrew the ball screw), so this is the only upper bound.
// These limits are only right while the legs have been homed since the ODrives last powered up.
// Returns false (the hardware then fails to initialize) if any leg has no lower limit.
bool ODriveHardwareInterface::load_position_limits() {
    auto const logger = rclcpp::get_logger("ODriveHardwareInterface");
    YAML::Node legs;
    auto const file = info_.hardware_parameters.find("limits_file");
    if (file != info_.hardware_parameters.end() && !file->second.empty()) {
        if (std::filesystem::exists(file->second)) {
            try {
                legs = YAML::LoadFile(file->second)["legs"];
            } catch (YAML::Exception const& e) {
                RCLCPP_ERROR(logger, "Could not read limits file '%s': %s", file->second.c_str(), e.what());
            }
        } else {
            RCLCPP_WARN(logger, "Limits file '%s' does not exist", file->second.c_str());
        }
    } else {
        RCLCPP_WARN(logger, "No limits_file hardware parameter set");
    }

    std::vector<std::string> missing;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        auto const& joint = info_.joints[i];
        Axis& axis = axes_[i];
        for (auto const& ci : joint.command_interfaces) {
            if (ci.name == hardware_interface::HW_IF_POSITION && !ci.max.empty()) {
                axis.max_pos_ = std::stod(ci.max);
            }
        }
        YAML::Node const leg = legs ? legs[joint.name] : YAML::Node();
        if (leg && leg["hard_min_m"]) {
            double const margin = leg["margin_mm"] ? leg["margin_mm"].as<double>() / 1000.0 : 0.003;
            axis.min_pos_ = leg["hard_min_m"].as<double>() + margin;
            axis.min_from_calibration_ = true;
            RCLCPP_INFO(logger, "%s (node %u): position limited to %.1f .. %.1f mm (homed %s)", joint.name.c_str(),
                axis.node_id_, axis.min_pos_ * 1000, axis.max_pos_ * 1000,
                leg["homed"] ? leg["homed"].as<std::string>().c_str() : "?");
        } else {
            missing.push_back(joint.name);
            RCLCPP_ERROR(logger, "%s (node %u): NO LOWER LIMIT - this leg has not been homed.",
                joint.name.c_str(), axis.node_id_);
        }
    }
    if (!missing.empty()) {
        RCLCPP_ERROR(logger, "%zu suspension leg(s) not homed - the ODrive hardware will not start. "
            "Run: ros2 run kridtbot calibrate_suspension", missing.size());
        return false;
    }
    return true;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ODriveHardwareInterface"),
            "Failed to initialize SocketCAN on %s",
            can_intf_name_.c_str()
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    can_open_ = true;
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    can_open_ = false;
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

// Ctrl+C on the launch: the controller manager shuts the hardware down, which ends up here
CallbackReturn ODriveHardwareInterface::on_shutdown(const State&) {
    park_legs_down("hardware shutdown");
    return CallbackReturn::SUCCESS;
}

// Drives every leg that has a calibrated lower limit down to it (the homed bottom end stop + margin_mm,
// from calibrate_suspension), all legs together on a linear ramp so they arrive
// at the same moment, then holds them there (or idles them with idle_when_unused). Runs at most once.
// A leg that disarms or needs more than park_max_torque is stopped and held where it is; after
// park_timeout the remaining legs hold wherever they got to.
void ODriveHardwareInterface::park_legs_down(const char* why) {
    if (!park_on_shutdown_ || parked_.exchange(true)) {
        return;
    }
    std::lock_guard<std::mutex> lock(io_mutex_);    // the control loop's read()/write() skip from now on
    auto const logger = rclcpp::get_logger("ODriveHardwareInterface");
    if (!can_open_) {
        RCLCPP_WARN(logger, "Parking (%s): CAN interface not open, legs not moved", why);
        return;
    }
    read_estimates(0.5);

    std::vector<size_t> legs;
    std::vector<double> start, target;
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        if (!axis.min_from_calibration_) {
            RCLCPP_WARN(logger, "Parking (%s): %s has no calibrated lower limit (run calibrate_suspension), not moved",
                why, info_.joints[i].name.c_str());
            continue;
        }
        if (!std::isfinite(axis.pos_estimate_)) {
            RCLCPP_WARN(logger, "Parking (%s): no position from %s, not moved", why, info_.joints[i].name.c_str());
            continue;
        }
        legs.push_back(i);
        start.push_back(axis.pos_estimate_);
        target.push_back(axis.min_pos_);
    }
    if (legs.empty()) {
        return;
    }

    // Position control, straight setpoints (the ramp is done here), starting from where each leg is.
    // Errors are not cleared, so an ODrive that tripped on a fault stays off.
    double max_dist = 0.0;
    for (size_t k = 0; k < legs.size(); ++k) {
        Axis& axis = axes_[legs[k]];
        axis.holding_ = false;
        Set_Controller_Mode_msg_t control_msg;
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
        control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
        axis.send(control_msg);
        Set_Input_Pos_msg_t pos_msg;
        pos_msg.Input_Pos = start[k] / 0.005;
        axis.send(pos_msg);
        Set_Axis_State_msg_t state_msg;
        state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;
        axis.send(state_msg);
        max_dist = std::max(max_dist, std::abs(target[k] - start[k]));
    }
    double const duration = max_dist / park_speed_;
    RCLCPP_INFO(logger, "Parking (%s): driving %zu leg(s) down to their lower soft limits (%.1f s) ...",
        why, legs.size(), duration);

    auto const t0 = std::chrono::steady_clock::now();
    std::vector<bool> stopped(legs.size(), false);
    bool timed_out = false;
    while (true) {
        while (can_intf_.read_nonblocking()) {
        }
        double const elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        double const frac = duration > 0.0 ? std::min(1.0, elapsed / duration) : 1.0;
        bool all_there = true;
        for (size_t k = 0; k < legs.size(); ++k) {
            if (stopped[k]) {
                continue;
            }
            Axis& axis = axes_[legs[k]];
            char const* name = info_.joints[legs[k]].name.c_str();
            if (elapsed > 1.0 && axis.have_heartbeat_ && axis.axis_state_ != AXIS_STATE_CLOSED_LOOP_CONTROL) {
                RCLCPP_ERROR(logger, "Parking: %s is not in closed loop control (state %u), left as it is",
                    name, axis.axis_state_);
                stopped[k] = true;
                continue;
            }
            if (std::isfinite(axis.torque_estimate_) && std::abs(axis.torque_estimate_) > park_max_torque_) {
                RCLCPP_ERROR(logger, "Parking: %s torque %.2f Nm is beyond %.2f Nm, holding it at %.1f mm",
                    name, axis.torque_estimate_, park_max_torque_, axis.pos_estimate_ * 1000);
                Set_Input_Pos_msg_t hold;
                hold.Input_Pos = axis.pos_estimate_ / 0.005;
                axis.send(hold);
                stopped[k] = true;
                continue;
            }
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = (start[k] + (target[k] - start[k]) * frac) / 0.005;
            axis.send(msg);
            if (frac < 1.0 || !(std::abs(axis.pos_estimate_ - target[k]) < 0.001)) {
                all_there = false;
            }
        }
        if (all_there) {
            break;
        }
        if (elapsed > std::max(park_timeout_, duration + 0.5)) {
            timed_out = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (size_t k = 0; k < legs.size(); ++k) {
        Axis& axis = axes_[legs[k]];
        RCLCPP_INFO(logger, "Parking: %s at %.1f mm (lower soft limit %.1f mm)",
            info_.joints[legs[k]].name.c_str(), axis.pos_estimate_ * 1000, target[k] * 1000);
        if (idle_when_unused_) {
            Set_Axis_State_msg_t idle_msg;
            idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
            axis.send(idle_msg);
        }
    }
    if (timed_out) {
        RCLCPP_WARN(logger, "Parking: not all legs reached their lower limit within %.1f s", park_timeout_);
    }
    RCLCPP_INFO(logger, "Parking done, legs %s", idle_when_unused_ ? "set to idle" : "holding their position");
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");

    // This can be called several seconds before the controller finishes starting. Until then every
    // axis holds the position it has now (or goes idle with idle_when_unused), which needs the
    // current position estimates first.
    read_estimates(0.5);

    active_ = true;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

// Process incoming CAN messages until every axis has a position estimate, or the timeout passes.
void ODriveHardwareInterface::read_estimates(double timeout_s) {
    auto const end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < end) {
        while (can_intf_.read_nonblocking()) {
        }
        if (std::all_of(axes_.begin(), axes_.end(), [](Axis const& a) { return std::isfinite(a.pos_estimate_); })) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

    active_ = false;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_ 
        ));
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}};

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            set_axis_command_mode(axis);
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    std::unique_lock<std::mutex> lock(io_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !can_open_) {
        return return_type::OK;     // parking on shutdown owns the CAN socket
    }
    timestamp_ = timestamp;

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    if (parked_) {
        return return_type::OK;     // shutting down: the legs stay parked, controllers don't move them again
    }
    std::unique_lock<std::mutex> lock(io_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !can_open_) {
        return return_type::OK;
    }
    auto const logger = rclcpp::get_logger("ODriveHardwareInterface");
    for (auto& axis : axes_) {
        // A velocity or torque that would push the leg further past one of its limits is not sent
        bool const at_max = std::isfinite(axis.pos_estimate_) && axis.pos_estimate_ >= axis.max_pos_;
        bool const at_min = std::isfinite(axis.pos_estimate_) && axis.pos_estimate_ <= axis.min_pos_;
        auto const limited = [&](double value) {
            if (!std::isfinite(value) || (at_max && value > 0.0) || (at_min && value < 0.0)) {
                return 0.0;
            }
            return value;
        };

        // Send the CAN message that fits the set of enabled setpoints
        if (axis.pos_input_enabled_) {
            double pos = axis.pos_setpoint_;
            if (!std::isfinite(pos)) {
                // No valid command yet (e.g. a controller that hasn't received one): hold where the leg is
                if (!std::isfinite(axis.pos_estimate_)) {
                    continue;
                }
                pos = axis.pos_estimate_;
            }
            double const clamped = std::clamp(pos, axis.min_pos_, axis.max_pos_);
            if (clamped != pos && !axis.limit_warned_) {
                RCLCPP_WARN(logger, "Node %u: position command %.1f mm is outside %.1f .. %.1f mm, limited to %.1f mm",
                    axis.node_id_, pos * 1000, axis.min_pos_ * 1000, axis.max_pos_ * 1000, clamped * 1000);
            }
            axis.limit_warned_ = clamped != pos;
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = clamped / (2 * M_PI) * (2 * M_PI) / 0.005;
            msg.Vel_FF = axis.vel_input_enabled_ ? (limited(axis.vel_setpoint_) / (2 * M_PI) * (2 * M_PI) / 0.005) : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? limited(axis.torque_setpoint_) : 0.0f;
            axis.send(msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel = limited(axis.vel_setpoint_) / (2 * M_PI) * (2 * M_PI) / 0.005;
            msg.Input_Torque_FF = axis.torque_input_enabled_ ? limited(axis.torque_setpoint_) : 0.0f;
            axis.send(msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = limited(axis.torque_setpoint_);
            axis.send(msg);
        } else if (axis.holding_) {
            // No controller: keep sending the held position (keeps the ODrive watchdog fed, if enabled)
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = axis.hold_pos_ / 0.005;
            msg.Vel_FF = 0.0f;
            msg.Torque_FF = 0.0f;
            axis.send(msg);
        } else {
            // no control enabled - don't send any setpoint
        }
    }

    return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

// Called when no controller is using the axis. Holds its current position in position control
// (without clearing errors, so an ODrive that tripped on a fault stays off), or switches it to idle
// with idle_when_unused or when its position is unknown.
void ODriveHardwareInterface::hold_or_idle(Axis& axis) {
    auto const logger = rclcpp::get_logger("ODriveHardwareInterface");
    if (idle_when_unused_ || !std::isfinite(axis.pos_estimate_)) {
        RCLCPP_INFO(logger, "Node %u: no controller%s, setting axis to idle.", axis.node_id_,
            std::isfinite(axis.pos_estimate_) ? "" : " and no position estimate");
        axis.holding_ = false;
        Set_Axis_State_msg_t idle_msg;
        idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(idle_msg);
        return;
    }
    axis.hold_pos_ = axis.pos_estimate_;
    axis.holding_ = true;
    RCLCPP_INFO(logger, "Node %u: no controller, holding position %.1f mm.", axis.node_id_, axis.hold_pos_ * 1000);

    Set_Controller_Mode_msg_t control_msg;
    control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
    control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
    axis.send(control_msg);
    Set_Input_Pos_msg_t pos_msg;
    pos_msg.Input_Pos = axis.hold_pos_ / 0.005;
    pos_msg.Vel_FF = 0.0f;
    pos_msg.Torque_FF = 0.0f;
    axis.send(pos_msg);
    Set_Axis_State_msg_t state_msg;
    state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;
    axis.send(state_msg);
}

void ODriveHardwareInterface::set_axis_command_mode(Axis& axis) {
    if (parked_) {
        return;     // parked on shutdown: leave the legs where parking put them
    }
    if (!active_) {
        // Launch stopping / hardware deactivated: hold (or idle) so the legs don't drop
        hold_or_idle(axis);
        axis.holding_ = false;      // write() isn't called any more after this
        return;
    }

    Set_Controller_Mode_msg_t control_msg;
    Clear_Errors_msg_t clear_error_msg;
    Set_Axis_State_msg_t state_msg;

    clear_error_msg.Identify = 0;
    control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
    state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

    if (axis.pos_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to position control.");
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
        if (trap_traj_) {
            // Smooth moves limited by the trajectory planner's vel/accel/decel limits
            control_msg.Input_Mode = INPUT_MODE_TRAP_TRAJ;
            if (std::isfinite(traj_vel_limit_)) {
                Set_Traj_Vel_Limit_msg_t vel_msg;
                vel_msg.Traj_Vel_Limit = traj_vel_limit_ / 0.005;               // m/s -> turns/s
                axis.send(vel_msg);
            }
            if (std::isfinite(traj_accel_limit_)) {
                Set_Traj_Accel_Limits_msg_t acc_msg;
                acc_msg.Traj_Accel_Limit = traj_accel_limit_ / 0.005;           // m/s^2 -> turns/s^2
                acc_msg.Traj_Decel_Limit = traj_accel_limit_ / 0.005;
                axis.send(acc_msg);
            }
        }
    } else if (axis.vel_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to velocity control.");
        control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    } else if (axis.torque_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to torque control.");
        control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    } else {
        // No controller using this axis (start-up, or between controllers): hold it, don't let it drop
        hold_or_idle(axis);
        return;
    }

    axis.holding_ = false;
    axis.send(control_msg);
    axis.send(clear_error_msg);
    axis.send(state_msg);
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Heartbeat_msg_t::cmd_id: {
            if (Heartbeat_msg_t msg; try_decode(msg)) {
                axis_state_ = msg.Axis_State;
                have_heartbeat_ = true;
            }
        } break;
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI) * 0.005 / (2 * M_PI);
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI) * 0.005 / (2 * M_PI);
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
