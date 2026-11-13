#include <algorithm>
#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

// Joystick height control for the suspension legs.
//
// Triangle / cross raise / lower the base height while held (height_speed), the right stick tilts the
// rover (up to +-tilt_range per leg). Every leg's setpoint moves towards its target at most max_leg_speed,
// so the suspension moves smoothly and slowly enough with the arm mounted. The setpoints start at the
// legs' real positions (from /joint_states), so starting the launch doesn't make the legs jump.
class LinearPositionPub : public rclcpp::Node {
public:
    LinearPositionPub()
        : Node("linear_position_pub") {

        // Declare parameters (to be loaded from YAML)
        this->declare_parameter("min_value", 0.0);
        this->declare_parameter("max_value", 0.10);
        this->declare_parameter("axis_index_x", 3);
        this->declare_parameter("axis_index_y", 4);
        this->declare_parameter("height_speed", 0.005);    // [m/s] base height change while a button is held
        this->declare_parameter("max_leg_speed", 0.01);    // [m/s] fastest any leg setpoint may move
        this->declare_parameter("tilt_range", 0.1);        // [m] leg offset at full stick deflection

        // Get parameter values
        min_value_ = this->get_parameter("min_value").as_double();
        max_value_ = this->get_parameter("max_value").as_double();
        axis_x_index_ = this->get_parameter("axis_index_x").as_int();
        axis_y_index_ = this->get_parameter("axis_index_y").as_int();
        height_speed_ = this->get_parameter("height_speed").as_double();
        max_leg_speed_ = this->get_parameter("max_leg_speed").as_double();
        tilt_range_ = this->get_parameter("tilt_range").as_double();

        button_cross_index_ = 0;
        triangle_cross_index_ = 2;

        // Publisher for joint positions
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/linear_position_control_joy/commands", 10);

        // Subscriber for joystick input
        joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&LinearPositionPub::joy_callback, this, std::placeholders::_1));

        // Subscriber for the legs' real positions (only used to start from them)
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&LinearPositionPub::joint_state_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&LinearPositionPub::update, this));
        last_update_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Suspension height control: %.1f..%.1f mm, height %.1f mm/s, legs at most %.1f mm/s, "
            "waiting for the legs' positions on /joint_states...", min_value_ * 1000, max_value_ * 1000,
            height_speed_ * 1000, max_leg_speed_ * 1000);
    }

private:
    // Same order as the joints of linear_position_control_joy
    const std::array<std::string, 4> joint_names_ {"left_front_linear", "left_rear_linear", "right_front_linear", "right_rear_linear"};
    std::array<double, 4> setpoints_ {};
    std::array<bool, 4> have_position_ {};
    bool initialized_ = false;

    double static_linear_position_ = 0.0;   // base height
    double x_axis_value_ = 0.0, y_axis_value_ = 0.0;
    double raise_ = 0.0, lower_ = 0.0;
    double min_value_, max_value_, height_speed_, max_leg_speed_, tilt_range_;
    int button_cross_index_, triangle_cross_index_, axis_x_index_, axis_y_index_;
    rclcpp::Time last_update_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (initialized_) {
            return;
        }
        for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            for (std::size_t j = 0; j < joint_names_.size(); ++j) {
                if (msg->name[i] == joint_names_[j]) {
                    setpoints_[j] = msg->position[i];
                    have_position_[j] = true;
                }
            }
        }
        if (std::all_of(have_position_.begin(), have_position_.end(), [](bool b) { return b; })) {
            double sum = 0.0;
            for (double p : setpoints_) {
                sum += p;
            }
            static_linear_position_ = std::clamp(sum / setpoints_.size(), min_value_, max_value_);
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting from the legs' positions: LF %.1f LR %.1f RF %.1f RR %.1f mm",
                setpoints_[0] * 1000, setpoints_[1] * 1000, setpoints_[2] * 1000, setpoints_[3] * 1000);
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() > static_cast<std::size_t>(std::max(axis_x_index_, axis_y_index_)) &&
            msg->buttons.size() > static_cast<std::size_t>(std::max(button_cross_index_, triangle_cross_index_))) {
            lower_ = msg->buttons[button_cross_index_];
            raise_ = msg->buttons[triangle_cross_index_];
            x_axis_value_ = msg->axes[axis_x_index_];
            y_axis_value_ = msg->axes[axis_y_index_];
        } else {
            RCLCPP_WARN(this->get_logger(), "Joystick message does not contain enough axes/buttons.");
        }
    }

    void update() {
        auto const now = this->now();
        double const dt = std::clamp((now - last_update_).seconds(), 0.0, 0.1);
        last_update_ = now;
        if (!initialized_) {
            return;
        }

        // Base height moves at height_speed while triangle (up) or cross (down) is held
        static_linear_position_ += (raise_ - lower_) * height_speed_ * dt;
        static_linear_position_ = std::clamp(static_linear_position_, min_value_, max_value_);

        // Target of each leg: base height plus the stick tilt
        double const x = x_axis_value_ * tilt_range_;
        double const y = y_axis_value_ * tilt_range_;
        std::array<double, 4> const targets {
            static_linear_position_ - x - y,   // left front
            static_linear_position_ - x + y,   // left rear
            static_linear_position_ + x - y,   // right front
            static_linear_position_ + x + y};  // right rear

        // Move every setpoint towards its target at no more than max_leg_speed
        double const max_step = max_leg_speed_ * dt;
        for (std::size_t i = 0; i < setpoints_.size(); ++i) {
            double const target = std::clamp(targets[i], min_value_, max_value_);
            setpoints_[i] += std::clamp(target - setpoints_[i], -max_step, max_step);
        }

        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {setpoints_.begin(), setpoints_.end()};
        publisher_->publish(message);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinearPositionPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
