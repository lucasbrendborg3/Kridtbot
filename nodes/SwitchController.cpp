#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

using std::placeholders::_1;

class ControllerSwitcher : public rclcpp::Node
{
public:
  ControllerSwitcher()
  : Node("controller_switcher"), prev_state_(0), use_suspension_(false)
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&ControllerSwitcher::joy_callback, this, _1));

    client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    // Startup switch to deactivate suspension controller
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ControllerSwitcher::deactivate_suspension_at_start, this)
    );


  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const int button_index = 4;

    if (msg->buttons.size() > button_index &&
        msg->buttons[button_index] == 1 && prev_state_ == 0)
    {
      use_suspension_ = !use_suspension_;
      RCLCPP_INFO(this->get_logger(), "Button toggled. Switching controller to: %s",
                  use_suspension_ ? "suspension_controller" : "linear_position_control_joy");
      switch_controller();
    }

    if (msg->buttons.size() > button_index)
      prev_state_ = msg->buttons[button_index];
  }

  void switch_controller()
  {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    // STRICT: switch both controllers or neither. With BEST_EFFORT, pressing L1 without the
    // suspension_controller loaded (active_suspension:=false) deactivated the joystick height
    // control and left the suspension without any controller.
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    if (use_suspension_) {
      request->deactivate_controllers = {"linear_position_control_joy"};
      request->activate_controllers = {"suspension_controller"};
    } else {
      request->deactivate_controllers = {"suspension_controller"};
      request->activate_controllers = {"linear_position_control_joy"};
    }

    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }

    client_->async_send_request(request,
      [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        if (future.get()->ok) {
          RCLCPP_INFO(this->get_logger(), "Controller switch succeeded.");
        } else {
          use_suspension_ = !use_suspension_;   // nothing changed, so neither does our idea of what's active
          RCLCPP_ERROR(this->get_logger(), "Controller switch failed (is suspension_controller loaded? It needs "
            "active_suspension:=true). Still using: %s", use_suspension_ ? "suspension_controller" : "linear_position_control_joy");
        }
      });
  }

  void deactivate_suspension_at_start()
  {
    if (!client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Startup: service not available yet, retrying...");
      return;
    }
    timer_->cancel();

    // Both controllers are spawned --inactive, so nothing needs deactivating here (and
    // suspension_controller may not even be loaded) - just activate the default, joystick one.
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = {"linear_position_control_joy"};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

    client_->async_send_request(request,
      [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        if (future.get()->ok) {
          RCLCPP_INFO(this->get_logger(), "Startup: linear_position_control_joy activated.");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Startup: controller switch failed.");
        }
      });
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  int prev_state_;
  bool use_suspension_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerSwitcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
