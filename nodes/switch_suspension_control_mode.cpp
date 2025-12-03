#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

class SwitchSuspensionControlMode : public rclcpp::Node {
public:
    SwitchSuspensionControlMode()
        : Node("switch_suspension_control_mode") {

        // Declare parameters (to be loaded from YAML)
        this->declare_parameter("axis_index", 10);

        // Get parameter values
        axis_index_ = this->get_parameter("axis_index").as_int();

        // Subscriber for joystick input
        joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&SwitchSuspensionControlMode::joy_callback, this, std::placeholders::_1));

        // Create service client for the switch_controller service
        switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

        // Wait for the service to be available
        while (!switch_controller_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for switch_controller service...");
        }
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->buttons.size() > 10 && msg->buttons[10] == 1) {
            // Button 10 pressed, switch the controller
            switch_controller("linear_position_control_joy", "suspension_controller");
        }
    }

    void switch_controller(const std::string& activate, const std::string& deactivate) {
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->start_controllers.push_back(activate);
        request->stop_controllers.push_back(deactivate);
        request->strict = true;

        // Call the service to switch controllers
        auto future = switch_controller_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Successfully switched controllers.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch controllers.");
        }
    }

    int axis_index_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwitchSuspensionControlMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
