#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

class SwitchSuspensionControlMode : public rclcpp::Node {
public:
    LinearPositionPub()
        : Node("switch_suspension_control_mode") {

        // Declare parameters (to be loaded from YAML)
        this->declare_parameter("axis_index", 10);

        // Get parameter values
        axis_index_ = this->get_parameter("axis_index").as_int();

        // Subscriber for joystick input
        joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&LinearPositionPub::joy_callback, this, std::placeholders::_1));

    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() > static_cast<std::size_t>(axis_index_))  {
            double joystick_value = msg->axes[axis_index_];


        } else {
            RCLCPP_WARN(this->get_logger(), "Joystick message does not contain enough axes.");
        }
    }

    int axis_index_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwitchSuspensionControlMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}