#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

class LinearPositionPub : public rclcpp::Node {
public:
    LinearPositionPub()
        : Node("linear_position_pub") {

        // Declare parameters (to be loaded from YAML)
        this->declare_parameter("min_value", -0.005);
        this->declare_parameter("max_value", 0.10);
        this->declare_parameter("axis_index", 4);

        // Get parameter values
        min_value_ = this->get_parameter("min_value").as_double();
        max_value_ = this->get_parameter("max_value").as_double();
        axis_index_ = this->get_parameter("axis_index").as_int();

        // Publisher for joint positions
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/linear_position_control_joy/commands", 10);

        // Subscriber for joystick input
        joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&LinearPositionPub::joy_callback, this, std::placeholders::_1));

        // Print out information
        // RCLCPP_INFO(this->get_logger(), "LinearPositionPub initialized with min_value: %.2f, max_value: %.2f, axis_index: %d", min_value_, max_value_, axis_index_);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() > static_cast<std::size_t>(axis_index_))  {
            double joystick_value = msg->axes[axis_index_];

            // map joystick input (-1 to 1) into (min_value to max_value)
            double scaled_value = ((joystick_value + 1) / 2) * (max_value_ - min_value_) + min_value_;

            // publish the values
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {scaled_value, scaled_value, scaled_value, scaled_value};
            publisher_->publish(message);

        } else {
            RCLCPP_WARN(this->get_logger(), "Joystick message does not contain enough axes.");
        }
    }

    double min_value_, max_value_;
    int axis_index_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinearPositionPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}