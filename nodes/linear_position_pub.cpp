#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

class LinearPositionPub : public rclcpp::Node {
public:
    LinearPositionPub()
        : Node("linear_position_pub"), static_linear_position_(0.0) {

        // Declare parameters (to be loaded from YAML)
        this->declare_parameter("min_value", 0.0);
        this->declare_parameter("max_value", 0.10);
        this->declare_parameter("axis_index_x", 3);
        this->declare_parameter("axis_index_y", 4);

        // Get parameter values
        min_value_ = this->get_parameter("min_value").as_double();
        max_value_ = this->get_parameter("max_value").as_double();
        axis_x_index_ = this->get_parameter("axis_index_x").as_int();
        axis_y_index_ = this->get_parameter("axis_index_y").as_int();

        button_cross_index_ = 0;
        triangle_cross_index_ = 2;

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
    double static_linear_position_; // Make variable presisntent acorss function calls
    double RF_pos_, RR_pos_, LF_pos_, LR_pos_;
    double min_value_, max_value_;
    int button_cross_index_, triangle_cross_index_, axis_x_index_, axis_y_index_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() > static_cast<std::size_t>(axis_x_index_))  {
            //double joystick_value = msg->axes[axis_index_linear_];
            //double joystick_value_cross = msg->axes[axis_index_cross_];

            // map joystick input (-1 to 1) into (min_value to max_value)
            //double scaled_value = ((joystick_value + 1) / 2) * (max_value_ - min_value_) + min_value_;
            //double cross_value = ((joystick_value_cross + 1) / 2) * 0.01;
            // publish the values
            //auto message = std_msgs::msg::Float64MultiArray();
            //message.data = {scaled_value, scaled_value, scaled_value, scaled_value};
            double cross_button_value = msg->buttons[button_cross_index_];
            double triangle_button_value = msg->buttons[triangle_cross_index_];
            
            double x_axis_value = msg->axes[axis_x_index_];
            double y_axis_value = msg->axes[axis_y_index_];

            // Update linear position based on button presses
            static_linear_position_ -= cross_button_value * 0.01;
            static_linear_position_ += triangle_button_value * 0.01;

            static_linear_position_ = std::clamp(static_linear_position_, min_value_, max_value_);

            // Determine the individual position of the actuators.
            RF_pos_ = std::clamp(static_linear_position_+x_axis_value*0.1-y_axis_value*0.1, min_value_, max_value_);
            RR_pos_ = std::clamp(static_linear_position_+x_axis_value*0.1+y_axis_value*0.1, min_value_, max_value_);
            LF_pos_ = std::clamp(static_linear_position_-x_axis_value*0.1-y_axis_value*0.1, min_value_, max_value_);
            LR_pos_ = std::clamp(static_linear_position_-x_axis_value*0.1+y_axis_value*0.1, min_value_, max_value_);

            // Create and publish message
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = {LF_pos_,LR_pos_,RF_pos_,RR_pos_};
            publisher_->publish(message);

        } else {
            RCLCPP_WARN(this->get_logger(), "Joystick message does not contain enough axes.");
        }
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinearPositionPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}