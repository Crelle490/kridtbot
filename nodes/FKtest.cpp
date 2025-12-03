#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class WheelVelocityPublisher : public rclcpp::Node
{
public:
  WheelVelocityPublisher() : Node("FKtest"), step_(0)
  {
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/one_wheel_trajectory/commands",10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&WheelVelocityPublisher::publish_commands, this));

    state_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Initialized FKtest node for square driving.");
  }
  ~WheelVelocityPublisher()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {0.0, 0.0, 0.0, 0.0};  // Stop all wheels
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Shutting down and stopping wheels.");
  }

private:
  void publish_commands()

  {
    std_msgs::msg::Float64MultiArray msg;
    rclcpp::Time now = this->now();
    double elapsed = (now - state_start_time_).seconds();

    if (step_ >= 1)
    {
      msg.data = {0.0, 0.0, 0.0, 0.0};  // Stop
      pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Finished square path.");
      timer_->cancel();
      return;
    }

    if (step_ % 2 == 0)
    {
      // Drive straight
      if (elapsed < forward_duration_)
      {
        //msg.data = {-2.22, -2.22, 2.22, 2.22};
        msg.data = {-4.44, -4.44, -4.44, -4.44};
      }
      else
      {
        step_++;
        state_start_time_ = now;
        return;
      }
    }
    else
    {
      // Turn in place
      if (elapsed < turn_duration_)
      {
        //msg.data = {direction_one_*4.44, direction_one_*4.44, -direction_one_*4.44, -direction_one_*4.44};
        //msg.data = {-direction_two_*2.22, direction_two_*2.22, direction_two_*2.22, -direction_two_*2.22};  // Left wheels reverse, right wheels forward
        
      }
      else
      {
        step_++;
        state_start_time_ = now;
        return;
      }
    }

    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time state_start_time_;
  int step_;
  const double forward_duration_ = 4.0;
  const double turn_duration_ = 2.55;
  double direction_one_ = 1.0;  
  double direction_two_ = 1.0; 
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocityPublisher>());
  rclcpp::shutdown();
  return 0;
}
