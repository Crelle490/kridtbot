#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SquareDriver : public rclcpp::Node
{
public:
  SquareDriver() : Node("square_driver"), step_(0)
  {
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mecanum_drive_controller/reference", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&SquareDriver::timer_callback, this));
    state_start_time_ = this->now();
  }

private:
  void timer_callback()
  {
    rclcpp::Time now = this->now();
    double elapsed = (now - state_start_time_).seconds();

    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = now;
    msg.header.frame_id = "base_link";  

    if (step_ < 8)
    {
      if (step_ % 2 == 0)  // Drive forward
      {
        if (elapsed < forward_duration_)
        {
          msg.twist.linear.x = 0.2;
        }
        else
        {
          step_++;
          state_start_time_ = now;
          return;
        }
      }
      else  // Turn 90 degrees
      {
        if (elapsed < turn_duration_)
        {
          msg.twist.angular.z = 0.5;
        }
        else
        {
          step_++;
          state_start_time_ = now;
          return;
        }
      }
    }
    else
    {
      pub_->publish(geometry_msgs::msg::TwistStamped());  // Stop
      RCLCPP_INFO(this->get_logger(), "Square path complete.");
      timer_->cancel();
      return;
    }

    pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time state_start_time_;
  int step_;
  const double forward_duration_ = 5.0;
  const double turn_duration_ = 3.2;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareDriver>());
  rclcpp::shutdown();
  return 0;
}
