#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuCovarianceNode : public rclcpp::Node
{
public:
  ImuCovarianceNode() : Node("imu_covariance_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_sensor_broadcaster/imu", 10,
      std::bind(&ImuCovarianceNode::imu_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_cov", 10);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto imu_msg = *msg;  // Copy the incoming message

    imu_msg.orientation_covariance = {
        0, 0,    0,
        0,    0, 0,
        0,    0,    0
      };
      
      imu_msg.angular_velocity_covariance = {
        0, 0,    0,
        0,    0, 0,
        0,    0,    0
      };
      
      imu_msg.linear_acceleration_covariance = {
        0, 0,     0,
        0,      0, 0,
        0,      0,     0
      };
      

    pub_->publish(imu_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCovarianceNode>());
  rclcpp::shutdown();
  return 0;
}
