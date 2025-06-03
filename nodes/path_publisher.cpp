#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class OdomPathPublisher : public rclcpp::Node
{
public:
  OdomPathPublisher()
  : Node("odom_path_publisher")
  {
    path_msg_.header.frame_id = "odom";

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OdomPathPublisher::odom_callback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/odom_path", 10);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    path_msg_.poses.push_back(pose);
    path_msg_.header.stamp = msg->header.stamp;
    diff_cont              diff_drive_controller/DiffDriveController      inactive

    path_pub_->publish(path_msg_);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
