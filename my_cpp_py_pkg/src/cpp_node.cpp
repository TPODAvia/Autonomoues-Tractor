#include "my_cpp_py_pkg/cpp_header.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <fstream>

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MyNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    std::ofstream file;
    file.open("/dev/ttyUSB0");
    if (file.is_open())
    {
      file << msg->linear.x << "\n";
      file << msg->linear.y << "\n";
      file << msg->linear.z << "\n";
      file << msg->angular.x << "\n";
      file << msg->angular.y << "\n";
      file << msg->angular.z << "\n";
      file.close();
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}