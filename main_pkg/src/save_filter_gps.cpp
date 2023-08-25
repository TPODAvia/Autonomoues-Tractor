#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GPSSubscriber : public rclcpp::Node
{
public:
  GPSSubscriber() : Node("gps_subscriber")
  {
    // Create the subscriber
    subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/filtered", 10, std::bind(&GPSSubscriber::gpsCallback, this, std::placeholders::_1));
  }

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // Save the data to a file
    std::ofstream file("/home/ubuntu/file.txt", std::ios::app);
    if (file.is_open())
    {
      file << "Latitude: " << msg->latitude << std::endl;
      file << "Longitude: " << msg->longitude << std::endl;
      file << "Altitude: " << msg->altitude << std::endl;
      file.close();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to open file");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GPSSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}