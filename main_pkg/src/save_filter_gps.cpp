#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

class GPSSubscriber : public rclcpp::Node
{
float Velocity_x_ = 0;
float old_timestamp_ = 0;

public:
  GPSSubscriber() : Node("gps_subscriber")
  {
    // Create the subscriber
    subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/filtered", 10, std::bind(&GPSSubscriber::gpsCallback, this, std::placeholders::_1));
    
    // Create a subscriber for the Imu messages
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_topic", 10, std::bind(&GPSSubscriber::imuCallback, this, std::placeholders::_1));
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
      file << "Velocity x: " << Velocity_x_ << std::endl;
      file << "position_covariance: " << msg->position_covariance << std::endl;
      file.close();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to open file");
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Get the current timestamp
    auto now = std::chrono::system_clock::now();
    float new_timestamp = std::chrono::system_clock::to_time_t(now);

    Velocity_x_ = Velocity_x_ + (msg->linear_acceleration.x)*(new_timestamp - old_timestamp_);

  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
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