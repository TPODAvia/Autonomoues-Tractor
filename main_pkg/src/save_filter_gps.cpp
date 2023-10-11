#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

//sudo apt-get install ros-humble-cv-bridge
class ImageFilterNode : public rclcpp::Node
{
public:
  ImageFilterNode()
  : Node("image_filter_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    publisher1_ = this->create_publisher<sensor_msgs::msg::Image>("oak/rgb/image_sync", 10);
    publisher2_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("oak/rgb/info_sync", 10);
    publisher3_ = this->create_publisher<sensor_msgs::msg::Image>("oak/stereo/image_blur", 10);

    subscription1_ = this->create_subscription<sensor_msgs::msg::Image>(
        "oak/rgb/image_raw", 10, std::bind(&ImageFilterNode::image_callback1, this, std::placeholders::_1));
        
    subscription2_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "oak/rgb/camera_info", 10, std::bind(&ImageFilterNode::camera_info_callback, this, std::placeholders::_1));

    subscription3_ = this->create_subscription<sensor_msgs::msg::Image>(
        "oak/stereo/image_raw", 10, std::bind(&ImageFilterNode::image_callback2, this, std::placeholders::_1));
  }

private:
  sensor_msgs::msg::Image image_msg; // Declare image_msg as a member variable
  sensor_msgs::msg::CameraInfo image_info; // Declare image_info as a member variable
  

  void image_callback1(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    image_msg = *msg;
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    image_info = *msg;
  }

  void image_callback2(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::medianBlur(cv_ptr->image, cv_ptr->image, 5);
    sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", cv_ptr->image).toImageMsg();

    auto current_time = this->get_clock()->now();
    image_msg.header.stamp = current_time;
    image_info.header.stamp = current_time;
    msg_out->header.stamp = current_time;

    publisher1_->publish(image_msg);
    publisher2_->publish(image_info);
    publisher3_->publish(*msg_out);

  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription1_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription3_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher1_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher2_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher3_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageFilterNode>());
  rclcpp::shutdown();
  return 0;
}