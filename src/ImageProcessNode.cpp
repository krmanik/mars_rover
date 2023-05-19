#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageProcessNode : public rclcpp::Node
{
public:
  ImageProcessNode()
  : Node("image_process_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_process", 10, std::bind(&ImageProcessNode::imageCallback, this, std::placeholders::_1));

    image_process_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_analysis", 10);

    RCLCPP_INFO(this->get_logger(), "ImageProcessNode created");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert ROS image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;

    // Convert the image to black and white
    cv::Mat bwImage;
    cv::cvtColor(image, bwImage, cv::COLOR_BGR2GRAY);

    // Convert the black and white image back to a ROS message
    cv_bridge::CvImage bwImageMsg;
    bwImageMsg.header = msg->header;
    bwImageMsg.encoding = sensor_msgs::image_encodings::MONO8;
    bwImageMsg.image = bwImage;    

    sensor_msgs::msg::Image::SharedPtr imagePtr = bwImageMsg.toImageMsg();
    image_process_publisher_->publish(*imagePtr);

    RCLCPP_INFO(this->get_logger(), "Published processd image to image_analysis topic");
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_process_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto image_process_node = std::make_shared<ImageProcessNode>();

  rclcpp::spin(image_process_node);
  rclcpp::shutdown();
  return 0;
}
