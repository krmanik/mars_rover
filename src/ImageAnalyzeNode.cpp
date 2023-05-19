#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageAnalyzeNode : public rclcpp::Node
{
public:
  ImageAnalyzeNode()
  : Node("image_analyze_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_analyze", 10, std::bind(&ImageAnalyzeNode::imageCallback, this, std::placeholders::_1));
    
    final_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("final_image", 10);

    RCLCPP_INFO(this->get_logger(), "ImageAnalyzeNode created");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;
    cv::Mat binaryImage;
    cv::threshold(image, binaryImage, 128, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int numContours = contours.size();

    final_image_publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Number of contours: %d", numContours);
    RCLCPP_INFO(this->get_logger(), "Image analysis completed and final image published");
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr final_image_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto image_analyze_node = std::make_shared<ImageAnalyzeNode>();

  rclcpp::spin(image_analyze_node);
  rclcpp::shutdown();
  return 0;
}
