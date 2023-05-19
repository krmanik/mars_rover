#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));
    
    image_analyze_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_process", 10);

    RCLCPP_INFO(this->get_logger(), "ImageSubscriberNode created");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Perform image processing operations using OpenCV
    cv::Mat image = cv_ptr->image;

    // Crop the image
    // Example region of interest (x, y, width, height)
    cv::Rect roi(100, 100, 200, 200);
    cv::Mat croppedImage = image(roi);

    // Convert the cropped image back to a ROS message
    cv_bridge::CvImage croppedImageMsg;
    croppedImageMsg.header = msg->header;
    croppedImageMsg.encoding = sensor_msgs::image_encodings::BGR8;
    croppedImageMsg.image = croppedImage;

    sensor_msgs::msg::Image::SharedPtr imagePtr = croppedImageMsg.toImageMsg();
    image_analyze_publisher_->publish(*imagePtr);

    RCLCPP_INFO(this->get_logger(), "Preprocessed and published for processing");
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_analyze_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto image_subscriber_node = std::make_shared<ImageSubscriber>();

  rclcpp::spin(image_subscriber_node);
  rclcpp::shutdown();
  return 0;
}
