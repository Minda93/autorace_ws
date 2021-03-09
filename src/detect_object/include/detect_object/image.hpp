#ifndef DETECT_OBJECT_IMAGE_HPP_
#define DETECT_OBJECT_IMAGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string>

namespace detect_object
{
  class Image : public rclcpp::Node
  {
  public:
    explicit Image(
      const std::string &name = "detect_image", 
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~Image() = default;
  
  protected:
    // show image
    void image_show(
      const cv::Mat &src, 
      bool showImage, 
      const std::string &windowName = "detect_image");

    // process
    virtual void process(const cv::Mat &src);

  protected:
    // subscriber
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // encoding image to mat type
    int encoding2mat_type(const std::string encoding);

  protected:
    // subscribe
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  private:
    bool showImage_;
  }; // class Image

} // namespace detect_object

#endif // DETECT_OBJECT_IMAGE_HPP_