#include "detect_object/image.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string>

namespace detect_object
{
  Image::Image(const std::string &name, const rclcpp::NodeOptions &options)
    : Node(name, options)
  {
    // param 
    showImage_ = false;

    // topic
    auto qosSensor = rclcpp::QoS(
      rclcpp::QoSInitialization(
        rmw_qos_profile_sensor_data.history,
        rmw_qos_profile_sensor_data.depth
      ),
      rmw_qos_profile_sensor_data
    );

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      qosSensor,
      std::bind(&Image::image_callback, this, std::placeholders::_1)
    );
  }

  void Image::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame{
      msg->height, msg->width,
      encoding2mat_type(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step};
    
    if(msg->encoding == "rgb8")
    {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    process(frame);
  }

  int Image::encoding2mat_type(const std::string encoding)
  {
    if (encoding == "mono8") {
      return CV_8UC1;
    } else if (encoding == "bgr8") {
      return CV_8UC3;
    } else if (encoding == "mono16") {
      return CV_16SC1;
    } else if (encoding == "rgba8") {
      return CV_8UC4;
    } else if (encoding == "bgra8") {
      return CV_8UC4;
    } else if (encoding == "32FC1") {
      return CV_32FC1;
    } else if (encoding == "rgb8") {
      return CV_8UC3;
    } else {
      throw std::runtime_error("Unsupported encoding type");
    }
  }

  void Image::image_show(
    const cv::Mat &src, bool showImage, const std::string &windowName)
  {
    if(showImage && !src.empty()){
      cv::imshow(windowName, src);
      
      // esc keyboard key = 27
      if(cv::waitKey(3) == 27)
      {
        cv::destroyWindow(windowName);
        rclcpp::shutdown();
      }
    }
  }

  void Image::process(const cv::Mat &src)
  {
    if(!src.empty())
    {
      image_show(src, showImage_);
    }
  }
} // namespace detect_object