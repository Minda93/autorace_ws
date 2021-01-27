#include "detect_object/detect_lane.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <memory>
#include <chrono>
#include <functional>
#include <string>

namespace detect_object
{
  DetectLane::DetectLane(const rclcpp::NodeOptions &options)
    : Node("detect_lane", options)
  {
    // param
    parse_parameters();

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
      std::bind(&DetectLane::image_callback, this, std::placeholders::_1)
    );
    
    // temporary setting
    startTimer_ = create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&DetectLane::process, this));
  }

  void DetectLane::parse_parameters()
  {
    this->declare_parameter<bool>("show_image", true);

    get_parameter_or<bool>(
      "show_image",
      cfg_.showImage,
      true);
  }

  void DetectLane::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame{
      msg->height, msg->width,
      encoding2mat_type(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step};
    
    if(msg->encoding == "rgb8")
    {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    frame.copyTo(src_);
  }

  int DetectLane::encoding2mat_type(const std::string encoding)
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

  void DetectLane::image_show(const cv::Mat &src, bool showImage)
  {
    if(showImage && !src.empty()){
      cv::imshow("Detect Lane Image", src);
      if(cv::waitKey(3) == 27)
      {
        rclcpp::shutdown();
      }
    }
  }

  void DetectLane::process()
  {
    image_show(src_, cfg_.showImage);
  }
} // detect_object

RCLCPP_COMPONENTS_REGISTER_NODE(detect_object::DetectLane)