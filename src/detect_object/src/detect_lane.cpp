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

    // Perspective Transform param
    this->declare_parameter<float>("top_x", 70);
    this->declare_parameter<float>("top_y", -40);
    this->declare_parameter<float>("bottom_x", 330);
    this->declare_parameter<float>("bottom_y", 240);

    get_parameter_or<bool>(
      "show_image",
      cfg_.showImage,
      true);
    
    get_parameter_or<float>(
      "top_x",
      cfg_.birdView.top_x,
      70);
    
    get_parameter_or<float>(
      "top_y",
      cfg_.birdView.top_y,
      -40);
    
    get_parameter_or<float>(
      "bottom_x",
      cfg_.birdView.bottom_x,
      330);
    
    get_parameter_or<float>(
      "bottom_y",
      cfg_.birdView.bottom_y,
      240);
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
      
      // esc keyboard key = 27
      if(cv::waitKey(3) == 27)
      {
        rclcpp::shutdown();
      }
    }
  }

  void DetectLane::homography_transform_process()
  {
    if(!src_.empty())
    {
      // src size(640, 480)
    cv::Point2f centerPoint{src_.size()/2};
    cv::Point2f srcVertices[4]{
      {centerPoint.x - cfg_.birdView.top_x, centerPoint.y - cfg_.birdView.top_y},
      {centerPoint.x + cfg_.birdView.top_x, centerPoint.y - cfg_.birdView.top_y},
      {centerPoint.x + cfg_.birdView.bottom_x, centerPoint.y + cfg_.birdView.bottom_y},
      {centerPoint.x - cfg_.birdView.bottom_x, centerPoint.y + cfg_.birdView.bottom_y}
    };

    // dst size(250, 300)
    // [200, 0], [800, 0], [800, 600], [200, 600]
    cv::Point2f dstVertices[4]{
      {50, 0},
      {200, 0},
      {200, 300},
      {50, 300}
    };

    // draw line for bird view
    for(int idx{0}; idx < 4; ++idx)
    {
      cv::line( src_, srcVertices[idx], srcVertices[(idx+1)%4], cv::Scalar( 255, 0, 0 ), 3, cv::LINE_8);
    }

    cv::Mat perspectiveMatrix{getPerspectiveTransform(srcVertices, dstVertices)};
    cv::warpPerspective(
      src_, dst_, perspectiveMatrix,
      cv::Size(250, 300), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }
  }

  void DetectLane::process()
  { 
    homography_transform_process();
    image_show(dst_, cfg_.showImage);
    // image_show(src_, cfg_.showImage);
  }
} // detect_object

RCLCPP_COMPONENTS_REGISTER_NODE(detect_object::DetectLane)