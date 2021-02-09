#ifndef DETECT_OBJECT_DETECT_LANE_HPP_
#define DETECT_OBJECT_DETECT_LANE_HPP_

#include "detect_object/detect_param.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <memory>
#include <string>

namespace detect_object
{
  class DetectLane : public rclcpp::Node 
  {
  public:
    explicit DetectLane(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~DetectLane() = default;

    void image_show(const cv::Mat &src, bool showImage);
    void process();
  
  private:
    // init param
    void parse_parameters();
    rcl_interfaces::msg::SetParametersResult dynamic_load_params(
      const std::vector<rclcpp::Parameter> &params);

    // subscribe image
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    // encoding image to mat type
    int encoding2mat_type(const std::string encoding);

    // process
    void homography_transform_process();
    void detect_lane();

  private:
    // param
    DetectLaneParam cfg_;
    BaseMode mode_;
    cv::Mat src_;
    cv::Mat dst_;

    // time
    rclcpp::TimerBase::SharedPtr startTimer_;
    
    // topic 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  };
} // class DetectLane

#endif // DETECT_OBJECT_DETECT_LANE_HPP_