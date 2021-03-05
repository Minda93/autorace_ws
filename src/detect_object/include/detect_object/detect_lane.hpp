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
#include <vector>

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
    void homography_transform_process(cv::Mat &src, cv::Mat &dst);
    void mask_lane(cv::Mat &src, cv::Mat &maskYellow, cv::Mat &maskWhite);
    void smooth_image();
    void sliding_window(const cv::Mat &src, const std::string &left_or_right, cv::Mat &dst);
    void line_fitting(const cv::Mat &src, cv::Mat &lane_fit, std::vector<float> &lane_fitX, cv::Mat &dst);

    // 
    cv::Mat polyfit(const std::vector<cv::Point2f> &points, int order, bool choose_x_input = true);
    std::vector<float> generate_lane_plotting(const cv::Mat &lane_fit, size_t size);
    
  private:
    // param
    DetectLaneParam cfg_;
    bool lossLane_;
    cv::Mat src_;
    
    cv::Mat yellowLaneFit_;
    std::vector<float> yellowLaneFitX_;

    cv::Mat whiteLaneFit_;
    std::vector<float> whiteLaneFitX_;

    // time
    rclcpp::TimerBase::SharedPtr startTimer_;
    
    // topic 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  };
} // class DetectLane

#endif // DETECT_OBJECT_DETECT_LANE_HPP_