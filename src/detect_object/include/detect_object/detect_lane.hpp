#ifndef DETECT_OBJECT_DETECT_LANE_HPP_
#define DETECT_OBJECT_DETECT_LANE_HPP_

#include "detect_object/image.hpp"
#include "detect_object/detect_param.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <memory>
#include <string>
#include <vector>

namespace detect_object
{
  class DetectLane : public Image 
  {
  public:
    explicit DetectLane(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~DetectLane() = default;
  
  protected:
    virtual void process(const cv::Mat &src) override;
  
  protected:
    // init param
    void parse_parameters();
    rcl_interfaces::msg::SetParametersResult dynamic_load_params(
      const std::vector<rclcpp::Parameter> &params);

    // process
    void homography_transform_process(const cv::Mat &src, cv::Mat &dst);
    void mask_lane(const cv::Mat &src, cv::Mat &maskYellow, cv::Mat &maskWhite);
    void smooth_image();
    void sliding_window(const cv::Mat &src, const std::string &left_or_right, cv::Mat &dst);
    void line_fitting(const cv::Mat &src, cv::Mat &lane_fit, std::vector<float> &lane_fitX, cv::Mat &dst);
    void make_lane(cv::Mat &dst);

    // math algorithm
    cv::Mat polyfit(const std::vector<cv::Point2f> &points, int order, bool choose_x_input = true);
    std::vector<float> generate_lane_plotting(const cv::Mat &lane_fit, size_t size);
    
    // topic
    void pub(double centerX);

  private:
    // param
    DetectLaneParam cfg_;
    bool lossLane_;
    
    cv::Mat yellowLaneFit_;
    std::vector<float> yellowLaneFitX_;

    cv::Mat whiteLaneFit_;
    std::vector<float> whiteLaneFitX_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lane_center_pub_;
  };
} // class DetectLane

#endif // DETECT_OBJECT_DETECT_LANE_HPP_