#ifndef CAM_TOOLS_CAM_CALIBRATION_HPP_
#define CAM_TOOLS_CAM_CALIBRATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace cam_tools
{
  class CamCalibration : public rclcpp::Node 
  {
  public:
    explicit CamCalibration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~CamCalibration() = default;
  
  private:
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  }; // class CamCalibration
} // namespace cam_tools

#endif // CAM_TOOLS_CAM_CALIBRATION_HPP_