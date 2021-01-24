#ifndef CAM_TOOLS_CAM_CALIBRATION_HPP_
#define CAM_TOOLS_CAM_CALIBRATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string>
#include <vector>

namespace cam_tools
{
  class CamCalibration : public rclcpp::Node 
  {
  public:
    explicit CamCalibration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~CamCalibration() = default;
  
  private:
    bool help(const std::vector<std::string> args);
    void parse_parameters();

    void load_image_lists();
    // cv::Mat image_calibration(cv::Mat &src);

  private:
    std::string imagesPath_;
    std::vector<std::string> imageLists_;

  }; // class CamCalibration
} // namespace cam_tools

#endif // CAM_TOOLS_CAM_CALIBRATION_HPP_