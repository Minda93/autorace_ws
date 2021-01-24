#include "cam_tools/cam_calibration.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace cam_tools
{
  CamCalibration::CamCalibration(const rclcpp::NodeOptions &options)
    : Node("cam_calibration", options)
  {

  }
  
} // namespace cam_tools

RCLCPP_COMPONENTS_REGISTER_NODE(cam_tools::CamCalibration)