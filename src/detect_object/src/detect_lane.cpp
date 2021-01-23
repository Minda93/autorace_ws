#include "detect_object/detect_lane.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace detect_object
{
  DetectLane::DetectLane(const rclcpp::NodeOptions &options)
    : Node("detect_lane", options)
  {

  }
} // detect_object

RCLCPP_COMPONENTS_REGISTER_NODE(detect_object::DetectLane)