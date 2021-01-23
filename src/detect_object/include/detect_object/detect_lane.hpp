#ifndef DETECT_OBJECT_DETECT_LANE_HPP_
#define DETECT_OBJECT_DETECT_LANE_HPP_

#include "detect_object/detect_param.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <memory>

namespace detect_object
{
  class DetectLane : public rclcpp::Node 
  {
  public:
    explicit DetectLane(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~DetectLane() = default;
  
  private:
    DetectLaneParam config_;
    BaseMode mode_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  };
} // class DetectLane

#endif // DETECT_OBJECT_DETECT_LANE_HPP_